/*
 * can_rs02.cpp - RS02 一体化电机 CAN 驱动
 * 平台: ESP32-S3 (Arduino框架) / ESP-IDF TWAI
 * 协议: 标准帧 (参考 STM32 已验证版本, 非手册MIT扩展帧)
 *
 * 关键点 (与 STM32 版一致):
 *   - 标准帧 CAN_ID = 电机ID
 *   - 使能/停止/设零: data[0..6]=0xFF, data[7]=子命令 (0xFC/0xFD/0xFE)
 *   - 运控: 12bit 混合打包 (速度/Kp/Kd/力矩各12bit, 位置16bit)
 *   - 反馈: data[0]=电机ID, data[1..2]=角度原始值
 *   - 角度公式: pos_raw/65535 * 25.14 - 12.57
 */

#include "can_rs02.h"
#include "config.h"
#include <Arduino.h>
#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <math.h>
#include <string.h>

/* ==================== 静态变量 ==================== */

float            can_rs02_feedback_rad[3] = {0.0f, 0.0f, 0.0f};
CANMotorFeedback can_rs02_fb[3];

static CANMotorTraj traj_motor[3];
static float cur_rad[3] = {0.0f, 0.0f, 0.0f};
static SemaphoreHandle_t xCANMutex = NULL;
static TaskHandle_t hCANMotorTask = NULL;

/* ==================== 内部函数声明 ==================== */

static uint16_t float_to_uint(float x, float x_min, float x_max, int bits);
static float quintic_smooth(float t);
static float traj_update(CANMotorTraj* t);
static void parse_feedback(uint8_t* data);
static void twai_rx_task(void *pvParameters);

/* ==================== 内部函数实现 ==================== */

static uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (uint16_t)((x - x_min) * ((float)((1 << bits) - 1)) / span);
}

static float quintic_smooth(float t) {
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * t * (10.0f - 15.0f * t + 6.0f * t * t);
}

static float traj_update(CANMotorTraj* t) {
    if (!t->active) return t->qf;
    float elapsed = (millis() - t->start_tick) / 1000.0f;
    float tau = elapsed / t->T;
    if (tau >= 1.0f) {
        t->active = false;
        return t->qf;
    }
    return t->q0 + (t->qf - t->q0) * quintic_smooth(tau);
}

/*
 * 反馈解析 — 与 STM32 版完全一致:
 *   data[0] = 电机ID
 *   data[1..2] = 角度原始值 [0~65535]
 *   公式: angle = pos_raw / 65535 * 25.14 - 12.57
 */
static void parse_feedback(uint8_t* data) {
    uint8_t motor_id = data[0];
    if (motor_id < 1 || motor_id > 2) return;

    uint16_t pos_raw = ((uint16_t)data[1] << 8) | data[2];
    float angle = ((float)pos_raw / 65535.0f) * 25.14f - 12.57f;

    char buf[64];
    snprintf(buf, sizeof(buf), "M%d 反馈: %.2f° (raw=%d)", motor_id, angle * 180.0f / PI, pos_raw);
    Serial.println(buf);

    if (xCANMutex && xSemaphoreTake(xCANMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        can_rs02_feedback_rad[motor_id] = angle;
        can_rs02_fb[motor_id].position_rad = angle;
        xSemaphoreGive(xCANMutex);
    }
}

/* ==================== TWAI 底层发送 ==================== */

/*
 * 标准帧发送 — 与 STM32 的 CAN_Send_Std() 完全一致
 *   StdId = 电机ID
 *   数据 = 8字节
 */
static bool twai_send_std(uint32_t stdId, uint8_t* data) {
    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.identifier = stdId;
    msg.extd = 0;           // 标准帧
    msg.data_length_code = 8;
    memcpy(msg.data, data, 8);
    return twai_transmit(&msg, pdMS_TO_TICKS(5)) == ESP_OK;
}

/* ==================== 接口函数实现 ==================== */

void CAN_RS02_Init(void) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_RS02_TX_PIN,
        (gpio_num_t)CAN_RS02_RX_PIN,
        TWAI_MODE_NORMAL
    );
    g_config.rx_queue_len = CAN_RS02_RX_QUEUE_LEN;
    g_config.tx_queue_len = 10;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("[CAN] TWAI driver install FAILED!");
        return;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("[CAN] TWAI start FAILED!");
        return;
    }
    Serial.println("[CAN] TWAI 1Mbps 标准帧 启动成功");

    xCANMutex = xSemaphoreCreateMutex();

    xTaskCreate(twai_rx_task, "CAN_Rx", 4096, NULL, 4, NULL);

    /* 上电不做任何电机操作，等待用户通过网页手动使能 */
    Serial.println("[CAN] 就绪，请通过网页手动使能电机");

    xTaskCreate(Task_CANMotorDrive, "CAN_Drv", 4096, NULL, 3, &hCANMotorTask);
    Serial.println("[CAN] 初始化完成");
}

/*
 * 使能 — 与 STM32 完全一致: data[7]=0xFC, 其余 0xFF
 */
void CAN_RS02_Enable(uint8_t id) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    twai_send_std(id, data);
}

/*
 * 停止 — 与 STM32 完全一致: data[7]=0xFD, 其余 0xFF
 */
void CAN_RS02_Disable(uint8_t id) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    twai_send_std(id, data);
}

/*
 * 设零 — 与 STM32 完全一致: data[7]=0xFE, 其余 0xFF
 */
void CAN_RS02_SetZero(uint8_t id) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    twai_send_std(id, data);
}

/*
 * MIT 运控 — 与 STM32 完全一致的 12bit 混合打包:
 *   data[0..1] = 位置 16bit  (±12.57 rad)
 *   data[2..3] = 速度 12bit  + Kp高4bit
 *   data[3..4] = Kp低8bit
 *   data[5..6] = Kd 12bit  + 力矩高4bit
 *   data[6..7] = 力矩低8bit
 */
void CAN_RS02_MIT_Ctrl(uint8_t id, float pos_rad, float vel_rads,
                        float kp, float kd, float torq_nm) {
    uint16_t p_int  = float_to_uint(pos_rad, -12.57f, 12.57f, 16);
    uint16_t v_int  = float_to_uint(vel_rads, -44.0f, 44.0f, 12);
    uint16_t kp_int = float_to_uint(kp, 0.0f, 500.0f, 12);
    uint16_t kd_int = float_to_uint(kd, 0.0f, 5.0f, 12);
    uint16_t t_int  = float_to_uint(torq_nm, -17.0f, 17.0f, 12);

    uint8_t data[8];
    data[0] = p_int >> 8;
    data[1] = p_int & 0xFF;
    data[2] = v_int >> 4;
    data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    data[4] = kp_int & 0xFF;
    data[5] = kd_int >> 4;
    data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    data[7] = t_int & 0xFF;

    twai_send_std(id, data);
}

void CAN_RS02_StartTraj(uint8_t id, float target_rad, float duration_sec) {
    if (id < 1 || id > 2) return;
    if (xCANMutex && xSemaphoreTake(xCANMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (duration_sec <= 0.0f) {
            cur_rad[id] = target_rad;
            traj_motor[id].active = false;
            traj_motor[id].qf = target_rad;
            CAN_RS02_MIT_Ctrl(id, target_rad, CAN_RS02_LOCK_VEL,
                               CAN_RS02_LOCK_KP, CAN_RS02_LOCK_KD, CAN_RS02_LOCK_TORQ);
        } else {
            traj_motor[id].q0 = cur_rad[id];
            traj_motor[id].qf = target_rad;
            traj_motor[id].T = duration_sec;
            traj_motor[id].start_tick = millis();
            traj_motor[id].active = true;
        }
        xSemaphoreGive(xCANMutex);
    }
}

bool CAN_RS02_IsMoving(uint8_t id) {
    if (id < 1 || id > 2) return false;
    bool moving = false;
    if (xCANMutex && xSemaphoreTake(xCANMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        moving = traj_motor[id].active;
        xSemaphoreGive(xCANMutex);
    }
    return moving;
}

float CAN_RS02_GetPosition(uint8_t id) {
    if (id < 1 || id > 2) return 0.0f;
    float pos = 0.0f;
    if (xCANMutex && xSemaphoreTake(xCANMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        pos = can_rs02_feedback_rad[id];
        xSemaphoreGive(xCANMutex);
    }
    return pos;
}

void CAN_RS02_ResetTraj(uint8_t id, float rad) {
    if (id < 1 || id > 2) return;
    if (xCANMutex && xSemaphoreTake(xCANMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        cur_rad[id] = rad;
        traj_motor[id].q0 = rad;
        traj_motor[id].qf = rad;
        traj_motor[id].active = false;
        xSemaphoreGive(xCANMutex);
    }
}

/* ==================== FreeRTOS 任务 ==================== */

/*
 * 接收任务 — 与 STM32 一致:
 *   只接受标准帧, data[0]=电机ID, data[1..2]=角度原始值
 */
static void twai_rx_task(void *pvParameters) {
    twai_message_t msg;
    while (1) {
        if (twai_receive(&msg, portMAX_DELAY) == ESP_OK) {
            if (!msg.extd && msg.data_length_code >= 8) {
                parse_feedback(msg.data);
            }
        }
    }
}

void Task_CANMotorDrive(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(CAN_RS02_TASK_PERIOD_MS);

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        if (xCANMutex && xSemaphoreTake(xCANMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            for (uint8_t id = 1; id <= 2; id++) {
                float new_rad = traj_update(&traj_motor[id]);
                cur_rad[id] = new_rad;
                CAN_RS02_MIT_Ctrl(id, new_rad, CAN_RS02_LOCK_VEL,
                                   CAN_RS02_LOCK_KP, CAN_RS02_LOCK_KD, CAN_RS02_LOCK_TORQ);
            }
            xSemaphoreGive(xCANMutex);
        }
    }
}