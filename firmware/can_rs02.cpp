/*
 * can_rs02.cpp - RS02 一体化电机 CAN 驱动 (29位扩展帧 + 运控MIT模式)
 * 平台: ESP32-S3 / ESP-IDF TWAI
 * 协议: 官方 robStride02 (29位扩展帧)
 *
 * ext_id = (type<<24) | (extra<<16) | (HOST_ID=0xFD<<8) | motor_id
 *   运控帧 extra = torque_u8 (8bit, 0Nm=0x7F, ±17Nm=0x00/0xFE)
 *
 *   type3  使能: data = 8x00
 *   type4  停止: data[0] = clear_error
 *   type6  设机械零位: data[0]=1, 延时 100ms
 *   type0x12 写参数: data[0..1]=Index(小端) data[4..7]=value
 *   type1  运控: data = 位置u16|速度u16|Kp u16|Kd u16 (大端, 各16bit)
 *   type2  反馈: 电机主动上报, data = 位置u16|速度u16|扭矩u16|温度u16 (大端)
 *
 * 运控为电机上电默认模式; Enable 前先写 run_mode=0 保险。
 * 归一化: 位置u16=(pos/12.566+1)*65535/2, 速度u16=(vel/44+1)*65535/2,
 *         Kp u16=Kp/500*65535, Kd u16=Kd/5*65535, torque_u8=(t/17+1)*255/2
 */

#include "can_rs02.h"
#include "config.h"
#include "web_server.h"
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
static volatile bool g_fb_received[3] = {false, false, false};

static CANMotorTraj traj_motor[3];
static float cur_rad[3] = {0.0f, 0.0f, 0.0f};
static SemaphoreHandle_t xCANMutex = NULL;
static TaskHandle_t hCANMotorTask = NULL;

/* ==================== 扩展帧常量 ==================== */

#define CAN_HOST_ID     0xFD
#define CAN_T_MOTION    0x01
#define CAN_T_FEEDBACK  0x02
#define CAN_T_ENABLE    0x03
#define CAN_T_STOP      0x04
#define CAN_T_SETZERO   0x06
#define CAN_T_WRITE     0x12
#define CAN_T_REPORT_ON 0x18

#define P_RUN_MODE      0x7005
#define P_LIMIT_SPD     0x7017
#define P_LIMIT_CUR     0x7018
#define P_REPORT_TIME   0x7026

#define RS02_P_MAX      12.566f
#define RS02_V_MAX      44.0f
#define RS02_T_MAX      17.0f
#define RS02_KP_MAX     500.0f
#define RS02_KD_MAX     5.0f

/* ==================== 内部函数声明 ==================== */

static float quintic_smooth(float t);
static float traj_update(CANMotorTraj* t);
static void parse_feedback(uint32_t ext_id, uint8_t* data);
static void twai_rx_task(void *pvParameters);

/* ==================== 工具 ==================== */

static uint32_t can_ext_id(uint8_t type, uint8_t motor_id) {
    return ((uint32_t)type << 24) | ((uint32_t)CAN_HOST_ID << 8) | (uint32_t)motor_id;
}

/* ext_id 带 torque(bit16-23, 8bit) 的运控帧 ID */
static uint32_t can_ext_id_torque(uint8_t type, uint8_t motor_id, uint8_t torque_u8) {
    return ((uint32_t)type << 24) | ((uint32_t)torque_u8 << 16)
         | ((uint32_t)CAN_HOST_ID << 8) | (uint32_t)motor_id;
}

static bool twai_send_ext(uint32_t extId, const uint8_t* data, uint8_t dlc) {
    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.identifier = extId;
    msg.extd = 1;
    msg.data_length_code = dlc;
    if (data && dlc) memcpy(msg.data, data, dlc);
    return twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK;
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

/* ==================== 反馈解析 (type2) ==================== */

static void parse_feedback(uint32_t ext_id, uint8_t* data) {
    uint8_t motor_id = (ext_id >> 8) & 0xFF;
    if (motor_id <1 || motor_id > 2) return;

    uint16_t pos_raw = ((uint16_t)data[0] << 8) | data[1];
    uint16_t vel_raw = ((uint16_t)data[2] << 8) | data[3];
    uint16_t tq_raw  = ((uint16_t)data[4] << 8) | data[5];
    uint16_t tmp_raw = ((uint16_t)data[6] << 8) | data[7];

    float angle = ((float)pos_raw / 65535.0f * 2.0f - 1.0f) * RS02_P_MAX;
    float vel   = ((float)vel_raw / 65535.0f * 2.0f - 1.0f) * RS02_V_MAX;
    float torque= ((float)tq_raw  / 65535.0f * 2.0f - 1.0f) * RS02_T_MAX;
    float temp  = (float)tmp_raw * 0.1f;

    if (xCANMutex && xSemaphoreTake(xCANMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        can_rs02_feedback_rad[motor_id] = angle;
        can_rs02_fb[motor_id].position_rad   = angle;
        can_rs02_fb[motor_id].velocity_rads  = vel;
        can_rs02_fb[motor_id].torque_nm      = torque;
        can_rs02_fb[motor_id].temp_c         = temp;
        g_fb_received[motor_id] = true;
        xSemaphoreGive(xCANMutex);

        static uint32_t cnt[3] = {0,0,0};
        cnt[motor_id]++;
        if (cnt[motor_id] % 100 == 0) {
            char b[64];
            snprintf(b, sizeof(b), "[FB] M%d 收到 %lu 帧", motor_id, (unsigned long)cnt[motor_id]);
            WebServer_SendLog(b);
        }
    }
}

/* ==================== 底层命令 ==================== */

void CAN_RS02_StartReport(uint8_t id) {
    uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x01, 0x00};
    twai_send_ext(can_ext_id(CAN_T_REPORT_ON, id), data, 8);
}

void CAN_RS02_StopReport(uint8_t id) {
    uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x00, 0x00};
    twai_send_ext(can_ext_id(CAN_T_REPORT_ON, id), data, 8);
}

void CAN_RS02_SetReportPeriod(uint8_t id, uint8_t code) {
    CAN_RS02_WriteParamU8(id, P_REPORT_TIME, code);
}

void CAN_RS02_WriteParamU8(uint8_t id, uint16_t index, uint8_t val) {
    uint8_t data[8] = {0};
    data[0] = index & 0xFF;
    data[1] = index >> 8;
    data[4] = val;
    twai_send_ext(can_ext_id(CAN_T_WRITE, id), data, 8);
}

void CAN_RS02_WriteParamFloat(uint8_t id, uint16_t index, float val) {
    uint8_t data[8] = {0};
    data[0] = index & 0xFF;
    data[1] = index >> 8;
    memcpy(&data[4], &val, 4);
    twai_send_ext(can_ext_id(CAN_T_WRITE, id), data, 8);
}

/* 使能: 先写 run_mode=0 (运控), 再发 type3 使能 */
void CAN_RS02_Enable(uint8_t id) {
    CAN_RS02_WriteParamU8(id, P_RUN_MODE, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    uint8_t data[8] = {0};
    twai_send_ext(can_ext_id(CAN_T_ENABLE, id), data, 8);
}

/* 停止: type4, data[0]=clear_error */
void CAN_RS02_Disable(uint8_t id) {
    uint8_t data[8] = {0};
    data[0] = 0x01;
    twai_send_ext(can_ext_id(CAN_T_STOP, id), data, 8);
}

/* 设机械零位: type6, data[0]=1, 延时 100ms, 同步清空固件缓存 */
void CAN_RS02_SetZero(uint8_t id) {
    uint8_t data[8] = {0};
    data[0] = 0x01;
    twai_send_ext(can_ext_id(CAN_T_SETZERO, id), data, 8);
    vTaskDelay(pdMS_TO_TICKS(100));
    if (xCANMutex && xSemaphoreTake(xCANMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        can_rs02_feedback_rad[id] = 0.0f;
        can_rs02_fb[id].position_rad = 0.0f;
        xSemaphoreGive(xCANMutex);
    }
}

/* 运控 type1 — 大端 16bit: 位置u16|速度u16|Kp u16|Kd u16, ext_id 带 torque */
void CAN_RS02_MIT_Ctrl(uint8_t id, float pos_rad, float vel_rads,
                        float kp, float kd, float torq_nm) {
    if (pos_rad >  RS02_P_MAX) pos_rad =  RS02_P_MAX;
    if (pos_rad < -RS02_P_MAX) pos_rad = -RS02_P_MAX;
    if (vel_rads >  RS02_V_MAX) vel_rads =  RS02_V_MAX;
    if (vel_rads < -RS02_V_MAX) vel_rads = -RS02_V_MAX;
    if (kp > RS02_KP_MAX) kp = RS02_KP_MAX; if (kp <0) kp = 0;
    if (kd > RS02_KD_MAX) kd = RS02_KD_MAX; if (kd <0) kd = 0;

    uint16_t p_u  = (uint16_t)((pos_rad / RS02_P_MAX + 1.0f) * 65535.0f / 2.0f);
    uint16_t v_u  = (uint16_t)((vel_rads / RS02_V_MAX + 1.0f) * 65535.0f / 2.0f);
    uint16_t kp_u = (uint16_t)(kp / RS02_KP_MAX * 65535.0f);
    uint16_t kd_u = (uint16_t)(kd / RS02_KD_MAX * 65535.0f);

    uint8_t data[8];
    data[0] = p_u >> 8;   data[1] = p_u & 0xFF;
    data[2] = v_u >> 8;   data[3] = v_u & 0xFF;
    data[4] = kp_u >> 8;  data[5] = kp_u & 0xFF;
    data[6] = kd_u >> 8;  data[7] = kd_u & 0xFF;

    uint8_t t_u8 = (uint8_t)((torq_nm / RS02_T_MAX + 1.0f) * 255.0f / 2.0f);
    twai_send_ext(can_ext_id_torque(CAN_T_MOTION, id, t_u8), data, 8);
}

/* ==================== 轨迹接口 ==================== */

void CAN_RS02_StartTraj(uint8_t id, float target_rad, float duration_sec) {
    if (id <1 || id > 2) return;
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
    if (id <1 || id > 2) return 0.0f;
    float pos = 0.0f;
    if (xCANMutex && xSemaphoreTake(xCANMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        pos = can_rs02_feedback_rad[id];
        xSemaphoreGive(xCANMutex);
    }
    return pos;
}

bool CAN_RS02_HasFeedback(uint8_t id) {
    if (id <1 || id > 2) return false;
    return g_fb_received[id];
}

void CAN_RS02_ResetTraj(uint8_t id, float rad) {
    if (id <1 || id > 2) return;
    if (xCANMutex && xSemaphoreTake(xCANMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        cur_rad[id] = rad;
        traj_motor[id].q0 = rad;
        traj_motor[id].qf = rad;
        traj_motor[id].active = false;
        xSemaphoreGive(xCANMutex);
    }
}

/* ==================== 初始化 ==================== */

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
    Serial.println("[CAN] TWAI 1Mbps 扩展帧 启动成功");

    xCANMutex = xSemaphoreCreateMutex();

    xTaskCreate(twai_rx_task, "CAN_Rx", 4096, NULL, 4, NULL);
    xTaskCreate(Task_CANMotorDrive, "CAN_Drv", 4096, NULL, 3, &hCANMotorTask);

    CAN_RS02_StartReport(1);
    CAN_RS02_StartReport(2);
    Serial.println("[CAN] 主动上报已开启, 请通过网页手动使能");
}

/* ==================== FreeRTOS 任务 ==================== */

static void twai_rx_task(void *pvParameters) {
    twai_message_t msg;
    while (1) {
        if (twai_receive(&msg, portMAX_DELAY) == ESP_OK) {
            if (msg.extd) {
                uint8_t type = (msg.identifier >> 24) & 0x1F;
                if (type == CAN_T_FEEDBACK && msg.data_length_code >= 8) {
                    parse_feedback(msg.identifier, msg.data);
                }
            }
        }
    }
}

void Task_CANMotorDrive(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(CAN_RS02_TASK_PERIOD_MS);
    uint32_t last_report = 0;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        if (millis() - last_report >= 2000) {
            last_report = millis();
            CAN_RS02_StartReport(1);
            CAN_RS02_StartReport(2);
        }

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