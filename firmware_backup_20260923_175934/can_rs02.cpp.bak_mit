/*
 * can_rs02.cpp - RS02 一体化电机 CAN 驱动 (官方扩展帧 + 位置CSP模式)
 * 平台: ESP32-S3 / ESP-IDF TWAI
 * 协议: 29 位扩展帧 (官方手册 product_manual_robStride02)
 *
 * ext_id = (type<<24) | (HOST_ID<<8) | motor_id
 *   type3  使能 / type4 停止 / type6 设零
 *   type0x12 写参数 (run_mode/limit_spd/limit_cur/loc_ref/report_time)
 *   type0x18 主动上报开关 (data[6]=01开/00关)
 *   type2 反馈: 位置u16|速度u16|扭矩u16|温度u16 (大端)
 *
 * 控制方式: 位置 CSP, 直接写 loc_ref(0x7016, float rad), 无 type1 运控帧
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

static bool csp_ready[3] = {false, false, false};

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
#define P_LOC_REF       0x7016
#define P_LIMIT_SPD     0x7017
#define P_LIMIT_CUR     0x7018
#define P_REPORT_TIME   0x7026

#define RS02_P_MAX      12.566f
#define RS02_V_MAX      44.0f
#define RS02_KP_MAX     500.0f
#define RS02_KD_MAX     5.0f

#define CSP_LIMIT_SPD   20.0f
#define CSP_LIMIT_CUR   8.0f

/* ==================== 内部函数声明 ==================== */

static float quintic_smooth(float t);
static float traj_update(CANMotorTraj* t);
static void parse_feedback(uint32_t ext_id, uint8_t* data);
static void twai_rx_task(void *pvParameters);

/* ==================== 工具 ==================== */

static uint32_t can_ext_id(uint8_t type, uint8_t motor_id) {
    return ((uint32_t)type << 24) | ((uint32_t)CAN_HOST_ID << 8) | (uint32_t)motor_id;
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
    if (motor_id < 1 || motor_id > 2) return;

    uint16_t pos_raw = ((uint16_t)data[0] << 8) | data[1];
    uint16_t vel_raw = ((uint16_t)data[2] << 8) | data[3];
    uint16_t tq_raw  = ((uint16_t)data[4] << 8) | data[5];
    uint16_t tmp_raw = ((uint16_t)data[6] << 8) | data[7];

    float angle = ((float)pos_raw / 65535.0f * 2.0f - 1.0f) * RS02_P_MAX;
    float vel   = ((float)vel_raw / 65535.0f * 2.0f - 1.0f) * RS02_V_MAX;
    float torque= ((float)tq_raw  / 65535.0f * 2.0f - 1.0f) * 17.0f;
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

void CAN_RS02_CSP_SetPos(uint8_t id, float pos_rad) {
    CAN_RS02_WriteParamFloat(id, P_LOC_REF, pos_rad);
}

void CAN_RS02_Enable(uint8_t id) {
    uint8_t data[8] = {0};
    twai_send_ext(can_ext_id(CAN_T_ENABLE, id), data, 8);
}

void CAN_RS02_Disable(uint8_t id) {
    uint8_t data[8] = {0};
    data[0] = 0x01;
    twai_send_ext(can_ext_id(CAN_T_STOP, id), data, 8);
    csp_ready[id] = false;
}

void CAN_RS02_SetZero(uint8_t id) {
    uint8_t data[8] = {0};
    data[0] = 0x01;
    twai_send_ext(can_ext_id(CAN_T_SETZERO, id), data, 8);
    vTaskDelay(pdMS_TO_TICKS(100));
}

void CAN_RS02_CSP_Setup(uint8_t id, float limit_spd, float limit_cur) {
    CAN_RS02_Disable(id);
    vTaskDelay(pdMS_TO_TICKS(30));

    CAN_RS02_WriteParamU8(id, P_RUN_MODE, 5);
    vTaskDelay(pdMS_TO_TICKS(30));

    CAN_RS02_Enable(id);
    vTaskDelay(pdMS_TO_TICKS(30));

    CAN_RS02_WriteParamFloat(id, P_LIMIT_SPD, limit_spd);
    vTaskDelay(pdMS_TO_TICKS(20));
    CAN_RS02_WriteParamFloat(id, P_LIMIT_CUR, limit_cur);
    vTaskDelay(pdMS_TO_TICKS(20));

    float cur = CAN_RS02_GetPosition(id);
    CAN_RS02_WriteParamFloat(id, P_LOC_REF, cur);
    vTaskDelay(pdMS_TO_TICKS(20));

    csp_ready[id] = true;
}

void CAN_RS02_MIT_Ctrl(uint8_t id, float pos_rad, float vel_rads,
                        float kp, float kd, float torq_nm) {
    (void)vel_rads; (void)kp; (void)kd; (void)torq_nm;
    CAN_RS02_CSP_SetPos(id, pos_rad);
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

/* ==================== 上电安全归零 ==================== */

static bool wait_feedback(uint8_t id, uint32_t timeout_ms) {
    if (id < 1 || id > 2) return false;
    char b[128];
    g_fb_received[id] = false;

    /* 补发一次开上报, 排除上报丢失 */
    snprintf(b, sizeof(b), "[HOME] M%d wait_fb: 补发 StartReport", id);
    Serial.println(b); WebServer_SendLog(b);
    CAN_RS02_StartReport(id);

    uint32_t t0 = millis();
    while (millis() - t0 < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (g_fb_received[id]) {
            snprintf(b, sizeof(b), "[HOME] M%d wait_fb: OK (%lums, fb1=%d fb2=%d)",
                     id, (unsigned long)(millis() - t0), g_fb_received[1], g_fb_received[2]);
            Serial.println(b); WebServer_SendLog(b);
            return true;
        }
    }
    snprintf(b, sizeof(b), "[HOME] M%d wait_fb: 超时%lums fb1=%d fb2=%d",
             id, (unsigned long)timeout_ms, g_fb_received[1], g_fb_received[2]);
    Serial.println(b); WebServer_SendLog(b);
    return false;
}

bool CAN_RS02_SafeHome(uint8_t id) {
    if (id < 1 || id > 2) return false;
    char buf[128];

    snprintf(buf, sizeof(buf), "[HOME] M%d ===== 开始 t=%lu =====", id, (unsigned long)millis());
    Serial.println(buf); WebServer_SendLog(buf);

    if (!wait_feedback(id, 3000)) {
        snprintf(buf, sizeof(buf), "[HOME] M%d 无反馈, 跳过归零", id);
        Serial.println(buf); WebServer_SendLog(buf);
        return false;
    }

    float cur_deg = CAN_RS02_GetPosition(id) * 180.0f / PI;
    snprintf(buf, sizeof(buf), "[HOME] M%d 当前角度: %.1f° t=%lu", id, cur_deg, (unsigned long)millis());
    Serial.println(buf); WebServer_SendLog(buf);

    if (cur_deg < CSP_HOME_SAFE_MIN || cur_deg > CSP_HOME_SAFE_MAX) {
        snprintf(buf, sizeof(buf), "[HOME][ALARM] M%d 角度 %.1f° 越界, 闭锁!", id, cur_deg);
        Serial.println(buf); WebServer_SendLog(buf);
        return false;
    }

    snprintf(buf, sizeof(buf), "[HOME] M%d 失能+清故障 t=%lu", id, (unsigned long)millis());
    Serial.println(buf); WebServer_SendLog(buf);
    CAN_RS02_Disable(id);
    vTaskDelay(pdMS_TO_TICKS(200));

    snprintf(buf, sizeof(buf), "[HOME] M%d CSP_Setup 开始 t=%lu", id, (unsigned long)millis());
    Serial.println(buf); WebServer_SendLog(buf);
    CAN_RS02_CSP_Setup(id, CSP_HOME_SPD, CSP_HOME_SAFE_CUR);

    snprintf(buf, sizeof(buf), "[HOME] M%d CSP_Setup 完成, 启动归零轨迹 t=%lu", id, (unsigned long)millis());
    Serial.println(buf); WebServer_SendLog(buf);
    CAN_RS02_ResetTraj(id, CAN_RS02_GetPosition(id));
    CAN_RS02_StartTraj(id, 0.0f, CSP_HOME_DUR);

    uint32_t t0 = millis();
    while (millis() - t0 < (uint32_t)(CSP_HOME_DUR * 1000.0f + 2000.0f)) {
        vTaskDelay(pdMS_TO_TICKS(20));
        if (!CAN_RS02_IsMoving(id)) break;
    }

    if (!CAN_RS02_IsMoving(id)) {
        snprintf(buf, sizeof(buf), "[HOME] M%d 归零成功, 恢复 %.1fA t=%lu",
                 id, CSP_WORK_CUR, (unsigned long)millis());
        Serial.println(buf); WebServer_SendLog(buf);
        CAN_RS02_WriteParamFloat(id, P_LIMIT_CUR, CSP_WORK_CUR);
        CAN_RS02_WriteParamFloat(id, P_LIMIT_SPD, CSP_WORK_SPD);
        vTaskDelay(pdMS_TO_TICKS(30));
        return true;
    } else {
        snprintf(buf, sizeof(buf), "[HOME][WARN] M%d 归零超时, 停止 t=%lu",
                 id, (unsigned long)millis());
        Serial.println(buf); WebServer_SendLog(buf);
        CAN_RS02_Disable(id);
        return false;
    }
}

/* 启动单电机归零 (不含等待), 成功返回 true */
static bool SafeHome_Start(uint8_t id) {
    if (id < 1 || id > 2) return false;
    char buf[128];

    snprintf(buf, sizeof(buf), "[HOME] M%d ===== 开始 t=%lu =====", id, (unsigned long)millis());
    Serial.println(buf); WebServer_SendLog(buf);

    if (!wait_feedback(id, 1000)) {
        snprintf(buf, sizeof(buf), "[HOME] M%d 无反馈, 跳过归零", id);
        Serial.println(buf); WebServer_SendLog(buf);
        return false;
    }

    float cur_deg = CAN_RS02_GetPosition(id) * 180.0f / PI;
    snprintf(buf, sizeof(buf), "[HOME] M%d 当前角度: %.1f°", id, cur_deg);
    Serial.println(buf); WebServer_SendLog(buf);

    if (cur_deg < CSP_HOME_SAFE_MIN || cur_deg > CSP_HOME_SAFE_MAX) {
        snprintf(buf, sizeof(buf), "[HOME][ALARM] M%d 角度 %.1f° 越界, 闭锁!", id, cur_deg);
        Serial.println(buf); WebServer_SendLog(buf);
        return false;
    }

    CAN_RS02_Disable(id);
    vTaskDelay(pdMS_TO_TICKS(50));
    CAN_RS02_CSP_Setup(id, CSP_HOME_SPD, CSP_HOME_SAFE_CUR);

    CAN_RS02_ResetTraj(id, CAN_RS02_GetPosition(id));
    CAN_RS02_StartTraj(id, 0.0f, CSP_HOME_DUR);
    return true;
}

/* 并行归零: 两电机同时启动, 同时等待 */
void CAN_RS02_AutoHome(void) {
    Serial.println("[HOME] === 上电安全归零(并行) ===");
    WebServer_SendLog("[HOME] === 上电安全归零(并行) ===");

    bool ok1 = SafeHome_Start(1);
    bool ok2 = SafeHome_Start(2);

    uint32_t t0 = millis();
    while (millis() - t0 < (uint32_t)(CSP_HOME_DUR * 1000.0f + 2000.0f)) {
        vTaskDelay(pdMS_TO_TICKS(20));
        bool moving1 = ok1 && CAN_RS02_IsMoving(1);
        bool moving2 = ok2 && CAN_RS02_IsMoving(2);
        if (!moving1 && !moving2) break;
    }

    if (ok1) {
        CAN_RS02_WriteParamFloat(1, P_LIMIT_CUR, CSP_WORK_CUR);
        CAN_RS02_WriteParamFloat(1, P_LIMIT_SPD, CSP_WORK_SPD);
    }
    if (ok2) {
        CAN_RS02_WriteParamFloat(2, P_LIMIT_CUR, CSP_WORK_CUR);
        CAN_RS02_WriteParamFloat(2, P_LIMIT_SPD, CSP_WORK_SPD);
    }
    vTaskDelay(pdMS_TO_TICKS(30));

    char b[96];
    snprintf(b, sizeof(b), "[HOME] === 归零结束 M1=%d M2=%d ===", ok1, ok2);
    Serial.println(b); WebServer_SendLog(b);
}

/* ==================== 轨迹接口 (CSP) ==================== */

void CAN_RS02_StartTraj(uint8_t id, float target_rad, float duration_sec) {
    if (id < 1 || id > 2) return;
    if (xCANMutex && xSemaphoreTake(xCANMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (duration_sec <= 0.0f) {
            cur_rad[id] = target_rad;
            traj_motor[id].active = false;
            traj_motor[id].qf = target_rad;
            CAN_RS02_CSP_SetPos(id, target_rad);
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
                if (traj_motor[id].active) {
                    float new_rad = traj_update(&traj_motor[id]);
                    cur_rad[id] = new_rad;
                    CAN_RS02_CSP_SetPos(id, new_rad);
                }
            }
            xSemaphoreGive(xCANMutex);
        }
    }
}