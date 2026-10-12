/*
 * auto_align.cpp - 自动纠偏状态机 (比例控制版)
 * 姿态1 (前腿悬停): M1=30 M2=100 M3=80, 前松后吸, 按 IMU 偏差比例控制纠偏RS02
 * 姿态2 (后腿悬停): M1=90 M2=90 M3=20, 前吸后松, 按 IMU 偏差比例控制纠偏RS02
 * 物理映射: error>0偏左→RS02正向; error<0偏右→RS02反向
 */

#include "auto_align.h"
#include "config.h"
#include "protocol.h"
#include "servo_driver.h"
#include "pump_driver.h"
#include "can_rs02.h"
#include "rs02_correct.h"
#include "imu_driver.h"
#include <Arduino.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <WiFi.h>
#include <HTTPClient.h>

extern volatile bool gait_estop;

/* ===== IMU 有线读取 (JY62 串口, 由 Task_IMU 周期采样) ===== */
static float IMU_FetchRoll(void) {
    return IMU_GetRoll();
}

typedef enum {
    AA_IDLE = 0,
    AA_PREP1, AA_MOVE_LIFT1, AA_WAIT_STABLE1, AA_READ_ERROR1, AA_WAIT_MOVE1, AA_HOME1, AA_WAIT_HOME1,
    AA_PREP2, AA_MOVE_LIFT2, AA_WAIT_STABLE2, AA_READ_ERROR2, AA_WAIT_MOVE2, AA_HOME2, AA_WAIT_HOME2,
    AA_DONE, AA_TIMEOUT, AA_ESTOP,
} AutoAlignState;

static AutoAlignState g_aa_state = AA_IDLE;
static uint32_t g_aa_timer = 0;
static bool g_aa_active = false;
static int g_aa_iter = 0;
static float g_aa_theta = 0.0f;
static bool g_is_pose1 = true;
static bool g_is_front_centered = false;  // 前腿是否已归中（偏差 < 1°）
static float g_imu_gnd_zero = 0.0f;  // IMU 管壁基准 Roll 值
static AutoAlignCorrMode g_aa_corr_mode = AA_CORR_NONE;  // 姿态2纠偏模式

/* ==================== 内部工具函数 ==================== */

static float calc_stdev(float* samples, int count) {
    float sum = 0, sq_sum = 0;
    for (int i = 0; i < count; i++) sum += samples[i];
    float mean = sum / count;
    for (int i = 0; i < count; i++) sq_sum += (samples[i] - mean) * (samples[i] - mean);
    return sqrtf(sq_sum / count);
}

/* ==================== 状态机任务 ==================== */

static void AutoAlignTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(50);

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        if (!g_aa_active) continue;

        if (gait_estop) {
            Serial.println("[AUTO-ALIGN] 急停触发!");
            Pump_Set(1, 1); Pump_Set(2, 1);
            CAN_RS02_StartTraj(1, 0.0f, 3.0f); CAN_RS02_StartTraj(2, 0.0f, 3.0f);
            Servo_SetM1(60.0f, 4.0f); Servo_SetM2(110.0f, 4.0f); Servo_SetM3(35.0f, 4.0f);
            g_aa_active = false; g_aa_state = AA_IDLE;
            continue;
        }

        switch (g_aa_state) {

            /* ===== 姿态1: 后腿悬停 (124,130.4,35) ===== */
            case AA_PREP1: {
                Serial.println("[AUTO-ALIGN] 姿态1: 前泵吸 后泵松");
                g_is_pose1 = true;
                g_aa_iter = 0; 
                g_aa_theta = CAN_RS02_GetPosition(1);
                g_imu_gnd_zero = Imu_GetGnd();
                Pump_Set(1, 1); Pump_Set(2, 0);
                vTaskDelay(pdMS_TO_TICKS(1000));

                float roll_start = IMU_FetchRoll();
                float err_start = fabsf(roll_start - g_imu_gnd_zero);
                g_is_front_centered = (err_start < 1.0f);

                if (g_is_front_centered) {
                    Serial.printf("[AUTO-ALIGN] P1跳过: roll=%.2f 已在±1°内, 直接进入前腿归零\n", roll_start);
                    Pump_Set(1, 1); Pump_Set(2, 1);
                    g_aa_state = AA_PREP2;
                    break;
                }
                g_aa_state = AA_MOVE_LIFT1;
                break;
            }

            case AA_MOVE_LIFT1:
                Servo_SetM1(140.0f, AA_LIFT_DUR);
                Servo_SetM2(130.0f, AA_LIFT_DUR);
                Servo_SetM3(50.0f, AA_LIFT_DUR);
                g_aa_timer = millis() + (uint32_t)(AA_LIFT_DUR * 1000) + 2000;
                g_aa_state = AA_WAIT_STABLE1;
                break;

            case AA_WAIT_STABLE1:
                if (millis() > g_aa_timer) {
                    g_aa_state = AA_READ_ERROR1;
                }
                break;

            /* ===== 后腿悬停: IMU 滚转角纠偏 ===== */
            case AA_READ_ERROR1: {
                g_aa_corr_mode = AA_CORR_IMU;
                float roll_now = IMU_FetchRoll();
                float error = roll_now - g_imu_gnd_zero;
                float abs_err = fabsf(error);

                Serial.printf("[AUTO-ALIGN] P1-IMU: roll=%.2f gnd=%.2f err=%.2f°\n",
                              roll_now, g_imu_gnd_zero, error);

                g_aa_theta = CAN_RS02_GetPosition(1);
                if (g_aa_iter == 0 && abs_err > AA_IMU_DEADZONE) {
                    float step = AA_STEP_BASE + (abs_err / AA_K_SYS) * AA_STEP_P;
                    if (step > AA_STEP_MAX) step = AA_STEP_MAX;
                    if (step <AA_STEP_MIN) step = AA_STEP_MIN;
                    float sign = (error > 0.0f) ? 1.0f : -1.0f;
                    g_aa_theta += sign * step;
                    if (g_aa_theta > 0.10f) g_aa_theta = 0.10f;
                    if (g_aa_theta < -0.10f) g_aa_theta = -0.10f;
                    Serial.printf("[AUTO-ALIGN] P1: 后腿抬起, RS02纠偏→%.4f rad\n", g_aa_theta);
                    RS02_DualCorrect(1, g_aa_theta, 2, g_aa_theta, RS02_CORRECT_TIMEOUT_MS);
                }
                g_aa_iter++;
                g_aa_state = AA_WAIT_MOVE1;
                break;
            }

            case AA_WAIT_MOVE1:
                /* 等 RS02 停稳 + 额外 1.5s */
                if (millis() > g_aa_timer) {
                    vTaskDelay(pdMS_TO_TICKS(1500));
                    g_aa_state = AA_HOME1;
                }
                break;

            case AA_HOME1:
                Serial.println("[AUTO-ALIGN] 姿态1完成, 归位...");
                Servo_SetM1(60.0f, AA_HOME_DUR);
                Servo_SetM2(110.0f, AA_HOME_DUR);
                Servo_SetM3(35.0f, AA_HOME_DUR);
                Pump_Set(1, 1); Pump_Set(2, 1);
                g_aa_timer = millis() + (uint32_t)(AA_HOME_DUR * 1000) + 1000;
                g_aa_state = AA_WAIT_HOME1;
                break;

            case AA_WAIT_HOME1:
                if (millis() > g_aa_timer) g_aa_state = AA_PREP2;
                break;

            /* ===== 姿态2: 前腿悬停 (50,130,130) ===== */
            case AA_PREP2: {
                Serial.println("[AUTO-ALIGN] 姿态2: 前泵松 后泵吸");
                g_is_pose1 = false;
                g_aa_iter = 0;
                g_imu_gnd_zero = Imu_GetGnd();
                Pump_Set(1, 0); Pump_Set(2, 1);
                vTaskDelay(pdMS_TO_TICKS(1000));
                g_aa_state = AA_MOVE_LIFT2;
                break;
            }

            case AA_MOVE_LIFT2:
                Servo_SetM1(60.0f, AA_LIFT_DUR);
                Servo_SetM2(120.0f, AA_LIFT_DUR);
                Servo_SetM3(115.0f, AA_LIFT_DUR);
                g_aa_timer = millis() + (uint32_t)(AA_LIFT_DUR * 1000) + 2000;
                g_aa_state = AA_WAIT_STABLE2;
                break;

            case AA_WAIT_STABLE2:
                if (millis() > g_aa_timer) {
                    g_aa_state = AA_READ_ERROR2;
                }
                break;

            case AA_READ_ERROR2: {
                g_aa_corr_mode = AA_CORR_NONE;
                g_aa_theta = 0.0f;
                Serial.printf("[AUTO-ALIGN] P2: 前腿抬起, RS02归零→0 rad\n");
                RS02_DualCorrect(1, 0.0f, 2, 0.0f, RS02_CORRECT_TIMEOUT_MS);
                g_aa_iter++;
                g_aa_state = AA_WAIT_MOVE2;
                break;
            }

            case AA_WAIT_MOVE2:
                if (millis() > g_aa_timer) {
                    vTaskDelay(pdMS_TO_TICKS(1500));
                    g_aa_state = AA_HOME2;
                }
                break;

            case AA_HOME2:
                Serial.println("[AUTO-ALIGN] 姿态2完成, 归位双吸...");
                Servo_SetM1(60.0f, AA_HOME_DUR);
                Servo_SetM2(110.0f, AA_HOME_DUR);
                Servo_SetM3(35.0f, AA_HOME_DUR);
                Pump_Set(1, 1); Pump_Set(2, 1);
                g_aa_timer = millis() + (uint32_t)(AA_HOME_DUR * 1000) + 1000;
                g_aa_state = AA_WAIT_HOME2;
                break;

            case AA_WAIT_HOME2:
                if (millis() > g_aa_timer) g_aa_state = AA_DONE;
                break;

            case AA_DONE:
                Serial.println("[AUTO-ALIGN] 全部完成!");
                g_aa_active = false;
                g_aa_state = AA_IDLE;
                break;

            case AA_TIMEOUT:
                Serial.println("[AUTO-ALIGN] 超时, 安全归位...");
                Pump_Set(1, 1); Pump_Set(2, 1);
                vTaskDelay(pdMS_TO_TICKS(1000));
                CAN_RS02_StartTraj(1, 0.0f, AA_HOME_DUR);
                CAN_RS02_StartTraj(2, 0.0f, AA_HOME_DUR);
                vTaskDelay(pdMS_TO_TICKS((uint32_t)(AA_HOME_DUR * 1000) + 1000));
                Servo_SetM1(60.0f, AA_HOME_DUR);
                Servo_SetM2(110.0f, AA_HOME_DUR);
                Servo_SetM3(35.0f, AA_HOME_DUR);
                g_aa_active = false; g_aa_state = AA_IDLE;
                break;

            default: break;
        }
    }
}

void AutoAlign_Init(void) {
    xTaskCreate(AutoAlignTask, "AutoAlign", 4096, NULL, 2, NULL);
    Serial.println("[AUTO-ALIGN] 初始化完成");
}

void AutoAlign_Start(void) {
    if (g_aa_active) return;
    gait_estop = false;
    g_aa_active = true;
    g_aa_state = AA_PREP1;
    Serial.println("[AUTO-ALIGN] 开始自动纠偏");
}

void AutoAlign_Stop(void) { gait_estop = true; }
bool AutoAlign_IsActive(void) { return g_aa_active; }
AutoAlignCorrMode AutoAlign_GetCorrMode(void) { return g_aa_corr_mode; }