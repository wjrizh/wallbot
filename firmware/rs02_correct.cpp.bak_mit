/*
 * rs02_correct.cpp - RS02 双轴PID位置修正 (接力微调版)
 * 策略: 先发1.0s长轨迹平滑走大段, 等电机停下来再检查是否需要补偿微调
 * 利用 CAN_RS02_IsMoving 避免运动中途叠加指令导致过冲弹跳
 * 参数: BASE=0.002 + P=0.1×|err|, MAX_WINDUP=0.15, deadzone=0.002
 */

#include "rs02_correct.h"
#include "config.h"
#include "can_rs02.h"
#include <Arduino.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern volatile bool gait_estop;

bool RS02_DualCorrect(uint8_t id1, float t1, uint8_t id2, float t2, uint32_t timeout_ms) {
    uint32_t t0 = millis();
    uint32_t settle_t0 = 0;
    float c1 = t1, c2 = t2;
    const float MAX_W = 0.15f, BASE = 0.002f, P = 0.1f;

    // 1. 首次下发长轨迹，平滑前往目标
    CAN_RS02_StartTraj(id1, c1, 1.0f);
    CAN_RS02_StartTraj(id2, c2, 1.0f);

    while (millis() - t0 < timeout_ms) {
        if (gait_estop) return false;

        float e1 = t1 - CAN_RS02_GetPosition(id1);
        float e2 = t2 - CAN_RS02_GetPosition(id2);
        bool ok1 = fabs(e1) < RS02_CORRECT_DEADZONE;
        bool ok2 = fabs(e2) < RS02_CORRECT_DEADZONE;

        // 2. 双双进入死区：保持写目标，等 settle 窗口吸收惯性后再退出
        if (ok1 && ok2) {
            if (settle_t0 == 0) settle_t0 = millis();
            CAN_RS02_ResetTraj(id1, t1); CAN_RS02_MIT_Ctrl(id1, t1, CAN_RS02_LOCK_VEL, CAN_RS02_LOCK_KP, CAN_RS02_LOCK_KD, CAN_RS02_LOCK_TORQ);
            CAN_RS02_ResetTraj(id2, t2); CAN_RS02_MIT_Ctrl(id2, t2, CAN_RS02_LOCK_VEL, CAN_RS02_LOCK_KP, CAN_RS02_LOCK_KD, CAN_RS02_LOCK_TORQ);
            if (millis() - settle_t0 >= RS02_SETTLE_MS) return true;
        } else {
            settle_t0 = 0;
        }

        // 3. 核心修正：只有当电机停止移动（轨迹走完），但仍然有稳态误差时，才进行补偿累加
        if (!ok1 && !CAN_RS02_IsMoving(id1)) {
            float s = BASE + fabs(e1) * P;
            if (s > RS02_CORRECT_MAX_STEP) s = RS02_CORRECT_MAX_STEP;
            c1 += (e1 > 0) ? s : -s;
            if (c1 > t1 + MAX_W) c1 = t1 + MAX_W;
            if (c1 < t1 - MAX_W) c1 = t1 - MAX_W;
            // 下发 0.2s 的短促平滑微调轨迹
            CAN_RS02_StartTraj(id1, c1, 0.2f);
        }

        if (!ok2 && !CAN_RS02_IsMoving(id2)) {
            float s = BASE + fabs(e2) * P;
            if (s > RS02_CORRECT_MAX_STEP) s = RS02_CORRECT_MAX_STEP;
            c2 += (e2 > 0) ? s : -s;
            if (c2 > t2 + MAX_W) c2 = t2 + MAX_W;
            if (c2 < t2 - MAX_W) c2 = t2 - MAX_W;
            CAN_RS02_StartTraj(id2, c2, 0.2f);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // 超时退出
    CAN_RS02_ResetTraj(id1, t1); CAN_RS02_MIT_Ctrl(id1, t1, CAN_RS02_LOCK_VEL, CAN_RS02_LOCK_KP, CAN_RS02_LOCK_KD, CAN_RS02_LOCK_TORQ);
    CAN_RS02_ResetTraj(id2, t2); CAN_RS02_MIT_Ctrl(id2, t2, CAN_RS02_LOCK_VEL, CAN_RS02_LOCK_KP, CAN_RS02_LOCK_KD, CAN_RS02_LOCK_TORQ);
    return false;
}