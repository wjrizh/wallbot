/*
 * rs02_correct.cpp - RS02 双轴位置修正 (单次到位版)
 * 只发一次目标轨迹, 由调用方算好的角度直接到位, 不做迭代补偿
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
    CAN_RS02_StartTraj(id1, t1, 1.0f);
    CAN_RS02_StartTraj(id2, t2, 1.0f);

    uint32_t t0 = millis();
    while (millis() - t0 <timeout_ms) {
        if (gait_estop) return false;
        if (!CAN_RS02_IsMoving(id1) && !CAN_RS02_IsMoving(id2)) {
            vTaskDelay(pdMS_TO_TICKS(300));
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return false;
}