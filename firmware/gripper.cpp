/*
 * gripper.cpp - 夹持器控制 (本地舵机, IO38 PWM)
 * 夹持器为第 5 路舵机, 上电初始角度 GRIPPER_HOME_DEG (130°)
 * 协同运行中 (AutoMode_IsActive) 拒绝手动下发, 避免冲突
 */
#include "gripper.h"
#include "config.h"
#include "servo_driver.h"
#include "auto_mode.h"
#include <Arduino.h>

#define GRIPPER_MOVE_DUR 1.5f

void Gripper_Init(void) {
    Servo_SetGrip(GRIPPER_HOME_DEG, 0.0f);
    Serial.printf("[GRIP] 初始化, 初始角度 %.1f deg\n", GRIPPER_HOME_DEG);
}

float Gripper_GetAngle(void) { return Servo_GetAngle(5); }
bool  Gripper_IsOnline(void) { return true; }

bool Gripper_SetTarget(float deg) {
    if (AutoMode_IsActive()) {
        Serial.println("[GRIP] 协同运行中, 拒绝手动下发");
        return false;
    }
    if (deg <0.0f) deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    Servo_SetGrip(deg, GRIPPER_MOVE_DUR);
    Serial.printf("[GRIP] 目标 -> %.1f\n", deg);
    return true;
}

void Gripper_Tick(void) {
    /* 本地舵机无需轮询 */
}