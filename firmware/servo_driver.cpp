/*
 * servo_driver.cpp - 四路舵机PWM驱动 (五次多项式平滑, 独立标定项目)
 * 频率: 50Hz, 14bit分辨率, FreeRTOS 50Hz任务
 */

#include "servo_driver.h"
#include "config.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <math.h>

static ServoTraj traj_m1;
static ServoTraj traj_m2;
static ServoTraj traj_m3;
static ServoTraj traj_pb9;
static ServoTraj traj_grip;

static float cur_m1 = 60.0f;
static float cur_m2 = 110.0f;
static float cur_m3 = 35.0f;
static float cur_pb9 = 160.0f;
static float cur_grip = GRIPPER_HOME_DEG;

static SemaphoreHandle_t xServoMutex = NULL;

/* ==================== 内部函数 ==================== */

static float quintic_smooth(float t) {
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * t * (10.0f - 15.0f * t + 6.0f * t * t);
}

static uint32_t angle_to_duty(float angle_deg) {
    if (angle_deg < SERVO_ANGLE_MIN) angle_deg = SERVO_ANGLE_MIN;
    if (angle_deg > SERVO_ANGLE_MAX) angle_deg = SERVO_ANGLE_MAX;
    float pulse_us = 500.0f + (angle_deg / 180.0f) * 2000.0f;
    float duty_cycle = pulse_us / 20000.0f;
    return (uint32_t)(duty_cycle * ((1 << SERVO_PWM_RESOLUTION) - 1));
}

static void traj_init(ServoTraj* t) {
    t->q0 = 0.0f; t->qf = 0.0f; t->T = 0.0f; t->start_tick = 0; t->active = false;
}

static void traj_start(ServoTraj* t, float q0, float qf, float T) {
    t->q0 = q0; t->qf = qf;
    t->T = (T > 0.0f) ? T : 0.001f;
    t->start_tick = millis();
    t->active = true;
}

static float traj_update(ServoTraj* t) {
    if (!t->active) return t->qf;
    float elapsed = (millis() - t->start_tick) / 1000.0f;
    float tau = elapsed / t->T;
    if (tau >= 1.0f) { t->active = false; return t->qf; }
    return t->q0 + (t->qf - t->q0) * quintic_smooth(tau);
}

/* ==================== 接口函数 ==================== */

void Servo_Init(void) {
    ledcAttach(SERVO_M1_PIN, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO_M2_PIN, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO_M3_PIN, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO_PB9_PIN, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
    ledcAttach(SERVO_GRIP_PIN, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);

    ledcWrite(SERVO_M1_PIN, angle_to_duty(cur_m1));
    ledcWrite(SERVO_M2_PIN, angle_to_duty(cur_m2));
    ledcWrite(SERVO_M3_PIN, angle_to_duty(cur_m3));
    ledcWrite(SERVO_PB9_PIN, angle_to_duty(cur_pb9));
    ledcWrite(SERVO_GRIP_PIN, angle_to_duty(cur_grip));

    traj_init(&traj_m1); traj_m1.qf = cur_m1;
    traj_init(&traj_m2); traj_m2.qf = cur_m2;
    traj_init(&traj_m3); traj_m3.qf = cur_m3;
    traj_init(&traj_pb9); traj_pb9.qf = cur_pb9;
    traj_init(&traj_grip); traj_grip.qf = cur_grip;

    xServoMutex = xSemaphoreCreateMutex();
    xTaskCreate(Task_ServoDrive, "ServoDrv", 4096, NULL, 3, NULL);
}

float Servo_GetAngle(uint8_t id) {
    float ang = 0.0f;
    if (xServoMutex && xSemaphoreTake(xServoMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        switch (id) {
            case 1: ang = cur_m1; break;
            case 2: ang = cur_m2; break;
            case 3: ang = cur_m3; break;
            case 9: ang = cur_pb9; break;
            case 5: ang = cur_grip; break;
        }
        xSemaphoreGive(xServoMutex);
    }
    return ang;
}

void Servo_SetM1(float angle_deg, float duration_sec) {
    if (angle_deg < SERVO_ANGLE_MIN) angle_deg = SERVO_ANGLE_MIN;
    if (angle_deg > SERVO_ANGLE_MAX) angle_deg = SERVO_ANGLE_MAX;
    if (xServoMutex && xSemaphoreTake(xServoMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (duration_sec <= 0.0f) {
            cur_m1 = angle_deg; traj_m1.active = false; traj_m1.qf = angle_deg;
            ledcWrite(SERVO_M1_PIN, angle_to_duty(angle_deg));
        } else {
            traj_start(&traj_m1, cur_m1, angle_deg, duration_sec);
        }
        xSemaphoreGive(xServoMutex);
    }
}

void Servo_SetM2(float angle_deg, float duration_sec) {
    if (angle_deg < SERVO_ANGLE_MIN) angle_deg = SERVO_ANGLE_MIN;
    if (angle_deg > SERVO_ANGLE_MAX) angle_deg = SERVO_ANGLE_MAX;
    if (xServoMutex && xSemaphoreTake(xServoMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (duration_sec <= 0.0f) {
            cur_m2 = angle_deg; traj_m2.active = false; traj_m2.qf = angle_deg;
            ledcWrite(SERVO_M2_PIN, angle_to_duty(angle_deg));
        } else {
            traj_start(&traj_m2, cur_m2, angle_deg, duration_sec);
        }
        xSemaphoreGive(xServoMutex);
    }
}

void Servo_SetM3(float angle_deg, float duration_sec) {
    if (angle_deg < SERVO_ANGLE_MIN) angle_deg = SERVO_ANGLE_MIN;
    if (angle_deg > SERVO_ANGLE_MAX) angle_deg = SERVO_ANGLE_MAX;
    if (xServoMutex && xSemaphoreTake(xServoMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (duration_sec <= 0.0f) {
            cur_m3 = angle_deg; traj_m3.active = false; traj_m3.qf = angle_deg;
            ledcWrite(SERVO_M3_PIN, angle_to_duty(angle_deg));
        } else {
            traj_start(&traj_m3, cur_m3, angle_deg, duration_sec);
        }
        xSemaphoreGive(xServoMutex);
    }
}

void Servo_SetPB9(float angle_deg, float duration_sec) {
    if (angle_deg < SERVO_ANGLE_MIN) angle_deg = SERVO_ANGLE_MIN;
    if (angle_deg > SERVO_ANGLE_MAX) angle_deg = SERVO_ANGLE_MAX;
    if (xServoMutex && xSemaphoreTake(xServoMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (duration_sec <= 0.0f) {
            cur_pb9 = angle_deg; traj_pb9.active = false; traj_pb9.qf = angle_deg;
            ledcWrite(SERVO_PB9_PIN, angle_to_duty(angle_deg));
        } else {
            traj_start(&traj_pb9, cur_pb9, angle_deg, duration_sec);
        }
        xSemaphoreGive(xServoMutex);
    }
}

void Servo_SetGrip(float angle_deg, float duration_sec) {
    if (angle_deg < SERVO_ANGLE_MIN) angle_deg = SERVO_ANGLE_MIN;
    if (angle_deg > SERVO_ANGLE_MAX) angle_deg = SERVO_ANGLE_MAX;
    if (xServoMutex && xSemaphoreTake(xServoMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (duration_sec <= 0.0f) {
            cur_grip = angle_deg; traj_grip.active = false; traj_grip.qf = angle_deg;
            ledcWrite(SERVO_GRIP_PIN, angle_to_duty(angle_deg));
        } else {
            traj_start(&traj_grip, cur_grip, angle_deg, duration_sec);
        }
        xSemaphoreGive(xServoMutex);
    }
}

void Servo_HoldPB9(void) {
    if (xServoMutex && xSemaphoreTake(xServoMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        traj_pb9.active = false;
        traj_pb9.qf = cur_pb9;
        traj_pb9.q0 = cur_pb9;
        xSemaphoreGive(xServoMutex);
    }
}

void Servo_HoldGrip(void) {
    if (xServoMutex && xSemaphoreTake(xServoMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        traj_grip.active = false;
        traj_grip.qf = cur_grip;
        traj_grip.q0 = cur_grip;
        xSemaphoreGive(xServoMutex);
    }
}

void Task_ServoDrive(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(SERVO_TASK_PERIOD_MS);
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        if (xServoMutex && xSemaphoreTake(xServoMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            float nm1 = traj_update(&traj_m1);
            float nm2 = traj_update(&traj_m2);
            float nm3 = traj_update(&traj_m3);
            float npb9 = traj_update(&traj_pb9);
            float ngrip = traj_update(&traj_grip);
            ledcWrite(SERVO_M1_PIN, angle_to_duty(nm1));
            ledcWrite(SERVO_M2_PIN, angle_to_duty(nm2));
            ledcWrite(SERVO_M3_PIN, angle_to_duty(nm3));
            ledcWrite(SERVO_PB9_PIN, angle_to_duty(npb9));
            ledcWrite(SERVO_GRIP_PIN, angle_to_duty(ngrip));
            cur_m1 = nm1; cur_m2 = nm2; cur_m3 = nm3; cur_pb9 = npb9; cur_grip = ngrip;
            xSemaphoreGive(xServoMutex);
        }
    }
}