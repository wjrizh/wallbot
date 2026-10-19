#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float    q0;
    float    qf;
    float    T;
    uint32_t start_tick;
    bool     active;
} ServoTraj;

void Servo_Init(void);
void Servo_SetM1(float angle_deg, float duration_sec);
void Servo_SetM2(float angle_deg, float duration_sec);
void Servo_SetM3(float angle_deg, float duration_sec);
void Servo_SetPB9(float angle_deg, float duration_sec);
void Servo_SetGrip(float angle_deg, float duration_sec);
float Servo_GetAngle(uint8_t id);
/* 冻结轨迹于当前角度, 停止插值但保持 PWM 输出不变 */
void Servo_HoldPB9(void);
void Servo_HoldGrip(void);
void Task_ServoDrive(void *pvParameters);

#endif