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
void Task_ServoDrive(void *pvParameters);

#endif