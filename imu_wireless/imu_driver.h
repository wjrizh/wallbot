#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdint.h>

void IMU_Init(void);
bool IMU_Update(void);
float IMU_GetRoll(void);
float IMU_GetPitch(void);
float IMU_GetYaw(void);

#endif