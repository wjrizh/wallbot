#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdint.h>

void IMU_Init(void);
bool IMU_Update(void);
float IMU_GetRoll(void);
float IMU_GetPitch(void);
float IMU_GetYaw(void);
/* 调试: 写入可读文本 (串口统计/帧统计/原始值), 返回写入长度 */
int  IMU_GetDebug(char* buf, unsigned int n);

#endif