#ifndef GRIPPER_H
#define GRIPPER_H

#include <stdint.h>

/* gripper: 本地舵机 (IO38 PWM, 第5路), 初始 GRIPPER_HOME_DEG */
void  Gripper_Init(void);
bool  Gripper_SetTarget(float deg);
float Gripper_GetAngle(void);
bool  Gripper_IsOnline(void);
void  Gripper_Tick(void);

#endif