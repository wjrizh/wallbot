#ifndef LASER_TOF_H
#define LASER_TOF_H

#include <stdint.h>
#include <stdbool.h>

/* VL53L5CX 8x8 ToF (TCA9548A CH0), 前方 +-15deg 扇形, 上半区 row0-3 */

void  Laser_Init(void);
bool  Laser_IsOnline(void);
/* 当前一次采样值 (前方扇形最小值, mm), 0=无效 */
uint16_t Laser_GetFront(void);
/* 后台任务周期调用: 选通道 + 读一帧, 更新 g_front */
void  Laser_Tick(void);

#endif
