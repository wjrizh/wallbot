#ifndef AUTO_MODE_H
#define AUTO_MODE_H
#include <stdint.h>
#include <stdbool.h>

/* 子母协同协议 (4 字节 ASCII)
 * 母->子: COOP(开夹持器130到位) / GOON(抬起160放行) / LOCK(预夹+夹紧完成)
 * 子->母: REDY(就绪) / DONE(工作完成) / ACKD(确认)
 */

void AutoMode_Init(void);
bool AutoMode_IsActive(void);
void AutoMode_StartCoop(void);
void AutoMode_Abort(void);
void AutoMode_Tick(void);

bool AutoMode_WantsSendFrame(uint8_t* out_buf, uint8_t* out_len);
void AutoMode_OnFrameSent(void);

float AutoMode_GetChildPitch(void);
float AutoMode_GetChildRoll(void);
bool AutoMode_IsChildIMUUpdated(void);
void AutoMode_ClearChildIMUFlag(void);

#endif