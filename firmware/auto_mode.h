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

/* 填 0x88 状态帧 11 字节 payload (web_server 周期推送调用)
 * [0]=CV状态(0-6)/200=协同非视觉阶段 [1]=纠偏轮次 [2]=rc(恒0)
 * [3..4]=phi*100 [5..6]=psi*100 [7..8]=ax [9]=aligned [10]=轮次上限 */
void AutoMode_FillVisionStatus(uint8_t out[11]);

bool AutoMode_WantsSendFrame(uint8_t* out_buf, uint8_t* out_len);
void AutoMode_OnFrameSent(void);

float AutoMode_GetChildPitch(void);
float AutoMode_GetChildRoll(void);
bool AutoMode_IsChildIMUUpdated(void);
void AutoMode_ClearChildIMUFlag(void);

#endif