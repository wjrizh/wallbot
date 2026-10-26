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
 * [0]=阶段: 0-6=CV视觉态; 210-226=协同机械/握手阶段; 201=纠偏测试
 * [1]=纠偏轮次
 * [2]=告警: 0正常 1=NRF发送失败补发中 2=超时中止 3=记参考失败
 *         4=测量失败 5=轮次上限 6=其他异常
 * [3..4]=phi*100 [5..6]=psi*100 [7..8]=ax [9]=aligned [10]=轮次上限 */
void AutoMode_FillVisionStatus(uint8_t out[11]);

/* 取待发协同帧 (不消费); 成功后必须 OnTxResult(true) 才推进并清除 */
bool AutoMode_WantsSendFrame(uint8_t* out_buf, uint8_t* out_len);
/* NRF 发送结果: true=推进; false=保留待发+告警, 下周期继续补发直到成功 */
void AutoMode_OnTxResult(bool ok);

float AutoMode_GetChildPitch(void);
float AutoMode_GetChildRoll(void);
bool AutoMode_IsChildIMUUpdated(void);
void AutoMode_ClearChildIMUFlag(void);

#endif