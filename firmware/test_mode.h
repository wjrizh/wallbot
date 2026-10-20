/* test_mode.h - 子机视觉纠偏测试模式 (无摄像头, 假偏差下发)
 *
 * 链路: App -> WS(0x30/0x31/0x32) -> 本模块 -> NRF24L01 -> 子机
 *   TSTA            母->子: 进入测试, 子机回 TSTB
 *   ADJ,<p>,<s>,<a> 母->子: 假视觉偏差, 子机执行后回 DONE
 *   TSTE            母->子: 退出测试
 *   'J' + 10B      子->母: 轮子运动遥测, 原样转发 App (0x86)
 */
#ifndef TEST_MODE_H
#define TEST_MODE_H

#include <stdint.h>
#include <stdbool.h>

void TestMode_Init(void);
void TestMode_Start(void);
void TestMode_SendAdj(float phi_deg, float psi_deg, float ax_mm);
void TestMode_Stop(void);

/* 主循环调用: 收 TSTB/DONE/'J' 并推进状态 */
void TestMode_Tick(void);

/* 发送队列: 由 JoypadSender_Task 取走 */
bool TestMode_WantsSendFrame(uint8_t* out_buf, uint8_t* out_len);
void TestMode_OnFrameSent(void);

bool     TestMode_IsActive(void);
uint8_t TestMode_GetState(void);
uint8_t TestMode_GetRound(void);

/* 供 web_server 拼状态帧 */
void TestMode_GetStatus(uint8_t* state, uint8_t* round, uint8_t* last_rc);

/* 遥测帧 'J' 转发出口, 由 web_server 注入 */
typedef void (*TestMode_TelemSink)(const uint8_t* frame, uint8_t len);
void TestMode_SetTelemetrySink(TestMode_TelemSink fn);

#endif /* TEST_MODE_H */
