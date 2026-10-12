#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

void Protocol_Init(void);
void Protocol_ProcessFrame(const uint8_t* data, uint16_t len);

/* 回执帧发送回调: 由 web_server.cpp 注入 (向发起命令的 WS 客户端回 AA 55 82 02 [CMD][CODE]) */
void Protocol_SetAckSender(void (*fn)(const uint8_t*, uint16_t));

/* 主动发送一帧回执 (供 web_server.cpp 的 onWsEvent 使用) */
void Protocol_SendAck(uint8_t cmd, uint8_t code);

uint16_t Protocol_CRC16(const uint8_t* data, uint16_t len);

/* ==================== IMU 自动标定 (RAM, 不写 NVS) ==================== */

extern float FetchIMURoll(void);

void  Imu_SetGnd(float roll_deg);
float Imu_GetGnd(void);
bool  Imu_IsCalibrated(void);
void  Imu_CalibrateOnce(void);

#endif