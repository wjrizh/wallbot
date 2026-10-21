#ifndef NRF_SENDER_H
#define NRF_SENDER_H

#include <stdint.h>

void NRF_Sender_Init(void);
bool NRF_SendFrame(const uint8_t* data, uint8_t len);
/* 带重试: 子机回包/发遥测时会短暂切 TX, 单发易收不到 ACK */
bool NRF_SendFrameRetry(const uint8_t* data, uint8_t len, uint8_t retries);

void NRF_StartListening(void);
void NRF_StopListening(void);
bool NRF_ReceiveFrame(uint8_t* buf, uint8_t maxlen, uint8_t* out_len);

#endif