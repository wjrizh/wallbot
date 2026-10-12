#ifndef NRF_SENDER_H
#define NRF_SENDER_H

#include <stdint.h>

void NRF_Sender_Init(void);
bool NRF_SendFrame(const uint8_t* data, uint8_t len);

void NRF_StartListening(void);
void NRF_StopListening(void);
bool NRF_ReceiveFrame(uint8_t* buf, uint8_t maxlen, uint8_t* out_len);

#endif