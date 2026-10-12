#ifndef JOYPAD_SENDER_H
#define JOYPAD_SENDER_H

#include <stdint.h>

void JoypadSender_Init(void);
void JoypadSender_Task(void *pvParameters);
uint8_t JoypadSender_GetSignalQuality(void);

#endif