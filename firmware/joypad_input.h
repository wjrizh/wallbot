#ifndef JOYPAD_INPUT_H
#define JOYPAD_INPUT_H

#include <stdint.h>
#include <stdbool.h>

void JoypadInput_Init(void);
void Joypad_SetAxis(uint8_t ch, int8_t val);
void Joypad_PressKey(uint8_t key);
bool Joypad_IsNewFrame(void);
void Joypad_GetFrame(int8_t* lh, int8_t* lv, int8_t* rh, int8_t* rv, uint8_t* key);
void Joypad_ClearKey(void);

#endif