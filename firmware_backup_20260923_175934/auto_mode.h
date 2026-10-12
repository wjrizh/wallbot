#ifndef AUTO_MODE_H
#define AUTO_MODE_H
#include <stdint.h>
#include <stdbool.h>

void AutoMode_Init(void);
bool AutoMode_IsActive(void);
void AutoMode_StartCoop(void);
void AutoMode_StartKey10(void);
void AutoMode_Tick(void);
bool AutoMode_WantsSendKey(uint8_t* out_key);
void AutoMode_OnSent(uint8_t key);

float AutoMode_GetChildPitch(void);
float AutoMode_GetChildRoll(void);
bool AutoMode_IsChildIMUUpdated(void);
void AutoMode_ClearChildIMUFlag(void);

#endif