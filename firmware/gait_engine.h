#ifndef GAIT_ENGINE_H
#define GAIT_ENGINE_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    GAIT_IDLE      = 0,
    GAIT_DEBOUNCE  = 1,
    GAIT_SCANNING  = 2,
    GAIT_EXECUTING = 3,
    GAIT_WAITING   = 5,
} GaitState;

typedef enum {
    GAIT_MODE_OFF    = 0,
    GAIT_MODE_NORMAL = 1,
    GAIT_MODE_COMBAT = 2,
} GaitMode;

extern volatile bool gait_estop;

void GaitEngine_Init(void);
void GaitEngine_Start(void);
void GaitEngine_CombatStart(void);
void GaitEngine_Stop(void);
void GaitEngine_ConfirmNext(void);
void GaitEngine_StepWalk(float step_mm);
void GaitEngine_Rotate(void);
void GaitEngine_UTurn(void);
uint8_t GaitEngine_GetState(void);
uint8_t GaitEngine_GetMode(void);
uint32_t GaitEngine_GetCombatSteps(void);
uint8_t GaitEngine_GetSuspendedFoot(void);
void GaitEngine_EnqueueCmd(uint8_t cmd, const uint8_t* data, uint8_t len);

#endif