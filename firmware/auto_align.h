#ifndef AUTO_ALIGN_H
#define AUTO_ALIGN_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    AA_CORR_NONE = 0,
    AA_CORR_IMU  = 1,
    AA_CORR_LASER = 2,
} AutoAlignCorrMode;

void AutoAlign_Init(void);
void AutoAlign_Start(void);
void AutoAlign_Stop(void);
bool AutoAlign_IsActive(void);
AutoAlignCorrMode AutoAlign_GetCorrMode(void);

#endif