#ifndef AVOID_MODE_H
#define AVOID_MODE_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    AVOID_OFF = 0,
    AVOID_SAMPLING = 1,
    AVOID_EXEC = 2,
    AVOID_WAIT = 3,
} AvoidState;

void    Avoid_Init(void);
void    Avoid_Start(void);
void    Avoid_Stop(void);
bool    Avoid_IsActive(void);
uint8_t Avoid_GetState(void);
uint16_t Avoid_GetDistance(void);

#endif
