#ifndef PUMP_DRIVER_H
#define PUMP_DRIVER_H

#include <stdint.h>

void Pump_Init(void);
void Pump_Set(uint8_t id, uint8_t state);
uint8_t Pump_GetState(uint8_t id);

#endif