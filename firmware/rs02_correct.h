#ifndef RS02_CORRECT_H
#define RS02_CORRECT_H

#include <stdint.h>
#include <stdbool.h>

bool RS02_DualCorrect(uint8_t id1, float t1, uint8_t id2, float t2, uint32_t timeout_ms);

#endif