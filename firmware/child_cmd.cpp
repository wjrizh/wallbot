/* child_cmd.cpp - 子机运动指令直通队列 */
#include "child_cmd.h"
#include <Arduino.h>
#include <string.h>
#include <freertos/FreeRTOS.h>

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t s_frame[32];
static bool s_pending = false;

void ChildCmd_Init(void) {
    portENTER_CRITICAL(&s_mux);
    memset(s_frame, 0, sizeof(s_frame));
    s_pending = false;
    portEXIT_CRITICAL(&s_mux);
}

bool ChildCmd_Set(int8_t speed, int8_t spin, uint8_t key) {
    uint8_t f[32];
    memset(f, 0, sizeof(f));
    f[0] = 0x00;
    f[1] = (uint8_t)speed;
    f[2] = (uint8_t)spin;
    f[5] = key;
    return ChildCmd_SetRaw(f);
}

bool ChildCmd_SetRaw(const uint8_t frame[32]) {
    portENTER_CRITICAL(&s_mux);
    memcpy(s_frame, frame, sizeof(s_frame));
    s_pending = true;
    portEXIT_CRITICAL(&s_mux);
    return true;
}

bool ChildCmd_Pop(uint8_t out[32]) {
    bool has;
    portENTER_CRITICAL(&s_mux);
    has = s_pending;
    if (has) {
        memcpy(out, s_frame, sizeof(s_frame));
        s_pending = false;
    }
    portEXIT_CRITICAL(&s_mux);
    return has;
}

void ChildCmd_Clear(void) {
    portENTER_CRITICAL(&s_mux);
    s_pending = false;
    portEXIT_CRITICAL(&s_mux);
}

bool ChildCmd_Pending(void) {
    bool has;
    portENTER_CRITICAL(&s_mux);
    has = s_pending;
    portEXIT_CRITICAL(&s_mux);
    return has;
}
