#include "joypad_input.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

static int8_t axis[4] = {0, 0, 0, 0};
static uint8_t key_val = 0;
static bool new_frame = false;
static SemaphoreHandle_t joy_mutex = NULL;

void JoypadInput_Init(void) {
    joy_mutex = xSemaphoreCreateMutex();
}

void Joypad_SetAxis(uint8_t ch, int8_t val) {
    if (ch >= 4) return;
    if (val < -100) val = -100;
    if (val > 100)  val = 100;
    if (joy_mutex) xSemaphoreTake(joy_mutex, portMAX_DELAY);
    axis[ch] = val;
    new_frame = true;
    if (joy_mutex) xSemaphoreGive(joy_mutex);
}

void Joypad_PressKey(uint8_t key) {
    if (joy_mutex) xSemaphoreTake(joy_mutex, portMAX_DELAY);
    key_val = key;
    new_frame = true;
    if (joy_mutex) xSemaphoreGive(joy_mutex);
}

bool Joypad_IsNewFrame(void) {
    return new_frame;
}

void Joypad_GetFrame(int8_t* lh, int8_t* lv, int8_t* rh, int8_t* rv, uint8_t* key) {
    if (joy_mutex) xSemaphoreTake(joy_mutex, portMAX_DELAY);
    *lh  = axis[0];
    *lv  = axis[1];
    *rh  = axis[2];
    *rv  = axis[3];
    *key = key_val;
    new_frame = false;
    if (joy_mutex) xSemaphoreGive(joy_mutex);
}

void Joypad_ClearKey(void) {
    if (joy_mutex) xSemaphoreTake(joy_mutex, portMAX_DELAY);
    if (key_val != 0) {
        key_val = 0;
        new_frame = true;
    }
    if (joy_mutex) xSemaphoreGive(joy_mutex);
}