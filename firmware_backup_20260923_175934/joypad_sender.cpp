#include "joypad_sender.h"
#include "joypad_input.h"
#include "nrf_sender.h"
#include "auto_mode.h"
#include "config.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

static uint8_t signal_quality = 0;

uint8_t JoypadSender_GetSignalQuality(void) {
    return signal_quality;
}

static uint8_t CalculateSuccessRatio(bool success) {
    static uint8_t history[10] = {0};
    static uint8_t p = 0;
    uint8_t count = 0;

    history[p] = success ? 1 : 0;
    p = (p + 1) % 10;

    for (int i = 0; i < 10; i++) {
        if (history[i] == 1) count++;
    }
    return count;
}

void JoypadSender_Init(void) {
    JoypadInput_Init();
    NRF_Sender_Init();
}

void JoypadSender_Task(void *pvParameters) {
    int8_t last_lh = 0, last_lv = 0, last_rh = 0, last_rv = 0;
    uint8_t last_key = 0;

    const TickType_t period = pdMS_TO_TICKS(JOYPAD_SEND_PERIOD_MS);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    Serial.printf("[JoySend] 发送任务启动, 周期=%dms\n", JOYPAD_SEND_PERIOD_MS);
    uint32_t loop_count = 0;

    while (1) {
        loop_count++;
        uint8_t buf[32] = {0};
        buf[0] = 0x00;

        if (Joypad_IsNewFrame()) {
            Joypad_GetFrame(&last_lh, &last_lv, &last_rh, &last_rv, &last_key);
        }

        if (last_key == 10 && !AutoMode_IsActive()) {
            AutoMode_StartKey10();
        }

        buf[1] = (uint8_t)(int8_t)last_lh;
        buf[2] = (uint8_t)(int8_t)last_lv;
        buf[3] = (uint8_t)(int8_t)last_rh;
        buf[4] = (uint8_t)(int8_t)last_rv;
        uint8_t send_key = last_key;
        if (AutoMode_WantsSendKey(&send_key)) {
            last_key = send_key;
        }
        buf[5] = send_key;

        if (AutoMode_IsActive()) {
            if (send_key != 0) {
                bool success = NRF_SendFrame(buf, sizeof(buf));
                if (success) {
                    last_key = 0;
                    Joypad_ClearKey();
                    AutoMode_OnSent(send_key);
                }
            }
            AutoMode_Tick();
            vTaskDelayUntil(&xLastWakeTime, period);
            continue;
        }

        bool success = NRF_SendFrame(buf, sizeof(buf));
        signal_quality = CalculateSuccessRatio(success);

        if (success && send_key != 0) {
            last_key = 0;
            Joypad_ClearKey();
        }

        vTaskDelayUntil(&xLastWakeTime, period);
    }
}