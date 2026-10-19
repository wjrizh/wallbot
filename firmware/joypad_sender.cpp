#include "joypad_sender.h"
#include "joypad_input.h"
#include "nrf_sender.h"
#include "auto_mode.h"
#include "child_cmd.h"
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

        buf[1] = (uint8_t)(int8_t)last_lh;
        buf[2] = (uint8_t)(int8_t)last_lv;
        buf[3] = (uint8_t)(int8_t)last_rh;
        buf[4] = (uint8_t)(int8_t)last_rv;
        buf[5] = last_key;

        if (AutoMode_IsActive()) {
            uint8_t fbuf[4]; uint8_t flen = 0;
            if (AutoMode_WantsSendFrame(fbuf, &flen)) {
                NRF_StopListening();
                NRF_SendFrame(fbuf, flen);
                AutoMode_OnFrameSent();   /* 无论成败都推进状态+切RX, 失败视为丢帧 */
            }
            AutoMode_Tick();
            vTaskDelayUntil(&xLastWakeTime, period);
            continue;
        }

        /* 协同已退出但仍有遗留帧 (如 STOP) 待发, 优先发出 */
        uint8_t fbuf[4]; uint8_t flen = 0;
        if (AutoMode_WantsSendFrame(fbuf, &flen)) {
            NRF_StopListening();
            NRF_SendFrame(fbuf, flen);
            AutoMode_OnFrameSent();
        }

        /* 子机指令直通 (App 按键 / 视觉纠偏): 优先于虚拟手柄帧 */
        uint8_t child_frame[32];
        if (ChildCmd_Pop(child_frame)) {
            NRF_StopListening();
            NRF_SendFrame(child_frame, sizeof(child_frame));
        }

        NRF_StopListening();
        bool success = NRF_SendFrame(buf, sizeof(buf));
        signal_quality = CalculateSuccessRatio(success);

        if (success && last_key != 0) {
            last_key = 0;
            Joypad_ClearKey();
        }

        vTaskDelayUntil(&xLastWakeTime, period);
    }
}