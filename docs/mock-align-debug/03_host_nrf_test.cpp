/* ==========================================================================
 * 03_host_nrf_test.cpp  --  host NRF + joypad + test state machine
 * ========================================================================== */

/* --------------------------------------------------------------------------
 * FILE: firmware/nrf_sender.h
 * -------------------------------------------------------------------------- */

#ifndef NRF_SENDER_H
#define NRF_SENDER_H

#include <stdint.h>

void NRF_Sender_Init(void);
bool NRF_SendFrame(const uint8_t* data, uint8_t len);

void NRF_StartListening(void);
void NRF_StopListening(void);
bool NRF_ReceiveFrame(uint8_t* buf, uint8_t maxlen, uint8_t* out_len);

#endif


/* --------------------------------------------------------------------------
 * FILE: firmware/nrf_sender.cpp
 * -------------------------------------------------------------------------- */

#include "nrf_sender.h"
#include "config.h"
#include <Arduino.h>
#include <RF24.h>
#include <string.h>

static RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
static uint8_t tx_addr[5] = {NRF_TX_ADDR_0, NRF_TX_ADDR_1, NRF_TX_ADDR_2, NRF_TX_ADDR_3, NRF_TX_ADDR_4};

/* 状态跟踪: 只在真正切换时 flush, 避免连续调用把已收到的帧冲掉 */
static bool s_listening = false;

void NRF_Sender_Init(void) {
    SPI.begin(NRF_SCK_PIN, NRF_MISO_PIN, NRF_MOSI_PIN, -1);
    if (!radio.begin()) {
        Serial.println("[NRF] 模块未找到!");
        return;
    }
    radio.setChannel(NRF_RF_CH);
    radio.setDataRate(RF24_2MBPS);
    radio.setCRCLength(RF24_CRC_16);
    radio.setAutoAck(true);
    radio.setRetries(0, 3);
    radio.setPayloadSize(32);
    radio.openReadingPipe(1, tx_addr);
    radio.openWritingPipe(tx_addr);
    radio.stopListening();
    s_listening = false;
    Serial.println("[NRF] PTX 模式就绪");
}

/* 固定 32 字节 payload: 发送前补齐, 避免长度不匹配 */
bool NRF_SendFrame(const uint8_t* data, uint8_t len) {
    uint8_t buf[32];
    uint8_t n = (len > 32) ? 32 : len;
    memset(buf, 0, sizeof(buf));
    memcpy(buf, data, n);
    return radio.write(buf, 32);
}

void NRF_StartListening(void) {
    if (s_listening) return;
    radio.flush_tx();
    radio.flush_rx();
    radio.startListening();
    s_listening = true;
}

void NRF_StopListening(void) {
    if (!s_listening) return;
    radio.stopListening();
    s_listening = false;
}

/* 固定 32 字节 payload: 不用 getDynamicPayloadSize (未启用时返回 0) */
bool NRF_ReceiveFrame(uint8_t* buf, uint8_t maxlen, uint8_t* out_len) {
    if (!radio.available()) return false;
    uint8_t n = radio.getPayloadSize();
    if (n == 0 || n > 32) n = 32;
    if (n > maxlen) n = maxlen;
    radio.read(buf, n);
    *out_len = n;
    return true;
}


/* --------------------------------------------------------------------------
 * FILE: firmware/joypad_sender.h
 * -------------------------------------------------------------------------- */

#ifndef JOYPAD_SENDER_H
#define JOYPAD_SENDER_H

#include <stdint.h>

void JoypadSender_Init(void);
void JoypadSender_Task(void *pvParameters);
uint8_t JoypadSender_GetSignalQuality(void);

#endif


/* --------------------------------------------------------------------------
 * FILE: firmware/joypad_sender.cpp
 * -------------------------------------------------------------------------- */

#include "joypad_sender.h"
#include "joypad_input.h"
#include "nrf_sender.h"
#include "auto_mode.h"
#include "child_cmd.h"
#include "test_mode.h"
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

        /* 视觉纠偏测试帧 (TSTA / ADJ / TSTE): 最高优先 */
        uint8_t tbuf[32]; uint8_t tlen = 0;
        if (TestMode_WantsSendFrame(tbuf, &tlen)) {
            NRF_StopListening();
            NRF_SendFrame(tbuf, tlen);
            TestMode_OnFrameSent();
        }

        /* 子机指令直通 (App 按键 / 视觉纠偏): 优先于虚拟手柄帧 */
        uint8_t child_frame[32];
        if (ChildCmd_Pop(child_frame)) {
            NRF_StopListening();
            NRF_SendFrame(child_frame, sizeof(child_frame));
        }

        /* 测试模式: 保持 RX 监听等 TSTB/DONE, 不发摇杆帧抢占 */
        if (TestMode_IsActive()) {
            NRF_StartListening();
            TestMode_Tick();
            vTaskDelayUntil(&xLastWakeTime, period);
            continue;
        }

        NRF_StopListening();
        bool success = NRF_SendFrame(buf, sizeof(buf));
        signal_quality = CalculateSuccessRatio(success);

        if (success && last_key != 0) {
            last_key = 0;
            Joypad_ClearKey();
        }

        /* 发完切回 RX, 下一轮才能收到子机应答 */
        NRF_StartListening();

        vTaskDelayUntil(&xLastWakeTime, period);
    }
}


/* --------------------------------------------------------------------------
 * FILE: firmware/test_mode.h
 * -------------------------------------------------------------------------- */

/* test_mode.h - 子机视觉纠偏测试模式 (无摄像头, 假偏差下发)
 *
 * 链路: App -> WS(0x30/0x31/0x32) -> 本模块 -> NRF24L01 -> 子机
 *   TSTA            母->子: 进入测试, 子机回 TSTB
 *   ADJ,<p>,<s>,<a> 母->子: 假视觉偏差, 子机执行后回 DONE
 *   TSTE            母->子: 退出测试
 *   'J' + 10B      子->母: 轮子运动遥测, 原样转发 App (0x86)
 */
#ifndef TEST_MODE_H
#define TEST_MODE_H

#include <stdint.h>
#include <stdbool.h>

void TestMode_Init(void);
void TestMode_Start(void);
void TestMode_SendAdj(float phi_deg, float psi_deg, float ax_mm);
void TestMode_Stop(void);

/* 主循环调用: 收 TSTB/DONE/'J' 并推进状态 */
void TestMode_Tick(void);

/* 发送队列: 由 JoypadSender_Task 取走 */
bool TestMode_WantsSendFrame(uint8_t* out_buf, uint8_t* out_len);
void TestMode_OnFrameSent(void);

bool     TestMode_IsActive(void);
uint8_t TestMode_GetState(void);
uint8_t TestMode_GetRound(void);

/* 供 web_server 拼状态帧 */
void TestMode_GetStatus(uint8_t* state, uint8_t* round, uint8_t* last_rc);

/* 遥测帧 'J' 转发出口, 由 web_server 注入 */
typedef void (*TestMode_TelemSink)(const uint8_t* frame, uint8_t len);
void TestMode_SetTelemetrySink(TestMode_TelemSink fn);

#endif /* TEST_MODE_H */



/* --------------------------------------------------------------------------
 * FILE: firmware/test_mode.cpp
 * -------------------------------------------------------------------------- */

/* test_mode.cpp - 子机视觉纠偏测试模式
 *
 * 状态机:
 *   IDLE --Start--> SEND_TSTA --> WAIT_TSTB --> READY
 *   READY --SendAdj--> SEND_ADJ --> WAIT_DONE --> READY (轮次+1)
 *   READY --Stop--> SEND_TSTE --> IDLE
 * 任一等待态超时回 IDLE 并置 last_rc=3。
 */
#include "test_mode.h"
#include "nrf_sender.h"
#include "config.h"
#include <Arduino.h>
#include <string.h>

#define TSTA_TIMEOUT_MS   3000
#define TSTB_TIMEOUT_MS   5000
#define DONE_TIMEOUT_MS  60000   /* 子机执行一段纠偏最长时限 */
#define TSTE_TIMEOUT_MS   1000

enum {
    TM_IDLE = 0,
    TM_SEND_TSTA,
    TM_WAIT_TSTB,
    TM_READY,
    TM_SEND_ADJ,
    TM_WAIT_DONE,
    TM_SEND_TSTE,
};

static uint8_t  s_state = TM_IDLE;
static uint8_t  s_round = 0;
static uint8_t  s_last_rc = 0;
static uint32_t s_deadline = 0;

static uint8_t  s_tx_buf[32];
static uint8_t  s_tx_len = 0;
static bool     s_tx_pending = false;

/* 遥测转发回调, 由 web_server 注入 */
typedef void (*TelemSink)(const uint8_t* frame, uint8_t len);
static TelemSink s_telem_sink = NULL;

void TestMode_SetTelemetrySink(TelemSink fn) {
    s_telem_sink = fn;
}

void TestMode_Init(void) {
    s_state = TM_IDLE;
    s_round = 0;
    s_last_rc = 0;
    s_tx_pending = false;
}

bool TestMode_IsActive(void) { return s_state != TM_IDLE; }
uint8_t TestMode_GetState(void) { return s_state; }
uint8_t TestMode_GetRound(void) { return s_round; }

void TestMode_GetStatus(uint8_t* state, uint8_t* round, uint8_t* last_rc) {
    if (state)   *state = s_state;
    if (round)   *round = s_round;
    if (last_rc) *last_rc = s_last_rc;
}

static void queue_str(const char* s) {
    size_t len = strlen(s);
    if (len > 32) len = 32;
    memset(s_tx_buf, 0, sizeof(s_tx_buf));
    memcpy(s_tx_buf, s, len);
    s_tx_len = (uint8_t)len;
    s_tx_pending = true;
}

void TestMode_Start(void) {
    s_round = 0;
    s_last_rc = 0;
    queue_str("TSTA");
    s_state = TM_SEND_TSTA;
    s_deadline = millis() + TSTA_TIMEOUT_MS;
    Serial.println("[TEST] start, send TSTA");
}

void TestMode_SendAdj(float phi_deg, float psi_deg, float ax_mm) {
    if (s_state != TM_READY) {
        Serial.printf("[TEST] SendAdj rejected, state=%u\n", s_state);
        return;
    }
    char buf[32];
    snprintf(buf, sizeof(buf), "ADJ,%.1f,%.1f,%.1f",
             (double)phi_deg, (double)psi_deg, (double)ax_mm);
    queue_str(buf);
    s_state = TM_SEND_ADJ;
    s_deadline = millis() + TSTB_TIMEOUT_MS;
    Serial.printf("[TEST] round %u send %s\n", (unsigned)s_round + 1, buf);
}

void TestMode_Stop(void) {
    if (s_state == TM_IDLE) return;
    queue_str("TSTE");
    s_state = TM_SEND_TSTE;
    s_deadline = millis() + TSTE_TIMEOUT_MS;
    Serial.println("[TEST] stop, send TSTE");
}

bool TestMode_WantsSendFrame(uint8_t* out_buf, uint8_t* out_len) {
    if (!s_tx_pending) return false;
    memcpy(out_buf, s_tx_buf, s_tx_len);
    *out_len = s_tx_len;
    s_tx_pending = false;
    return true;
}

void TestMode_OnFrameSent(void) {
    NRF_StartListening();
    if (s_state == TM_SEND_TSTA)      { s_state = TM_WAIT_TSTB; s_deadline = millis() + TSTB_TIMEOUT_MS; }
    else if (s_state == TM_SEND_ADJ)  { s_state = TM_WAIT_DONE; s_deadline = millis() + DONE_TIMEOUT_MS; }
    else if (s_state == TM_SEND_TSTE) { s_state = TM_IDLE; }
}

void TestMode_Tick(void) {
    if (s_state == TM_IDLE) return;

    uint8_t buf[32];
    uint8_t len = 0;
    while (NRF_ReceiveFrame(buf, sizeof(buf), &len)) {
        if (len >= 11 && buf[0] == 'J') {
            if (s_telem_sink) s_telem_sink(buf, len);
            continue;
        }
        if (len >= 4 && memcmp(buf, "TSTB", 4) == 0 && s_state == TM_WAIT_TSTB) {
            Serial.println("[TEST] TSTB, child ready");
            s_state = TM_READY;
            s_last_rc = 0;
        } else if (len >= 4 && memcmp(buf, "DONE", 4) == 0 && s_state == TM_WAIT_DONE) {
            Serial.printf("[TEST] DONE round %u\n", (unsigned)s_round + 1);
            s_round++;
            s_last_rc = 0;
            s_state = TM_READY;
        }
    }

    if ((int32_t)(millis() - s_deadline) >= 0) {
        Serial.printf("[TEST] timeout in state %u\n", s_state);
        s_last_rc = 3;
        s_state = TM_IDLE;
    }
}


