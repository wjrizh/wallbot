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
#define TSTB_TIMEOUT_MS   2000   /* 实测子机 5ms 内回 TSTB, 2s 足够 */
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
