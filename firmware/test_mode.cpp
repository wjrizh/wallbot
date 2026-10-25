/* test_mode.cpp - 子机视觉纠偏测试模式
 *
 * 状态机:
 *   IDLE --Start--> SEND_TSTA --> WAIT_TSTB --> READY
 *   READY --SendAdj--> SEND_ADJ --> WAIT_DONE --> READY (轮次+1)
 *   READY --Stop--> SEND_TSTE --> IDLE
 * 任一等待态超时回 IDLE 并置 last_rc=3。
 *
 * 并发: JoypadSender_Task 取帧/收帧, AlignVision_Task 或 WS 回调发起命令,
 *       状态与发送缓冲统一用 portMUX 短临界区保护。
 */
#include "test_mode.h"
#include "nrf_sender.h"
#include "auto_mode.h"
#include "config.h"
#include <Arduino.h>
#include <string.h>
#include <freertos/FreeRTOS.h>

#define TSTA_TIMEOUT_MS   3000
#define TSTB_TIMEOUT_MS   2000   /* 实测子机 5ms 内回 TSTB, 2s 足够 */
#define DONE_TIMEOUT_MS  60000   /* 子机执行一段纠偏最长时限 */
#define TSTE_TIMEOUT_MS   1000
#define TLM_LOG_PERIOD_MS 500    /* 'J' 遥测串口打印节流 */

enum {
    TM_IDLE = 0,
    TM_SEND_TSTA,
    TM_WAIT_TSTB,
    TM_READY,
    TM_SEND_ADJ,
    TM_WAIT_DONE,
    TM_SEND_TSTE,
};

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

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
    portENTER_CRITICAL(&s_mux);
    s_state = TM_IDLE;
    s_round = 0;
    s_last_rc = 0;
    s_tx_pending = false;
    portEXIT_CRITICAL(&s_mux);
}

bool TestMode_IsActive(void) { return s_state != TM_IDLE; }
uint8_t TestMode_GetState(void) { return s_state; }
uint8_t TestMode_GetRound(void) { return s_round; }

void TestMode_GetStatus(uint8_t* state, uint8_t* round, uint8_t* last_rc) {
    if (state)   *state = s_state;
    if (round)   *round = s_round;
    if (last_rc) *last_rc = s_last_rc;
}

/* 调用方需已进入临界区 */
static void queue_str_locked(const char* s) {
    size_t len = strlen(s);
    if (len > 32) len = 32;
    memset(s_tx_buf, 0, sizeof(s_tx_buf));
    memcpy(s_tx_buf, s, len);
    s_tx_len = (uint8_t)len;
    s_tx_pending = true;
}

void TestMode_Start(void) {
    if (AutoMode_IsActive()) {
        Serial.println("[TEST] start rejected: AutoMode active");
        return;
    }
    portENTER_CRITICAL(&s_mux);
    s_round = 0;
    s_last_rc = 0;
    queue_str_locked("TSTA");
    s_state = TM_SEND_TSTA;
    s_deadline = millis() + TSTA_TIMEOUT_MS;
    portEXIT_CRITICAL(&s_mux);
    Serial.println("[TEST] start, send TSTA");
}

void TestMode_SendAdj(float phi_deg, float psi_deg, float ax_mm) {
    if (s_state != TM_READY) {
        Serial.printf("[TEST] SendAdj rejected, state=%u (need 3=READY)\n", s_state);
        return;
    }
    char buf[32];
    snprintf(buf, sizeof(buf), "ADJ,%.1f,%.1f,%.1f",
             (double)phi_deg, (double)psi_deg, (double)ax_mm);
    portENTER_CRITICAL(&s_mux);
    queue_str_locked(buf);
    s_state = TM_SEND_ADJ;
    s_deadline = millis() + TSTB_TIMEOUT_MS;
    portEXIT_CRITICAL(&s_mux);
    Serial.printf("[TEST] round %u send %s\n", (unsigned)s_round + 1, buf);
}

void TestMode_Stop(void) {
    if (s_state == TM_IDLE) return;
    portENTER_CRITICAL(&s_mux);
    queue_str_locked("TSTE");
    s_state = TM_SEND_TSTE;
    s_deadline = millis() + TSTE_TIMEOUT_MS;
    portEXIT_CRITICAL(&s_mux);
    Serial.println("[TEST] stop, send TSTE");
}

bool TestMode_WantsSendFrame(uint8_t* out_buf, uint8_t* out_len) {
    bool has = false;
    portENTER_CRITICAL(&s_mux);
    if (s_tx_pending) {
        memcpy(out_buf, s_tx_buf, s_tx_len);
        *out_len = s_tx_len;
        s_tx_pending = false;
        has = true;
    }
    portEXIT_CRITICAL(&s_mux);
    return has;
}

void TestMode_OnFrameSent(void) {
    NRF_StartListening();
    portENTER_CRITICAL(&s_mux);
    if (s_state == TM_SEND_TSTA)      { s_state = TM_WAIT_TSTB; s_deadline = millis() + TSTB_TIMEOUT_MS; }
    else if (s_state == TM_SEND_ADJ)  { s_state = TM_WAIT_DONE; s_deadline = millis() + DONE_TIMEOUT_MS; }
    else if (s_state == TM_SEND_TSTE) { s_state = TM_IDLE; }
    portEXIT_CRITICAL(&s_mux);
}

/* 子机轮子遥测: 串口打印便于台架观察 */
static void logTelem(const uint8_t* buf) {
    static uint32_t lastLog = 0;
    static uint8_t lastStage = 0, lastPct = 0;
    uint32_t now = millis();
    int16_t a, b, tgt;
    memcpy(&a, &buf[3], 2);
    memcpy(&b, &buf[5], 2);
    memcpy(&tgt, &buf[7], 2);
    uint8_t stage = buf[1], pct = buf[9], rc = buf[10];
    /* 阶段切换 / 有 rc / 500ms 节流打印 */
    bool due = (stage != lastStage) || (rc != 0) || (now - lastLog >= TLM_LOG_PERIOD_MS);
    if (!due) return;
    lastStage = stage; lastPct = pct; lastLog = now;
    Serial.printf("[TLM] st=%u dir=%d A=%d B=%d tgt=%d pct=%u rc=%u\n",
                  (unsigned)stage, (int)(int8_t)buf[2], (int)a, (int)b,
                  (int)tgt, (unsigned)pct, (unsigned)rc);
}

void TestMode_Tick(void) {
    if (s_state == TM_IDLE) return;

    uint8_t buf[32];
    uint8_t len = 0;
    while (NRF_ReceiveFrame(buf, sizeof(buf), &len)) {
        if (len >= 11 && buf[0] == 'J') {
            logTelem(buf);
            if (s_telem_sink) s_telem_sink(buf, len);
            continue;
        }
        /* 子机无线日志 'L': [0]='L' [1]=n [2..] 文本. 子机 printf 无 _write, 只能走这条 */
        if (len >= 2 && buf[0] == 'L' && buf[1] <= 30) {
            char msg[32];
            memcpy(msg, &buf[2], buf[1]);
            msg[buf[1]] = '\0';
            Serial.printf("[CHILD] %s\n", msg);
            continue;
        }
        if (len >= 4 && memcmp(buf, "TSTB", 4) == 0 && s_state == TM_WAIT_TSTB) {
            Serial.println("[TEST] TSTB, child ready");
            portENTER_CRITICAL(&s_mux);
            s_state = TM_READY;
            s_last_rc = 0;
            portEXIT_CRITICAL(&s_mux);
        } else if (len >= 4 && memcmp(buf, "DONE", 4) == 0 && s_state == TM_WAIT_DONE) {
            Serial.printf("[TEST] DONE round %u\n", (unsigned)s_round + 1);
            portENTER_CRITICAL(&s_mux);
            s_round++;
            s_last_rc = 0;
            s_state = TM_READY;
            portEXIT_CRITICAL(&s_mux);
        }
    }

    /* TM_READY 是稳态: 等用户点"发送模拟偏差", 不该有超时.
       原先进入 READY 时没更新 s_deadline, 仍是 WAIT_TSTB 的期限,
       TSTB 一到就被判超时 (实测 timeout in state 3) */
    if (s_state == TM_READY) return;

    /* 就绪态不等待回复; 不能沿用 TSTB/DONE 的旧截止时间把测试误判超时。 */
    if ((s_state == TM_WAIT_TSTB || s_state == TM_WAIT_DONE) &&
        (int32_t)(millis() - s_deadline) >= 0) {
        Serial.printf("[TEST] timeout in state %u\n", s_state);
        portENTER_CRITICAL(&s_mux);
        s_last_rc = 3;
        s_state = TM_IDLE;
        portEXIT_CRITICAL(&s_mux);
    }
}
