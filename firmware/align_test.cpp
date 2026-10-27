#include "align_test.h"
#include "nrf_sender.h"
#include "coop_vision.h"
#include "auto_mode.h"
#include "child_cmd.h"
#include "web_server.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

static const uint8_t FRAME_TSTA[4] = {'T','S','T','A'};
static const uint8_t FRAME_TSTE[4] = {'T','S','T','E'};
static const uint8_t FRAME_GOON[4] = {'G','O','O','N'};
static const uint8_t FRAME_REDY[4] = {'R','E','D','Y'};
static const uint8_t FRAME_DONE[4] = {'D','O','N','E'};

typedef enum {
    AT_IDLE = 0,
    AT_SEND_TSTA,
    AT_WAIT_REDY,
    AT_REF,
    AT_MEASURE,
    AT_SEND_ADJ,
    AT_WAIT_STEP,
    AT_SEND_GOON,
    AT_WAIT_DONE,        /* GOON 后等子机 DONE, 再发 TSTE */
    AT_SEND_TSTE,
} AtState;

static AtState g_st = AT_IDLE;
static uint32_t g_waitDeadline = 0;
static uint16_t g_iter = 0;
static uint8_t  g_alignedHits = 0;
static uint8_t  g_refRetry = 0, g_measRetry = 0;

static bool g_wants_send = false;
static uint8_t g_send_buf[32] = {0};
static uint8_t g_send_len = 0;

static uint8_t g_rx_buf[8] = {0};
static uint8_t g_rx_idx = 0;

static void request_send(const uint8_t* frame) {
    memcpy(g_send_buf, frame, 4);
    g_send_len = 4;
    g_wants_send = true;
}

static void request_send_adj(float phi, float psi, float ax) {
    int n = snprintf((char*)g_send_buf, sizeof(g_send_buf),
                     "ADJ,%.1f,%.1f,%.1f", (double)phi, (double)psi, (double)ax);
    if (n < 0) n = 0;
    if (n > (int)sizeof(g_send_buf)) n = (int)sizeof(g_send_buf);
    g_send_len = (uint8_t)n;
    g_wants_send = true;
}

static bool recv_match(const uint8_t* target) {
    uint8_t len = 0;
    uint8_t tmp[32];
    while (NRF_ReceiveFrame(tmp, sizeof(tmp), &len)) {
        for (uint8_t i = 0; i < len; i++) {
            g_rx_buf[g_rx_idx] = tmp[i]; g_rx_idx++;
            if (g_rx_idx >= 4) {
                if (memcmp(g_rx_buf, target, 4) == 0) {
                    g_rx_idx = 0; memset(g_rx_buf, 0, 8);
                    return true;
                }
                memmove(g_rx_buf, g_rx_buf + 1, 3); g_rx_idx = 3;
            }
        }
    }
    return false;
}

static void finish_idle(const char* reason) {
    Serial.printf("[AT] %s -> IDLE\n", reason ? reason : "done");
    if (reason) WebServer_SendLog(reason);
    CoopVision_Reset();
    g_iter = 0;
    g_alignedHits = 0;
    g_refRetry = 0;
    g_measRetry = 0;
    g_rx_idx = 0;
    memset(g_rx_buf, 0, 8);
    g_st = AT_IDLE;
    NRF_StopListening();
}

void AlignTest_Init(void) { g_st = AT_IDLE; }

bool AlignTest_IsActive(void) { return g_st != AT_IDLE; }

void AlignTest_Start(void) {
    if (g_st != AT_IDLE) {
        Serial.printf("[AT] already active (state=%u), reject\n", (unsigned)g_st);
        WebServer_SendLog("[AT] already active, reject");
        return;
    }
    if (AutoMode_IsActive()) {
        Serial.println("[AT] coop active, reject align-test");
        WebServer_SendLog("[AT] coop active, reject");
        return;
    }
    CoopVision_Reset();
    ChildCmd_Clear();
    g_iter = 0;
    g_alignedHits = 0;
    g_refRetry = 0;
    g_measRetry = 0;
    g_rx_idx = 0;
    memset(g_rx_buf, 0, 8);
    g_wants_send = false;
    Serial.println("[AT] start: TSTA, wait REDY");
    WebServer_SendLog("[AT] align-test start (TSTA->REDY)");
    request_send(FRAME_TSTA);
    g_st = AT_SEND_TSTA;
}

void AlignTest_Stop(void) {
    if (g_st == AT_IDLE) return;
    Serial.println("[AT] stop, send TSTE");
    WebServer_SendLog("[AT] align-test stop");
    CoopVision_Reset();
    ChildCmd_Clear();
    g_wants_send = false;
    request_send(FRAME_TSTE);
    g_st = AT_SEND_TSTE;
}

bool AlignTest_WantsSendFrame(uint8_t* out_buf, uint8_t* out_len) {
    if (g_wants_send) {
        memcpy(out_buf, g_send_buf, g_send_len);
        *out_len = g_send_len;
        g_wants_send = false;
        return true;
    }
    return false;
}

void AlignTest_OnFrameSent(void) {
    NRF_StartListening();
    g_rx_idx = 0;
    memset(g_rx_buf, 0, 8);
    if (g_st == AT_SEND_TSTA) {
        g_st = AT_WAIT_REDY;
        g_waitDeadline = millis() + 10000;
    } else if (g_st == AT_SEND_ADJ) {
        g_st = AT_WAIT_STEP;
    } else if (g_st == AT_SEND_GOON) {
        g_st = AT_WAIT_DONE;
        g_waitDeadline = millis() + 10000;
    } else if (g_st == AT_SEND_TSTE) {
        finish_idle("[AT] TSTE sent, done");
    }
}

void AlignTest_Tick(void) {
    switch (g_st) {
    case AT_IDLE:
    case AT_SEND_TSTA:
    case AT_SEND_ADJ:
    case AT_SEND_GOON:
    case AT_SEND_TSTE:
        break;

    case AT_WAIT_DONE:
        if (recv_match(FRAME_DONE)) {
            Serial.println("[AT] got DONE, send TSTE");
            WebServer_SendLog("[AT] child DONE, exit test");
            request_send(FRAME_TSTE);
            g_st = AT_SEND_TSTE;
        } else if (millis() > g_waitDeadline) {
            Serial.println("[AT] wait DONE timeout, send TSTE");
            WebServer_SendLog("[AT] wait DONE timeout");
            request_send(FRAME_TSTE);
            g_st = AT_SEND_TSTE;
        }
        break;

    case AT_WAIT_REDY:
        if (recv_match(FRAME_REDY)) {
            Serial.println("[AT] got REDY, request ref");
            WebServer_SendLog("[AT] child ready, take ref");
            CoopVision_RequestRef();
            g_refRetry = 0;
            g_waitDeadline = millis() + 15000;
            g_st = AT_REF;
        } else if (millis() > g_waitDeadline) {
            Serial.println("[AT] wait REDY timeout, TSTE");
            WebServer_SendLog("[AT] wait REDY timeout");
            request_send(FRAME_TSTE);
            g_st = AT_SEND_TSTE;
        }
        break;

    case AT_REF: {
        CoopVisionState cv = CoopVision_GetState();
        if (cv == CV_REF_DONE) {
            Serial.println("[AT] ref done, measure");
            WebServer_SendLog("[AT] ref done");
            g_iter = 0;
            g_alignedHits = 0;
            g_measRetry = 0;
            CoopVision_RequestMeasure();
            g_waitDeadline = millis() + 15000;
            g_st = AT_MEASURE;
        } else if (cv == CV_REF_FAIL) {
            g_refRetry++;
            if (g_refRetry >= 3) {
                Serial.println("[AT] ref fail x3, exit");
                WebServer_SendLog("[AT] ref fail x3");
                request_send(FRAME_TSTE);
                g_st = AT_SEND_TSTE;
            } else {
                Serial.printf("[AT] ref fail, retry %u/3\n", (unsigned)g_refRetry);
                CoopVision_RequestRef();
            }
        } else if (millis() > g_waitDeadline) {
            Serial.println("[AT] ref timeout, exit");
            WebServer_SendLog("[AT] ref timeout");
            request_send(FRAME_TSTE);
            g_st = AT_SEND_TSTE;
        }
        break;
    }

    case AT_MEASURE: {
        CoopVisionState cv = CoopVision_GetState();
        if (cv == CV_MEAS_DONE) {
            float phi, psi, ax;
            bool aligned;
            CoopVision_GetDev(&phi, &psi, &ax, &aligned);
            g_iter++;
            Serial.printf("[AT] it=%u meas phi=%.2f psi=%.2f ax=%.1f aligned=%d\n",
                          (unsigned)g_iter, (double)phi, (double)psi, (double)ax,
                          aligned ? 1 : 0);
            char lb[96];
            snprintf(lb, sizeof(lb), "[AT] it=%u phi=%.2f psi=%.2f ax=%.1f%s",
                     (unsigned)g_iter, (double)phi, (double)psi, (double)ax,
                     aligned ? " OK" : "");
            WebServer_SendLog(lb);

            if (g_iter >= AV_SAFETY_MAX_ITER) {
                Serial.println("[AT] max iter, exit");
                WebServer_SendLog("[AT] max iter");
                request_send(FRAME_TSTE);
                g_st = AT_SEND_TSTE;
                break;
            }
            if (aligned) {
                g_alignedHits++;
                if (g_alignedHits >= AV_ALIGNED_HITS) {
                    Serial.printf("[AT] converged after %u, GOON+TSTE\n", (unsigned)g_iter);
                    WebServer_SendLog("[AT] converged");
                    request_send(FRAME_GOON);
                    g_st = AT_SEND_GOON;
                } else {
                    Serial.printf("[AT] aligned hit %u/%u, remeasure\n",
                                  (unsigned)g_alignedHits, (unsigned)AV_ALIGNED_HITS);
                    CoopVision_RequestMeasure();
                    g_waitDeadline = millis() + 15000;
                }
            } else {
                g_alignedHits = 0;
                float cPhi, cPsi, cAx;
                CoopVision_ComputeCmd(phi, psi, ax, &cPsi, &cPhi, &cAx);
                Serial.printf("[AT] it=%u cmd phi=%.2f psi=%.2f ax=%.1f\n",
                              (unsigned)g_iter, (double)cPhi, (double)cPsi, (double)cAx);
                if (cPhi == 0.0f && cPsi == 0.0f && cAx == 0.0f) {
                    Serial.println("[AT] deadzone, treat as converged");
                    WebServer_SendLog("[AT] deadzone converge");
                    request_send(FRAME_GOON);
                    g_st = AT_SEND_GOON;
                } else {
                    request_send_adj(cPhi, cPsi, cAx);
                    g_waitDeadline = millis() + 70000;
                    g_st = AT_SEND_ADJ;
                }
            }
        } else if (cv == CV_MEAS_FAIL) {
            g_measRetry++;
            if (g_measRetry >= 3) {
                Serial.println("[AT] meas fail x3, exit");
                WebServer_SendLog("[AT] meas fail x3");
                request_send(FRAME_TSTE);
                g_st = AT_SEND_TSTE;
            } else {
                Serial.printf("[AT] meas fail, retry %u/3\n", (unsigned)g_measRetry);
                CoopVision_RequestMeasure();
            }
        } else if (millis() > g_waitDeadline) {
            Serial.println("[AT] meas timeout, exit");
            WebServer_SendLog("[AT] meas timeout");
            request_send(FRAME_TSTE);
            g_st = AT_SEND_TSTE;
        }
        break;
    }

    case AT_WAIT_STEP:
        if (recv_match(FRAME_REDY)) {
            Serial.printf("[AT] it=%u step done, remeasure\n", (unsigned)g_iter);
            CoopVision_RequestMeasure();
            g_waitDeadline = millis() + 15000;
            g_st = AT_MEASURE;
        } else if (millis() > g_waitDeadline) {
            Serial.println("[AT] step timeout, exit");
            WebServer_SendLog("[AT] step timeout");
            request_send(FRAME_TSTE);
            g_st = AT_SEND_TSTE;
        }
        break;

    default:
        break;
    }
}

void AlignTest_FillVisionStatus(uint8_t out[11]) {
    float phi = 0.0f, psi = 0.0f, ax = 0.0f;
    bool aligned = false;
    CoopVision_GetDev(&phi, &psi, &ax, &aligned);
    uint8_t st;
    switch (g_st) {
    case AT_REF:
    case AT_MEASURE:
    case AT_WAIT_STEP:
        st = (uint8_t)CoopVision_GetState();
        break;
    case AT_IDLE:
        st = (uint8_t)CV_IDLE;
        break;
    default:
        st = 201; /* align-test active, non-vision phase */
        break;
    }
    out[0] = st;
    out[1] = (uint8_t)g_iter;
    out[2] = 0;
    int16_t p = (int16_t)lroundf(phi * 100.0f);
    int16_t q = (int16_t)lroundf(psi * 100.0f);
    int16_t a = (int16_t)lroundf(ax);
    memcpy(&out[3], &p, 2);
    memcpy(&out[5], &q, 2);
    memcpy(&out[7], &a, 2);
    out[9] = aligned ? 1 : 0;
    out[10] = (uint8_t)AV_SAFETY_MAX_ITER;
}
