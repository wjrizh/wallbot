/* coop_vision.cpp - 协同内视觉纠偏后端 (K230 HTTP)
 *
 * 链路: auto_mode 状态机 -> Request*() -> 本任务 httpGet K230
 *       /align/ref (记参考) / /align/once (共识测量)
 * 结果: s_cv 状态 + s_phi/s_psi/s_ax 偏差缓存, auto_mode 轮询取用;
 *       原始 JSON 经 0x89 推 App。
 *
 * 增益固定 AV_DEFAULT_GAIN_PCT(100%), 不再由 App 传入。
 */
#include "coop_vision.h"
#include "config.h"
#include "web_server.h"
#include <Arduino.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define CV_LOG(fmt, ...)  do { Serial.printf("[CV] " fmt "\n", ##__VA_ARGS__); } while (0)

static volatile CoopVisionState s_cv = CV_IDLE;
static volatile bool s_reqRef = false, s_reqMeas = false;
static float s_phi = 0.0f, s_psi = 0.0f, s_ax = 0.0f;
static bool s_aligned = false;
static char s_base[48] = "";
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

/* 标定 (NVS 持久化) */
static int8_t  s_signPhi = 1, s_signPsi = 1, s_signAx = 1;
static uint8_t s_betaDeg = 20, s_speedPct = 100;
static Preferences s_prefs;

/* ==================== JSON 解析 (原 align_vision) ==================== */

static bool jsonNum(const char* body, const char* key, float* out) {
    const char* p = strstr(body, key);
    if (!p) return false;
    p += strlen(key);
    while (*p == ' ' || *p == ':' || *p == '\"') p++;
    char* end = NULL;
    double v = strtod(p, &end);
    if (end == p) return false;
    *out = (float)v;
    return true;
}

static bool jsonBool(const char* body, const char* key, bool* out) {
    const char* p = strstr(body, key);
    if (!p) return false;
    p += strlen(key);
    while (*p == ' ' || *p == ':' || *p == '\"') p++;
    if (strncmp(p, "true", 4) == 0)  { *out = true;  return true; }
    if (strncmp(p, "false", 5) == 0) { *out = false; return true; }
    return false;
}

/* ==================== HTTP ==================== */

static bool httpGet(const char* base, const char* path, char* out, size_t cap,
                    int* codeOut, uint32_t* dtMsOut) {
    char url[96];
    snprintf(url, sizeof(url), "%s%s", base, path);
    HTTPClient http;
    http.setTimeout(AV_HTTP_TIMEOUT_MS);
    http.setConnectTimeout(AV_HTTP_TIMEOUT_MS);
    if (!http.begin(url)) {
        if (codeOut) *codeOut = -1;
        return false;
    }
    uint32_t t0 = millis();
    int code = http.GET();
    uint32_t dt = millis() - t0;
    if (dtMsOut) *dtMsOut = dt;
    if (codeOut) *codeOut = code;
    if (code != 200) {
        http.end();
        return false;
    }
    String body = http.getString();
    http.end();
    size_t n = body.length();
    if (n >= cap) n = cap - 1;
    memcpy(out, body.c_str(), n);
    out[n] = '\0';
    return true;
}

static float clampf(float v, float lim) {
    if (v >  lim) return  lim;
    if (v < -lim) return -lim;
    return v;
}

/* ==================== 参数持久化 ==================== */

static void saveTune(void) {
    if (!s_prefs.begin("wallbot-cv", false)) return;
    s_prefs.putChar("sPhi", s_signPhi);
    s_prefs.putChar("sPsi", s_signPsi);
    s_prefs.putChar("sAx",  s_signAx);
    s_prefs.putUChar("beta", s_betaDeg);
    s_prefs.putUChar("spd",  s_speedPct);
    s_prefs.putUChar("init", 1);
    s_prefs.end();
}

static void normalizeTune(void) {
    if (s_signPhi != -1) s_signPhi = 1;
    if (s_signPsi != -1) s_signPsi = 1;
    if (s_signAx  != -1) s_signAx  = 1;
    if (s_betaDeg < 10 || s_betaDeg > 40) s_betaDeg = 20;
    if (s_speedPct < 20 || s_speedPct > 100) s_speedPct = 100;
}

static void loadTune(void) {
    if (s_prefs.begin("wallbot-cv", false) && s_prefs.getUChar("init", 0) == 1) {
        s_signPhi  = (int8_t)s_prefs.getChar("sPhi", 1);
        s_signPsi  = (int8_t)s_prefs.getChar("sPsi", 1);
        s_signAx   = (int8_t)s_prefs.getChar("sAx", 1);
        s_betaDeg  = (uint8_t)s_prefs.getUChar("beta", 20);
        s_speedPct = (uint8_t)s_prefs.getUChar("spd", 100);
        s_prefs.end();
        normalizeTune();
        return;
    }
    s_prefs.end();
    /* 首次启动: 从旧 align_vision 的 wallbot-av 迁移标定值 */
    bool migrated = false;
    if (s_prefs.begin("wallbot-av", true)) {
        s_signPhi  = (int8_t)s_prefs.getChar("sPhi", 1);
        s_signPsi  = (int8_t)s_prefs.getChar("sPsi", 1);
        s_signAx   = (int8_t)s_prefs.getChar("sAx", 1);
        s_betaDeg  = (uint8_t)s_prefs.getUChar("beta", 20);
        s_speedPct = (uint8_t)s_prefs.getUChar("spd", 100);
        s_prefs.end();
        migrated = true;
    }
    normalizeTune();
    saveTune();
    if (migrated) CV_LOG("tune migrated from wallbot-av");
}

static void loadBase(void) {
    uint8_t ip[4] = {0};
    if (s_prefs.begin("wallbot-cv", true)) {
        size_t n = s_prefs.getBytes("vip", ip, 4);
        s_prefs.end();
        if (n == 4) {
            snprintf(s_base, sizeof(s_base), "http://%u.%u.%u.%u",
                     (unsigned)ip[0], (unsigned)ip[1],
                     (unsigned)ip[2], (unsigned)ip[3]);
            CV_LOG("base from NVS: %s", s_base);
        }
    }
}

/* ==================== 对外接口 ==================== */

void CoopVision_Init(void) {
    s_cv = CV_IDLE;
    s_reqRef = false;
    s_reqMeas = false;
    s_phi = s_psi = s_ax = 0.0f;
    s_aligned = false;
    loadTune();
    loadBase();
    CV_LOG("init sign=%+d/%+d/%+d beta=%u spd=%u%% gain=%u%%",
           s_signPhi, s_signPsi, s_signAx, s_betaDeg, s_speedPct,
           (unsigned)AV_DEFAULT_GAIN_PCT);
}

void CoopVision_SetBase(const uint8_t ip[4]) {
    portENTER_CRITICAL(&s_mux);
    snprintf(s_base, sizeof(s_base), "http://%u.%u.%u.%u",
             (unsigned)ip[0], (unsigned)ip[1],
             (unsigned)ip[2], (unsigned)ip[3]);
    portEXIT_CRITICAL(&s_mux);
    if (s_prefs.begin("wallbot-cv", false)) {
        s_prefs.putBytes("vip", ip, 4);
        s_prefs.putUChar("init", 1);
        s_prefs.end();
    }
    CV_LOG("base set %s", s_base);
}

void CoopVision_RequestRef(void) {
    s_reqRef = false;
    s_cv = CV_REF_BUSY;
    s_reqRef = true;
}

void CoopVision_RequestMeasure(void) {
    s_reqMeas = false;
    s_cv = CV_MEAS_BUSY;
    s_reqMeas = true;
}

void CoopVision_Reset(void) {
    s_reqRef = false;
    s_reqMeas = false;
    s_cv = CV_IDLE;
}

CoopVisionState CoopVision_GetState(void) {
    return s_cv;
}

void CoopVision_GetDev(float* phi, float* psi, float* ax, bool* aligned) {
    if (phi) *phi = s_phi;
    if (psi) *psi = s_psi;
    if (ax)  *ax  = s_ax;
    if (aligned) *aligned = s_aligned;
}

void CoopVision_SetTune(int8_t signPhi, int8_t signPsi, int8_t signAx,
                        uint8_t betaDeg, uint8_t speedPct) {
    s_signPhi = (signPhi < 0) ? -1 : 1;
    s_signPsi = (signPsi < 0) ? -1 : 1;
    s_signAx  = (signAx  < 0) ? -1 : 1;
    if (betaDeg >= 10 && betaDeg <= 40) s_betaDeg = betaDeg;
    if (speedPct >= 20 && speedPct <= 100) s_speedPct = speedPct;
    saveTune();
    CV_LOG("tune sign=%+d/%+d/%+d beta=%u spd=%u%% (saved)",
           s_signPhi, s_signPsi, s_signAx, s_betaDeg, s_speedPct);
}

void CoopVision_GetTune(int8_t* signPhi, int8_t* signPsi, int8_t* signAx,
                        uint8_t* betaDeg, uint8_t* speedPct) {
    if (signPhi) *signPhi = s_signPhi;
    if (signPsi) *signPsi = s_signPsi;
    if (signAx)  *signAx  = s_signAx;
    if (betaDeg) *betaDeg = s_betaDeg;
    if (speedPct) *speedPct = s_speedPct;
}

void CoopVision_ComputeCmd(float phi, float psi, float ax,
                           float* cPsi, float* cPhi, float* cAx) {
    float gain = (float)AV_DEFAULT_GAIN_PCT / 100.0f;
    float fPsi = clampf(psi * (float)s_signPsi * gain, AV_CMD_MAX_PSI);
    float fPhi = clampf(phi * (float)s_signPhi * gain, AV_CMD_MAX_PHI);
    float fAx  = clampf(ax  * (float)s_signAx  * gain, AV_CMD_MAX_AX);
    /* 只纠超出容差的轴: 进容差清零, 防止小误差被最小命令放大 */
    if (fabsf(phi) <= AV_RAW_TOL_PHI) fPhi = 0.0f;
    if (fabsf(psi) <= AV_RAW_TOL_PSI) fPsi = 0.0f;
    if (fabsf(ax)  <= AV_RAW_TOL_AX)  fAx  = 0.0f;
    /* 增益过小导致命令低于子机最小可动量: 抬到最小量, 防止空转烧轮次 */
    if (fPhi != 0.0f && fabsf(fPhi) < AV_CMD_MIN_PHI)
        fPhi = (fPhi > 0.0f) ? AV_CMD_MIN_PHI : -AV_CMD_MIN_PHI;
    if (fPsi != 0.0f && fabsf(fPsi) < AV_CMD_MIN_PSI)
        fPsi = (fPsi > 0.0f) ? AV_CMD_MIN_PSI : -AV_CMD_MIN_PSI;
    if (fAx != 0.0f && fabsf(fAx) < AV_CMD_MIN_AX)
        fAx  = (fAx  > 0.0f) ? AV_CMD_MIN_AX  : -AV_CMD_MIN_AX;
    if (cPsi) *cPsi = fPsi;
    if (cPhi) *cPhi = fPhi;
    if (cAx)  *cAx  = fAx;
}

/* ==================== FreeRTOS 任务 ==================== */

void CoopVision_Task(void* pvParameters) {
    (void)pvParameters;
    CV_LOG("task started");
    char body[224];
    for (;;) {
        if (s_reqRef) {
            s_reqRef = false;
            int code = 0;
            bool ok = httpGet(s_base, "/align/ref", body, sizeof(body), &code, NULL);
            CV_LOG("ref http=%d %s", code, ok ? body : "(fail)");
            if (!ok) { body[0] = '\0'; }
            /* 结果态只在仍是本请求的 BUSY 时写入: Reset 后丢弃过期结果 */
            if (s_cv == CV_REF_BUSY) {
                bool rok = false;
                if (ok && jsonBool(body, "\"ok\"", &rok) && rok) {
                    s_cv = CV_REF_DONE;
                    WebServer_SendLog("[CV] 参考位完成");
                } else {
                    s_cv = CV_REF_FAIL;
                }
            }
        }
        if (s_reqMeas) {
            s_reqMeas = false;
            int code = 0;
            uint32_t dt = 0;
            bool ok = httpGet(s_base, "/align/once", body, sizeof(body), &code, &dt);
            CV_LOG("once %ums http=%d %s", (unsigned)dt, code, ok ? body : "(fail)");
            if (!ok) { body[0] = '\0'; }
            if (ok) WebServer_PushAlignJson(body);
            if (s_cv == CV_MEAS_BUSY) {
                bool mok = false;
                if (ok && jsonBool(body, "\"ok\"", &mok) && mok) {
                    jsonNum(body, "\"phi_deg\"", &s_phi);
                    jsonNum(body, "\"psi_deg\"", &s_psi);
                    jsonNum(body, "\"axial_mm\"", &s_ax);
                    jsonBool(body, "\"aligned\"", &s_aligned);
                    s_cv = CV_MEAS_DONE;
                } else {
                    s_cv = CV_MEAS_FAIL;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
