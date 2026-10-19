/* align_bridge.cpp - 子机视觉纠偏闭环 */
#include "align_bridge.h"
#include "config.h"
#include "child_cmd.h"
#include "web_server.h"
#include <Arduino.h>
#include <HTTPClient.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

static bool s_active = false;
static char s_base[48];
static uint32_t s_deadline = 0;
static uint32_t s_last_poll = 0;
static int s_aligned_hits = 0;
static char s_last_err[48] = "idle";

void AlignBridge_Init(void) {
    s_active = false;
    s_base[0] = 0;
    s_aligned_hits = 0;
}

bool AlignBridge_IsActive(void) { return s_active; }

bool AlignBridge_Start(const uint8_t ip[4]) {
    snprintf(s_base, sizeof(s_base), "http://%u.%u.%u.%u",
             (unsigned)ip[0], (unsigned)ip[1], (unsigned)ip[2], (unsigned)ip[3]);
    s_active = true;
    s_aligned_hits = 0;
    s_last_poll = 0;
    s_deadline = millis() + ALIGN_BRIDGE_TIMEOUT_MS;
    snprintf(s_last_err, sizeof(s_last_err), "started");
    ChildCmd_Clear();
    WebServer_SendLog("[ALIGN] 视觉纠偏启动");
    Serial.printf("[ALIGN] start cam=%s\n", s_base);
    return true;
}

void AlignBridge_Stop(void) {
    if (!s_active) return;
    s_active = false;
    ChildCmd_Clear();
    ChildCmd_Set(0, 0, 0);
    WebServer_SendLog("[ALIGN] 视觉纠偏停止");
    Serial.printf("[ALIGN] stop (%s)\n", s_last_err);
}

/* 从 JSON 文本取数值字段, 找不到返回 false */
static bool json_num(const char* body, const char* key, float* out) {
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

static bool json_true(const char* body, const char* key, bool* out) {
    const char* p = strstr(body, key);
    if (!p) return false;
    p += strlen(key);
    while (*p == ' ' || *p == ':') p++;
    if (strncmp(p, "true", 4) == 0) { *out = true; return true; }
    if (strncmp(p, "false", 5) == 0) { *out = false; return true; }
    return false;
}

static int8_t clamp_i8(float v, float lim) {
    if (v > lim) v = lim;
    if (v < -lim) v = -lim;
    return (int8_t)lroundf(v);
}

void AlignBridge_Tick(void) {
    if (!s_active) return;
    uint32_t now = millis();
    if (now > s_deadline) {
        snprintf(s_last_err, sizeof(s_last_err), "timeout");
        WebServer_SendLog("[ALIGN] 纠偏超时");
        AlignBridge_Stop();
        return;
    }
    if (now - s_last_poll < ALIGN_BRIDGE_PERIOD_MS) return;
    s_last_poll = now;

    char url[64];
    snprintf(url, sizeof(url), "%s/align/get", s_base);
    HTTPClient http;
    http.setTimeout(ALIGN_HTTP_TIMEOUT_MS);
    if (!http.begin(url)) {
        snprintf(s_last_err, sizeof(s_last_err), "http_begin_fail");
        return;
    }
    int code = http.GET();
    if (code != 200) {
        snprintf(s_last_err, sizeof(s_last_err), "http_%d", code);
        http.end();
        return;
    }
    String body = http.getString();
    http.end();

    bool ok = false;
    json_true(body.c_str(), "\"ok\"", &ok);
    if (!ok) {
        snprintf(s_last_err, sizeof(s_last_err), "cam_nok");
        ChildCmd_Set(0, 0, 0);
        return;
    }

    float psi = 0, axial = 0, phi = 0;
    bool has_psi   = json_num(body.c_str(), "\"psi_deg\"", &psi);
    bool has_axial = json_num(body.c_str(), "\"axial_mm\"", &axial);
    bool has_phi   = json_num(body.c_str(), "\"phi_deg\"", &phi);
    bool aligned   = false;
    json_true(body.c_str(), "\"aligned\"", &aligned);

    if (aligned) {
        s_aligned_hits++;
        ChildCmd_Set(0, 0, 0);
        snprintf(s_last_err, sizeof(s_last_err), "aligned");
        if (s_aligned_hits >= 2) {
            WebServer_SendLog("[ALIGN] 纠偏达标, 已停止");
            AlignBridge_Stop();
        }
        return;
    }
    s_aligned_hits = 0;

    float spin_v = 0.0f;
    if (has_psi && fabsf(psi) > ALIGN_PSI_DEAD_DEG) {
        spin_v = -ALIGN_PSI_K * psi;
    }
    float speed_v = 0.0f;
    if (has_axial && fabsf(axial) > ALIGN_AXIAL_DEAD_MM) {
        speed_v = -ALIGN_AXIAL_K * (axial / 10.0f);
    }

    int8_t spin  = clamp_i8(spin_v,  (float)ALIGN_SPIN_MAX);
    int8_t speed = clamp_i8(speed_v, (float)ALIGN_SPEED_MAX);
    ChildCmd_Set(speed, spin, 9);   /* key=9: 子机手动模式 */

    snprintf(s_last_err, sizeof(s_last_err), "phi=%.1f", has_phi ? (double)phi : 0.0);
    if (has_phi && fabsf(phi) > 6.0f) {
        static uint32_t last_warn = 0;
        if (now - last_warn > 3000) {
            last_warn = now;
            WebServer_SendLog("[ALIGN] phi 超差, 子机无周向自由度");
        }
    }
}
