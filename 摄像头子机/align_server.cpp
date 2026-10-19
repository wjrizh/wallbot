/* align_server.cpp - 视觉纠偏 HTTP API
 *
 * 闭环用法 (对应执行文档 6.3 伪代码, 母机/App 侧实现):
 *   1. 放下前: GET /align/ref            记录参考位 (1 秒)
 *   2. 回程到预对接点停稳后: GET /align/get
 *      -> {"ok":true,"phi_deg":..,"psi_deg":..,"arc_mm":..,"aligned":..}
 *      |phi|>4° 发 MV,arc 给子机; |psi|>4° 发 SPIN,-psi; 全达标 -> 夹取
 */
#include <Arduino.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "config.h"
#include "cam_module.h"
#include "tag_detector.h"
#include "align_measure.h"
#include "align_server.h"

/* 上次测量缓存 (供 /align/status) */
static Deviation g_last_dev;
static bool g_last_valid = false;
static char g_last_reason[32] = "never_run";

/* 连拍 ALIGN_FRAMES 帧投票检测; 全部命中且一致 -> 返回平均观测 */
static bool measure_consensus(TagObservation *avg, const char **reason) {
    if (!Cam_EnterDetectMode()) {
        *reason = "cam_failed";
        return false;
    }
    TagObservation obs[ALIGN_FRAMES];
    int n_ok = 0;
    for (int k = 0; k < ALIGN_FRAMES; k++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) continue;
        TagObservation o;
        bool got = TagDetector_Detect(fb->buf, fb->width, fb->height, fb->width, &o);
        esp_camera_fb_return(fb);
        if (got) obs[n_ok++] = o;
    }
    Cam_ExitDetectMode();

    if (n_ok < ALIGN_FRAMES) {           /* 连续一致才采信: 有一帧丢失即不采信 */
        *reason = (n_ok == 0) ? "no_tag" : "tag_unstable";
        return false;
    }

    /* 一致性: theta 展幅与中心位移 */
    float th_min = 1e9f, th_max = -1e9f;
    float u_min = 1e9f, u_max = -1e9f, v_min = 1e9f, v_max = -1e9f;
    float sin_sum = 0, cos_sum = 0;
    for (int k = 0; k < ALIGN_FRAMES; k++) {
        if (obs[k].theta < th_min) th_min = obs[k].theta;
        if (obs[k].theta > th_max) th_max = obs[k].theta;
        if (obs[k].u < u_min) u_min = obs[k].u;
        if (obs[k].u > u_max) u_max = obs[k].u;
        if (obs[k].v < v_min) v_min = obs[k].v;
        if (obs[k].v > v_max) v_max = obs[k].v;
        sin_sum += sinf(obs[k].theta);
        cos_sum += cosf(obs[k].theta);
    }
    if ((th_max - th_min) * 180.0f / (float)M_PI > ALIGN_THETA_SPREAD ||
        (u_max - u_min) > ALIGN_UV_SPREAD || (v_max - v_min) > ALIGN_UV_SPREAD) {
        *reason = "jitter";
        return false;
    }


    memset(avg, 0, sizeof(*avg));
    avg->ok = true;
    avg->id = obs[0].id;
    avg->hamming = obs[0].hamming;
    avg->u = 0; avg->v = 0; avg->edge_a = 0; avg->edge_p = 0; avg->margin = 0; avg->contrast = 0;
    for (int k = 0; k < ALIGN_FRAMES; k++) {
        avg->u += obs[k].u / ALIGN_FRAMES;
        avg->v += obs[k].v / ALIGN_FRAMES;
        avg->edge_a += obs[k].edge_a / ALIGN_FRAMES;
        avg->edge_p += obs[k].edge_p / ALIGN_FRAMES;
        avg->margin += obs[k].margin / ALIGN_FRAMES;
        avg->contrast += obs[k].contrast / ALIGN_FRAMES;
        memcpy(avg->corners, obs[0].corners, sizeof(avg->corners));
    }
    avg->theta = atan2f(sin_sum, cos_sum);
    return true;
}

/* ---- GET /align/get ---- */
static esp_err_t handle_align_get(httpd_req_t *req) {
    char buf[384];
    const AlignRefPose *ref = AlignMeasure_GetRef();
    if (!ref->valid) {
        snprintf(g_last_reason, sizeof(g_last_reason), "no_ref");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send(req, "{\"ok\":false,\"reason\":\"no_ref\"}", HTTPD_RESP_USE_STRLEN);
    }

    TagObservation avg;
    const char *reason = "unknown";
    uint32_t t0 = millis();
    bool got = measure_consensus(&avg, &reason);
    uint32_t dt = millis() - t0;

    if (!got) {
        snprintf(g_last_reason, sizeof(g_last_reason), "%s", reason);
        int n = snprintf(buf, sizeof(buf),
                         "{\"ok\":false,\"reason\":\"%s\",\"dt_ms\":%u}", reason, (unsigned)dt);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send(req, buf, n);
    }

    Deviation dev;
    if (!AlignMeasure_Compute(&avg, &dev)) {
        snprintf(g_last_reason, sizeof(g_last_reason), "compute_failed");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send(req, "{\"ok\":false,\"reason\":\"compute_failed\"}", HTTPD_RESP_USE_STRLEN);
    }

    g_last_dev = dev;
    g_last_valid = true;
    snprintf(g_last_reason, sizeof(g_last_reason), "ok");
    int n = snprintf(buf, sizeof(buf),
        "{\"ok\":true,\"dt_ms\":%u,\"id\":%d,\"hamming\":%d,"
        "\"phi_deg\":%.2f,\"psi_deg\":%.2f,\"phi_shape_deg\":%.2f,"
        "\"axial_mm\":%.1f,\"arc_mm\":%.1f,\"perp_mm\":%.1f,"
        "\"aligned\":%s,\"angle_tol\":%.1f}",
        (unsigned)dt, avg.id, avg.hamming,
        (double)dev.phi_deg, (double)dev.psi_deg, (double)dev.phi_shape_deg,
        (double)dev.axial_mm, (double)dev.arc_mm, (double)dev.perp_mm,
        dev.aligned ? "true" : "false", (double)ALIGN_ANGLE_TOL);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, buf, n);
}

/* ---- GET /align/ref (或 ?clear=1) ---- */
static esp_err_t handle_align_ref(httpd_req_t *req) {
    char qbuf[64] = {0};
    httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf) - 1);
    if (strstr(qbuf, "clear=1")) {
        AlignMeasure_ClearRef();
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send(req, "{\"ok\":true,\"ref\":false}", HTTPD_RESP_USE_STRLEN);
    }

    TagObservation avg;
    const char *reason = "unknown";
    bool got = measure_consensus(&avg, &reason);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    if (!got) {
        char buf[128];
        int n = snprintf(buf, sizeof(buf),
                         "{\"ok\":false,\"reason\":\"%s\"}", reason);
        return httpd_resp_send(req, buf, n);
    }
    bool saved = AlignMeasure_SetRef(&avg);
    char buf[192];
    int n = snprintf(buf, sizeof(buf),
        "{\"ok\":%s,\"ref\":true,\"id\":%d,\"u\":%.1f,\"v\":%.1f,"
        "\"theta_deg\":%.2f,\"edge_px\":%.1f}",
        saved ? "true" : "false", avg.id,
        (double)avg.u, (double)avg.v,
        (double)(avg.theta * 180.0f / (float)M_PI), (double)avg.edge_a);
    Serial.printf("[ALIGN] 参考位已记录: u=%.1f v=%.1f edge=%.1fpx\n", avg.u, avg.v, avg.edge_a);
    return httpd_resp_send(req, buf, n);
}

/* ---- GET /align/status ---- */
static esp_err_t handle_align_status(httpd_req_t *req) {
    char buf[512];
    const AlignRefPose *ref = AlignMeasure_GetRef();
    int n;
    if (ref->valid) {
        n = snprintf(buf, sizeof(buf),
            "ref: u=%.1f v=%.1f theta=%.2fdeg edge=%.1fpx\n"
            "last(%s): phi=%.2fdeg psi=%.2fdeg axial=%.1fmm arc=%.1fmm aligned=%d\n"
            "cfg: tag=%.0fmm R_TAG=%.0fmm pipe_R=%.0fmm tol=%.1fdeg axial_tol=%.0fmm frames=%d\n",
            ref->u, ref->v, ref->theta * 180.0f / (float)M_PI, ref->edge_a,
            g_last_reason,
            g_last_valid ? (double)g_last_dev.phi_deg : 0.0,
            g_last_valid ? (double)g_last_dev.psi_deg : 0.0,
            g_last_valid ? (double)g_last_dev.axial_mm : 0.0,
            g_last_valid ? (double)g_last_dev.arc_mm : 0.0,
            (g_last_valid && g_last_dev.aligned) ? 1 : 0,
            (double)TAG_REAL_MM, (double)R_TAG_MM, (double)PIPE_RADIUS_MM,
            (double)ALIGN_ANGLE_TOL, (double)ALIGN_AXIAL_TOL_MM, ALIGN_FRAMES);
    } else {
        n = snprintf(buf, sizeof(buf),
            "ref: none (last=%s)\n"
            "cfg: tag=%.0fmm R_TAG=%.0fmm pipe_R=%.0fmm tol=%.1fdeg axial_tol=%.0fmm frames=%d\n",
            g_last_reason,
            (double)TAG_REAL_MM, (double)R_TAG_MM, (double)PIPE_RADIUS_MM,
            (double)ALIGN_ANGLE_TOL, (double)ALIGN_AXIAL_TOL_MM, ALIGN_FRAMES);
    }
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, buf, n);
}

void AlignServer_Register(httpd_handle_t control_httpd) {
    httpd_uri_t get_uri = {.uri = "/align/get", .method = HTTP_GET,
                           .handler = handle_align_get, .user_ctx = NULL};
    httpd_uri_t ref_uri = {.uri = "/align/ref", .method = HTTP_GET,
                           .handler = handle_align_ref, .user_ctx = NULL};
    httpd_uri_t status_uri = {.uri = "/align/status", .method = HTTP_GET,
                              .handler = handle_align_status, .user_ctx = NULL};
    if (!control_httpd) return;
    httpd_register_uri_handler(control_httpd, &get_uri);
    httpd_register_uri_handler(control_httpd, &ref_uri);
    httpd_register_uri_handler(control_httpd, &status_uri);
    Serial.println("[ALIGN] API ready: /align/ref /align/get /align/status");
}
