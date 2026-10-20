/* align_server.cpp - 视觉纠偏 HTTP API + 后台检测任务
 *
 * 闭环用法 (对应执行文档 6.3 伪代码, 母机/App 侧实现):
 *   1. 放下前: GET /align/ref            记录参考位
 *   2. 回程到预对接点停稳后: GET /align/get
 *      -> {"ok":true,"phi_deg":..,"psi_deg":..,"arc_mm":..,"aligned":..}
 *      |phi|>4° 发 MV,arc 给子机; |psi|>4° 发 SPIN,-psi; 全达标 -> 夹取
 *
 * 架构 (关键: 检测与推流解耦):
 *   - align_worker_task 后台循环: 取帧 -> JPEG 1/2 缩放解码 -> 灰度 -> 检测
 *     -> 更新 ALIGN_FRAMES 帧滑动窗口共识, 每帧后主动让出 CPU
 *   - 取帧优先向 :81/stream 索要旁路帧 (AlignServer_FeedFrame 投喂, memcpy <0.2ms)
 *     若 :81/stream 无活动 (>250ms 未投喂), worker 自行 esp_camera_fb_get()
 *   - /align/get 只续租 3s 并读缓存, <1ms 返回, 绝不触碰相机与算法
 */
#include <Arduino.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "jpeg_decoder.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "config.h"
#include "cam_module.h"
#include "tag_detector.h"
#include "align_measure.h"
#include "align_server.h"
#include "stream_server.h"

/* 上次测量缓存 (供 /align/status) */
static Deviation g_last_dev;
static bool g_last_valid = false;
static char g_last_reason[32] = "never_run";
static TagDiag g_last_diag;   /* 最近一次检测的诊断 (失败时用于日志/排障) */

/* 检测互斥: TagDetector 内部全局工作区不可并发, worker 与 /view(:81) 共用 */
SemaphoreHandle_t g_measure_lock = NULL;

/* ---- worker 诊断计数 (排查"取不到帧/解不出码/检不到 tag"到底卡在哪一步) ---- */
static volatile uint32_t s_diag_frames = 0;    /* 拿到帧次数 */
static volatile uint32_t s_diag_nofeed = 0;    /* 没拿到任何帧次数 */
static volatile uint32_t s_diag_decode_fail = 0;
static volatile uint32_t s_diag_detect_ok = 0;
static volatile uint32_t s_diag_detect_fail = 0;
static volatile int      s_diag_last_decode_err = 0;   /* 最近一次 esp_jpeg_decode 返回码 */
static volatile bool     s_worker_alive = false;
static volatile int      s_diag_last_w = 0, s_diag_last_h = 0; /* 最近一次投喂帧宽高 */
static volatile int      s_diag_last_ow = 0, s_diag_last_oh = 0; /* 最近一次解码输出宽高 */

/* 检测工作缓冲 (ALIGN_DETECT_W x ALIGN_DETECT_H): JPEG 缩放解码 -> RGB888 -> 灰度 */
static uint8_t *s_dec_rgb = NULL;
static uint8_t *s_dec_gray = NULL;

static bool measure_bufs_init(void) {
    if (s_dec_rgb && s_dec_gray) return true;
    const size_t npix = (size_t)ALIGN_DETECT_W * ALIGN_DETECT_H;
    if (!s_dec_rgb) {
        s_dec_rgb = (uint8_t *)heap_caps_malloc(npix * 3, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_dec_rgb) s_dec_rgb = (uint8_t *)malloc(npix * 3);
    }
    if (!s_dec_gray) {
        s_dec_gray = (uint8_t *)heap_caps_malloc(npix, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_dec_gray) s_dec_gray = (uint8_t *)malloc(npix);
    }
    return s_dec_rgb && s_dec_gray;
}

/* JPEG 帧 -> 灰度 (检测用)。
 * 640x480 走 JPEG_IMAGE_SCALE_1_2 在解码器内部直接缩到 320x240;
 * 320x240 输入则不缩放。其他尺寸一律拒绝。 */
bool AlignDecodeJpegToGray(const uint8_t *jpg, size_t jlen, int fw, int fh,
                           const uint8_t **out_gray, int *out_w, int *out_h) {
    if (!jpg || jlen == 0) return false;
    esp_jpeg_image_scale_t scale;
    if (fw == ALIGN_DETECT_W && fh == ALIGN_DETECT_H) {
        scale = JPEG_IMAGE_SCALE_0;
    } else if (fw == ALIGN_DETECT_W * 2 && fh == ALIGN_DETECT_H * 2) {
        scale = JPEG_IMAGE_SCALE_1_2;
    } else {
        return false;
    }
    if (!measure_bufs_init()) return false;

    const size_t npix = (size_t)ALIGN_DETECT_W * ALIGN_DETECT_H;
    esp_jpeg_image_cfg_t cfg = {};
    cfg.indata = (uint8_t *)jpg;
    cfg.indata_size = (uint32_t)jlen;
    cfg.outbuf = s_dec_rgb;
    cfg.outbuf_size = (uint32_t)(npix * 3);
    cfg.out_format = JPEG_IMAGE_FORMAT_RGB888;
    cfg.out_scale = scale;
    cfg.advanced.working_buffer = NULL;   /* 由 esp_jpeg_decode 自行分配 */
    cfg.advanced.working_buffer_size = 0;

    esp_jpeg_image_output_t out = {};
    esp_err_t derr = esp_jpeg_decode(&cfg, &out);
    s_diag_last_decode_err = (int)derr;
    s_diag_last_ow = out.width;
    s_diag_last_oh = out.height;
    if (derr != ESP_OK) return false;
    if (out.width != ALIGN_DETECT_W || out.height != ALIGN_DETECT_H) return false;
    if (out.output_len < npix * 3) return false;

    for (size_t i = 0; i < npix; i++) {
        const uint8_t *px = &s_dec_rgb[i * 3];
        s_dec_gray[i] = (uint8_t)((px[0] * 77 + px[1] * 151 + px[2] * 28) >> 8);
    }
    *out_gray = s_dec_gray;
    *out_w = ALIGN_DETECT_W;
    *out_h = ALIGN_DETECT_H;
    return true;
}

/* ================= 旁路帧投喂槽 ================= */
#define FEED_JPG_MAX  (48 * 1024)
static uint8_t *s_feed_jpg = NULL;
static size_t   s_feed_len = 0;
static int      s_feed_w = 0, s_feed_h = 0;
static volatile bool     s_want_frame = false;
static volatile bool     s_has_frame  = false;
static volatile uint32_t s_last_stream_feed_ms = 0;
static volatile uint32_t s_detect_active_until = 0; /* 后台检测租约到期时间(ms) */
/* 保护 s_feed_jpg/s_feed_len/s_feed_w/h/s_has_frame/s_want_frame 的互斥:
 * :81/stream 任务写入与 worker 任务读取可能并发, 不加锁会读到半新半旧数据 */
static portMUX_TYPE s_feed_mux = portMUX_INITIALIZER_UNLOCKED;

/* ================= 共识结果缓存 ================= */
static portMUX_TYPE s_res_mux = portMUX_INITIALIZER_UNLOCKED;
static TagObservation s_win_obs[ALIGN_FRAMES];
static int            s_win_count = 0;
static TagObservation s_consensus_obs;
static bool           s_consensus_ok = false;
static char           s_consensus_reason[32] = "no_tag";
static uint32_t       s_last_dt_ms = 0;
static TagObservation s_last_good;      /* 最近一次单帧有效观测 (记参考位兜底用) */
static bool           s_last_good_ok = false;

/* 供 :81/stream 调用: 仅当 worker 正在等帧时才拷走一帧, 耗时 <0.2ms */
void AlignServer_FeedFrame(const uint8_t *jpg, size_t len, int w, int h) {
    s_last_stream_feed_ms = millis();
    if (!jpg || len == 0 || len > FEED_JPG_MAX) return;
    portENTER_CRITICAL(&s_feed_mux);
    bool take = s_want_frame && !s_has_frame && s_feed_jpg;
    if (take) {
        memcpy(s_feed_jpg, jpg, len);
        s_feed_len = len;
        s_feed_w = w;
        s_feed_h = h;
        s_has_frame = true;
        s_want_frame = false;
    }
    portEXIT_CRITICAL(&s_feed_mux);
}

/* 滑动窗口一致性校验 */
static void update_sliding_consensus(bool got, const TagObservation *o, uint32_t dt_ms) {
    portENTER_CRITICAL(&s_res_mux);
    s_last_dt_ms = dt_ms;
    if (!got) {
        s_win_count = 0;
        s_consensus_ok = false;
        strcpy(s_consensus_reason, "no_tag");
        portEXIT_CRITICAL(&s_res_mux);
        return;
    }
    if (s_win_count < ALIGN_FRAMES) {
        s_win_obs[s_win_count++] = *o;
    } else {
        for (int i = 1; i < ALIGN_FRAMES; i++) s_win_obs[i - 1] = s_win_obs[i];
        s_win_obs[ALIGN_FRAMES - 1] = *o;
    }
    if (s_win_count < ALIGN_FRAMES) {
        s_consensus_ok = false;
        strcpy(s_consensus_reason, "tag_unstable");
        portEXIT_CRITICAL(&s_res_mux);
        return;
    }

    float th_min = 1e9f, th_max = -1e9f;
    float u_min = 1e9f, u_max = -1e9f, v_min = 1e9f, v_max = -1e9f;
    float sin_sum = 0, cos_sum = 0;
    for (int k = 0; k < ALIGN_FRAMES; k++) {
        if (s_win_obs[k].theta < th_min) th_min = s_win_obs[k].theta;
        if (s_win_obs[k].theta > th_max) th_max = s_win_obs[k].theta;
        if (s_win_obs[k].u < u_min) u_min = s_win_obs[k].u;
        if (s_win_obs[k].u > u_max) u_max = s_win_obs[k].u;
        if (s_win_obs[k].v < v_min) v_min = s_win_obs[k].v;
        if (s_win_obs[k].v > v_max) v_max = s_win_obs[k].v;
        sin_sum += sinf(s_win_obs[k].theta);
        cos_sum += cosf(s_win_obs[k].theta);
    }
    if ((th_max - th_min) * 180.0f / (float)M_PI > ALIGN_THETA_SPREAD ||
        (u_max - u_min) > ALIGN_UV_SPREAD || (v_max - v_min) > ALIGN_UV_SPREAD) {
        s_consensus_ok = false;
        strcpy(s_consensus_reason, "jitter");
        portEXIT_CRITICAL(&s_res_mux);
        return;
    }

    TagObservation avg;
    memset(&avg, 0, sizeof(avg));
    avg.ok = true;
    avg.id = s_win_obs[0].id;
    avg.hamming = s_win_obs[0].hamming;
    for (int k = 0; k < ALIGN_FRAMES; k++) {
        avg.u        += s_win_obs[k].u / ALIGN_FRAMES;
        avg.v        += s_win_obs[k].v / ALIGN_FRAMES;
        avg.edge_a   += s_win_obs[k].edge_a / ALIGN_FRAMES;
        avg.edge_p   += s_win_obs[k].edge_p / ALIGN_FRAMES;
        avg.margin   += s_win_obs[k].margin / ALIGN_FRAMES;
        avg.contrast += s_win_obs[k].contrast / ALIGN_FRAMES;
    }
    memcpy(avg.corners, s_win_obs[ALIGN_FRAMES - 1].corners, sizeof(avg.corners));
    avg.theta = atan2f(sin_sum, cos_sum);

    s_consensus_obs = avg;
    s_consensus_ok = true;
    strcpy(s_consensus_reason, "ok");
    portEXIT_CRITICAL(&s_res_mux);
}

/* 后台检测任务: Core 0, 优先级 2 (低于 httpd 的 5), 绝不阻塞 :81 推流与 :80 */
static void align_worker_task(void *arg) {
    (void)arg;
    s_feed_jpg = (uint8_t *)heap_caps_malloc(FEED_JPG_MAX, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_feed_jpg) s_feed_jpg = (uint8_t *)malloc(FEED_JPG_MAX);
    if (!s_feed_jpg) Stream_Log("align_wk: feed buf alloc FAIL");
    else Stream_Log("align_wk: feed buf ok");
    if (!measure_bufs_init()) Stream_Log("align_wk: decode bufs alloc FAIL");
    s_worker_alive = true;
    Stream_Log("align_wk: started");

    for (;;) {
        uint32_t now = millis();
        if ((int32_t)(s_detect_active_until - now) <= 0) {
            if (Cam_InDetectMode()) Cam_ExitDetectMode();
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        if (!Cam_InDetectMode()) Cam_EnterDetectMode();

        /* 优先向 :81/stream 索要旁路帧; 若 :81 未活跃 (>250ms 无投喂) 则自行取帧 */
        portENTER_CRITICAL(&s_feed_mux);
        s_has_frame = false;
        s_want_frame = true;
        portEXIT_CRITICAL(&s_feed_mux);

        uint32_t w0 = millis();
        while ((millis() - w0) < 80) {
            portENTER_CRITICAL(&s_feed_mux);
            bool has = s_has_frame;
            portEXIT_CRITICAL(&s_feed_mux);
            if (has) break;
            if ((millis() - s_last_stream_feed_ms) > 250) break;
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        size_t frame_len = 0;
        int frame_w = 0, frame_h = 0;
        portENTER_CRITICAL(&s_feed_mux);
        bool has = s_has_frame;
        s_want_frame = false;
        if (has) { frame_len = s_feed_len; frame_w = s_feed_w; frame_h = s_feed_h; }
        portEXIT_CRITICAL(&s_feed_mux);

        if (!has) {
            /* 回退: :81/stream 没在跑, worker 自己取帧 (此时无推流竞争) */
            camera_fb_t *fb = esp_camera_fb_get();
            if (fb) {
                if (fb->format == PIXFORMAT_JPEG && fb->len <= FEED_JPG_MAX && s_feed_jpg) {
                    portENTER_CRITICAL(&s_feed_mux);
                    memcpy(s_feed_jpg, fb->buf, fb->len);
                    s_feed_len = fb->len;
                    s_feed_w = fb->width;
                    s_feed_h = fb->height;
                    portEXIT_CRITICAL(&s_feed_mux);
                    frame_len = fb->len;
                    frame_w = fb->width;
                    frame_h = fb->height;
                    has = true;
                }
                esp_camera_fb_return(fb);
            }
        }

        if (!has) {
            s_diag_nofeed++;
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        s_diag_frames++;
        s_diag_last_w = frame_w;
        s_diag_last_h = frame_h;

        uint32_t t0 = millis();
        TagObservation o;
        TagDiag d;
        memset(&o, 0, sizeof(o));
        memset(&d, 0, sizeof(d));
        bool got = false;
        if (g_measure_lock && xSemaphoreTake(g_measure_lock, pdMS_TO_TICKS(500)) == pdTRUE) {
            const uint8_t *gray = NULL;
            int gw = 0, gh = 0;
            if (AlignDecodeJpegToGray(s_feed_jpg, frame_len, frame_w, frame_h, &gray, &gw, &gh)) {
                got = TagDetector_DetectEx(gray, gw, gh, gw, &o, &d);
                g_last_diag = d;
                if (got) {
                    s_diag_detect_ok++;
                    portENTER_CRITICAL(&s_res_mux);
                    s_last_good = o;
                    s_last_good_ok = true;
                    portEXIT_CRITICAL(&s_res_mux);
                } else {
                    s_diag_detect_fail++;
                }
            } else {
                s_diag_decode_fail++;
            }
            xSemaphoreGive(g_measure_lock);
        }
        uint32_t dt = millis() - t0;
        update_sliding_consensus(got, &o, dt);

        /* 主动让出 CPU, 保证 WiFi 栈、:81 推流与 loop() UDP 发现不被饿死 */
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

/* ---- GET /align/get (非阻塞读缓存, <1ms 返回) ---- */
static esp_err_t handle_align_get(httpd_req_t *req) {
    char buf[384];
    s_detect_active_until = millis() + 3000;   /* 续租 3s, 让 worker 继续检测 */

    const AlignRefPose *ref = AlignMeasure_GetRef();
    /* 无参考位时不直接报 no_ref: 仍返回原始观测 (u/v/theta/edge),
     * 让页面不点"记参考位"也能看到实时检测数据。 */
    if (!ref->valid) {
        TagObservation o;
        bool has;
        uint32_t dt;
        portENTER_CRITICAL(&s_res_mux);
        has = s_last_good_ok;
        o = s_last_good;
        dt = s_last_dt_ms;
        portEXIT_CRITICAL(&s_res_mux);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        if (!has) {
            snprintf(g_last_reason, sizeof(g_last_reason), "no_tag");
            return httpd_resp_send(req, "{\"ok\":false,\"reason\":\"no_tag\"}", HTTPD_RESP_USE_STRLEN);
        }
        snprintf(g_last_reason, sizeof(g_last_reason), "no_ref");
        int n = snprintf(buf, sizeof(buf),
            "{\"ok\":true,\"no_ref\":true,\"dt_ms\":%u,\"id\":%d,\"hamming\":%d,"
            "\"u\":%.1f,\"v\":%.1f,\"theta_deg\":%.2f,\"edge_px\":%.1f}",
            (unsigned)dt, o.id, o.hamming, (double)o.u, (double)o.v,
            (double)(o.theta * 180.0f / (float)M_PI), (double)o.edge_a);
        return httpd_resp_send(req, buf, n);
    }

    TagObservation avg;
    bool got;
    char reason[32];
    uint32_t dt;
    portENTER_CRITICAL(&s_res_mux);
    got = s_consensus_ok;
    avg = s_consensus_obs;
    strncpy(reason, s_consensus_reason, sizeof(reason) - 1);
    reason[sizeof(reason) - 1] = '\0';
    dt = s_last_dt_ms;
    portEXIT_CRITICAL(&s_res_mux);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    if (!got) {
        snprintf(g_last_reason, sizeof(g_last_reason), "%s", reason);
        int n = snprintf(buf, sizeof(buf),
                         "{\"ok\":false,\"reason\":\"%s\",\"dt_ms\":%u}", reason, (unsigned)dt);
        return httpd_resp_send(req, buf, n);
    }

    Deviation dev;
    if (!AlignMeasure_Compute(&avg, &dev)) {
        snprintf(g_last_reason, sizeof(g_last_reason), "compute_failed");
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

    /* 激活后台检测, 轮询等待最多 800ms 拿到稳定共识 */
    s_detect_active_until = millis() + 3000;
    uint32_t w0 = millis();
    bool got = false;
    TagObservation avg;
    memset(&avg, 0, sizeof(avg));
    char reason[32] = "no_tag";

    /* 记参考位是用户主动触发的一次性操作, 不该被 3 帧抖动卡死:
     * 等 3s 拿稳定共识; 超时但已有单帧有效观测时, 直接用它记参考位。 */
    TagObservation single;
    bool has_single = false;
    memset(&single, 0, sizeof(single));
    while ((millis() - w0) < 3000) {
        portENTER_CRITICAL(&s_res_mux);
        got = s_consensus_ok;
        avg = s_consensus_obs;
        strncpy(reason, s_consensus_reason, sizeof(reason) - 1);
        reason[sizeof(reason) - 1] = '\0';
        if (!got && s_last_good_ok) { single = s_last_good; has_single = true; }
        portEXIT_CRITICAL(&s_res_mux);
        if (got) break;
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    if (!got && has_single) {
        avg = single;
        got = true;
        strcpy(reason, "single");
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    if (!got) {
        Stream_Log("ref FAIL %s", reason);
        char buf[128];
        int n = snprintf(buf, sizeof(buf), "{\"ok\":false,\"reason\":\"%s\"}", reason);
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
    Stream_Log("ref saved u=%.1f v=%.1f theta=%.1f edge=%.1fpx",
               (double)avg.u, (double)avg.v,
               (double)(avg.theta * 180.0f / (float)M_PI), (double)avg.edge_a);
    return httpd_resp_send(req, buf, n);
}

/* ---- GET /align/status ---- */
static esp_err_t handle_align_status(httpd_req_t *req) {
    char buf[768];
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
    int k = snprintf(buf + n, sizeof(buf) - n,
        "worker: alive=%d lease_ms=%d frames=%u nofeed=%u dec_fail=%u det_ok=%u det_fail=%u\n"
        "last: feed=%dx%d dec_ow=%d oh=%d derr=%d\n",
        s_worker_alive ? 1 : 0,
        (int)(s_detect_active_until - millis()),
        (unsigned)s_diag_frames, (unsigned)s_diag_nofeed,
        (unsigned)s_diag_decode_fail, (unsigned)s_diag_detect_ok,
        (unsigned)s_diag_detect_fail,
        s_diag_last_w, s_diag_last_h, s_diag_last_ow, s_diag_last_oh,
        s_diag_last_decode_err);
    if (k > 0) n += k;
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, buf, n);
}

void AlignServer_Register(httpd_handle_t control_httpd) {
    if (!g_measure_lock) g_measure_lock = xSemaphoreCreateMutex();
    /* 后台检测任务: 栈 8KB, 优先级 2 (低于 httpd 的 5), 绑定 Core 0 */
    xTaskCreatePinnedToCore(align_worker_task, "align_wk", 8192, NULL, 2, NULL, 0);

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
    Serial.println("[ALIGN] API ready: /align/ref /align/get /align/status (async worker)");
}
