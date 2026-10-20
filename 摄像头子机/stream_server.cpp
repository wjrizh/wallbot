/* stream_server.cpp - MJPEG 流服务 + 截图/帧率/诊断接口 */
#include <Arduino.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "config.h"
#include "cam_module.h"
#include "align_server.h"
#include "align_view.h"
#include "log_page.h"
#include "stream_server.h"

static const char *PART_BOUNDARY = "123456789000000000000987654321";
static const char *STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" "123456789000000000000987654321";
static const char *STREAM_BOUNDARY =
    "\r\n--" "123456789000000000000987654321" "\r\n";
static const char *STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static httpd_handle_t s_stream_httpd = NULL;
static httpd_handle_t s_capture_httpd = NULL;
static httpd_handle_t s_view_httpd = NULL;

static volatile float g_fps = 0.0f;
static volatile uint32_t g_softenc_fail = 0;   /* 检测模式下软编码失败帧计数 */

/* ===== 日志环形缓冲 (供 App 通过 /log 拉取, 替代串口) ===== */
#define LOG_LINES      24
#define LOG_LINE_LEN   96
static char     g_log[LOG_LINES][LOG_LINE_LEN];
static uint32_t g_log_seq[LOG_LINES];
static volatile uint32_t g_log_head = 0;   /* 下一个写入槽 */
static volatile uint32_t g_log_total = 0;  /* 累计写入条数 */

void Stream_Log(const char *fmt, ...) {
    char line[LOG_LINE_LEN];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    uint32_t slot = g_log_head % LOG_LINES;
    uint32_t ms = millis();
    snprintf(g_log[slot], LOG_LINE_LEN, "%7u %s", (unsigned)ms, line);
    g_log_seq[slot] = ++g_log_total;
    g_log_head = (g_log_head + 1) % LOG_LINES;
}

/* GET /log -> 按时间顺序输出最近的日志行 */
static esp_err_t log_handler(httpd_req_t *req) {
    static char out[LOG_LINES * LOG_LINE_LEN + 32];
    size_t n = 0;
    uint32_t total = g_log_total;
    uint32_t first = (total > LOG_LINES) ? (total - LOG_LINES + 1) : 1;
    for (uint32_t seq = first; seq <= total; seq++) {
        uint32_t slot = (seq - 1) % LOG_LINES;
        if (g_log_seq[slot] != seq) continue;
        int k = snprintf(out + n, sizeof(out) - n, "%s\n", g_log[slot]);
        if (k <= 0) break;
        n += (size_t)k;
        if (n + LOG_LINE_LEN + 2 >= sizeof(out)) break;
    }
    if (n == 0) n = (size_t)snprintf(out, sizeof(out), "(no log)\n");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, out, n);
}

static esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    char part_buf[64];

    res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "30");

    uint32_t fps_frames = 0;
    uint32_t fps_t0 = millis();

    /* 发送缓冲预分配一次复用: 避免每帧 malloc/free 造成 PSRAM 碎片 */
    size_t cap = 128 * 1024;
    char *frame_buf = (char *)malloc(cap);
    if (!frame_buf) {
        frame_buf = (char *)heap_caps_malloc(cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!frame_buf) return ESP_FAIL;
    }

    while (true) {
        /* 分辨率固定 VGA, 本流全程出帧; 检测不再切 framesize, 画面尺寸恒定。 */
        fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        /* 顺手把当前 JPEG 帧投喂给后台检测任务 (仅当它正在等帧时才 memcpy, <0.2ms) */
        if (fb->format == PIXFORMAT_JPEG) {
            AlignServer_FeedFrame(fb->buf, fb->len, fb->width, fb->height);
        }

        fps_frames++;
        uint32_t now = millis();
        if (now - fps_t0 >= 1000) {
            g_fps = fps_frames * 1000.0f / (now - fps_t0);
            fps_frames = 0;
            fps_t0 = now;
        }

        uint8_t *jpg_buf = NULL;
        size_t jpg_len = 0;
        bool soft = false;
        if (fb->format == PIXFORMAT_JPEG) {
            jpg_buf = fb->buf;
            jpg_len = fb->len;
        } else {
            /* 灰度模式防御分支 (固定 JPEG 后正常不会走到): 软编码失败只丢这一帧,
             * 不能断开流, 否则 /align/* 期间画面直接黑屏 */
            if (!frame2jpg(fb, 12, &jpg_buf, &jpg_len)) {
                g_softenc_fail++;
                esp_camera_fb_return(fb);
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            soft = true;
        }

        size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, (unsigned)jpg_len);
        /* boundary + part header + jpeg body 合并为单次发送, 避免 TCP 拆段抖动 */
        size_t blen = strlen(STREAM_BOUNDARY);
        size_t total = blen + hlen + jpg_len;
        if (total > cap) {
            /* 帧异常大: 扩容一次, 仍失败则丢弃该帧 */
            char *nb = (char *)realloc(frame_buf, total);
            if (!nb) {
                if (soft && jpg_buf) free(jpg_buf);
                esp_camera_fb_return(fb);
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;
            }
            frame_buf = nb;
            cap = total;
        }
        memcpy(frame_buf, STREAM_BOUNDARY, blen);
        memcpy(frame_buf + blen, part_buf, hlen);
        memcpy(frame_buf + blen + hlen, jpg_buf, jpg_len);

        /* 关键: 已拷贝进 frame_buf, 立即归还相机 fb, 再做阻塞的网络发送。
         * fb 占用时间从数十 ms 降到 <0.2ms, DMA 缓冲立刻可用于采集下一帧。 */
        if (soft && jpg_buf) free(jpg_buf);
        esp_camera_fb_return(fb);
        fb = NULL;

        res = httpd_resp_send_chunk(req, frame_buf, total);
        if (res != ESP_OK) break;

        /* 让出 2ms 给低优先级任务 (loop 中的 UDP 发现) 与 IDLE 看门狗 */
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (frame_buf) free(frame_buf);
    return res;
}

static esp_err_t root_handler(httpd_req_t *req) {
    char buf[384];
    camera_fb_t *fb = esp_camera_fb_get();
    int fb_ok = fb ? 1 : 0;
    size_t fb_len = fb ? fb->len : 0;
    int fb_fmt = fb ? (int)fb->format : -1;
    if (fb) esp_camera_fb_return(fb);
    int n = snprintf(buf, sizeof(buf),
        "cam_init=%d cam_err=0x%x jpeg_ok=%d pid=0x%x err2=0x%x init_ms=%u "
        "detect_mode=%d fb_get=%d fb_len=%u fb_fmt=%d psram=%u heap=%u\n",
        Cam_IsOk() ? 1 : 0, Cam_Err(), Cam_JpegOk() ? 1 : 0,
        Cam_SensorPid(), Cam_Err2(), (unsigned)Cam_InitMs(),
        Cam_InDetectMode() ? 1 : 0,
        fb_ok, (unsigned)fb_len, fb_fmt,
        (unsigned)ESP.getPsramSize(), (unsigned)ESP.getFreeHeap());
    n += snprintf(buf + n, sizeof(buf) - n, "softenc_fail=%u\n", (unsigned)g_softenc_fail);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, buf, n);
}

static esp_err_t fps_handler(httpd_req_t *req) {
    char buf[32];
    int n = snprintf(buf, sizeof(buf), "%.1f\n", (double)g_fps);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, buf, n);
}

static esp_err_t jpg_handler(httpd_req_t *req) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req, "fb_get failed", HTTPD_RESP_USE_STRLEN);
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    esp_err_t res;
    if (fb->format == PIXFORMAT_JPEG) {
        res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    } else {
        uint8_t *jpg_buf = NULL;
        size_t jpg_len = 0;
        if (frame2jpg(fb, 12, &jpg_buf, &jpg_len)) {
            res = httpd_resp_send(req, (const char *)jpg_buf, jpg_len);
            free(jpg_buf);
        } else {
            res = httpd_resp_send(req, "encode failed", HTTPD_RESP_USE_STRLEN);
        }
    }
    esp_camera_fb_return(fb);
    return res;
}

bool StreamServer_Start(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.stack_size = 16384;   /* /align/get 同步做检测, 局部数组较大, 默认4K不够 */
    /* 默认 max_uri_handlers=8, :80 实际注册 9 个(不含 /view),
     * 不调大会让后注册的路由静默注册失败(返回 ESP_ERR_HTTPD_HANDLERS_FULL 但没人检查) */
    config.max_uri_handlers = 16;

    httpd_uri_t stream_uri = {.uri = "/stream", .method = HTTP_GET,
                              .handler = stream_handler, .user_ctx = NULL};
    httpd_uri_t jpg_uri = {.uri = "/jpg", .method = HTTP_GET,
                           .handler = jpg_handler, .user_ctx = NULL};
    httpd_uri_t diag_uri = {.uri = "/diag", .method = HTTP_GET,
                            .handler = root_handler, .user_ctx = NULL};
    httpd_uri_t fps_uri = {.uri = "/fps", .method = HTTP_GET,
                           .handler = fps_handler, .user_ctx = NULL};
    httpd_uri_t log_uri = {.uri = "/log", .method = HTTP_GET,
                           .handler = log_handler, .user_ctx = NULL};

    // Control server on :80
    if (httpd_start(&s_capture_httpd, &config) != ESP_OK) return false;
    httpd_register_uri_handler(s_capture_httpd, &jpg_uri);
    httpd_register_uri_handler(s_capture_httpd, &diag_uri);
    httpd_register_uri_handler(s_capture_httpd, &fps_uri);
    httpd_register_uri_handler(s_capture_httpd, &log_uri);

    // 视觉纠偏 API (/align/*) 挂在 :80 控制服务上
    AlignServer_Register(s_capture_httpd);
    // 内置网页日志页 (:80/logs) 与状态 (:80/status)
    LogPage_Register(s_capture_httpd);
    /* 注意: /view 是 while(true) 推流 handler, 不能挂在 :80。
     * esp_http_server 每个实例是单线程, 推流会永久占用该任务,
     * 导致同实例上的 /align/* /log /logs 全部无响应。放到 :81 流实例上。 */

    // Stream server on :81 (专用流媒体实例, 与控制接口物理隔离)
    config.server_port = 81;
    config.ctrl_port = 32769;
    if (httpd_start(&s_stream_httpd, &config) != ESP_OK) return false;
    httpd_register_uri_handler(s_stream_httpd, &stream_uri);
    /* 注意: /view 也是 while(true) 推流 handler。esp_http_server 每个实例单线程,
     * 与 /stream 同挂 :81 会互相阻塞 (一个在推流时另一个永远排不上队),
     * 因此 /view 必须用独立实例。 */
    config.server_port = 82;
    config.ctrl_port = 32770;
    if (httpd_start(&s_view_httpd, &config) == ESP_OK) {
        AlignView_Register(s_view_httpd);
    } else {
        Stream_Log(":82 start failed, /view disabled");
    }
    return true;
}
