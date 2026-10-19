/* stream_server.cpp - MJPEG 流服务 + 截图/帧率/诊断接口 */
#include <Arduino.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_http_server.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "config.h"
#include "cam_module.h"
#include "align_server.h"
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

static volatile float g_fps = 0.0f;

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

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            // /align 切检测模式瞬间会短暂取不到帧: 重试 2s 而不是直接断流
            uint32_t wait0 = millis();
            bool recovered = false;
            while (millis() - wait0 < 2000) {
                delay(20);
                fb = esp_camera_fb_get();
                if (fb) { recovered = true; break; }
            }
            if (!recovered) { res = ESP_FAIL; break; }
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
        if (fb->format == PIXFORMAT_JPEG) {
            jpg_buf = fb->buf;
            jpg_len = fb->len;
        } else {
            // 灰度检测模式期间: 软编码; 编码失败跳帧续传, 不断流
            if (!frame2jpg(fb, 12, &jpg_buf, &jpg_len)) {
                esp_camera_fb_return(fb);
                delay(20);
                continue;
            }
        }

        size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, (unsigned)jpg_len);
        // boundary + part header + jpeg body 合并为单次发送, 避免 TCP 拆段抖动
        size_t blen = strlen(STREAM_BOUNDARY);
        size_t total = blen + hlen + jpg_len;
        char *frame_buf = (char *)malloc(total);
        if (!frame_buf) {
            if (fb->format != PIXFORMAT_JPEG && jpg_buf) free(jpg_buf);
            esp_camera_fb_return(fb);
            res = ESP_FAIL;
            break;
        }
        memcpy(frame_buf, STREAM_BOUNDARY, blen);
        memcpy(frame_buf + blen, part_buf, hlen);
        memcpy(frame_buf + blen + hlen, jpg_buf, jpg_len);
        res = httpd_resp_send_chunk(req, frame_buf, total);
        free(frame_buf);

        if (fb->format != PIXFORMAT_JPEG && jpg_buf) free(jpg_buf);
        esp_camera_fb_return(fb);
        if (res != ESP_OK) break;
    }
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

    httpd_uri_t stream_uri = {.uri = "/stream", .method = HTTP_GET,
                              .handler = stream_handler, .user_ctx = NULL};
    httpd_uri_t jpg_uri = {.uri = "/jpg", .method = HTTP_GET,
                           .handler = jpg_handler, .user_ctx = NULL};
    httpd_uri_t root_uri = {.uri = "/", .method = HTTP_GET,
                            .handler = root_handler, .user_ctx = NULL};
    httpd_uri_t fps_uri = {.uri = "/fps", .method = HTTP_GET,
                           .handler = fps_handler, .user_ctx = NULL};

    // Control server on :80
    if (httpd_start(&s_capture_httpd, &config) != ESP_OK) return false;
    httpd_register_uri_handler(s_capture_httpd, &jpg_uri);
    httpd_register_uri_handler(s_capture_httpd, &root_uri);
    httpd_register_uri_handler(s_capture_httpd, &fps_uri);

    // 视觉纠偏 API (/align/*) 挂在 :80 控制服务上
    AlignServer_Register(s_capture_httpd);

    // Stream server on :81
    config.server_port = 81;
    config.ctrl_port = 32769;
    if (httpd_start(&s_stream_httpd, &config) != ESP_OK) return false;
    httpd_register_uri_handler(s_stream_httpd, &stream_uri);
    return true;
}
