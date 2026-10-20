/* ==================== config.h ==================== */
/* config.h - 摄像头子机全部可调参数 (集中一处, 上机调参只改这里)
 *
 * 模块划分:
 *   摄像头子机.ino  主入口: 初始化/看门狗/模块编排
 *   cam_module      摄像头初始化 + 测检模式切换 + 诊断计数
 *   discovery       UDP 自动发现 (端口 48899)
 *   stream_server   MJPEG 流 (:81/stream) + :80 的 /jpg /fps / 诊断
 *   tag_detector    板端 AprilTag 36h11 检测 (灰度帧 -> 角点/ID)
 *   align_measure   视觉纠偏测量 (参考位 + 偏差解算, 移植 tag_utils.py)
 *   align_server    纠偏 HTTP API (/align/*), 母机/App 闭环调用的接口
 */
#ifndef WALLBOT_CAM_CONFIG_H
#define WALLBOT_CAM_CONFIG_H

#include "esp_camera.h"   /* FRAMESIZE_*/

/* ---- WiFi (与母机/平板同一热点) ---- */
#define WIFI_SSID     "robot"
#define WIFI_PASS     "12345678"
/* NOTE: DHCP 获取与平板热点同网段地址, 具体 IP 由 App 通过 UDP 发现获得 */

/* ---- UDP 自动发现 ---- */
#define DISCOVERY_PORT        48899
#define DISCOVERY_REQ         "WALLBOT_DISCOVER"
#define DEVICE_ROLE           "camchild"

/* ---- 摄像头 ---- */
/* 流媒体分辨率/画质 (MJPEG 硬编码) */
#define STREAM_FRAMESIZE      FRAMESIZE_HD
#define STREAM_JPEG_QUALITY   11
/* ---- 检测分辨率 (检测模式只切分辨率, 保持 JPEG; 板上解码为灰度后检测) ----
 * 注意: esp32-camera 的 DMA 解析模式在初始化时按 JPEG 定死,
 *       运行时刻切换 pixformat 会产生坏帧(检测必 no_tag + 画面花屏),
 *       所以全程保持 JPEG, 只切 framesize。 */
#define ALIGN_DETECT_FRAMESIZE  FRAMESIZE_QVGA
#define ALIGN_DETECT_W          320   /* FRAMESIZE_QVGA 对应的宽 */
#define ALIGN_DETECT_H          240   /* FRAMESIZE_QVGA 对应的高 */

/* ---- AprilTag / 视觉纠偏几何参数 (与 code/tag_utils.py 保持一致) ---- */
#define TAG_REAL_MM       50.0f  /* 标签黑框真实边长(mm), 打印后实测填入 */
#define R_TAG_MM          60.0f  /* 标签黑框中心平面到管道轴线的垂直距离(mm), 贴好后必须实测 */
#define PIPE_RADIUS_MM    60.0f  /* 管道内半径(mm), 仅用于把 phi 换算成沿管壁弧长 */

#define TAG_ID            0      /* 使用的标签 ID */

/* ---- 检测器参数 ---- */
#define DET_EDGE_PX_MIN   24     /* 标签黑框边长像素下限 (QVGA 下的预期范围) */
#define DET_EDGE_PX_MAX   200    /* 标签黑框边长像素上限 */
#define DET_MAX_HAMMING   1      /* 解码允许的最大汉明距离 */
#define DET_CONTRAST_MIN  25.0f  /* 黑白电平最小对比度 (抗全黑/全白误检) */
#define DET_ADAPT_WIN     45     /* 自适应阈值窗口(奇数, 像素)。
                                  * 须大于最大标签的黑框厚度(edge_max/8*1=25px):
                                  * 窗口大->厚黑框核心也判暗(或成闭合厚环带), 不断裂 */
#define DET_ADAPT_BIAS    25     /* 判黑偏置: gray < mean - bias 视为暗。
                                  * 取 ~2.5x 噪声sigma: 噪声误判率<1%, 背景不絮凝;
                                  * 黑框会变细环带(连续等值线,不断裂), 由填洞补实心 */

/* ---- 纠偏闭环参数 (执行文档 6.3 节: ANGLE_TOL=4°, 3 帧一致才采信) ---- */
#define ALIGN_FRAMES        3      /* 每次测量连拍帧数 (识别要连续一致) */
#define ALIGN_THETA_SPREAD  1.5f   /* 多帧 theta 一致性阈值 (deg) */
#define ALIGN_UV_SPREAD     3.0f   /* 多帧中心一致性阈值 (px) */
#define ALIGN_ANGLE_TOL     4.0f   /* 夹取判据: |phi|,|psi| <= 4° (夹爪 5° 留 1° 裕量) */
#define ALIGN_AXIAL_TOL_MM  20.0f  /* 轴向进/退偏差容差(mm), 夹爪开口容差相关 */

/* ---- NVS 存储命名空间 (参考位掉电保存) ---- */
#define ALIGN_NVS_NS        "wallbot-cam"

#endif /* WALLBOT_CAM_CONFIG_H */

/* ==================== cam_module.h ==================== */
/* cam_module.h - 摄像头初始化 / 检测模式切换 / 诊断信息 */
#ifndef WALLBOT_CAM_MODULE_H
#define WALLBOT_CAM_MODULE_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_camera.h"

void        Cam_Init(void);          /* 初始化相机, 失败不阻塞启动 */
bool        Cam_IsOk(void);
int         Cam_Err(void);           /* esp_camera_init 的返回码 */
int         Cam_Err2(void);          /* 二次探测错误码 */
bool        Cam_JpegOk(void);
int         Cam_SensorPid(void);
uint32_t    Cam_InitMs(void);        /* 初始化耗时 */

/* 检测模式: 切到灰度低分辨率跑 AprilTag 检测, 用完必须 Exit 配对调用。
 * 期间 /stream 仍可用 (灰度帧走软编码), 但帧率下降, 属正常现象。 */
bool        Cam_EnterDetectMode(void);
void        Cam_ExitDetectMode(void);
bool        Cam_InDetectMode(void);

/* 抓一帧并丢弃 (模式切换后等 AEC/AGC 稳定用) */
void        Cam_FlushFrames(int n, int delay_ms);

#endif /* WALLBOT_CAM_MODULE_H */

/* ==================== cam_module.cpp ==================== */
/* cam_module.cpp - 摄像头初始化 / 检测模式切换 / 诊断信息 */
#include <Arduino.h>
#include "esp_camera.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "config.h"
#include "cam_module.h"
#include "stream_server.h"

/* ---- Camera pins: CAMERA_MODEL_ESP32S3_EYE ---- */
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   15
#define SIOD_GPIO_NUM   4
#define SIOC_GPIO_NUM   5
#define Y2_GPIO_NUM     11
#define Y3_GPIO_NUM     9
#define Y4_GPIO_NUM     8
#define Y5_GPIO_NUM     10
#define Y6_GPIO_NUM     12
#define Y7_GPIO_NUM     18
#define Y8_GPIO_NUM     17
#define Y9_GPIO_NUM     16
#define VSYNC_GPIO_NUM  6
#define HREF_GPIO_NUM   7
#define PCLK_GPIO_NUM   13

static bool    s_cam_ok = false;
static int     s_cam_err = 0;
static int     s_cam_err2 = 0;
static bool    s_jpeg_ok = false;
static int     s_sensor_pid = -1;
static uint32_t s_init_ms = 0;
static bool    s_detect_mode = false;
static int     s_detect_refs = 0;   /* 检测模式引用计数: /view 与 /align/* 可能同时占用 */
/* /view(:81) 与 /align/*(:80) 在不同 httpd 任务里并发调用 enter/exit,
 * 引用计数必须原子更新, 否则计数丢失会让分辨率卡在 QVGA。 */
static portMUX_TYPE s_mode_mux = portMUX_INITIALIZER_UNLOCKED;

bool Cam_IsOk(void)        { return s_cam_ok; }
int  Cam_Err(void)         { return s_cam_err; }
int  Cam_Err2(void)        { return s_cam_err2; }
bool Cam_JpegOk(void)      { return s_jpeg_ok; }
int  Cam_SensorPid(void)   { return s_sensor_pid; }
uint32_t Cam_InitMs(void)  { return s_init_ms; }
bool Cam_InDetectMode(void){ return s_detect_mode; }

void Cam_Init(void) {
    camera_config_t config = {};
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = STREAM_FRAMESIZE;
    config.jpeg_quality = STREAM_JPEG_QUALITY;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;

    uint32_t t0 = millis();
    esp_err_t err = esp_camera_init(&config);
    s_cam_err = (int)err;
    s_init_ms = millis() - t0;
    if (err != ESP_OK) return;
    s_cam_ok = true;

    sensor_t *s = esp_camera_sensor_get();
    s_sensor_pid = s ? s->id.PID : -1;
    if (s) {
        // OV3660/OV5640/OV2640 output is flipped vertically by default
        if (s->id.PID == OV3660_PID || s->id.PID == OV5640_PID || s->id.PID == OV2640_PID) {
            s->set_vflip(s, 1);
        }
        s->set_brightness(s, 0);
        s->set_contrast(s, 0);
        s->set_saturation(s, 0);
    }

    // 试抓一帧: 确认 JPEG 帧可用
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
        s_jpeg_ok = (fb->format == PIXFORMAT_JPEG && fb->len > 0);
        esp_camera_fb_return(fb);
    }
    // 复位一次错误码, 探测是否稳定
    camera_fb_t *fb2 = esp_camera_fb_get();
    if (fb2) esp_camera_fb_return(fb2);
    else s_cam_err2 = (int)ESP_FAIL;
}

/* 检测模式: 只切分辨率 (像素格式保持 JPEG 不变!)
 * 原因: esp32-camera 的 DMA 解析模式在初始化时按 JPEG 定死,
 *       运行时刻 set_pixformat 会导致驱动仍按 JPEG 解析灰度数据 -> 全是坏帧。
 * 引用计数: 多路(/view 与 /align/*)同时请求只切一次, 全部释放才切回。 */
bool Cam_EnterDetectMode(void) {
    if (!s_cam_ok) return false;
    bool need_switch = false;
    portENTER_CRITICAL(&s_mode_mux);
    if (s_detect_mode) {
        s_detect_refs++;
    } else {
        s_detect_mode = true;
        s_detect_refs = 1;
        need_switch = true;
    }
    portEXIT_CRITICAL(&s_mode_mux);
    if (!need_switch) return true;

    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        portENTER_CRITICAL(&s_mode_mux);
        s_detect_mode = false;
        s_detect_refs = 0;
        portEXIT_CRITICAL(&s_mode_mux);
        return false;
    }
    s->set_framesize(s, ALIGN_DETECT_FRAMESIZE);
    Cam_FlushFrames(2, 10);   /* 只冲刷 2 帧: 原 6x30ms 会在每次测量时阻塞 180ms */
    Stream_Log("detect mode ON (gray qvga)");
    return true;
}

void Cam_ExitDetectMode(void) {
    bool need_switch = false;
    portENTER_CRITICAL(&s_mode_mux);
    if (s_detect_mode) {
        if (s_detect_refs > 0) s_detect_refs--;
        if (s_detect_refs == 0) {
            s_detect_mode = false;
            need_switch = true;
        }
    }
    portEXIT_CRITICAL(&s_mode_mux);
    if (!need_switch) return;

    sensor_t *s = esp_camera_sensor_get();
    if (s) s->set_framesize(s, STREAM_FRAMESIZE);
    Cam_FlushFrames(1, 10);
    Stream_Log("detect mode OFF (jpeg hd)");
}

void Cam_FlushFrames(int n, int delay_ms) {
    for (int i = 0; i < n; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) esp_camera_fb_return(fb);
        if (delay_ms > 0) delay(delay_ms);
    }
}

/* ==================== stream_server.h ==================== */
/* stream_server.h - MJPEG 流服务 + 截图/帧率/诊断接口
 *
 * :81/stream  multipart/x-mixed-replace MJPEG 实时流
 * :80/jpg     单帧 JPEG
 * :80/fps     当前流帧率
 * :80/        诊断信息 (cam_init/pid/psram/heap)
 */
#ifndef WALLBOT_STREAM_SERVER_H
#define WALLBOT_STREAM_SERVER_H

#include <stdbool.h>

bool StreamServer_Start(void);   /* 启动 :80 控制服务 + :81 流服务 */

/* 写入日志环形缓冲 (App 通过 GET /log 拉取, 替代串口) */
void Stream_Log(const char *fmt, ...);

#endif /* WALLBOT_STREAM_SERVER_H */

/* ==================== stream_server.cpp ==================== */
/* stream_server.cpp - MJPEG 流服务 + 截图/帧率/诊断接口 */
#include <Arduino.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_http_server.h"
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

    while (true) {
        /* 检测模式只切分辨率(保持 JPEG), 本流照常出帧: 画面短暂变小, 不会断流。 */
        fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
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
            // 灰度检测模式期间流被拉低: 走软编码
            // 软编码失败(PSRAM 紧张)只丢这一帧, 不能断开流: 否则 /align/* 期间画面直接黑屏
            if (!frame2jpg(fb, 12, &jpg_buf, &jpg_len)) {
                g_softenc_fail++;
                esp_camera_fb_return(fb);
                vTaskDelay(pdMS_TO_TICKS(10));
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
    httpd_uri_t root_uri = {.uri = "/", .method = HTTP_GET,
                            .handler = root_handler, .user_ctx = NULL};
    httpd_uri_t fps_uri = {.uri = "/fps", .method = HTTP_GET,
                           .handler = fps_handler, .user_ctx = NULL};
    httpd_uri_t log_uri = {.uri = "/log", .method = HTTP_GET,
                           .handler = log_handler, .user_ctx = NULL};

    // Control server on :80
    if (httpd_start(&s_capture_httpd, &config) != ESP_OK) return false;
    httpd_register_uri_handler(s_capture_httpd, &jpg_uri);
    httpd_register_uri_handler(s_capture_httpd, &root_uri);
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
    // 检测可视化流 (:81/view): 灰度帧 + 候选框 + 诊断文字
    AlignView_Register(s_stream_httpd);
    return true;
}

/* ==================== align_server.h ==================== */
/* align_server.h - 视觉纠偏 HTTP API (挂在 :80 控制服务上)
 *
 * GET /align/ref          连拍投票检测, 将当前位姿记录为参考位 (放子机前、夹爪对中时调用)
 * GET /align/ref?clear=1  清除参考位
 * GET /align/get          测量当前偏差 -> JSON (闭环主接口: phi/psi/axial/arc/aligned)
 * GET /align/status       参考位 + 上次测量 + 配置概览 (纯文本)
 */
#ifndef WALLBOT_ALIGN_SERVER_H
#define WALLBOT_ALIGN_SERVER_H

#include "esp_http_server.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/* 检测互斥锁: TagDetector 内部使用全局静态工作区 g_sc, 不可并发。
 * /align/* (:80) 与 /view (:81) 在不同 httpd 任务上, 必须共用这把锁。 */
extern SemaphoreHandle_t g_measure_lock;

void AlignServer_Register(httpd_handle_t control_httpd);

#endif /* WALLBOT_ALIGN_SERVER_H */

/* ==================== align_server.cpp ==================== */
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
#include "img_converters.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
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

/* 模式切换 + 检测互斥: /align/get 与 /align/ref 串行,
 * 且 /view (:81) 也共用此锁 (TagDetector 内部全局工作区不可并发) */
SemaphoreHandle_t g_measure_lock = NULL;

/* 检测工作缓冲 (QVGA): JPEG 解码 -> RGB888 -> 灰度, 首次调用分配一次复用 */
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

/* JPEG 帧 -> 灰度 (检测用); 帧尺寸必须是 QVGA, 否则返回 false */
bool AlignDecodeJpegToGray(const uint8_t *jpg, size_t jlen, int fw, int fh,
                           const uint8_t **out_gray, int *out_w, int *out_h) {
    if (!jpg || jlen == 0) return false;
    if (fw != ALIGN_DETECT_W || fh != ALIGN_DETECT_H) return false;
    if (!measure_bufs_init()) return false;
    if (!fmt2rgb888(jpg, jlen, PIXFORMAT_JPEG, s_dec_rgb)) return false;
    const uint32_t npix = (uint32_t)ALIGN_DETECT_W * ALIGN_DETECT_H;
    for (uint32_t i = 0; i < npix; i++) {
        const uint8_t *px = &s_dec_rgb[i * 3];
        s_dec_gray[i] = (uint8_t)((px[0] * 77 + px[1] * 151 + px[2] * 28) >> 8);
    }
    *out_gray = s_dec_gray;
    *out_w = ALIGN_DETECT_W;
    *out_h = ALIGN_DETECT_H;
    return true;
}

/* 连拍 ALIGN_FRAMES 帧投票检测; 全部命中且一致 -> 返回平均观测
 * once=true: 只抓 1 帧直接采信 (调试/快速探测用, 不做一致性校验) */
static bool measure_consensus(TagObservation *avg, const char **reason, bool once) {
    if (!g_measure_lock) g_measure_lock = xSemaphoreCreateMutex();
    /* 锁等太久会把 :80 httpd 任务挂死: /view 正在检测时最多等 1.5s 就报 busy */
    if (g_measure_lock && xSemaphoreTake(g_measure_lock, pdMS_TO_TICKS(1500)) != pdTRUE) {
        *reason = "busy";
        return false;
    }
    if (!Cam_EnterDetectMode()) {
        if (g_measure_lock) xSemaphoreGive(g_measure_lock);
        *reason = "cam_failed";
        return false;
    }
    const int need = once ? 1 : ALIGN_FRAMES;
    /* 无 tag 时不要硬跑满 ALIGN_FRAMES*3 次: 每次都要整幅连通域+解码,
     * 无 tag 是常态, 跑满 9 次会让接口耗时数百 ms 且拖住 /view。 */
    const int max_try = once ? 2 : 4;
    TagObservation obs[ALIGN_FRAMES];
    int n_ok = 0;
    for (int k = 0; k < max_try && n_ok < need; k++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            uint32_t w0 = millis();
            while (!fb && millis() - w0 < 100) {
                vTaskDelay(pdMS_TO_TICKS(10));
                fb = esp_camera_fb_get();
            }
            if (!fb) continue;
        }
        TagObservation o;
        TagDiag d;
        bool got = false;
        /* 相机全程保持 JPEG; 检测前解码为灰度 (检测模式已切到 QVGA 分辨率) */
        const uint8_t *gray = NULL;
        int gw = 0, gh = 0;
        if (fb->format == PIXFORMAT_JPEG &&
            AlignDecodeJpegToGray(fb->buf, fb->len, fb->width, fb->height, &gray, &gw, &gh)) {
            got = TagDetector_DetectEx(gray, gw, gh, gw, &o, &d);
            g_last_diag = d;   /* 保留最后一帧诊断, 失败时用于日志 */
        }
        esp_camera_fb_return(fb);
        if (got) obs[n_ok++] = o;
    }
    Cam_ExitDetectMode();

    if (n_ok < need) {
        *reason = (n_ok == 0) ? "no_tag" : "tag_unstable";
        if (g_measure_lock) xSemaphoreGive(g_measure_lock);
        return false;
    }
    if (once) {
        *avg = obs[0];
        if (g_measure_lock) xSemaphoreGive(g_measure_lock);
        return true;
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
        if (g_measure_lock) xSemaphoreGive(g_measure_lock);
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
    if (g_measure_lock) xSemaphoreGive(g_measure_lock);
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

    char qbuf[32] = {0};
    httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf) - 1);
    bool once = (strstr(qbuf, "once=1") != NULL);

    TagObservation avg;
    const char *reason = "unknown";
    uint32_t t0 = millis();
    bool got = measure_consensus(&avg, &reason, once);
    uint32_t dt = millis() - t0;

    if (!got) {
        Stream_Log("get FAIL %s (%ums)", reason, (unsigned)dt);
        Stream_Log("  diag comp=%d cand=%d bbox=%dx%d fill=%d%% dropS=%d L=%d F=%d A=%d ham=%d try=%d st=%d",
                   g_last_diag.ncomps, g_last_diag.ncand,
                   g_last_diag.cand_bbox[2], g_last_diag.cand_bbox[3],
                   (int)(g_last_diag.cand_fill * 100),
                   g_last_diag.drop_small, g_last_diag.drop_large,
                   g_last_diag.drop_fill, g_last_diag.drop_aspect,
                   g_last_diag.best_hamming, g_last_diag.decode_tried, g_last_diag.stage);
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
    Stream_Log("get OK phi=%.1f psi=%.1f axial=%.1f (%ums)",
               (double)dev.phi_deg, (double)dev.psi_deg, (double)dev.axial_mm, (unsigned)dt);
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
    bool got = measure_consensus(&avg, &reason, false);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    if (!got) {
        Stream_Log("ref FAIL %s", reason);
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
    Stream_Log("ref saved u=%.1f v=%.1f theta=%.1f edge=%.1fpx",
               (double)avg.u, (double)avg.v,
               (double)(avg.theta * 180.0f / (float)M_PI), (double)avg.edge_a);
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

/* ==================== align_view.h ==================== */
/* align_view.h - 检测可视化流 (:80/view MJPEG)
 *
 * 输出灰度检测帧, 并在帧上叠加: 候选框/角点/连通域数/丢弃原因/汉明距离。
 * 用途: 不再依赖串口, 直接在 App 里看到"检测到了什么"。
 */
#ifndef WALLBOT_ALIGN_VIEW_H
#define WALLBOT_ALIGN_VIEW_H

#include <stdbool.h>
#include "esp_http_server.h"

/* 把 /view 路由注册到控制服务 (端口 80) */
void AlignView_Register(httpd_handle_t httpd);

#endif /* WALLBOT_ALIGN_VIEW_H */

/* ==================== align_view.cpp ==================== */
/* align_view.cpp - 检测可视化流 (:80/view MJPEG)
 *
 * 每帧流程: 抓灰度帧 -> 检测(带诊断) -> 叠加图形/文字 -> 软编码 JPEG -> 推送。
 * 叠加内容:
 *   - 最大候选包围盒 (白框)
 *   - 成功时画 4 个角点连线 (白线) 与中心十字
 *   - 左上角文字: comps/cand/drop 原因/hamming/contrast/margin
 * 文字用 5x7 点阵字库放大 2 倍绘制。
 */
#include <Arduino.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_http_server.h"
#include <string.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "config.h"
#include "cam_module.h"
#include "tag_detector.h"
#include "align_view.h"
#include "align_server.h"
#include "stream_server.h"
#include "esp_heap_caps.h"

/* 由 align_server.cpp 提供: JPEG -> 可写灰度 (QVGA) */
bool AlignDecodeJpegToGray(const uint8_t *jpg, size_t jlen, int fw, int fh,
                           const uint8_t **out_gray, int *out_w, int *out_h);

static const char *VIEW_BOUNDARY = "123456789000000000000987654321";
static const char *VIEW_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" "123456789000000000000987654321";
static const char *VIEW_BOUND =
    "\r\n--" "123456789000000000000987654321" "\r\n";
static const char *VIEW_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

/* ---------------- 5x7 点阵字体 (仅大写+数字+少量符号) ---------------- */
typedef struct { char c; uint8_t col[5]; } Glyph;

static const Glyph FONT[] = {
    {' ', {0x00,0x00,0x00,0x00,0x00}},
    {'0', {0x3E,0x51,0x49,0x45,0x3E}}, {'1', {0x00,0x42,0x7F,0x40,0x00}},
    {'2', {0x42,0x61,0x51,0x49,0x46}}, {'3', {0x21,0x41,0x45,0x4B,0x31}},
    {'4', {0x18,0x14,0x12,0x7F,0x10}}, {'5', {0x27,0x45,0x45,0x45,0x39}},
    {'6', {0x3C,0x4A,0x49,0x49,0x30}}, {'7', {0x01,0x71,0x09,0x05,0x03}},
    {'8', {0x36,0x49,0x49,0x49,0x36}}, {'9', {0x06,0x49,0x49,0x29,0x1E}},
    {'A', {0x7E,0x11,0x11,0x11,0x7E}}, {'B', {0x7F,0x49,0x49,0x49,0x36}},
    {'C', {0x3E,0x41,0x41,0x41,0x22}}, {'D', {0x7F,0x41,0x41,0x22,0x1C}},
    {'E', {0x7F,0x49,0x49,0x49,0x41}}, {'F', {0x7F,0x09,0x09,0x09,0x01}},
    {'G', {0x3E,0x41,0x49,0x49,0x7A}}, {'H', {0x7F,0x08,0x08,0x08,0x7F}},
    {'I', {0x00,0x41,0x7F,0x41,0x00}}, {'J', {0x20,0x40,0x41,0x3F,0x01}},
    {'K', {0x7F,0x08,0x14,0x22,0x41}}, {'L', {0x7F,0x40,0x40,0x40,0x40}},
    {'M', {0x7F,0x02,0x0C,0x02,0x7F}}, {'N', {0x7F,0x04,0x08,0x10,0x7F}},
    {'O', {0x3E,0x41,0x41,0x41,0x3E}}, {'P', {0x7F,0x09,0x09,0x09,0x06}},
    {'Q', {0x3E,0x41,0x51,0x21,0x5E}}, {'R', {0x7F,0x09,0x19,0x29,0x46}},
    {'S', {0x46,0x49,0x49,0x49,0x31}}, {'T', {0x01,0x01,0x7F,0x01,0x01}},
    {'U', {0x3F,0x40,0x40,0x40,0x3F}}, {'V', {0x1F,0x20,0x40,0x20,0x1F}},
    {'W', {0x3F,0x40,0x38,0x40,0x3F}}, {'X', {0x63,0x14,0x08,0x14,0x63}},
    {'Y', {0x07,0x08,0x70,0x08,0x07}}, {'Z', {0x61,0x51,0x49,0x45,0x43}},
    {':', {0x00,0x36,0x36,0x00,0x00}}, {'.', {0x00,0x60,0x60,0x00,0x00}},
    {'-', {0x08,0x08,0x08,0x08,0x08}}, {'/', {0x20,0x10,0x08,0x04,0x02}},
    {'=', {0x14,0x14,0x14,0x14,0x14}}, {'%', {0x23,0x13,0x08,0x64,0x62}},
    {'(', {0x00,0x1C,0x22,0x41,0x00}}, {')', {0x00,0x41,0x22,0x1C,0x00}},
};

static const uint8_t *glyph_of(char c) {
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
    for (size_t i = 0; i < sizeof(FONT) / sizeof(FONT[0]); i++)
        if (FONT[i].c == c) return FONT[i].col;
    return FONT[0].col;
}

/* 在灰度图上画字符 (scale 倍), 颜色 0=黑 255=白 */
static void draw_char(uint8_t *img, int w, int h, int x0, int y0, char c, int scale, uint8_t color) {
    const uint8_t *g = glyph_of(c);
    for (int col = 0; col < 5; col++) {
        uint8_t bits = g[col];
        for (int row = 0; row < 7; row++) {
            if (!(bits & (1 << row))) continue;
            for (int dy = 0; dy < scale; dy++) {
                int py = y0 + row * scale + dy;
                if (py < 0 || py >= h) continue;
                for (int dx = 0; dx < scale; dx++) {
                    int px = x0 + col * scale + dx;
                    if (px < 0 || px >= w) continue;
                    img[py * w + px] = color;
                }
            }
        }
    }
}

static void draw_text(uint8_t *img, int w, int h, int x, int y, const char *s, int scale, uint8_t color) {
    int cx = x;
    for (; *s; s++) {
        draw_char(img, w, h, cx, y, *s, scale, color);
        cx += 6 * scale;
        if (cx > w - 6 * scale) break;
    }
}

/* 半透明底衬: 让文字在任意灰度背景上都可读 */
static void draw_text_bg(uint8_t *img, int w, int h, int x, int y, int cw, int ch) {
    for (int py = y - 2; py < y + ch + 2; py++) {
        if (py < 0 || py >= h) continue;
        for (int px = x - 2; px < x + cw + 2; px++) {
            if (px < 0 || px >= w) continue;
            img[py * w + px] = (uint8_t)(img[py * w + px] / 3);
        }
    }
}

static void draw_rect(uint8_t *img, int w, int h, int x, int y, int rw, int rh, uint8_t color) {
    for (int i = 0; i < rw; i++) {
        int px = x + i;
        if (px < 0 || px >= w) continue;
        if (y >= 0 && y < h) img[y * w + px] = color;
        if (y + rh - 1 >= 0 && y + rh - 1 < h) img[(y + rh - 1) * w + px] = color;
    }
    for (int j = 0; j < rh; j++) {
        int py = y + j;
        if (py < 0 || py >= h) continue;
        if (x >= 0 && x < w) img[py * w + x] = color;
        if (x + rw - 1 >= 0 && x + rw - 1 < w) img[py * w + (x + rw - 1)] = color;
    }
}

static void draw_line(uint8_t *img, int w, int h, int x0, int y0, int x1, int y1, uint8_t color) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        if (x0 >= 0 && x0 < w && y0 >= 0 && y0 < h) img[y0 * w + x0] = color;
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

/* ---------------- 处理一帧 ---------------- */
static uint32_t g_view_fail = 0;

/* 可写的灰度工作帧 (JPEG 解码后在此叠加图形/文字, 再编码输出) */
static uint8_t *s_view_gray = NULL;

static bool view_process_frame(uint8_t **out_jpg, size_t *out_len) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        if ((g_view_fail++ % 20) == 0) Stream_Log("view fb_get fail n=%u", (unsigned)g_view_fail);
        return false;
    }
    if (fb->format != PIXFORMAT_JPEG) {
        esp_camera_fb_return(fb);
        if ((g_view_fail++ % 20) == 0)
            Stream_Log("view fb format=%d (expect jpeg)", (int)fb->format);
        return false;
    }
    const int w = ALIGN_DETECT_W, h = ALIGN_DETECT_H;
    if (fb->width != w || fb->height != h) {
        /* 检测模式未生效(仍是 HD), 本帧丢弃并等待下一帧 */
        esp_camera_fb_return(fb);
        if ((g_view_fail++ % 20) == 0)
            Stream_Log("view fb size=%dx%d (expect %dx%d)", (int)fb->width, (int)fb->height, w, h);
        return false;
    }
    if (!s_view_gray) {
        s_view_gray = (uint8_t *)heap_caps_malloc((size_t)w * h, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_view_gray) s_view_gray = (uint8_t *)malloc((size_t)w * h);
        if (!s_view_gray) { esp_camera_fb_return(fb); return false; }
    }
    /* 相机全程保持 JPEG; 这里解码为可写灰度帧再叠加。
     * AlignDecodeJpegToGray 与 TagDetector 都用全局静态工作区,
     * :81/view 与 :80/align/* 在不同 httpd 任务, 必须同一把锁串行。 */
    if (!g_measure_lock) g_measure_lock = xSemaphoreCreateMutex();
    if (g_measure_lock && xSemaphoreTake(g_measure_lock, pdMS_TO_TICKS(1000)) != pdTRUE) {
        esp_camera_fb_return(fb);
        return false;
    }
    const uint8_t *gray = NULL;
    int gw = 0, gh = 0;
    bool decoded = AlignDecodeJpegToGray(fb->buf, fb->len, fb->width, fb->height, &gray, &gw, &gh);
    TagObservation obs;
    TagDiag diag;
    bool ok = false;
    if (decoded) {
        memcpy(s_view_gray, gray, (size_t)w * h);
        ok = TagDetector_DetectEx(s_view_gray, w, h, w, &obs, &diag);
    }
    if (g_measure_lock) xSemaphoreGive(g_measure_lock);
    esp_camera_fb_return(fb);
    if (!decoded) {
        if ((g_view_fail++ % 20) == 0) Stream_Log("view jpeg decode fail");
        return false;
    }

    uint8_t *img = s_view_gray;

    /* 叠加图形 */
    if (diag.cand_bbox[2] > 0 && diag.cand_bbox[3] > 0) {
        draw_rect(img, w, h, diag.cand_bbox[0], diag.cand_bbox[1],
                  diag.cand_bbox[2], diag.cand_bbox[3], 255);
    }
    if (ok) {
        for (int i = 0; i < 4; i++) {
            int x0 = (int)obs.corners[i][0], y0 = (int)obs.corners[i][1];
            int x1 = (int)obs.corners[(i + 1) % 4][0], y1 = (int)obs.corners[(i + 1) % 4][1];
            draw_line(img, w, h, x0, y0, x1, y1, 255);
        }
        int cu = (int)obs.u, cv = (int)obs.v;
        draw_line(img, w, h, cu - 6, cv, cu + 6, cv, 255);
        draw_line(img, w, h, cu, cv - 6, cu, cv + 6, 255);
    }

    /* 叠加文字 (4 行, 每行最多 ~26 字符) */
    char line[4][40];
    snprintf(line[0], sizeof(line[0]), "COMP %d CAND %d AREA %d",
             diag.ncomps, diag.ncand, diag.cand_area);
    snprintf(line[1], sizeof(line[1]), "BBOX %dx%d FILL %d%%",
             diag.cand_bbox[2], diag.cand_bbox[3], (int)(diag.cand_fill * 100));
    snprintf(line[2], sizeof(line[2]), "DROP S%d L%d F%d A%d",
             diag.drop_small, diag.drop_large, diag.drop_fill, diag.drop_aspect);
    if (ok) {
        snprintf(line[3], sizeof(line[3]), "OK ID%d HAM%d C%d",
                 obs.id, obs.hamming, (int)obs.contrast);
    } else {
        snprintf(line[3], sizeof(line[3]), "FAIL ST%d HAM%d TRY%d",
                 diag.stage, diag.best_hamming, diag.decode_tried);
    }

    const int scale = 2;
    const int lh = 7 * scale + 4;
    for (int i = 0; i < 4; i++) {
        int y = 4 + i * lh;
        int cw = (int)strlen(line[i]) * 6 * scale;
        draw_text_bg(img, w, h, 4, y, cw, 7 * scale);
        draw_text(img, w, h, 4, y, line[i], scale, 255);
    }

    /* 把叠加后的灰度帧编码成 JPEG 输出 */
    return fmt2jpg(s_view_gray, (size_t)w * h, w, h, PIXFORMAT_GRAYSCALE, 12, out_jpg, out_len);
}

static esp_err_t view_handler(httpd_req_t *req) {
    esp_err_t res = httpd_resp_set_type(req, VIEW_CONTENT_TYPE);
    if (res != ESP_OK) return res;
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char part_buf[64];
    /* 本流独占检测模式: 进入时切灰度, 断开时切回 JPEG HD。
     * 这段时间内 :81/stream 会暂停出帧(保持连接), 由 App 端保证两者不同时拉。 */
    Cam_EnterDetectMode();
    while (true) {
        uint8_t *jpg = NULL;
        size_t jlen = 0;
        if (!view_process_frame(&jpg, &jlen)) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        int hlen = snprintf(part_buf, sizeof(part_buf), VIEW_PART, (unsigned)jlen);
        size_t blen = strlen(VIEW_BOUND);
        size_t total = blen + (size_t)hlen + jlen;
        char *buf = (char *)malloc(total);
        if (!buf) { free(jpg); res = ESP_FAIL; break; }
        memcpy(buf, VIEW_BOUND, blen);
        memcpy(buf + blen, part_buf, (size_t)hlen);
        memcpy(buf + blen + hlen, jpg, jlen);
        res = httpd_resp_send_chunk(req, buf, total);
        free(buf);
        free(jpg);
        if (res != ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    /* 客户端断开: 恢复 JPEG HD, 让 :81/stream 继续可用 */
    Cam_ExitDetectMode();
    Stream_Log("view stream closed");
    return res;
}

void AlignView_Register(httpd_handle_t httpd) {
    if (!httpd) return;
    httpd_uri_t view_uri = {.uri = "/view", .method = HTTP_GET,
                            .handler = view_handler, .user_ctx = NULL};
    httpd_register_uri_handler(httpd, &view_uri);
}

/* ==================== tag_detector.h ==================== */
/* tag_detector.h - 板端 AprilTag 36h11 检测 (纯算法, 输入灰度帧)
 *
 * 流程: 自适应阈值 -> 连通域(黑框) -> 主轴对角极值取角点 -> 边缘直线拟合
 *       亚像素角点 -> 单应矩阵 -> 采样 36 数据位 -> 码表匹配(容 1 bit 错)
 * 解码约定与官方 AprilRobotics/apriltag 一致 (见 tag36h11_table.h)。
 */
#ifndef WALLBOT_TAG_DETECTOR_H
#define WALLBOT_TAG_DETECTOR_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool  ok;
    int   id;                 /* 解码出的标签 ID */
    int   hamming;            /* 与码表的汉明距离 (0 = 完全一致) */
    float corners[4][2];      /* 图像坐标, c0左上 c1右上 c2右下 c3左下 (标签自身朝向) */
    float u, v;               /* 中心 (像素) */
    float theta;              /* c0->c1 边方向角 (弧度), 与 tag_utils.py 的 theta 一致 */
    float edge_a;             /* 轴向两边平均边长 (像素), 比例尺基准 */
    float edge_p;             /* 周向两边平均边长 (像素) */
    float margin;             /* 平均判决裕度 (灰度) */
    float contrast;           /* 白黑电平差 */
} TagObservation;

/* 诊断信息: 检测失败时也能看出卡在哪一步 (供 /align/view 叠加显示) */
typedef struct {
    int  ncomps;          /* 连通域数量 */
    int  ncand;           /* 通过面积初筛的候选数 */
    int  cand_bbox[4];    /* 最大候选包围盒 x,y,w,h (无候选时全 0) */
    int  cand_area;
    float cand_fill;      /* 面积/包围盒面积 */
    int  drop_small;      /* 因太小被丢弃的候选数 */
    int  drop_large;      /* 因太大被丢弃 */
    int  drop_fill;       /* 因填充率不符被丢弃 */
    int  drop_aspect;     /* 因长宽比不符被丢弃 */
    int  decode_tried;    /* 实际尝试解码的候选数 */
    int  best_hamming;    /* 尝试过的最优汉明距离 (255=未尝试) */
    float best_contrast;  /* 最优候选的对比度 */
    float best_margin;
    int  stage;           /* 失败阶段: 0=ok 1=无连通域 2=候选全被筛掉 3=解码失败 */
} TagDiag;

/* 在灰度图中检测 TAG_ID 标签; 找到返回 true 并填充 out。
 * diag 可为 NULL; 非 NULL 时无论成功失败都填充诊断信息。 */
bool TagDetector_Detect(const uint8_t *gray, int w, int h, int stride,
                        TagObservation *out);

bool TagDetector_DetectEx(const uint8_t *gray, int w, int h, int stride,
                          TagObservation *out, TagDiag *diag);

#endif /* WALLBOT_TAG_DETECTOR_H */

/* ==================== tag36h11_table.h ==================== */
/* tag36h11_table.h - AprilTag 36h11 官方码表 (自动生成, 勿手改)
 *
 * 数据来源: AprilRobotics/apriltag 库 tag36h11.c (BSD-2 许可, 版权归
 * The Regents of The University of Michigan / APRIL Robotics Lab)。
 * 由 视觉/code/gen_tag36h11_table.py 从官方源码提取生成。
 *
 * 约定 (与官方解码一致):
 *   - 标签黑框 = 8x8 单元: 最外圈 1 圈黑框 + 内部 6x6 数据位
 *   - 36 个数据位按 bit_x[i]/bit_y[i] (螺旋序) 取值, MSB 在前
 *     (bit=1 表示该单元为白)
 *   - 旋转匹配: 90° 旋转 = 位索引循环移位 9 (36/4)
 *   - TAG36H11_NCODES 个码, 与检测码的汉明距离最小者即 ID
 */
#ifndef TAG36H11_TABLE_H
#define TAG36H11_TABLE_H

#include <stdint.h>

#define TAG36H11_NCODES        587
#define TAG36H11_NBITS         36
#define TAG36H11_WIDTH_AT_BORDER 8   /* 黑框边长(格) = 1圈黑框 + 6x6 数据 */
#define TAG36H11_TOTAL_WIDTH   10    /* 含 1 格白色静区 */

/* 36 个数据位在 8x8 网格中的坐标 (0~7, 外圈 0/7 为黑框) */
static const uint8_t TAG36H11_BIT_X[TAG36H11_NBITS] = {
    1,
    2,
    3,
    4,
    5,
    2,
    3,
    4,
    3,
    6,
    6,
    6,
    6,
    6,
    5,
    5,
    5,
    4,
    6,
    5,
    4,
    3,
    2,
    5,
    4,
    3,
    4,
    1,
    1,
    1,
    1,
    1,
    2,
    2,
    2,
    3};
static const uint8_t TAG36H11_BIT_Y[TAG36H11_NBITS] = {
    1,
    1,
    1,
    1,
    1,
    2,
    2,
    2,
    3,
    1,
    2,
    3,
    4,
    5,
    2,
    3,
    4,
    3,
    6,
    6,
    6,
    6,
    6,
    5,
    5,
    5,
    4,
    6,
    5,
    4,
    3,
    2,
    5,
    4,
    3,
    4};

/* 587 个 36bit 码, MSB = bit[0] (螺旋序第 0 位) */
static const uint64_t TAG36H11_CODES[TAG36H11_NCODES] = {
    0x0000000d7e00984bULL,
    0x0000000dda664ca7ULL,
    0x0000000dc4a1c821ULL,
    0x0000000e17b470e9ULL,
    0x0000000ef91d01b1ULL,
    0x0000000f429cdd73ULL,
    0x000000005da29225ULL,
    0x00000001106cba43ULL,
    0x0000000223bed79dULL,
    0x000000021f51213cULL,
    0x000000033eb19ca6ULL,
    0x00000003f76eb0f8ULL,
    0x0000000469a97414ULL,
    0x000000045dcfe0b0ULL,
    0x00000004a6465f72ULL,
    0x000000051801db96ULL,
    0x00000005eb946b4eULL,
    0x000000068a7cc2ecULL,
    0x00000006f0ba2652ULL,
    0x000000078765559dULL,
    0x000000087b83d129ULL,
    0x000000086cc4a5c5ULL,
    0x00000008b64df90fULL,
    0x00000009c577b611ULL,
    0x0000000a3810f2f5ULL,
    0x0000000af4d75b83ULL,
    0x0000000b59a03fefULL,
    0x0000000bb1096f85ULL,
    0x0000000d1b92fc76ULL,
    0x0000000d0dd509d2ULL,
    0x0000000e2cfda160ULL,
    0x00000002ff497c63ULL,
    0x000000047240671bULL,
    0x00000005047a2e55ULL,
    0x0000000635ca87c7ULL,
    0x0000000691254166ULL,
    0x000000068f43d94aULL,
    0x00000006ef24bdb6ULL,
    0x00000008cdd8f886ULL,
    0x00000009de96b718ULL,
    0x0000000aff6e5a8aULL,
    0x0000000bae46f029ULL,
    0x0000000d225b6d59ULL,
    0x0000000df8ba8c01ULL,
    0x0000000e3744a22fULL,
    0x0000000fbb59375dULL,
    0x000000018a916828ULL,
    0x000000022f29c1baULL,
    0x0000000286887d58ULL,
    0x000000041392322eULL,
    0x000000075d18ecd1ULL,
    0x000000087c302743ULL,
    0x00000008c6317ba9ULL,
    0x00000009e40f36d7ULL,
    0x0000000c0e5a806aULL,
    0x0000000cc78cb87cULL,
    0x000000012d2f2d01ULL,
    0x0000000379f36a21ULL,
    0x00000006973f59acULL,
    0x00000007789ea9f4ULL,
    0x00000008f1c73e84ULL,
    0x00000008dd287a20ULL,
    0x000000094a4eee4cULL,
    0x0000000a455379b5ULL,
    0x0000000a9e92987dULL,
    0x0000000bd25cb40bULL,
    0x0000000be98d3582ULL,
    0x0000000d3d5972b2ULL,
    0x000000014c53d7c7ULL,
    0x00000004f1796936ULL,
    0x00000004e71fed1aULL,
    0x000000066d46fae0ULL,
    0x0000000a55abb933ULL,
    0x0000000ebee1accaULL,
    0x00000001ad4ba6a4ULL,
    0x0000000305b17571ULL,
    0x0000000553611351ULL,
    0x000000059ca62775ULL,
    0x00000007819cb6a1ULL,
    0x0000000edb7bc9ebULL,
    0x00000005b2694212ULL,
    0x000000072e12d185ULL,
    0x0000000ed6152e2cULL,
    0x00000005bcdadbf3ULL,
    0x000000078e0aa0c6ULL,
    0x0000000c60a0b909ULL,
    0x0000000ef9a34b0dULL,
    0x0000000398a6621aULL,
    0x0000000a8a27c944ULL,
    0x00000004b564304eULL,
    0x000000052902b4e2ULL,
    0x0000000857280b56ULL,
    0x0000000a91b2c84bULL,
    0x0000000e91df939bULL,
    0x00000001fa405f28ULL,
    0x000000023793ab86ULL,
    0x000000068c17729fULL,
    0x00000009fbf3b840ULL,
    0x000000036922413cULL,
    0x00000004eb5f946eULL,
    0x0000000533fe2404ULL,
    0x000000063de7d35eULL,
    0x0000000925eddc72ULL,
    0x000000099b8b3896ULL,
    0x0000000aace4c708ULL,
    0x0000000c22994af0ULL,
    0x00000008f1eae41bULL,
    0x0000000d95fb486cULL,
    0x000000013fb77857ULL,
    0x00000004fe0983a3ULL,
    0x0000000d559bf8a9ULL,
    0x0000000e1855d78dULL,
    0x0000000fec8daaadULL,
    0x000000071ecb6d95ULL,
    0x0000000dc9e50e4cULL,
    0x0000000ca3a4c259ULL,
    0x0000000740d12bbfULL,
    0x0000000aeedd18e0ULL,
    0x0000000b509b9c8eULL,
    0x00000005232fea1cULL,
    0x000000019282d18bULL,
    0x000000076c22d67bULL,
    0x0000000936beb34bULL,
    0x000000008a5ea8ddULL,
    0x0000000679eadc28ULL,
    0x0000000a08e119c5ULL,
    0x000000020a6e3e24ULL,
    0x00000007eab9c239ULL,
    0x000000096632c32eULL,
    0x0000000470d06e44ULL,
    0x00000008a70212fbULL,
    0x00000000a7e4251bULL,
    0x00000009ec762cc0ULL,
    0x0000000d8a3a1f48ULL,
    0x0000000db680f346ULL,
    0x00000004a1e93a9dULL,
    0x0000000638ddc04fULL,
    0x00000004c2fcc993ULL,
    0x000000001ef28c95ULL,
    0x0000000bf0d9792dULL,
    0x00000006d27557c3ULL,
    0x0000000623f977f4ULL,
    0x000000035b43be57ULL,
    0x0000000bb0c428d5ULL,
    0x0000000a6f01474dULL,
    0x00000005a70c9749ULL,
    0x000000020ddabc3bULL,
    0x00000002eabd78cfULL,
    0x000000090aa18f88ULL,
    0x0000000a9ea89350ULL,
    0x00000003cdb39b22ULL,
    0x0000000839a08f34ULL,
    0x0000000169bb814eULL,
    0x00000001a575ab08ULL,
    0x0000000a04d3d5a2ULL,
    0x0000000bf7902f2bULL,
    0x0000000095a5e65cULL,
    0x000000092e8fce94ULL,
    0x000000067ef48d12ULL,
    0x00000006400dbcacULL,
    0x0000000b12d8fb9fULL,
    0x00000000347f45d3ULL,
    0x0000000b35826f56ULL,
    0x0000000c546ac6e4ULL,
    0x000000081cc35b66ULL,
    0x000000041d14bd57ULL,
    0x00000000c052b168ULL,
    0x00000007d6ce5018ULL,
    0x0000000ab4ed5edeULL,
    0x00000005af817119ULL,
    0x0000000d1454b182ULL,
    0x00000002badb090bULL,
    0x000000003fcb4c0cULL,
    0x00000002f1c28fd8ULL,
    0x000000093608c6f7ULL,
    0x00000004c93ba2b5ULL,
    0x000000007d950a5dULL,
    0x0000000e54b3d3fcULL,
    0x000000015560cf9dULL,
    0x0000000189e4958aULL,
    0x000000062140e9d2ULL,
    0x0000000723bc1cdbULL,
    0x00000002063f26faULL,
    0x0000000fa08ab19fULL,
    0x00000007955641dbULL,
    0x0000000646b01daaULL,
    0x000000071cd427ccULL,
    0x000000009a42f7d4ULL,
    0x0000000717edc643ULL,
    0x000000015eb94367ULL,
    0x00000008392e6bb2ULL,
    0x0000000832408542ULL,
    0x00000002b9b874beULL,
    0x0000000b21f4730dULL,
    0x0000000b5d8f24c9ULL,
    0x00000007dbaf6931ULL,
    0x00000001b4e33629ULL,
    0x000000013452e710ULL,
    0x0000000e974af612ULL,
    0x00000001df61d29aULL,
    0x000000099f2532adULL,
    0x0000000e50ec71b4ULL,
    0x00000005df0a36e8ULL,
    0x00000004934e4ceaULL,
    0x0000000e34a0b4bdULL,
    0x0000000b7b26b588ULL,
    0x00000000f255118dULL,
    0x0000000d0c8fa31eULL,
    0x000000006a50c94fULL,
    0x0000000f28aa9f06ULL,
    0x0000000131d194d8ULL,
    0x0000000622e3da79ULL,
    0x0000000ac7478303ULL,
    0x0000000c8f2521d7ULL,
    0x00000006c9c881f5ULL,
    0x000000049e38b60aULL,
    0x0000000513d8df65ULL,
    0x0000000d7c2b0785ULL,
    0x00000009f6f9d75aULL,
    0x00000009f6966020ULL,
    0x00000001e1a54e33ULL,
    0x0000000c04d63419ULL,
    0x0000000946e04cd7ULL,
    0x00000001bdac5902ULL,
    0x000000056469b830ULL,
    0x0000000ffad59569ULL,
    0x000000086970e7d8ULL,
    0x00000008a4b41e12ULL,
    0x0000000ad4688e3bULL,
    0x000000085f8f5df4ULL,
    0x0000000d833a0893ULL,
    0x00000002a36fdd7cULL,
    0x0000000d6a857cf2ULL,
    0x00000008829bc35cULL,
    0x00000005e50d79bcULL,
    0x0000000fbb8035e4ULL,
    0x0000000c1a95bebfULL,
    0x0000000036b0baf8ULL,
    0x0000000e0da964eaULL,
    0x0000000b6483689bULL,
    0x00000007c8e2f4c1ULL,
    0x00000005b856a23bULL,
    0x00000002fc183995ULL,
    0x0000000e914b6d70ULL,
    0x0000000b31041969ULL,
    0x00000001bb478493ULL,
    0x0000000063e2b456ULL,
    0x0000000f2a082b9cULL,
    0x00000008e5e646eaULL,
    0x000000008172f8f6ULL,
    0x00000000dacd923eULL,
    0x0000000e5dcf0e2eULL,
    0x0000000bf9446baeULL,
    0x00000004822d50d1ULL,
    0x000000026e710bf5ULL,
    0x0000000b90ba2a24ULL,
    0x0000000f3b25aa73ULL,
    0x0000000809ad589bULL,
    0x000000094cc1e254ULL,
    0x00000005334a3adbULL,
    0x0000000592886b2fULL,
    0x0000000bf64704aaULL,
    0x0000000566dbf24cULL,
    0x000000072203e692ULL,
    0x000000064e61e809ULL,
    0x0000000d7259aad6ULL,
    0x00000007b924aedcULL,
    0x00000002df2184e8ULL,
    0x0000000353d1eca7ULL,
    0x0000000fce30d7ceULL,
    0x0000000f7b0f436eULL,
    0x000000057e8d8f68ULL,
    0x00000008c79e60dbULL,
    0x00000009c8362b2bULL,
    0x000000063a5804f2ULL,
    0x00000009298353dcULL,
    0x00000006f98a71c8ULL,
    0x0000000a5731f693ULL,
    0x000000021ca5c870ULL,
    0x00000001c2107fd3ULL,
    0x00000006181f6c39ULL,
    0x000000019e574304ULL,
    0x0000000329937606ULL,
    0x0000000043d5c70dULL,
    0x00000009b18ff162ULL,
    0x00000008e2ccfebfULL,
    0x000000072b7b9b54ULL,
    0x00000009b71f4f3cULL,
    0x0000000935d7393eULL,
    0x000000065938881aULL,
    0x00000006a5bd6f2dULL,
    0x0000000a19783306ULL,
    0x0000000e6472f4d7ULL,
    0x000000081163df5aULL,
    0x0000000a838e1cbdULL,
    0x0000000982748477ULL,
    0x0000000050c54febULL,
    0x00000000d82fbb58ULL,
    0x00000002c4c72799ULL,
    0x000000097d259ad6ULL,
    0x000000022d9a43edULL,
    0x0000000fdb162a9fULL,
    0x00000000cb4a727dULL,
    0x00000004fae2e371ULL,
    0x0000000535b5be8bULL,
    0x000000048795908aULL,
    0x0000000ce7c18962ULL,
    0x00000004ea154d80ULL,
    0x000000050c064889ULL,
    0x00000008d97fc75dULL,
    0x0000000c8bd9ec61ULL,
    0x000000083ee8e8bbULL,
    0x0000000c8431419aULL,
    0x00000001aa78079dULL,
    0x00000008111aa4a5ULL,
    0x0000000dfa3a69feULL,
    0x000000051630d83fULL,
    0x00000002d930fb3fULL,
    0x00000002133116e5ULL,
    0x0000000ae5395522ULL,
    0x0000000bc07a4e8aULL,
    0x000000057bf08ba0ULL,
    0x00000006cb18036aULL,
    0x0000000f0e2e4b75ULL,
    0x00000003eb692b6fULL,
    0x0000000d8178a3faULL,
    0x0000000238cce6a6ULL,
    0x0000000e97d5cdd7ULL,
    0x0000000fe10d8d5eULL,
    0x0000000b39584a1dULL,
    0x0000000ca03536fdULL,
    0x0000000aa61f3998ULL,
    0x000000072ff23ec2ULL,
    0x000000015aa7d770ULL,
    0x000000057a3a1282ULL,
    0x0000000d1f3902dcULL,
    0x00000006554c9388ULL,
    0x0000000fd01283c7ULL,
    0x0000000e8baa42c5ULL,
    0x000000072cee6adfULL,
    0x0000000f6614b3faULL,
    0x000000095c3778a2ULL,
    0x00000007da4cea7aULL,
    0x0000000d18a5912cULL,
    0x0000000d116426e5ULL,
    0x000000027c17bc1cULL,
    0x0000000b95b53bc1ULL,
    0x0000000c8f937a05ULL,
    0x0000000ed220c9bdULL,
    0x00000000c97d72abULL,
    0x00000008fb1217aeULL,
    0x000000025ca8a5a1ULL,
    0x0000000b261b871bULL,
    0x00000001bef0a056ULL,
    0x0000000806a51179ULL,
    0x0000000eed249145ULL,
    0x00000003f82aecebULL,
    0x0000000cc56e9acfULL,
    0x00000002e78d01ebULL,
    0x0000000102cee17fULL,
    0x000000037caad3d5ULL,
    0x000000016ac5b1eeULL,
    0x00000002af164eceULL,
    0x0000000d4cd81dc9ULL,
    0x000000012263a7e7ULL,
    0x000000057ac7d117ULL,
    0x00000009391d9740ULL,
    0x00000007aedaa77fULL,
    0x00000009675a3c72ULL,
    0x0000000277f25191ULL,
    0x0000000ebb6e64b9ULL,
    0x00000007ad3ef747ULL,
    0x000000012759b181ULL,
    0x0000000948257d4dULL,
    0x0000000b63a850f6ULL,
    0x00000003a52a8f75ULL,
    0x00000004a019532cULL,
    0x0000000a021a7529ULL,
    0x0000000cc661876dULL,
    0x00000004085afd05ULL,
    0x0000000e7048e089ULL,
    0x00000003f979cdc6ULL,
    0x0000000d9da9071bULL,
    0x0000000ed2fc5b68ULL,
    0x000000079d64c3a1ULL,
    0x0000000fd44e2361ULL,
    0x00000008eea46a74ULL,
    0x000000042233b9c2ULL,
    0x0000000ae4d1765dULL,
    0x00000007303a094cULL,
    0x00000002d7033abeULL,
    0x00000003dcc2b0b4ULL,
    0x00000000f0967d09ULL,
    0x000000006f0cd7deULL,
    0x000000009807aca0ULL,
    0x00000003a295cad3ULL,
    0x00000002b106b202ULL,
    0x00000003f38a828eULL,
    0x000000078af46596ULL,
    0x0000000bda2dc713ULL,
    0x00000009a8c8c9d9ULL,
    0x00000006a0f2ddceULL,
    0x0000000a76af6fe2ULL,
    0x0000000086f66fa4ULL,
    0x0000000d52d63f8dULL,
    0x000000089f7a6e73ULL,
    0x0000000cc6b23362ULL,
    0x0000000b4ebf3c39ULL,
    0x0000000564f300faULL,
    0x0000000e8de3a706ULL,
    0x000000079a033b61ULL,
    0x0000000765e160c5ULL,
    0x0000000a266a4f85ULL,
    0x0000000a68c38c24ULL,
    0x0000000dca0711fbULL,
    0x000000085fba85baULL,
    0x000000037a207b46ULL,
    0x0000000158fcc4d0ULL,
    0x00000000569d79b3ULL,
    0x00000007b1a25555ULL,
    0x0000000a8ae22468ULL,
    0x00000007c592bdfdULL,
    0x00000000c59a5f66ULL,
    0x0000000b1115daa3ULL,
    0x0000000f17c87177ULL,
    0x00000006769d766bULL,
    0x00000002b637356dULL,
    0x000000013d8685acULL,
    0x0000000f24cb6ec0ULL,
    0x00000000bd0b56d1ULL,
    0x000000042ff0e26dULL,
    0x0000000b41609267ULL,
    0x000000096f9518afULL,
    0x0000000c56f96636ULL,
    0x00000004a8e10349ULL,
    0x0000000863512171ULL,
    0x0000000ea455d86cULL,
    0x0000000bd0e25279ULL,
    0x0000000e65e3f761ULL,
    0x000000036c84a922ULL,
    0x000000085fd1b38fULL,
    0x0000000657c91539ULL,
    0x000000015033fe04ULL,
    0x000000009051c921ULL,
    0x0000000ab27d80d8ULL,
    0x0000000f92f7d0a1ULL,
    0x00000008eb6bb737ULL,
    0x000000010b5b0f63ULL,
    0x00000006c9c7ad63ULL,
    0x0000000f66fe70aeULL,
    0x0000000ca579bd92ULL,
    0x0000000956198e4dULL,
    0x000000029e4405e5ULL,
    0x0000000e44eb885cULL,
    0x000000041612456cULL,
    0x0000000ea45e0abfULL,
    0x0000000d326529bdULL,
    0x00000007b2c33cefULL,
    0x000000080bc9b558ULL,
    0x00000007169b9740ULL,
    0x0000000c37f99209ULL,
    0x000000031ff6dab9ULL,
    0x0000000c795190edULL,
    0x0000000a7636e95fULL,
    0x00000009df075841ULL,
    0x000000055a083932ULL,
    0x0000000a7cbdf630ULL,
    0x0000000409ea4ef0ULL,
    0x000000092a1991b6ULL,
    0x00000004b078dee9ULL,
    0x0000000ae18ce9e4ULL,
    0x00000005a6e1ef35ULL,
    0x00000001a403bd59ULL,
    0x000000031ea70a83ULL,
    0x00000002bc3c4f3aULL,
    0x00000005c921b3cbULL,
    0x0000000042da05c5ULL,
    0x00000001f667d16bULL,
    0x0000000416a368cfULL,
    0x0000000fbc0a7a3bULL,
    0x00000009419f0c7cULL,
    0x000000081be2fa03ULL,
    0x000000034e2c172fULL,
    0x000000028648d8aeULL,
    0x0000000c7acbb885ULL,
    0x000000045f31eb6aULL,
    0x0000000d1cfc0a7bULL,
    0x000000042c4d260dULL,
    0x0000000cf6584097ULL,
    0x000000094b132b14ULL,
    0x00000003c5c5df75ULL,
    0x00000008ae596fefULL,
    0x0000000aea8054ebULL,
    0x00000000ae9cc573ULL,
    0x0000000496fb731bULL,
    0x0000000ebf105662ULL,
    0x0000000af9c83a37ULL,
    0x0000000c0d64cd6bULL,
    0x00000007b608159aULL,
    0x0000000e74431642ULL,
    0x0000000d6fb9d900ULL,
    0x0000000291e99de0ULL,
    0x000000010500ba9aULL,
    0x00000005cd05d037ULL,
    0x0000000a87254fb2ULL,
    0x00000009d7824a37ULL,
    0x00000008b2c7b47cULL,
    0x000000030c788145ULL,
    0x00000002f4e5a8beULL,
    0x0000000badb884daULL,
    0x0000000026e0d5c9ULL,
    0x00000006fdbaa32eULL,
    0x000000034758eb31ULL,
    0x0000000565cd1b4fULL,
    0x00000002bfd90fb0ULL,
    0x0000000093052a6bULL,
    0x0000000d3c13c4b9ULL,
    0x00000002daea43bfULL,
    0x0000000a279762bcULL,
    0x0000000f1bd9f22cULL,
    0x00000004b7fec94fULL,
    0x0000000545761d5aULL,
    0x00000007327df411ULL,
    0x00000001b52a442eULL,
    0x000000049b0ce108ULL,
    0x000000024c764bc8ULL,
    0x0000000374563045ULL,
    0x0000000a3e8f91c6ULL,
    0x00000000e6bd2241ULL,
    0x0000000e0e52ee3cULL,
    0x000000007e8e3caaULL,
    0x000000096c2b7372ULL,
    0x000000033acbdfdaULL,
    0x0000000b15d91e54ULL,
    0x0000000464759ac1ULL,
    0x00000006886a1998ULL,
    0x000000057f5d3958ULL,
    0x00000005a1f5c1f5ULL,
    0x00000000b58158adULL,
    0x0000000e712053fbULL,
    0x00000005352ddb25ULL,
    0x0000000414b98ea0ULL,
    0x000000074f89f546ULL,
    0x000000038a56b3c3ULL,
    0x000000038db0dc17ULL,
    0x0000000aa016a755ULL,
    0x0000000dc72366f5ULL,
    0x00000000cee93d75ULL,
    0x0000000b2fe7a56bULL,
    0x0000000a847ed390ULL,
    0x00000008713ef88cULL,
    0x0000000a217cc861ULL,
    0x00000008bca25d7bULL,
    0x0000000455526818ULL,
    0x0000000ea3a7a180ULL,
    0x0000000a9536e5e0ULL,
    0x00000009b64a1975ULL,
    0x00000005bfc756bcULL,
    0x0000000046aa169bULL,
    0x000000053a17f76fULL,
    0x00000004d6815274ULL,
    0x0000000cca9cf3f6ULL,
    0x00000004013fcb8bULL,
    0x00000003d26cdfa5ULL,
    0x00000005786231f7ULL,
    0x00000007d4ab09abULL,
    0x0000000960b5ffbcULL,
    0x00000008914df0d4ULL,
    0x00000002fc6f2213ULL,
    0x0000000ac235637eULL,
    0x0000000151b28ed3ULL,
    0x000000046f79b6dbULL,
    0x00000001382e0c9fULL,
    0x000000053abf983aULL,
    0x0000000383c47adeULL,
    0x00000003fcf88978ULL,
    0x0000000eb9079df7ULL,
    0x000000009af0714dULL,
    0x0000000da19d1bb7ULL,
    0x00000009a02749f8ULL,
    0x00000001c62dab9bULL,
    0x00000001a137e44bULL,
    0x00000002867718c7ULL,
    0x000000035815525bULL,
    0x00000007cd35c550ULL,
    0x00000002164f73a0ULL,
    0x0000000e8b772fe0ULL};

#endif /* TAG36H11_TABLE_H */

/* ==================== tag_detector.cpp ==================== */
/* tag_detector.cpp - 板端 AprilTag 36h11 检测
 *
 * 简化版检测器, 针对"单标签、近正对、可控补光"的纠偏场景:
 *  1) 积分图自适应阈值二值化 (暗=1)
 *  2) 8 邻域连通域 + 矩统计, 面积/长宽比预筛
 *  3) 主轴对角方向极值定位 4 角点, 边缘射线采样 + 直线拟合求亚像素角点
 *  4) 4 点单应 -> 环形黑白电平平面拟合 -> 采样 36 数据位 -> 码表汉明匹配
 * 内存: 积分图/二值/标签/并查集约 0.7MB, 优先 PSRAM 分配, 只分配一次复用。
 */
#include <Arduino.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "esp_heap_caps.h"
#include "config.h"
#include "tag_detector.h"
#include "tag36h11_table.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DET_MAX_COMPS     4096   /* 连通域统计上限 (噪声场景碎域多, 需留余量) */
#define DET_MAX_CANDS     8      /* 最多尝试解码的候选数 */

typedef struct {
    int32_t area;
    int16_t minx, maxx, miny, maxy;
    double  sx, sy, sxx, syy, sxy;
} CompStat;

/* ================= 工作区 (按尺寸惰性分配复用) ================= */
typedef struct {
    int      w, h;
    uint32_t *ii;      /* (w+1)*(h+1) 积分图 */
    uint8_t  *bin;     /* w*h 二值 */
    uint16_t *lab;     /* w*h 连通域标签 */
    uint16_t *parent;  /* 并查集 */
    CompStat *stats;   /* 暗域统计 */
    uint16_t (*wbbox)[4]; /* 白域包围盒 (填洞用) */
} Scratch;

static Scratch g_sc;
static int g_ncomps = 0;

static void *psram_malloc(size_t n) {
    void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) p = malloc(n);
    return p;
}

static bool scratch_ensure(int w, int h) {
    if (g_sc.w == w && g_sc.h == h) return true;
    free(g_sc.ii); free(g_sc.bin); free(g_sc.lab); free(g_sc.parent);
    free(g_sc.stats); free(g_sc.wbbox);
    memset(&g_sc, 0, sizeof(g_sc));
    g_sc.ii     = (uint32_t *)psram_malloc((size_t)(w + 1) * (h + 1) * 4);
    g_sc.bin    = (uint8_t *)psram_malloc((size_t)w * h);
    g_sc.lab    = (uint16_t *)psram_malloc((size_t)w * h * 2);
    g_sc.parent = (uint16_t *)psram_malloc((size_t)w * h * 2);
    g_sc.stats  = (CompStat *)psram_malloc((size_t)DET_MAX_COMPS * sizeof(CompStat));
    g_sc.wbbox  = (uint16_t (*)[4])psram_malloc((size_t)DET_MAX_COMPS * 4 * 2);
    if (!g_sc.ii || !g_sc.bin || !g_sc.lab || !g_sc.parent ||
        !g_sc.stats || !g_sc.wbbox) {
        free(g_sc.ii); free(g_sc.bin); free(g_sc.lab); free(g_sc.parent);
        free(g_sc.stats); free(g_sc.wbbox);
        memset(&g_sc, 0, sizeof(g_sc));
        return false;
    }
    memset(g_sc.parent, 0, (size_t)w * h * 2);   /* 保证 label 0 的 parent 也是 0 */
    g_sc.w = w; g_sc.h = h;
    return true;
}

/* ================= 双线性灰度采样 (越界返回 -1) ================= */
static float samp_bilinear(const uint8_t *im, int w, int h, int stride, float px, float py) {
    if (px < 0 || py < 0 || px > w - 1 || py > h - 1) return -1.0f;
    int x0 = (int)px, y0 = (int)py;
    int x1 = x0 + 1 < w ? x0 + 1 : x0;
    int y1 = y0 + 1 < h ? y0 + 1 : y0;
    float fx = px - x0, fy = py - y0;
    float g00 = im[y0 * stride + x0], g10 = im[y0 * stride + x1];
    float g01 = im[y1 * stride + x0], g11 = im[y1 * stride + x1];
    return g00 * (1 - fx) * (1 - fy) + g10 * fx * (1 - fy)
         + g01 * (1 - fx) * fy       + g11 * fx * fy;
}

/* ================= 积分图 + 自适应阈值 ================= */
static void threshold_adaptive(const uint8_t *im, int w, int h, int stride) {
    const int win = DET_ADAPT_WIN, half = win / 2;
    uint32_t *ii = g_sc.ii;
    memset(ii, 0, (size_t)(w + 1) * 4);
    for (int y = 0; y < h; y++) {
        const uint8_t *row = im + (size_t)y * stride;
        uint32_t *iirow = ii + (size_t)(y + 1) * (w + 1);
        uint32_t *iiprev = ii + (size_t)y * (w + 1);
        uint32_t run = 0;
        for (int x = 0; x < w; x++) {
            run += row[x];
            iirow[x + 1] = iiprev[x + 1] + run;
        }
    }
    for (int y = 0; y < h; y++) {
        int y0 = y - half, y1 = y + half;
        if (y0 < 0) y0 = 0;
        if (y1 >= h) y1 = h - 1;
        uint8_t *brow = g_sc.bin + (size_t)y * w;
        const uint8_t *grow = im + (size_t)y * stride;
        for (int x = 0; x < w; x++) {
            int x0 = x - half, x1 = x + half;
            if (x0 < 0) x0 = 0;
            if (x1 >= w) x1 = w - 1;
            uint32_t area = (uint32_t)(x1 - x0 + 1) * (y1 - y0 + 1);
            uint32_t sum = ii[(size_t)(y1 + 1) * (w + 1) + (x1 + 1)]
                         - ii[(size_t)(y0) * (w + 1) + (x1 + 1)]
                         - ii[(size_t)(y1 + 1) * (w + 1) + (x0)]
                         + ii[(size_t)(y0) * (w + 1) + (x0)];
            float mean = (float)sum / area;
            brow[x] = (grow[x] < mean - DET_ADAPT_BIAS) ? 1 : 0;
        }
    }
}

/* ================= 并查集 ================= */
static uint16_t uf_find(uint16_t *p, uint16_t a) {
    while (p[a] != a) { p[a] = p[p[a]]; a = p[a]; }
    return a;
}

/* ================= 封闭白区填洞 =================
 * 自适应阈值窗口(15px)小于标签特征厚度(黑框边长/8*1, edge>=120px 时 >15px)时,
 * 大片黑色的内部局部均值≈自身, 不会被判暗 -> 黑框被掏空成细带, 连通域断裂。
 * 处理: 对"白"做连通域标记, 不接触图像边界的白区一律判为洞 -> 填黑。
 * 效果: 标签变成实心黑方块 (对角点提取更稳), 解码仍直接采灰度原图, 不受影响。 */
static void fill_holes(int w, int h) {
    uint16_t *lab = g_sc.lab, *parent = g_sc.parent;
    uint16_t (*wbbox)[4] = g_sc.wbbox;
    uint16_t next = 1;

    for (int y = 0; y < h; y++) {
        uint8_t *brow = g_sc.bin + (size_t)y * w;
        uint16_t *lrow = lab + (size_t)y * w;
        uint16_t *lup = (y > 0) ? lab + (size_t)(y - 1) * w : NULL;
        for (int x = 0; x < w; x++) {
            if (brow[x]) { lrow[x] = 0; continue; }   /* 黑色不属于白区 */
            uint16_t m = 0;
            /* 白区用 4 邻域: 暗格网的对角搭桥不漏, 封闭性才成立 */
            if (x > 0 && lrow[x - 1]) m = lrow[x - 1];
            if (y > 0 && lup[x] && (!m || lup[x] < m)) m = lup[x];
            if (!m) {
                if (next >= DET_MAX_COMPS) { lrow[x] = 0; continue; }
                m = next++;
                parent[m] = m;
                wbbox[m][0] = wbbox[m][1] = x;
                wbbox[m][2] = wbbox[m][3] = y;
            } else {
                m = uf_find(parent, m);
            }
            lrow[x] = m;
            if (x > 0 && lrow[x - 1] && lrow[x - 1] != m) {
                uint16_t a = uf_find(parent, lrow[x - 1]);
                if (a != m) parent[a] = m;
            }
            if (y > 0 && lup[x] && lup[x] != m) {
                uint16_t a = uf_find(parent, lup[x]);
                if (a != m) parent[a] = m;
            }
        }
    }

    /* 不触边的白区 -> 填黑 (洞) */
    for (int y = 0; y < h; y++) {
        uint8_t *brow = g_sc.bin + (size_t)y * w;
        uint16_t *lrow = lab + (size_t)y * w;
        for (int x = 0; x < w; x++) {
            if (brow[x]) continue;
            uint16_t l = lrow[x];
            if (l == 0) continue;                    /* 未标注 = 背景 */
            uint16_t r = uf_find(parent, l);
            if (r >= DET_MAX_COMPS) continue;        /* 超上限按背景处理 */
            if (wbbox[r][0] > 0 || wbbox[r][1] < w - 1 ||
                wbbox[r][2] > 0 || wbbox[r][3] < h - 1) continue;
            brow[x] = 1;
        }
    }
}

/* ================= 连通域 (8 邻域, union-find) ================= */
static void label_components(int w, int h) {
    uint16_t *lab = g_sc.lab, *parent = g_sc.parent;
    uint16_t next = 1;

    for (int y = 0; y < h; y++) {
        uint8_t *brow = g_sc.bin + (size_t)y * w;
        uint16_t *lrow = lab + (size_t)y * w;
        uint16_t *lup = (y > 0) ? lab + (size_t)(y - 1) * w : NULL;
        for (int x = 0; x < w; x++) {
            if (!brow[x]) { lrow[x] = 0; continue; }
            uint16_t m = 0;
            if (x > 0 && lrow[x - 1]) m = lrow[x - 1];
            if (y > 0) {
                if (lup[x] && (!m || lup[x] < m)) m = lup[x];
                if (x > 0 && lup[x - 1] && (!m || lup[x - 1] < m)) m = lup[x - 1];
                if (x + 1 < w && lup[x + 1] && (!m || lup[x + 1] < m)) m = lup[x + 1];
            }
            if (!m) {
                if (next >= 65535) { lrow[x] = 0; continue; }
                m = next++;
                parent[m] = m;
                if (m < DET_MAX_COMPS) {
                    CompStat *st = &g_sc.stats[m];
                    st->minx = st->maxx = x; st->miny = st->maxy = y;
                }
            } else {
                m = uf_find(parent, m);
            }
            lrow[x] = m;
            if (x > 0 && lrow[x - 1] && lrow[x - 1] != m) {
                uint16_t a = uf_find(parent, lrow[x - 1]);
                if (a != m) parent[a] = m;
            }
            if (y > 0) {
                uint16_t ns[3] = {(uint16_t)lup[x], (uint16_t)((x > 0) ? lup[x - 1] : 0),
                                  (uint16_t)((x + 1 < w) ? lup[x + 1] : 0)};
                for (int k = 0; k < 3; k++) {
                    if (ns[k] && ns[k] != m) {
                        uint16_t a = uf_find(parent, ns[k]);
                        if (a != m) parent[a] = m;
                    }
                }
            }
        }
    }

    /* 累加统计 (路径压缩到根) */
    g_ncomps = next - 1;
    memset(g_sc.stats, 0, (size_t)DET_MAX_COMPS * sizeof(CompStat));
    for (int y = 0; y < h; y++) {
        uint16_t *lrow = lab + (size_t)y * w;
        for (int x = 0; x < w; x++) {
            uint16_t l = lrow[x];
            if (!l) continue;
            uint16_t r = uf_find(parent, l);
            lrow[x] = r;
            if (r >= DET_MAX_COMPS) continue;   /* 超上限的域不统计 */
            CompStat *st = &g_sc.stats[r];
            double fx = x, fy = y;
            if (st->area == 0) { st->minx = st->maxx = x; st->miny = st->maxy = y; }
            st->area++;
            st->sx += fx; st->sy += fy;
            st->sxx += fx * fx; st->syy += fy * fy; st->sxy += fx * fy;
            if (x < st->minx) st->minx = x;
            if (x > st->maxx) st->maxx = x;
            if (y < st->miny) st->miny = y;
            if (y > st->maxy) st->maxy = y;
        }
    }
}

/* ================= 3x3 与 8x8 线性求解 ================= */
static bool solve3(double A[3][4]) {
    for (int i = 0; i < 3; i++) {
        int piv = i;
        for (int k = i + 1; k < 3; k++)
            if (fabs(A[k][i]) > fabs(A[piv][i])) piv = k;
        if (fabs(A[piv][i]) < 1e-12) return false;
        if (piv != i) for (int j = i; j < 4; j++) { double t = A[i][j]; A[i][j] = A[piv][j]; A[piv][j] = t; }
        for (int k = i + 1; k < 3; k++) {
            double f = A[k][i] / A[i][i];
            for (int j = i; j < 4; j++) A[k][j] -= f * A[i][j];
        }
    }
    for (int i = 2; i >= 0; i--) {
        double s = A[i][3];
        for (int j = i + 1; j < 3; j++) s -= A[i][j] * A[j][3];
        A[i][3] = s / A[i][i];
    }
    return true;
}

/* 4 点对应 (tag坐标 [-1,1]² -> 图像) 解单应 H, 使 px = H00 xt+H01 yt+H02 / (H20 xt+H21 yt+1) */
static bool homography_solve(const float A[4][2], double H[3][3]) {
    static const float TC[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    double M[8][9] = {{0}};
    for (int i = 0; i < 4; i++) {
        double xt = TC[i][0], yt = TC[i][1];
        double px = A[i][0], py = A[i][1];
        double *r1 = &M[2 * i][0], *r2 = &M[2 * i + 1][0];
        r1[0] = xt; r1[1] = yt; r1[2] = 1; r1[6] = -px * xt; r1[7] = -px * yt; r1[8] = px;
        r2[3] = xt; r2[4] = yt; r2[5] = 1; r2[6] = -py * xt; r2[7] = -py * yt; r2[8] = py;
    }
    for (int i = 0; i < 8; i++) {           /* Gauss 消元 (部分主元) */
        int piv = i;
        for (int k = i + 1; k < 8; k++)
            if (fabs(M[k][i]) > fabs(M[piv][i])) piv = k;
        if (fabs(M[piv][i]) < 1e-12) return false;
        if (piv != i) for (int j = i; j < 9; j++) { double t = M[i][j]; M[i][j] = M[piv][j]; M[piv][j] = t; }
        for (int k = i + 1; k < 8; k++) {
            double f = M[k][i] / M[i][i];
            for (int j = i; j < 9; j++) M[k][j] -= f * M[i][j];
        }
    }
    double x[8];
    for (int i = 7; i >= 0; i--) {
        double s = M[i][8];
        for (int j = i + 1; j < 8; j++) s -= M[i][j] * x[j];
        x[i] = s / M[i][i];
    }
    H[0][0] = x[0]; H[0][1] = x[1]; H[0][2] = x[2];
    H[1][0] = x[3]; H[1][1] = x[4]; H[1][2] = x[5];
    H[2][0] = x[6]; H[2][1] = x[7]; H[2][2] = 1;
    return true;
}

static void h_project(const double H[3][3], double xt, double yt, double *px, double *py) {
    double d = H[2][0] * xt + H[2][1] * yt + 1.0;
    *px = (H[0][0] * xt + H[0][1] * yt + H[0][2]) / d;
    *py = (H[1][0] * xt + H[1][1] * yt + H[1][2]) / d;
}

/* ================= 灰度电平平面拟合 ================= */
typedef struct { double sx, sy, sxx, syy, sxy, sv, sxv, syv; int n; } GrayModel;

static void gm_add(GrayModel *g, double x, double y, double v) {
    g->n++; g->sx += x; g->sy += y; g->sxx += x * x; g->syy += y * y;
    g->sxy += x * y; g->sv += v; g->sxv += x * v; g->syv += y * v;
}

/* 解 v = a + b x + c y; 求解结果存 a,b,c, 失败返回 false */
static bool gm_solve(const GrayModel *g, double *a, double *b, double *c) {
    if (g->n < 6) return false;
    double A[3][4];
    A[0][0] = g->n;    A[0][1] = g->sx;  A[0][2] = g->sy;  A[0][3] = g->sv;
    A[1][0] = g->sx;   A[1][1] = g->sxx; A[1][2] = g->sxy; A[1][3] = g->sxv;
    A[2][0] = g->sy;   A[2][1] = g->sxy; A[2][2] = g->syy; A[2][3] = g->syv;
    if (!solve3(A)) return false;
    *a = A[0][3]; *b = A[1][3]; *c = A[2][3];
    return true;
}

static double gm_eval(double a, double b, double c, double x, double y) { return a + b * x + c * y; }

/* ================= 解码一次 (给定 TL/TR/BR/BL 角点赋值) ================= */
static int decode_attempt(const uint8_t *im, int w, int h, int stride,
                          const float A[4][2], TagObservation *out) {
    double H[3][3];
    if (!homography_solve(A, H)) return 255;

    const int WAB = TAG36H11_WIDTH_AT_BORDER;
    /* 黑白电平采样线 (同官方): 白在黑框外 0.5 格, 黑在框内 0.5 格 */
    GrayModel gw, gb;
    memset(&gw, 0, sizeof(gw)); memset(&gb, 0, sizeof(gb));
    for (int i = 0; i < 8; i++) {
        const double off = 0.5, span = (double)i + 0.5;
        const double coords[8][2] = {
            {-off, span}, {WAB + off, span},          /* 左右白列 */
            {off, span},  {WAB - off, span},          /* 左右黑列 */
            {span, -off}, {span, WAB + off},          /* 上下白行 */
            {span, off},  {span, WAB - off},          /* 上下黑行 */
        };
        for (int k = 0; k < 8; k++) {
            double cx = coords[k][0], cy = coords[k][1];
            double xt = 2.0 * cx / WAB - 1.0, yt = 2.0 * cy / WAB - 1.0;
            double px, py;
            h_project(H, xt, yt, &px, &py);
            float v = samp_bilinear(im, w, h, stride, (float)px, (float)py);
            if (v < 0) continue;
            bool is_white = (k < 2 || (k >= 4 && k < 6));
            gm_add(is_white ? &gw : &gb, xt, yt, v);
        }
    }
    double wa, wb, wc, ba, bb, bc;
    if (!gm_solve(&gw, &wa, &wb, &wc) || !gm_solve(&gb, &ba, &bb, &bc)) return 255;
    double contrast = gm_eval(wa, wb, wc, 0, 0) - gm_eval(ba, bb, bc, 0, 0);
    if (contrast < DET_CONTRAST_MIN) return 255;

    /* 采样 36 数据位, MSB 在前 (官方螺旋序) */
    uint64_t rcode = 0;
    double margin_sum = 0;
    for (int i = 0; i < TAG36H11_NBITS; i++) {
        double cx = TAG36H11_BIT_X[i] + 0.5, cy = TAG36H11_BIT_Y[i] + 0.5;
        double xt = 2.0 * cx / WAB - 1.0, yt = 2.0 * cy / WAB - 1.0;
        double px, py;
        h_project(H, xt, yt, &px, &py);
        float v = samp_bilinear(im, w, h, stride, (float)px, (float)py);
        if (v < 0) return 255;                       /* 位采样出图 -> 放弃 */
        double thresh = 0.5 * (gm_eval(wa, wb, wc, xt, yt) + gm_eval(ba, bb, bc, xt, yt));
        margin_sum += fabs(v - thresh);
        rcode = (rcode << 1) | (uint64_t)(v > thresh ? 1 : 0);
    }

    /* 码表匹配 (含容错位) */
    int best_h = 100, best_id = -1;
    for (int i = 0; i < TAG36H11_NCODES; i++) {
        if (TAG36H11_CODES[i] == rcode) { best_h = 0; best_id = i; break; }
    }
    if (best_h != 0) {
        for (int i = 0; i < TAG36H11_NCODES; i++) {
            int d = __builtin_popcountll(TAG36H11_CODES[i] ^ rcode);
            if (d < best_h) { best_h = d; best_id = i; if (d <= DET_MAX_HAMMING) break; }
        }
    }
    if (best_h > DET_MAX_HAMMING || best_id < 0) return 255;

    /* 角点约定: 官方位网格朝向与 cv2 渲染的图案相差恒定 180°
     * (见 视觉/code/pc_check/check_convention.py 的验证)。
     * 输出角点移 2 位, 使 c0..c3 = cv2 渲染图案的 左上/右上/右下/左下,
     * theta/phi/arc/axial 的符号与 Python 工具链 (tag_utils.py) 完全一致。 */
    float S[4][2];
    for (int i = 0; i < 4; i++) {
        S[i][0] = A[(i + 2) % 4][0];
        S[i][1] = A[(i + 2) % 4][1];
    }

    /* 与 tag_utils.py 一致: c0左上 c1右上, theta = c0->c1 方向角 */
    float e01 = hypotf(S[1][0] - S[0][0], S[1][1] - S[0][1]);
    float e12 = hypotf(S[2][0] - S[1][0], S[2][1] - S[1][1]);
    float e23 = hypotf(S[3][0] - S[2][0], S[3][1] - S[2][1]);
    float e30 = hypotf(S[0][0] - S[3][0], S[0][1] - S[3][1]);

    memset(out, 0, sizeof(*out));
    out->ok = true;
    out->id = best_id;
    out->hamming = best_h;
    out->u = (S[0][0] + S[1][0] + S[2][0] + S[3][0]) * 0.25f;
    out->v = (S[0][1] + S[1][1] + S[2][1] + S[3][1]) * 0.25f;
    out->theta = atan2f(S[1][1] - S[0][1], S[1][0] - S[0][0]);
    out->edge_a = 0.5f * (e01 + e23);
    out->edge_p = 0.5f * (e12 + e30);
    out->margin = (float)(margin_sum / TAG36H11_NBITS);
    out->contrast = (float)contrast;
    memcpy(out->corners, S, sizeof(out->corners));
    return best_h;
}

/* ================= 角点精修 (边缘射线 + 直线拟合 + 求交) ================= */
typedef struct { double x, y, dx, dy; bool valid; } Line2D;

static bool fit_side_line(const uint8_t *im, int w, int h, int stride,
                          const float PA[2], const float PB[2],
                          const float centroid[2], Line2D *ln) {
    double sx = PB[0] - PA[0], sy = PB[1] - PA[1];
    double len = hypot(sx, sy);
    if (len < 8) return false;
    double dx = sx / len, dy = sy / len;
    double nx = -dy, ny = dx;
    /* 法向指向远离形心一侧 (黑框外侧) */
    double mx = 0.5 * (PA[0] + PB[0]) - centroid[0];
    double my = 0.5 * (PA[1] + PB[1]) - centroid[1];
    if (nx * mx + ny * my < 0) { nx = -nx; ny = -ny; }

    double pts[9][2];
    int npts = 0;
    const int RAY = 5;
    for (int k = 0; k < 9; k++) {
        double t = 0.2 + k * 0.075;
        double bx = PA[0] + t * sx, by = PA[1] + t * sy;
        float g[2 * RAY + 1];
        for (int s = 0; s <= 2 * RAY; s++)
            g[s] = samp_bilinear(im, w, h, stride,
                                 (float)(bx + (RAY - s) * nx), (float)(by + (RAY - s) * ny));
        float gmax = -1e9f, gmin = 1e9f;
        for (int s = 0; s <= 2 * RAY; s++) {
            if (g[s] < 0) { gmax = -1; break; }
            if (g[s] > gmax) gmax = g[s];
            if (g[s] < gmin) gmin = g[s];
        }
        if (gmax < 0 || gmax - gmin < DET_CONTRAST_MIN) continue;
        float T = 0.5f * (gmax + gmin);
        /* s=0 对应最外侧, 向内找首次变暗, 线性插值亚像素 */
        for (int s = 0; s < 2 * RAY; s++) {
            if (g[s] >= T && g[s + 1] < T) {
                float frac = (g[s] - T) / (g[s] - g[s + 1]);
                float sd = (RAY - s) - frac;      /* 距中心的外法向像素数 */
                pts[npts][0] = bx + sd * nx;
                pts[npts][1] = by + sd * ny;
                npts++;
                break;
            }
        }
    }
    if (npts < 5) return false;

    double cx = 0, cy = 0;
    for (int i = 0; i < npts; i++) { cx += pts[i][0]; cy += pts[i][1]; }
    cx /= npts; cy /= npts;
    double mxx = 0, myy = 0, mxy = 0;
    for (int i = 0; i < npts; i++) {
        double ax = pts[i][0] - cx, ay = pts[i][1] - cy;
        mxx += ax * ax; myy += ay * ay; mxy += ax * ay;
    }
    double th = 0.5 * atan2(2 * mxy, mxx - myy);
    ln->x = cx; ln->y = cy;
    ln->dx = cos(th); ln->dy = sin(th);
    ln->valid = true;
    return true;
}

static bool line_intersect(const Line2D *l1, const Line2D *l2, float out[2]) {
    double det = l1->dx * l2->dy - l2->dx * l1->dy;
    if (fabs(det) < 1e-9) return false;
    double t = ((l2->x - l1->x) * l2->dy - (l2->y - l1->y) * l2->dx) / det;
    out[0] = (float)(l1->x + t * l1->dx);
    out[1] = (float)(l1->y + t * l1->dy);
    return true;
}

static void refine_corners(const uint8_t *im, int w, int h, int stride,
                           const float P[4][2], const float centroid[2], float R[4][2]) {
    Line2D lines[4];
    for (int i = 0; i < 4; i++) {
        const float *PA = P[i], *PB = P[(i + 1) % 4];
        lines[i].valid = false;
        fit_side_line(im, w, h, stride, PA, PB, centroid, &lines[i]);
    }
    for (int i = 0; i < 4; i++) {
        R[i][0] = P[i][0]; R[i][1] = P[i][1];      /* 默认退回原角点 */
        int a = (i + 3) % 4, b = i;                /* 角 i = side(a) 与 side(b) 交点 */
        if (lines[a].valid && lines[b].valid)
            line_intersect(&lines[a], &lines[b], R[i]);
    }
}

/* ================= 候选 -> 角点 -> 解码 ================= */
typedef struct { int id; int32_t area; } Cand;

static int cand_cmp(const void *a, const void *b) {
    return ((const Cand *)b)->area - ((const Cand *)a)->area;
}

static bool comp_extremes(const CompStat *st, uint16_t id, int w, int h, float P[4][2]) {
    /* 主轴方向 u, 对角方向 d1 = u+v, d2 = u-v; 4 角点 = d1/d2 的极值点 */
    double cx = st->sx / st->area, cy = st->sy / st->area;
    double mu20 = st->sxx / st->area - cx * cx;
    double mu02 = st->syy / st->area - cy * cy;
    double mu11 = st->sxy / st->area - cx * cy;
    double th = 0.5 * atan2(2 * mu11, mu20 - mu02);
    double ux = cos(th), uy = sin(th);
    double d1x = ux - uy, d1y = ux + uy;     /* u + v (v = (-uy, ux)) */
    double d2x = ux + uy, d2y = uy - ux;

    double b1 = -1e18, b2 = 1e18, b3 = -1e18, b4 = 1e18;
    int p1 = -1, p2 = -1, p3 = -1, p4 = -1;
    for (int y = st->miny; y <= st->maxy; y++) {
        const uint16_t *lrow = g_sc.lab + (size_t)y * w;
        for (int x = st->minx; x <= st->maxx; x++) {
            if (lrow[x] != id) continue;
            double v1 = x * d1x + y * d1y;
            double v2 = x * d2x + y * d2y;
            if (v1 > b1) { b1 = v1; p1 = y * w + x; }
            if (v1 < b2) { b2 = v1; p2 = y * w + x; }
            if (v2 > b3) { b3 = v2; p3 = y * w + x; }
            if (v2 < b4) { b4 = v2; p4 = y * w + x; }
        }
    }
    if (p1 < 0 || p2 < 0 || p3 < 0 || p4 < 0) return false;
    P[0][0] = p1 % w; P[0][1] = p1 / w;
    P[1][0] = p2 % w; P[1][1] = p2 / w;
    P[2][0] = p3 % w; P[2][1] = p3 / w;
    P[3][0] = p4 % w; P[3][1] = p4 / w;
    return true;
}

/* ================= 对外入口 ================= */
bool TagDetector_Detect(const uint8_t *gray, int w, int h, int stride, TagObservation *out) {
    return TagDetector_DetectEx(gray, w, h, stride, out, NULL);
}

bool TagDetector_DetectEx(const uint8_t *gray, int w, int h, int stride,
                          TagObservation *out, TagDiag *diag) {
    memset(out, 0, sizeof(*out));
    out->ok = false;
    if (diag) {
        memset(diag, 0, sizeof(*diag));
        diag->best_hamming = 255;
        diag->stage = 1;
    }
    if (!gray || w <= 0 || h <= 0) return false;
    if (!scratch_ensure(w, h)) return false;

    threshold_adaptive(gray, w, h, stride);
    fill_holes(w, h);
    label_components(w, h);
    if (diag) diag->ncomps = g_ncomps;
    if (g_ncomps == 0) return false;

    /* 按面积取前几个候选 */
    Cand cands[DET_MAX_COMPS];
    int ncand = 0;
    for (int i = 1; i < DET_MAX_COMPS && i < 65535; i++) {
        if (g_sc.stats[i].area > 0) {
            cands[ncand].id = i;
            cands[ncand].area = g_sc.stats[i].area;
            ncand++;
        }
    }
    if (ncand == 0) return false;
    qsort(cands, ncand, sizeof(Cand), cand_cmp);
    if (diag) {
        diag->ncand = ncand;
        const CompStat *top = &g_sc.stats[cands[0].id];
        diag->cand_bbox[0] = top->minx;
        diag->cand_bbox[1] = top->miny;
        diag->cand_bbox[2] = top->maxx - top->minx + 1;
        diag->cand_bbox[3] = top->maxy - top->miny + 1;
        diag->cand_area = (int)top->area;
        int bw0 = top->maxx - top->minx + 1, bh0 = top->maxy - top->miny + 1;
        diag->cand_fill = (bw0 > 0 && bh0 > 0) ? (float)top->area / (float)(bw0 * bh0) : 0.0f;
        diag->stage = 2;
    }
    if (ncand > DET_MAX_CANDS) ncand = DET_MAX_CANDS;

    TagObservation best;
    int best_h = 255;

    for (int ci = 0; ci < ncand && best_h > DET_MAX_HAMMING; ci++) {
        const CompStat *st = &g_sc.stats[cands[ci].id];
        float bw = st->maxx - st->minx + 1, bh = st->maxy - st->miny + 1;
        /* 预筛: 旋转方形黑框的包围盒/面积特征 */
#ifdef DET_DEBUG
        fprintf(stderr, "[cand %d] id=%d area=%d bbox=(%.0f,%.0f) fill=%.2f\n",
                ci, cands[ci].id, (int)st->area, bw, bh,
                (double)st->area / (bw * bh));
#endif
        if (bw < 0.7f * DET_EDGE_PX_MIN || bh < 0.7f * DET_EDGE_PX_MIN) {
            if (diag) diag->drop_small++;
            continue;
        }
        if (bw > 1.6f * DET_EDGE_PX_MAX || bh > 1.6f * DET_EDGE_PX_MAX) {
            if (diag) diag->drop_large++;
            continue;
        }
        double fill = (double)st->area / (bw * bh);
        /* 下限 0.15: 环带形态(黑框+暗位连不齐)约 0.2+, 细碎轮廓线 <0.1;
         * 上限 0.99: 填洞后是实心方块 (正对时 fill≈0.98) */
        if (fill < 0.15 || fill > 0.99) {
            if (diag) diag->drop_fill++;
            continue;
        }
        double ar = bw / bh;
        if (ar < 0.4 || ar > 2.5) {
            if (diag) diag->drop_aspect++;
            continue;
        }
        if (diag) diag->decode_tried++;

        float P[4][2];
        float centroid[2] = {(float)(st->sx / st->area), (float)(st->sy / st->area)};
        if (!comp_extremes(st, cands[ci].id, w, h, P)) continue;

        /* 角点按绕形心方位角排序, 得到一致环序 */
        float ang[4];
        for (int i = 0; i < 4; i++)
            ang[i] = atan2f(P[i][1] - centroid[1], P[i][0] - centroid[0]);
        for (int i = 0; i < 3; i++)
            for (int j = i + 1; j < 4; j++)
                if (ang[j] < ang[i]) {
                    float t = ang[i]; ang[i] = ang[j]; ang[j] = t;
                    float tmp[2] = {P[i][0], P[i][1]};
                    P[i][0] = P[j][0]; P[i][1] = P[j][1];
                    P[j][0] = tmp[0]; P[j][1] = tmp[1];
                }

        float R[4][2];
        refine_corners(gray, w, h, stride, P, centroid, R);

        /* 2 种绕向 x 4 个起始角 = 8 种角点赋值, 取最小汉明 */
        const float *base[2][4] = {
            {R[0], R[1], R[2], R[3]},
            {R[0], R[3], R[2], R[1]},
        };
        for (int ori = 0; ori < 2 && best_h > DET_MAX_HAMMING; ori++) {
            for (int r = 0; r < 4 && best_h > DET_MAX_HAMMING; r++) {
                float A[4][2];
                for (int i = 0; i < 4; i++) {
                    const float *src = base[ori][(r + i) % 4];
                    A[i][0] = src[0]; A[i][1] = src[1];
                }
                TagObservation o;
                int ham = decode_attempt(gray, w, h, stride, A, &o);
                if (diag && ham < diag->best_hamming) {
                    diag->best_hamming = ham;
                    diag->best_contrast = o.contrast;
                    diag->best_margin = o.margin;
                }
                if (ham < best_h) { best_h = ham; best = o; }
            }
        }
    }

    if (best_h > DET_MAX_HAMMING) {
        if (diag) diag->stage = (diag->decode_tried > 0) ? 3 : 2;
        return false;
    }
    if (diag) diag->stage = 0;
    *out = best;
    return true;
}

/* ==================== align_measure.h ==================== */
/* align_measure.h - 视觉纠偏测量 (参考位管理 + 偏差解算)
 *
 * 测量模型移植自 视觉/code/tag_utils.py (俯视安装 v2):
 *   - 相机装夹爪正上方垂直向下拍, 标签贴小车顶面, c0->c1 边平行管道轴线
 *   - phi (周向滚转角): 标签中心垂直于管道方向的位移反解, phi = asin(perp/R_TAG_MM)
 *   - psi (机身歪头角): 面内角差 theta - theta0
 *   - axial (轴向进/退): 中心沿管道方向的位移
 *   - arc: 周向纠偏应走的弧长 = R_PIPE * phi
 */
#ifndef WALLBOT_ALIGN_MEASURE_H
#define WALLBOT_ALIGN_MEASURE_H

#include <stdint.h>
#include <stdbool.h>
#include "tag_detector.h"

/* 参考位 (放子机前、夹爪对中时拍) */
typedef struct {
    bool  valid;
    float u;        /* 参考中心 (像素) */
    float v;
    float theta;    /* 参考面内角 (弧度) = 图像里的管道轴线方向 */
    float edge_a;   /* 参考轴向边长 (像素) */
} AlignRefPose;

/* 偏差报告 (一次测量) */
typedef struct {
    float phi_deg;        /* 周向滚转角 (deg), 主判据 */
    float psi_deg;        /* 机身航向/歪头角 (deg) */
    float phi_shape_deg;  /* 边长收缩反解的滚转角 (无符号, 交叉验证) */
    float axial_mm;       /* 沿管道方向位移 (mm), 正负=进/退 (上机确认) */
    float perp_mm;        /* 垂直管道方向位移 (mm) */
    float arc_mm;         /* 周向纠偏应走弧长 (mm), 带方向 */
    float mm_per_px;      /* 当前帧比例尺 */
    bool  aligned;        /* |phi|,|psi| <= ALIGN_ANGLE_TOL 且 |axial| <= 轴向容差 */
} Deviation;

void             AlignMeasure_Init(void);      /* 从 NVS 加载参考位 */
const AlignRefPose *AlignMeasure_GetRef(void);
bool             AlignMeasure_SetRef(const TagObservation *obs);  /* 保存并写 NVS */
void             AlignMeasure_ClearRef(void);  /* 清除参考位 */
bool             AlignMeasure_Compute(const TagObservation *obs, Deviation *dev);

#endif /* WALLBOT_ALIGN_MEASURE_H */

/* ==================== align_measure.cpp ==================== */
/* align_measure.cpp - 视觉纠偏测量 (参考位管理 + 偏差解算)
 * 数学与 视觉/code/tag_utils.py 的 delta_report/arc_mm_for 一一对应。
 */
#include <Arduino.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>
#include "config.h"
#include "align_measure.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static AlignRefPose g_ref;
static Preferences g_nvs;

static float wrap_deg(float d) { return fmodf(d + 180.0f, 360.0f) - 180.0f; }

void AlignMeasure_Init(void) {
    memset(&g_ref, 0, sizeof(g_ref));
    g_nvs.begin(ALIGN_NVS_NS, true);
    g_ref.valid = g_nvs.getBool("valid", false);
    if (g_ref.valid) {
        g_ref.u      = g_nvs.getFloat("u", 0);
        g_ref.v      = g_nvs.getFloat("v", 0);
        g_ref.theta  = g_nvs.getFloat("th", 0);
        g_ref.edge_a = g_nvs.getFloat("ea", 0);
        Serial.printf("[ALIGN] 参考位已从NVS恢复: u=%.1f v=%.1f theta=%.3fdeg edge=%.1fpx\n",
                      g_ref.u, g_ref.v, g_ref.theta * 180.0f / (float)M_PI, g_ref.edge_a);
    }
    g_nvs.end();
}

const AlignRefPose *AlignMeasure_GetRef(void) { return &g_ref; }

bool AlignMeasure_SetRef(const TagObservation *obs) {
    if (!obs || !obs->ok) return false;
    g_ref.valid = true;
    g_ref.u = obs->u;
    g_ref.v = obs->v;
    g_ref.theta = obs->theta;
    g_ref.edge_a = obs->edge_a;

    g_nvs.begin(ALIGN_NVS_NS, false);
    g_nvs.putBool("valid", true);
    g_nvs.putFloat("u", g_ref.u);
    g_nvs.putFloat("v", g_ref.v);
    g_nvs.putFloat("th", g_ref.theta);
    g_nvs.putFloat("ea", g_ref.edge_a);
    g_nvs.end();
    return true;
}

void AlignMeasure_ClearRef(void) {
    g_ref.valid = false;
    g_nvs.begin(ALIGN_NVS_NS, false);
    g_nvs.putBool("valid", false);
    g_nvs.end();
}

bool AlignMeasure_Compute(const TagObservation *obs, Deviation *dev) {
    if (!obs || !obs->ok || !g_ref.valid || !dev) return false;

    /* 管道轴线方向由参考位 theta 定义, 自动补偿相机安装旋转 */
    float th = g_ref.theta;
    float dir_ax_x = cosf(th),  dir_ax_y = sinf(th);   /* 图像里管道轴线方向 */
    float dir_pp_x = -sinf(th), dir_pp_y = cosf(th);   /* 垂直管道方向 */

    float scale = TAG_REAL_MM / (obs->edge_a > 1e-3f ? obs->edge_a : 1e-3f);
    float du = obs->u - g_ref.u;
    float dv = obs->v - g_ref.v;
    float axial = (du * dir_ax_x + dv * dir_ax_y) * scale;
    float perp  = (du * dir_pp_x + dv * dir_pp_y) * scale;

    float perp_ratio = perp / R_TAG_MM;
    if (perp_ratio > 1.0f) perp_ratio = 1.0f;
    if (perp_ratio < -1.0f) perp_ratio = -1.0f;
    float phi = asinf(perp_ratio) * 180.0f / (float)M_PI;
    float psi = wrap_deg((obs->theta - g_ref.theta) * 180.0f / (float)M_PI);

    float ratio = obs->edge_p / (obs->edge_a > 1e-3f ? obs->edge_a : 1e-3f);
    if (ratio > 1.0f) ratio = 1.0f;
    if (ratio < -1.0f) ratio = -1.0f;
    float phi_shape = acosf(ratio) * 180.0f / (float)M_PI;

    dev->phi_deg = phi;
    dev->psi_deg = psi;
    dev->phi_shape_deg = phi_shape;
    dev->axial_mm = axial;
    dev->perp_mm = perp;
    dev->arc_mm = phi * (float)M_PI / 180.0f * PIPE_RADIUS_MM;
    dev->mm_per_px = scale;
    dev->aligned = (fabsf(phi) <= ALIGN_ANGLE_TOL &&
                    fabsf(psi) <= ALIGN_ANGLE_TOL &&
                    fabsf(axial) <= ALIGN_AXIAL_TOL_MM);
    return true;
}

/* ==================== log_page.h ==================== */
/* log_page.h - 内置网页日志页 (浏览器直接看, 不依赖 App) */
#ifndef WALLBOT_LOG_PAGE_H
#define WALLBOT_LOG_PAGE_H

#include "esp_http_server.h"

void LogPage_Register(httpd_handle_t httpd);

#endif /* WALLBOT_LOG_PAGE_H */

/* ==================== log_page.cpp ==================== */
/* log_page.cpp - 内置网页日志页 (GET /logs)
 *
 * 用途: 不依赖 App, 手机/电脑浏览器直接打开 http://<板子IP>/logs
 *       即可看到从开机起的日志, 每 1s 自动刷新, 并显示 WiFi/摄像头状态。
 *       /logs.txt 为纯文本接口。
 */
#include <Arduino.h>
#include <WiFi.h>
#include "esp_http_server.h"
#include <stdio.h>
#include <string.h>
#include "cam_module.h"
#include "log_page.h"
#include "stream_server.h"

static const char *PAGE_HEAD =
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>WallBot Camera Log</title>"
    "<style>body{margin:0;background:#111;color:#ddd;font:13px/1.5 ui-monospace,Menlo,monospace}"
    "#h{padding:10px 14px;background:#1c1c1e;border-bottom:1px solid #333;position:sticky;top:0}"
    "#s{color:#8e8e93;font-size:12px;margin-top:4px}"
    "#l{padding:10px 14px;white-space:pre-wrap;word-break:break-all}"
    "</style></head><body>"
    "<div id='h'><b>摄像头子机日志</b><div id='s'>加载中…</div></div>"
    "<div id='l'>…</div>"
    "<script>"
    "function st(){fetch('/status',{cache:'no-store'}).then(function(r){return r.text()})"
    ".then(function(t){document.getElementById('s').textContent=t;})['catch'](function(e){document.getElementById('s').textContent='状态获取失败: '+e;});}"
    "function lg(){fetch('/log',{cache:'no-store'}).then(function(r){return r.text()})"
    ".then(function(t){var d=document.getElementById('l');d.textContent=t||'(空)';})['catch'](function(e){"
    "document.getElementById('l').textContent='日志获取失败: '+e;});}"
    "st();lg();setInterval(st,2000);setInterval(lg,1000);"
    "</script></body></html>";

static esp_err_t logs_page_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, PAGE_HEAD, HTTPD_RESP_USE_STRLEN);
}

/* /status: 板子当前状态, 一眼看出是 WiFi 问题还是摄像头问题 */
static esp_err_t status_handler(httpd_req_t *req) {
    char buf[320];
    int n = snprintf(buf, sizeof(buf),
        "wifi=%s ip=%s rssi=%d ch=%d | cam ok=%d err=0x%x pid=0x%x detect=%d | heap=%u psram=%u | up=%us",
        (WiFi.status() == WL_CONNECTED) ? "connected" : "disconnected",
        WiFi.localIP().toString().c_str(),
        (int)WiFi.RSSI(), (int)WiFi.channel(),
        Cam_IsOk() ? 1 : 0, Cam_Err(), Cam_SensorPid(),
        Cam_InDetectMode() ? 1 : 0,
        (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getPsramSize(),
        (unsigned)(millis() / 1000));
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, buf, n);
}

void LogPage_Register(httpd_handle_t httpd) {
    if (!httpd) return;
    httpd_uri_t logs_uri = {.uri = "/logs", .method = HTTP_GET,
                            .handler = logs_page_handler, .user_ctx = NULL};
    httpd_uri_t status_uri = {.uri = "/status", .method = HTTP_GET,
                              .handler = status_handler, .user_ctx = NULL};
    httpd_register_uri_handler(httpd, &logs_uri);
    httpd_register_uri_handler(httpd, &status_uri);
    Stream_Log("log page ready: /logs  status: /status");
}

/* ==================== discovery.h ==================== */
/* discovery.h - UDP 自动发现 (母机/App 广播询问, 本机应答角色+IP) */
#ifndef WALLBOT_DISCOVERY_H
#define WALLBOT_DISCOVERY_H

void Discovery_Poll(void);              /* 主循环轮询, 应答发现请求 */
void Discovery_Reset(void);             /* WiFi 重连后重置 socket */

#endif /* WALLBOT_DISCOVERY_H */

/* ==================== discovery.cpp ==================== */
/* discovery.cpp - UDP 自动发现 (母机/App 广播询问, 本机应答角色+IP) */
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <string.h>
#include <stdio.h>
#include "config.h"
#include "discovery.h"

static WiFiUDP g_udp;
static bool g_started = false;

void Discovery_Reset(void) {
    g_udp.stop();
    g_started = false;
}

void Discovery_Poll(void) {
    if (!g_started) {
        if (WiFi.status() != WL_CONNECTED) return;
        if (!g_udp.begin(DISCOVERY_PORT)) {
            Serial.printf("[UDP] begin :%u failed\n", DISCOVERY_PORT);
            return;
        }
        g_started = true;
        Serial.printf("[UDP] discovery listen :%u\n", DISCOVERY_PORT);
    }

    int size = g_udp.parsePacket();
    if (size <= 0) return;

    char rx[64];
    if (size >= (int)sizeof(rx)) { g_udp.flush(); return; }
    int len = g_udp.read(rx, sizeof(rx) - 1);
    if (len <= 0) return;
    rx[len] = 0;
    if (strncmp(rx, DISCOVERY_REQ, strlen(DISCOVERY_REQ)) != 0) return;

    IPAddress rip = g_udp.remoteIP();
    uint16_t rport = g_udp.remotePort();
    char reply[48];
    int n = snprintf(reply, sizeof(reply), "WALLBOT:%s:%s", DEVICE_ROLE,
                     WiFi.localIP().toString().c_str());
    g_udp.beginPacket(rip, rport);
    g_udp.write((const uint8_t *)reply, n);
    g_udp.endPacket();
    Serial.printf("[UDP] reply %s -> %s\n", reply, rip.toString().c_str());
}

/* ==================== 摄像头子机.ino ==================== */
/* 摄像头子机 - 主入口
 * ESP32-S3-CAM (Hiwonder), 俯视安装于母机夹爪处。
 *
 * 职责 (实现按模块分文件, 参数集中在 config.h):
 *   cam_module     摄像头初始化 + 检测模式切换
 *   discovery      UDP 自动发现 (:48899, 角色 camchild)
 *   stream_server  MJPEG 流 :81/stream + :80 /jpg /fps / 诊断
 *   tag_detector   板端 AprilTag 36h11 检测
 *   align_measure  视觉纠偏测量 (参考位 + 偏差解算)
 *   align_server   纠偏 API: /align/ref /align/get /align/status
 *
 * 纠偏闭环 (执行文档 6.3): 母机松爪前调 /align/ref 记参考位;
 * 子机回程到预对接点停稳后轮询 /align/get, 按 phi 发 MV、按 psi 发 SPIN,
 * 三角全部 <=4° 判据达标再夹取。
 */
#include <WiFi.h>
#include "config.h"
#include "cam_module.h"
#include "discovery.h"
#include "stream_server.h"
#include "align_measure.h"
#include "align_server.h"
#include "stream_server.h"
#include "log_page.h"

static void wifi_connect() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("Connecting to %s", WIFI_SSID);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Stream_Log("WiFi ok ip=%s", WiFi.localIP().toString().c_str());
    } else {
        Stream_Log("ERR WiFi timeout, retry in loop");
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println();

    Stream_Log("=== boot ===");
    Stream_Log("chip=%s cores=%d flash=%uMB psram=%uKB",
               ESP.getChipModel(), (int)ESP.getChipCores(),
               (unsigned)(ESP.getFlashChipSize() / (1024 * 1024)),
               (unsigned)(ESP.getPsramSize() / 1024));
    Stream_Log("ssid=%s build=%s %s", WIFI_SSID, __DATE__, __TIME__);

    AlignMeasure_Init();          /* 先恢复参考位 (NVS) */

    Cam_Init();
    if (!Cam_IsOk()) {
        Stream_Log("ERR Camera init failed err=0x%x", Cam_Err());
    } else {
        Stream_Log("Cam ok pid=0x%x %ums jpeg=%d",
                   Cam_SensorPid(), (unsigned)Cam_InitMs(), Cam_JpegOk() ? 1 : 0);
    }

    wifi_connect();

    if (StreamServer_Start()) {
        Stream_Log("HTTP ready :80 /logs /status /view /log /align/*, :81 /stream");
    } else {
        Stream_Log("ERR HTTP server start failed");
    }
}

void loop() {
    static uint32_t last_check = 0;
    uint32_t now = millis();

    // WiFi watchdog: reconnect if dropped
    if (now - last_check > 3000) {
        last_check = now;
        if (WiFi.status() != WL_CONNECTED) {
            Stream_Log("WiFi lost, reconnecting...");
            WiFi.disconnect();
            WiFi.begin(WIFI_SSID, WIFI_PASS);
            Discovery_Reset();
        }
    }
    Discovery_Poll();
    delay(20);
}

