// MJPEG stream server for ESP32-S3-CAM (Hiwonder), STA mode + static IP.
// Sensor: OV3660 (hardware JPEG), same camera config as the master board.
// Endpoints:
//   GET /       -> diagnostics (cam_init/pid/psram)
//   GET /jpg    -> single JPEG frame
//   GET /fps    -> current fps
//   GET :81/stream -> multipart/x-mixed-replace MJPEG
#include "esp_camera.h"
#include "img_converters.h"
#include <WiFi.h>
#include "esp_http_server.h"

// ---- WiFi ----
static const char *WIFI_SSID = "robot";
static const char *WIFI_PASS = "12345678";
static IPAddress STATIC_IP(10, 32, 7, 215);
static IPAddress STATIC_GW(10, 32, 7, 1);
static IPAddress STATIC_MASK(255, 255, 255, 0);
static IPAddress STATIC_DNS(10, 32, 7, 1);
// NOTE: 平板 robot 热点网段为 10.32.7.x/24, 母机=10.32.7.214, 子机=10.32.7.215
// 同网段通信不依赖网关, 若网关实际不是 .1 也不影响板子与平板互通

// ---- Camera pins: CAMERA_MODEL_ESP32S3_EYE ----
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
            res = ESP_FAIL;
            break;
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
            // non-JPEG sensor -> JPEG soft encode
            if (!frame2jpg(fb, 12, &jpg_buf, &jpg_len)) {
                esp_camera_fb_return(fb);
                res = ESP_FAIL;
                break;
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

static bool g_cam_ok = false;
static int g_cam_err = 0;
static int g_cam_err2 = 0;
static bool g_jpeg_ok = false;
static int g_sensor_pid = -1;

static esp_err_t root_handler(httpd_req_t *req) {
    char buf[256];
    camera_fb_t *fb = esp_camera_fb_get();
    int fb_ok = fb ? 1 : 0;
    size_t fb_len = fb ? fb->len : 0;
    int fb_fmt = fb ? (int)fb->format : -1;
    if (fb) esp_camera_fb_return(fb);
    int n = snprintf(buf, sizeof(buf),
        "cam_init=%d cam_err=0x%x jpeg_ok=%d pid=0x%x err2=0x%x fb_get=%d fb_len=%u fb_fmt=%d psram=%u heap=%u\n",
        g_cam_ok ? 1 : 0, g_cam_err, g_jpeg_ok ? 1 : 0, g_sensor_pid, g_cam_err2,
        fb_ok, (unsigned)fb_len, fb_fmt,
        (unsigned)ESP.getPsramSize(), (unsigned)ESP.getFreeHeap());
    httpd_resp_set_type(req, "text/plain");
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

static bool start_servers() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

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

    // Stream server on :81
    config.server_port = 81;
    config.ctrl_port = 32769;
    if (httpd_start(&s_stream_httpd, &config) != ESP_OK) return false;
    httpd_register_uri_handler(s_stream_httpd, &stream_uri);
    return true;
}

static bool init_camera() {
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
    config.frame_size = FRAMESIZE_HD;
    config.jpeg_quality = 11;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;

    esp_err_t err = esp_camera_init(&config);
    g_cam_err = (int)err;
    if (err != ESP_OK) return false;
    sensor_t *s = esp_camera_sensor_get();
    g_sensor_pid = s ? s->id.PID : -1;
    if (s) {
        // OV3660/OV5640/OV2640 output is flipped vertically by default
        if (s->id.PID == OV3660_PID || s->id.PID == OV5640_PID || s->id.PID == OV2640_PID) {
            s->set_vflip(s, 1);
        }
        // GC2145 default output is mirrored horizontally
        if (s->id.PID == GC2145_PID) {
            s->set_hmirror(s, 1);
        }
        s->set_brightness(s, 0);
        s->set_contrast(s, 0);
        s->set_saturation(s, 0);
    }
    return true;
}

static void wifi_connect() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    if (!WiFi.config(STATIC_IP, STATIC_GW, STATIC_MASK, STATIC_DNS)) {
        Serial.println("WiFi.config failed");
    }
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("Connecting to %s", WIFI_SSID);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start <20000) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Camera Ready! http://");
        Serial.print(WiFi.localIP());
        Serial.println(":81/stream");
    } else {
        Serial.println("WiFi connect timeout, will retry in loop");
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println();

    g_cam_ok = init_camera();
    if (!g_cam_ok) {
        Serial.println("Camera init failed");
    } else {
        Serial.println("Camera init ok");
    }

    wifi_connect();

    if (start_servers()) {
        Serial.println("HTTP servers started (:80 /jpg, :81 /stream)");
    } else {
        Serial.println("HTTP server start failed");
    }
}

void loop() {
    static uint32_t last_check = 0;
    uint32_t now = millis();

    // WiFi watchdog: reconnect if dropped
    if (now - last_check > 3000) {
        last_check = now;
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi lost, reconnecting...");
            WiFi.disconnect();
            WiFi.begin(WIFI_SSID, WIFI_PASS);
        }
    }
    delay(1000);
}