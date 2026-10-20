/* T3: T2 全部 + 8KB 栈后台任务调用 TagDetector_DetectEx(未修复的 32KB 栈数组版).
 * 输入为合成的 320x240 灰度图(带一个方块), 不依赖相机解码, 隔离变量。 */
#include <WiFi.h>
#include <esp_http_server.h>
#include <esp_camera.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "tag_detector.h"

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

static httpd_handle_t s_httpd = NULL;
static httpd_handle_t s_stream_httpd = NULL;
static bool s_cam_ok = false;
static volatile float g_fps = 0;
static volatile uint32_t g_stream_starts = 0;
static volatile uint32_t g_det_runs = 0;
static volatile uint32_t g_det_ok = 0;
static volatile int      g_det_stage = -1;
static volatile bool     g_det_enable = false;

static const char *STREAM_CT = "multipart/x-mixed-replace;boundary=123456789000000000000987654321";
static const char *STREAM_BOUND = "\r\n--123456789000000000000987654321\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static uint8_t *s_gray = NULL;

static void cam_init() {
    camera_config_t c = {};
    c.ledc_channel = LEDC_CHANNEL_0; c.ledc_timer = LEDC_TIMER_0;
    c.pin_d0=Y2_GPIO_NUM; c.pin_d1=Y3_GPIO_NUM; c.pin_d2=Y4_GPIO_NUM; c.pin_d3=Y5_GPIO_NUM;
    c.pin_d4=Y6_GPIO_NUM; c.pin_d5=Y7_GPIO_NUM; c.pin_d6=Y8_GPIO_NUM; c.pin_d7=Y9_GPIO_NUM;
    c.pin_xclk=XCLK_GPIO_NUM; c.pin_pclk=PCLK_GPIO_NUM;
    c.pin_vsync=VSYNC_GPIO_NUM; c.pin_href=HREF_GPIO_NUM;
    c.pin_sccb_sda=SIOD_GPIO_NUM; c.pin_sccb_scl=SIOC_GPIO_NUM;
    c.pin_pwdn=PWDN_GPIO_NUM; c.pin_reset=RESET_GPIO_NUM;
    c.xclk_freq_hz=20000000; c.pixel_format=PIXFORMAT_JPEG;
    c.frame_size=FRAMESIZE_VGA; c.jpeg_quality=12; c.fb_count=3;
    c.fb_location=CAMERA_FB_IN_PSRAM; c.grab_mode=CAMERA_GRAB_LATEST;
    esp_err_t e = esp_camera_init(&c);
    s_cam_ok = (e == ESP_OK);
    Serial.printf("T3 cam err=0x%x ok=%d\n", e, s_cam_ok ? 1 : 0);
}

/* 8KB 栈任务: 调未修复版 TagDetector_DetectEx */
static void det_task(void *arg) {
    (void)arg;
    const int W = 320, H = 240;
    s_gray = (uint8_t *)heap_caps_malloc((size_t)W * H, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_gray) s_gray = (uint8_t *)malloc((size_t)W * H);
    if (!s_gray) { Serial.println("T3 gray alloc FAIL"); vTaskDelete(NULL); }
    Serial.println("T3 det_task started");
    for (;;) {
        if (!g_det_enable) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }
        /* 合成图: 灰底 + 中间偏黑方块 */
        memset(s_gray, 180, (size_t)W * H);
        for (int y = 80; y < 160; y++)
            for (int x = 120; x < 200; x++)
                s_gray[y * W + x] = 30;
        TagObservation o;
        TagDiag d;
        g_det_runs++;
        bool ok = TagDetector_DetectEx(s_gray, W, H, W, &o, &d);
        g_det_stage = d.stage;
        if (ok) g_det_ok++;
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

static esp_err_t stream_handler(httpd_req_t *req) {
    g_stream_starts++;
    httpd_resp_set_type(req, STREAM_CT);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    size_t cap = 128 * 1024;
    char *buf = (char *)heap_caps_malloc(cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) buf = (char *)malloc(cap);
    if (!buf) return ESP_FAIL;
    uint32_t frames = 0, t0 = millis();
    char part[64];
    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }
        frames++;
        uint32_t now = millis();
        if (now - t0 >= 1000) { g_fps = frames * 1000.0f / (now - t0); frames = 0; t0 = now; }
        size_t hlen = snprintf(part, sizeof(part), STREAM_PART, (unsigned)fb->len);
        size_t blen = strlen(STREAM_BOUND);
        size_t total = blen + hlen + fb->len;
        if (total > cap) { esp_camera_fb_return(fb); continue; }
        memcpy(buf, STREAM_BOUND, blen);
        memcpy(buf + blen, part, hlen);
        memcpy(buf + blen + hlen, fb->buf, fb->len);
        esp_camera_fb_return(fb);
        if (httpd_resp_send_chunk(req, buf, total) != ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    free(buf);
    return ESP_OK;
}

static esp_err_t root_handler(httpd_req_t *req) {
    const char *page =
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'><title>T3</title></head>"
        "<body style='background:#111;color:#eee;font:16px sans-serif;padding:16px'>"
        "<h2>T3: 摄像头 + 推流 + 8KB栈检测(未修复)</h2>"
        "<div id='s'>…</div>"
        "<button onclick=\"fetch('/det?on=1')\">开启检测</button>"
        "<button onclick=\"fetch('/det?on=0')\">关闭检测</button>"
        "<img id='pic' style='width:100%;max-width:640px;background:#000;border-radius:8px'>"
        "<script>"
        "function st(){fetch('/status',{cache:'no-store'}).then(r=>r.text())"
        ".then(t=>{document.getElementById('s').textContent=t;})}"
        "setTimeout(()=>{document.getElementById('pic').src=location.protocol+'//'+location.hostname+':81/stream';},300);"
        "st();setInterval(st,1000);"
        "</script></body></html>";
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t det_handler(httpd_req_t *req) {
    char q[32] = {0};
    httpd_req_get_url_query_str(req, q, sizeof(q) - 1);
    g_det_enable = (strstr(q, "on=1") != NULL);
    char buf[64];
    int n = snprintf(buf, sizeof(buf), "det_enable=%d", g_det_enable ? 1 : 0);
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, buf, n);
}

static esp_err_t status_handler(httpd_req_t *req) {
    char buf[320];
    int n = snprintf(buf, sizeof(buf),
        "up=%us heap=%u psram=%u cam=%d fps=%.1f starts=%u | det_en=%d runs=%u ok=%u stage=%d",
        (unsigned)(millis()/1000), (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getPsramSize(),
        s_cam_ok ? 1 : 0, (double)g_fps, (unsigned)g_stream_starts,
        g_det_enable ? 1 : 0, (unsigned)g_det_runs, (unsigned)g_det_ok, g_det_stage);
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, buf, n);
}

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.printf("\nT3 boot rst=%d\n", (int)esp_reset_reason());
    cam_init();

    WiFi.persistent(false);
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
    delay(100);
    WiFi.mode(WIFI_AP);
    WiFi.setTxPower(WIFI_POWER_11dBm);
    WiFi.setSleep(false);
    bool up = WiFi.softAP("wallbot-t3", NULL, 6, 0, 4);
    delay(100);
    IPAddress ip(192,168,4,1);
    WiFi.softAPConfig(ip, ip, IPAddress(255,255,255,0));
    delay(200);
    Serial.printf("T3 AP up=%d ip=%s\n", up?1:0, WiFi.softAPIP().toString().c_str());

    /* 关键: 8KB 栈任务, 与主固件 align_worker_task 一致 */
    xTaskCreatePinnedToCore(det_task, "det", 8192, NULL, 2, NULL, 0);

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 8192; cfg.max_uri_handlers = 8; cfg.server_port = 80;
    if (httpd_start(&s_httpd, &cfg) == ESP_OK) {
        httpd_uri_t r = {.uri="/", .method=HTTP_GET, .handler=root_handler, .user_ctx=NULL};
        httpd_uri_t s = {.uri="/status", .method=HTTP_GET, .handler=status_handler, .user_ctx=NULL};
        httpd_uri_t d = {.uri="/det", .method=HTTP_GET, .handler=det_handler, .user_ctx=NULL};
        httpd_register_uri_handler(s_httpd, &r);
        httpd_register_uri_handler(s_httpd, &s);
        httpd_register_uri_handler(s_httpd, &d);
    }
    cfg.server_port = 81; cfg.ctrl_port = 32769;
    if (httpd_start(&s_stream_httpd, &cfg) == ESP_OK) {
        httpd_uri_t st = {.uri="/stream", .method=HTTP_GET, .handler=stream_handler, .user_ctx=NULL};
        httpd_register_uri_handler(s_stream_httpd, &st);
    }
    Serial.println("T3 setup done");
}

void loop() { delay(100); }
