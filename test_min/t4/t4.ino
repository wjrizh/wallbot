/* T4: AP + 纯净推流 + 真实相机帧解码检测 + 页面底部数据栏.
 * 图像不做任何叠加; 识别数据只走 /align/get 显示在网页底部。
 * 检测在独立 8KB 栈任务里跑, 推流循环顺手投喂一帧。 */
#include <WiFi.h>
#include <esp_http_server.h>
#include <esp_camera.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <string.h>
#include "jpeg_decoder.h"
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

#define DET_W 320
#define DET_H 240
#define FEED_MAX (48 * 1024)

static httpd_handle_t s_httpd = NULL;
static httpd_handle_t s_stream_httpd = NULL;
static bool s_cam_ok = false;
static volatile float g_fps = 0;
static volatile uint32_t g_stream_starts = 0;

/* 投喂槽 */
static uint8_t *s_feed = NULL;
static size_t   s_feed_len = 0;
static int      s_feed_w = 0, s_feed_h = 0;
static volatile bool s_want = false, s_has = false;
static volatile uint32_t s_last_feed_ms = 0;
static portMUX_TYPE s_feed_mux = portMUX_INITIALIZER_UNLOCKED;

/* 解码缓冲 */
static uint8_t *s_rgb = NULL, *s_gray = NULL;
static SemaphoreHandle_t s_det_lock = NULL;

/* 检测结果缓存 */
static portMUX_TYPE s_res_mux = portMUX_INITIALIZER_UNLOCKED;
static bool  s_res_ok = false;
static char  s_res_reason[24] = "no_tag";
static int   s_res_id = -1, s_res_ham = -1;
static float s_res_u = 0, s_res_v = 0, s_res_theta = 0, s_res_edge = 0;
static float s_res_corners[4][2];
static uint32_t s_res_dt = 0, s_res_seq = 0;

/* 诊断 */
static volatile uint32_t g_frames = 0, g_nofeed = 0, g_decfail = 0, g_detok = 0, g_detfail = 0;
static volatile int g_derr = 0, g_ow = 0, g_oh = 0;

static const char *STREAM_CT = "multipart/x-mixed-replace;boundary=123456789000000000000987654321";
static const char *STREAM_BOUND = "\r\n--123456789000000000000987654321\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

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
    Serial.printf("T4 cam err=0x%x ok=%d\n", e, s_cam_ok ? 1 : 0);
}

static bool decode_gray(const uint8_t *jpg, size_t jlen, int fw, int fh,
                        const uint8_t **out_gray) {
    if (!jpg || !jlen) return false;
    esp_jpeg_image_scale_t scale;
    if (fw == DET_W && fh == DET_H) scale = JPEG_IMAGE_SCALE_0;
    else if (fw == DET_W * 2 && fh == DET_H * 2) scale = JPEG_IMAGE_SCALE_1_2;
    else return false;

    if (!s_rgb) {
        s_rgb = (uint8_t *)heap_caps_malloc((size_t)DET_W*DET_H*3, MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
        if (!s_rgb) s_rgb = (uint8_t *)malloc((size_t)DET_W*DET_H*3);
    }
    if (!s_gray) {
        s_gray = (uint8_t *)heap_caps_malloc((size_t)DET_W*DET_H, MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
        if (!s_gray) s_gray = (uint8_t *)malloc((size_t)DET_W*DET_H);
    }
    if (!s_rgb || !s_gray) return false;

    esp_jpeg_image_cfg_t cfg = {};
    cfg.indata = (uint8_t *)jpg;
    cfg.indata_size = (uint32_t)jlen;
    cfg.outbuf = s_rgb;
    cfg.outbuf_size = (uint32_t)(DET_W*DET_H*3);
    cfg.out_format = JPEG_IMAGE_FORMAT_RGB888;
    cfg.out_scale = scale;
    cfg.advanced.working_buffer = NULL;
    cfg.advanced.working_buffer_size = 0;

    esp_jpeg_image_output_t out = {};
    esp_err_t e = esp_jpeg_decode(&cfg, &out);
    g_derr = (int)e; g_ow = out.width; g_oh = out.height;
    if (e != ESP_OK) return false;
    if (out.width != DET_W || out.height != DET_H) return false;

    size_t npix = (size_t)DET_W * DET_H;
    for (size_t i = 0; i < npix; i++) {
        const uint8_t *p = &s_rgb[i*3];
        s_gray[i] = (uint8_t)((p[0]*77 + p[1]*151 + p[2]*28) >> 8);
    }
    *out_gray = s_gray;
    return true;
}

static void det_task(void *arg) {
    (void)arg;
    s_feed = (uint8_t *)heap_caps_malloc(FEED_MAX, MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    if (!s_feed) s_feed = (uint8_t *)malloc(FEED_MAX);
    Serial.printf("T4 det_task start feed=%p\n", s_feed);
    if (!s_feed) vTaskDelete(NULL);

    for (;;) {
        portENTER_CRITICAL(&s_feed_mux);
        s_has = false; s_want = true;
        portEXIT_CRITICAL(&s_feed_mux);

        uint32_t w0 = millis();
        while ((millis() - w0) < 100) {
            portENTER_CRITICAL(&s_feed_mux);
            bool has = s_has;
            portEXIT_CRITICAL(&s_feed_mux);
            if (has) break;
            if ((millis() - s_last_feed_ms) > 300) break;
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        size_t len = 0; int fw = 0, fh = 0;
        portENTER_CRITICAL(&s_feed_mux);
        bool has = s_has; s_want = false;
        if (has) { len = s_feed_len; fw = s_feed_w; fh = s_feed_h; }
        portEXIT_CRITICAL(&s_feed_mux);

        if (!has) {
            camera_fb_t *fb = esp_camera_fb_get();
            if (fb) {
                if (fb->format == PIXFORMAT_JPEG && fb->len <= FEED_MAX) {
                    memcpy(s_feed, fb->buf, fb->len);
                    len = fb->len; fw = fb->width; fh = fb->height; has = true;
                }
                esp_camera_fb_return(fb);
            }
        }
        if (!has) { g_nofeed++; vTaskDelay(pdMS_TO_TICKS(20)); continue; }
        g_frames++;

        uint32_t t0 = millis();
        TagObservation o; TagDiag d;
        memset(&o, 0, sizeof(o)); memset(&d, 0, sizeof(d));
        bool got = false;
        if (xSemaphoreTake(s_det_lock, pdMS_TO_TICKS(500)) == pdTRUE) {
            const uint8_t *gray = NULL;
            if (decode_gray(s_feed, len, fw, fh, &gray)) {
                got = TagDetector_DetectEx(gray, DET_W, DET_H, DET_W, &o, &d);
                if (got) g_detok++; else g_detfail++;
            } else {
                g_decfail++;
            }
            xSemaphoreGive(s_det_lock);
        }
        uint32_t dt = millis() - t0;

        portENTER_CRITICAL(&s_res_mux);
        s_res_dt = dt; s_res_seq++;
        if (got) {
            s_res_ok = true;
            strcpy(s_res_reason, "ok");
            s_res_id = o.id; s_res_ham = o.hamming;
            s_res_u = o.u; s_res_v = o.v; s_res_theta = o.theta; s_res_edge = o.edge_a;
            memcpy(s_res_corners, o.corners, sizeof(s_res_corners));
        } else {
            s_res_ok = false;
            strcpy(s_res_reason, "no_tag");
        }
        portEXIT_CRITICAL(&s_res_mux);

        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

static esp_err_t stream_handler(httpd_req_t *req) {
    g_stream_starts++;
    httpd_resp_set_type(req, STREAM_CT);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    size_t cap = 128 * 1024;
    char *buf = (char *)heap_caps_malloc(cap, MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    if (!buf) buf = (char *)malloc(cap);
    if (!buf) return ESP_FAIL;
    uint32_t frames = 0, t0 = millis();
    char part[64];
    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }
        /* 顺手投喂 (纯净: 不改动 fb->buf) */
        if (fb->format == PIXFORMAT_JPEG) {
            s_last_feed_ms = millis();
            portENTER_CRITICAL(&s_feed_mux);
            if (s_want && !s_has && fb->len <= FEED_MAX) {
                memcpy(s_feed, fb->buf, fb->len);
                s_feed_len = fb->len; s_feed_w = fb->width; s_feed_h = fb->height;
                s_has = true; s_want = false;
            }
            portEXIT_CRITICAL(&s_feed_mux);
        }
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
        "<meta name='viewport' content='width=device-width,initial-scale=1'><title>WallBot Cam</title>"
        "<style>body{margin:0;background:#111;color:#eee;font:14px sans-serif}"
        "#pic{width:100%;display:block;background:#000}"
        "#bar{padding:12px;background:#1a1a1d;border-top:1px solid #333}"
        "#bar h3{margin:0 0 8px;font-size:12px;color:#8e8e93;letter-spacing:.5px}"
        "table{width:100%;border-collapse:collapse;font:13px ui-monospace,monospace}"
        "td{padding:5px 6px;border-bottom:1px solid #232326}"
        "td.k{color:#8e8e93;width:38%}td.v{text-align:right}"
        ".good{color:#4ade80}.bad{color:#ff6b6b}"
        "#st{padding:8px 12px;font-size:12px;color:#8e8e93}"
        "</style></head><body>"
        "<img id='pic' style='width:100%;display:block;background:#000'>"
        "<div id='st'>连接中…</div>"
        "<div id='bar'><h3>识别数据 /align/get</h3>"
        "<table>"
        "<tr><td class='k'>状态</td><td class='v' id='f_st'>--</td></tr>"
        "<tr><td class='k'>Tag ID</td><td class='v' id='f_id'>--</td></tr>"
        "<tr><td class='k'>汉明距</td><td class='v' id='f_ham'>--</td></tr>"
        "<tr><td class='k'>中心 u</td><td class='v' id='f_u'>--</td></tr>"
        "<tr><td class='k'>中心 v</td><td class='v' id='f_v'>--</td></tr>"
        "<tr><td class='k'>角度 theta</td><td class='v' id='f_th'>--</td></tr>"
        "<tr><td class='k'>边长 px</td><td class='v' id='f_edge'>--</td></tr>"
        "<tr><td class='k'>耗时</td><td class='v' id='f_dt'>--</td></tr>"
        "</table></div>"
        "<script>"
        "var B=location.protocol+'//'+location.hostname,bA=0,bS=0;"
        "function n(v,d){return (v===undefined||v===null)?'--':Number(v).toFixed(d);}"
        "function st(){if(bS)return;bS=1;fetch(B+'/status',{cache:'no-store'}).then(r=>r.text())"
        ".then(t=>{bS=0;document.getElementById('st').textContent=t;})"
        ".catch(e=>{bS=0;document.getElementById('st').textContent='状态失败';});}"
        "function ag(){if(bA)return;bA=1;fetch(B+'/align/get',{cache:'no-store'}).then(r=>r.json())"
        ".then(d=>{bA=0;var s=document.getElementById('f_st');"
        "document.getElementById('f_id').textContent=d.id;"
        "document.getElementById('f_ham').textContent=d.hamming;"
        "document.getElementById('f_u').textContent=n(d.u,1);"
        "document.getElementById('f_v').textContent=n(d.v,1);"
        "document.getElementById('f_th').textContent=n(d.theta_deg,2)+' °';"
        "document.getElementById('f_edge').textContent=n(d.edge_px,1);"
        "document.getElementById('f_dt').textContent=n(d.dt_ms,0)+' ms';})"
        ".catch(e=>{bA=0;var s=document.getElementById('f_st');s.textContent='不可达';s.className='v bad';});}"
        "setTimeout(()=>{document.getElementById('pic').src=B+':81/stream';},300);"
        "st();ag();setInterval(st,2000);setInterval(ag,700);"
        "</script></body></html>";
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t align_get_handler(httpd_req_t *req) {
    char buf[320];
    bool ok; char reason[24]; int id, ham; float u, v, th, edge; uint32_t dt, seq;
    float corners[4][2];
    portENTER_CRITICAL(&s_res_mux);
    ok = s_res_ok; strncpy(reason, s_res_reason, sizeof(reason)-1); reason[sizeof(reason)-1]=0;
    id = s_res_id; ham = s_res_ham; u = s_res_u; v = s_res_v;
    th = s_res_theta; edge = s_res_edge; dt = s_res_dt; seq = s_res_seq;
    memcpy(corners, s_res_corners, sizeof(corners));
    portEXIT_CRITICAL(&s_res_mux);
    (void)seq;
    int n;
    if (!ok) {
        n = snprintf(buf, sizeof(buf), "{\"ok\":false,\"reason\":\"%s\",\"dt_ms\":%u}",
                     reason, (unsigned)dt);
    } else {
        n = snprintf(buf, sizeof(buf),
            "{\"ok\":true,\"dt_ms\":%u,\"id\":%d,\"hamming\":%d,"
            "\"u\":%.1f,\"v\":%.1f,\"theta_deg\":%.2f,\"edge_px\":%.1f,"
            "\"dw\":%d,\"dh\":%d,"
            "\"corners\":[[%.1f,%.1f],[%.1f,%.1f],[%.1f,%.1f],[%.1f,%.1f]]}",
            (unsigned)dt, id, ham, (double)u, (double)v,
            (double)(th * 180.0f / (float)M_PI), (double)edge,
            DET_W, DET_H,
            (double)corners[0][0], (double)corners[0][1],
            (double)corners[1][0], (double)corners[1][1],
            (double)corners[2][0], (double)corners[2][1],
            (double)corners[3][0], (double)corners[3][1]);
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, buf, n);
}

static esp_err_t status_handler(httpd_req_t *req) {
    char buf[384];
    int n = snprintf(buf, sizeof(buf),
        "up=%us heap=%u psram=%u cam=%d fps=%.1f starts=%u | fr=%u nofeed=%u decfail=%u ok=%u fail=%u derr=%d ow=%d oh=%d",
        (unsigned)(millis()/1000), (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getPsramSize(),
        s_cam_ok?1:0, (double)g_fps, (unsigned)g_stream_starts,
        (unsigned)g_frames, (unsigned)g_nofeed, (unsigned)g_decfail,
        (unsigned)g_detok, (unsigned)g_detfail, g_derr, g_ow, g_oh);
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, buf, n);
}

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.printf("\nT4 boot rst=%d\n", (int)esp_reset_reason());
    cam_init();
    s_det_lock = xSemaphoreCreateMutex();

    WiFi.persistent(false);
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
    delay(100);
    WiFi.mode(WIFI_AP);
    WiFi.setTxPower(WIFI_POWER_11dBm);
    WiFi.setSleep(false);
    bool up = WiFi.softAP("wallbot-t4", NULL, 6, 0, 4);
    delay(100);
    IPAddress ip(192,168,4,1);
    WiFi.softAPConfig(ip, ip, IPAddress(255,255,255,0));
    delay(200);
    Serial.printf("T4 AP up=%d ip=%s\n", up?1:0, WiFi.softAPIP().toString().c_str());

    xTaskCreatePinnedToCore(det_task, "det", 8192, NULL, 2, NULL, 0);

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 12288; cfg.max_uri_handlers = 8; cfg.server_port = 80;
    if (httpd_start(&s_httpd, &cfg) == ESP_OK) {
        httpd_uri_t r = {.uri="/", .method=HTTP_GET, .handler=root_handler, .user_ctx=NULL};
        httpd_uri_t s = {.uri="/status", .method=HTTP_GET, .handler=status_handler, .user_ctx=NULL};
        httpd_uri_t a = {.uri="/align/get", .method=HTTP_GET, .handler=align_get_handler, .user_ctx=NULL};
        httpd_register_uri_handler(s_httpd, &r);
        httpd_register_uri_handler(s_httpd, &s);
        httpd_register_uri_handler(s_httpd, &a);
        Serial.println("T4 :80 ok");
    }
    cfg.server_port = 81; cfg.ctrl_port = 32769;
    if (httpd_start(&s_stream_httpd, &cfg) == ESP_OK) {
        httpd_uri_t st = {.uri="/stream", .method=HTTP_GET, .handler=stream_handler, .user_ctx=NULL};
        httpd_register_uri_handler(s_stream_httpd, &st);
        Serial.println("T4 :81 ok");
    }
    Serial.println("T4 setup done");
}

void loop() { delay(100); }
