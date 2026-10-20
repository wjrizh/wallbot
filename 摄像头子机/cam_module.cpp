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
    config.fb_count = STREAM_FB_COUNT;
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

    /* 不再切 framesize: 分辨率固定 VGA, 检测侧靠解码缩放得到 320x240。
     * 这里只是把"当前有检测使用者"的状态记下来, 供诊断页展示。 */
    Stream_Log("detect mode ON (vga fixed)");
    return true;
}

void Cam_ExitDetectMode(void) {
    bool released = false;
    portENTER_CRITICAL(&s_mode_mux);
    if (s_detect_mode) {
        if (s_detect_refs > 0) s_detect_refs--;
        if (s_detect_refs == 0) {
            s_detect_mode = false;
            released = true;
        }
    }
    portEXIT_CRITICAL(&s_mode_mux);
    if (released) Stream_Log("detect mode OFF");
}

void Cam_FlushFrames(int n, int delay_ms) {
    for (int i = 0; i < n; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) esp_camera_fb_return(fb);
        if (delay_ms > 0) delay(delay_ms);
    }
}
