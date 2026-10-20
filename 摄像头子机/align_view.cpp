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
