/*
 * imu_web_test.ino - 独立 IMU (JY62) 网页测试 (ESP32-S3)
 * 完全独立: 不依赖主固件任何文件。
 * 接线: IMU TX -> GPIO17 (本板 RX)
 *       IMU RX -> GPIO18 (本板 TX, 只发不收可悬空)
 * STA 连热点 robot, 静态 IP 10.32.7.217
 * 浏览器 http://10.32.7.217/  实时网页
 *        http://10.32.7.217/imu 纯文本调试
 */

#include "Arduino.h"
#include "HardwareSerial.h"
#include "WiFi.h"
#include "WebServer.h"

static const int  IMU_RX_PIN = 17;
static const int  IMU_TX_PIN = 18;
static const long IMU_BAUD   = 115200;
static const char AP_SSID[] = "imu_test";
static const char AP_PASS[] = "12345678";

static HardwareSerial IMUSerial(1);
static WebServer server(80);

static uint8_t rx_buf[32];
static uint8_t rx_idx = 0;

static uint32_t dbg_bytes = 0;
static uint32_t dbg_frames = 0;
static uint32_t dbg_ang = 0;
static uint32_t dbg_sum_fail = 0;
static uint32_t dbg_last_ms = 0;
static uint32_t dbg_updates = 0;

static float g_roll = 0, g_pitch = 0, g_yaw = 0;
static bool  g_has_data = false;

static uint8_t last_frame[11];
static bool has_frame = false;

static char raw_hex[33 * 3 + 1];
static uint8_t raw_len = 0;

static uint8_t calc_sum(uint8_t* d, uint8_t n) {
    uint8_t s = 0;
    for (uint8_t i = 0; i < n; i++) s += d[i];
    return s & 0xFF;
}

static void push_raw(uint8_t b) {
    char hi = "0123456789ABCDEF"[b >> 4];
    char lo = "0123456789ABCDEF"[b & 0x0F];
    if (raw_len <32) {
        raw_hex[raw_len * 3] = hi;
        raw_hex[raw_len * 3 + 1] = lo;
        raw_hex[raw_len * 3 + 2] = ' ';
        raw_len++;
        raw_hex[raw_len * 3] = '\0';
    } else {
        memmove(raw_hex, raw_hex + 3, 31 * 3);
        raw_hex[31 * 3] = hi;
        raw_hex[31 * 3 + 1] = lo;
        raw_hex[31 * 3 + 2] = ' ';
        raw_hex[32 * 3] = '\0';
    }
}

static void imu_poll(void) {
    dbg_updates++;
    while (IMUSerial.available()) {
        uint8_t b = IMUSerial.read();
        dbg_bytes++;
        push_raw(b);
        if (b == 0x55) {
            rx_idx = 0;
            rx_buf[rx_idx++] = b;
        } else if (rx_idx > 0) {
            rx_buf[rx_idx++] = b;
            if (rx_idx == 11) {
                uint8_t sum = calc_sum(rx_buf, 10);
                if (sum == rx_buf[10]) {
                    dbg_frames++;
                    memcpy(last_frame, rx_buf, 11);
                    has_frame = true;
                    if (rx_buf[1] == 0x53) {
                        int16_t rr = (int16_t)rx_buf[3] << 8 | rx_buf[2];
                        int16_t pr = (int16_t)rx_buf[5] << 8 | rx_buf[4];
                        int16_t yr = (int16_t)rx_buf[7] << 8 | rx_buf[6];
                        g_roll  = (float)rr / 32768.0f * 180.0f;
                        g_pitch = (float)pr / 32768.0f * 180.0f;
                        g_yaw   = (float)yr / 32768.0f * 180.0f;
                        g_has_data = true;
                        dbg_ang++;
                        dbg_last_ms = millis();
                    }
                } else {
                    dbg_sum_fail++;
                }
                rx_idx = 0;
            }
        }
    }
}

static void build_debug(char* buf, size_t n) {
    uint32_t age = dbg_last_ms ? (millis() - dbg_last_ms) : 0;
    char fhex[11 * 3 + 1];
    fhex[0] = '\0';
    if (has_frame) {
        int k = 0;
        for (int i = 0; i < 11; i++) {
            fhex[k++] = "0123456789ABCDEF"[last_frame[i] >> 4];
            fhex[k++] = "0123456789ABCDEF"[last_frame[i] & 0x0F];
            fhex[k++] = ' ';
        }
        fhex[k] = '\0';
    }
    const char* verdict =
        (!has_frame && dbg_bytes == 0) ? "NO_SERIAL_DATA -> 检查接线/波特率/引脚" :
        (!has_frame && dbg_bytes > 0 && dbg_frames == 0) ? "GARBAGE_BYTES -> 有字节无有效帧, 波特率/协议不符" :
        (!has_frame && dbg_frames > 0) ? "FRAMES_BUT_NONE" :
        (dbg_ang == 0) ? "NO_0x53 -> 帧类型不对(未见角度帧)" :
        (age > 2000) ? "STALE -> 最后角度帧超过2s" :
        (g_roll == 0 && g_pitch == 0 && g_yaw == 0) ? "ALL_ZERO -> 传感器未输出角度" :
        "OK -> 转动 IMU 板看数值变化";
    snprintf(buf, n,
        "IMU DEBUG\n"
        "init: RX=GPIO%d TX=GPIO%d baud=%ld\n"
        "updates=%lu\nbytes=%lu\nframes_ok=%lu\nang_frames_0x53=%lu\n"
        "sum_fail=%lu\nlast_frame_age_ms=%lu\nhas_data=%d\n"
        "raw: roll=%.3f pitch=%.3f yaw=%.3f\n"
        "last_frame_hex: %s\nrx_tail_hex: %s\nverdict: %s\n",
        IMU_RX_PIN, IMU_TX_PIN, IMU_BAUD,
        (unsigned long)dbg_updates, (unsigned long)dbg_bytes,
        (unsigned long)dbg_frames, (unsigned long)dbg_ang,
        (unsigned long)dbg_sum_fail, (unsigned long)age,
        g_has_data ? 1 : 0, g_roll, g_pitch, g_yaw,
        has_frame ? fhex : "(none)",
        raw_len ? raw_hex : "(none)", verdict);
}

static void handleRoot() {
    char dbg[1024];
    build_debug(dbg, sizeof(dbg));
    uint32_t age = dbg_last_ms ? (millis() - dbg_last_ms) : 0;
    char buf[2048];
    snprintf(buf, sizeof(buf),
        "\x3c!DOCTYPE html\x3e\x3chtml\x3e\x3chead\x3e\x3cmeta charset='utf-8'\x3e"
        "\x3cmeta http-equiv='refresh' content='1'\x3e\x3ctitle\x3eIMU Test\x3c/title\x3e"
        "\x3cstyle\x3ebody{font-family:monospace;background:#111;color:#0f0;padding:20px}"
        "pre{font-size:14px;white-space:pre-wrap}.big{font-size:30px}\x3c/style\x3e\x3c/head\x3e\x3cbody\x3e"
        "\x3ch2\x3eIMU (JY62) Test\x3c/h2\x3e"
        "\x3cp class='big'\x3eR=%7.2f  P=%7.2f  Y=%7.2f\x3c/p\x3e"
        "\x3cp\x3eage=%lums %s\x3c/p\x3e\x3chr\x3e\x3cpre\x3e%s\x3c/pre\x3e\x3c/body\x3e\x3c/html\x3e",
        g_roll, g_pitch, g_yaw,
        (unsigned long)age, (age > 0 && age < 1000) ? "LIVE" : "STALE",
        dbg);
    server.send(200, "text/html; charset=utf-8", buf);
}

static void handleImu() {
    char dbg[1024];
    build_debug(dbg, sizeof(dbg));
    server.send(200, "text/plain; charset=utf-8", dbg);
}

static void Task_IMU(void*) {
    while (1) {
        imu_poll();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== IMU web test (ESP32-S3) ===");
    Serial.printf("IMU: RX=GPIO%d TX=GPIO%d baud=%ld\n", IMU_RX_PIN, IMU_TX_PIN, IMU_BAUD);
    IMUSerial.begin(IMU_BAUD, SERIAL_8N1, IMU_RX_PIN, IMU_TX_PIN);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("\n[WiFi] AP: %s  IP: %s\n", AP_SSID,
                  WiFi.softAPIP().toString().c_str());

    xTaskCreate(Task_IMU, "IMU", 4096, NULL, 2, NULL);

    server.on("/", handleRoot);
    server.on("/imu", handleImu);
    server.begin();
    Serial.printf("[HTTP] open http://%s/\n", WiFi.softAPIP().toString().c_str());
}

void loop() {
    server.handleClient();
    delay(1);
}
