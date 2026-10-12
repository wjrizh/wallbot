/*
 * laser_multitest.ino - 多路激光 (TCA9548A + VL53L5CX) 健康检查
 * 完全独立于主固件。
 * 只做"是否正常"判断:
 *   1. TCA9548A (0x70) 是否应答
 *   2. 每个通道 (0~7) 上的 VL53L5CX 是否存在 (0x29 ACK)
 *   3. 对存在的通道尝试 begin()+读一帧, 判断"正常工作"
 *   4. 网页显示每路 OK / FAIL:  http://192.168.4.1/
 * 硬件: SDA=GPIO4, SCL=GPIO5, TCA9548A=0x70
 * WiFi: AP "laser_test" / "12345678"  IP 192.168.4.1
 */
#include "Arduino.h"
#include "Wire.h"
#include "WiFi.h"
#include "WebServer.h"
#include "Adafruit_VL53L5CX.h"

#define AP_SSID   "laser_test"
#define AP_PASS   "12345678"

#define TCA9548A_ADDR   0x70
#define TOF_I2C_SDA     4
#define TOF_I2C_SCL     5

#define FAN_ROW_START   4
#define FAN_ROW_END     7
#define FAN_COL_START   1
#define FAN_COL_END     6
#define TOF_VALID_STATUS 5

static WebServer server(80);

static bool g_tca_ok = false;
static bool g_ch_present[8];
static bool g_ch_ok[8];
static uint16_t g_ch_front[8];
static uint32_t g_ch_frames[8];

static Adafruit_VL53L5CX sensor;
static VL53L5CX_ResultsData results;

static void tcaSelect(uint8_t ch) {
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << ch);
    Wire.endTransmission();
}

static bool probe29(uint8_t ch) {
    tcaSelect(ch);
    Wire.beginTransmission(0x29);
    return (Wire.endTransmission() == 0);
}

static void checkChannel(uint8_t ch) {
    if (!g_ch_present[ch]) { g_ch_ok[ch] = false; return; }
    tcaSelect(ch);
    if (!sensor.begin(VL53L5CX_DEFAULT_ADDRESS, &Wire, 400000)) {
        g_ch_ok[ch] = false;
        return;
    }
    sensor.setResolution(64);
    sensor.setRangingFrequency(15);
    sensor.startRanging();
    uint32_t t0 = millis();
    while (millis() - t0 < 1000) {
        tcaSelect(ch);
        if (sensor.isDataReady() && sensor.getRangingData(&results)) {
            uint16_t front = 0;
            for (int r = FAN_ROW_START; r <= FAN_ROW_END; r++)
                for (int c = FAN_COL_START; c <= FAN_COL_END; c++) {
                    int idx = r * 8 + c;
                    if (results.target_status[idx] != TOF_VALID_STATUS) continue;
                    uint16_t d = results.distance_mm[idx];
                    if (d == 0) continue;
                    if (front == 0 || d < front) front = d;
                }
            g_ch_front[ch] = front;
            g_ch_frames[ch]++;
            g_ch_ok[ch] = true;
            return;
        }
        delay(10);
    }
    g_ch_ok[ch] = false;
}

static void runTest() {
    Wire.beginTransmission(TCA9548A_ADDR);
    g_tca_ok = (Wire.endTransmission() == 0);
    Serial.printf("TCA9548A: %s\n", g_tca_ok ? "OK" : "FAIL");
    for (uint8_t ch = 0; ch < 8; ch++) {
        g_ch_present[ch] = probe29(ch);
        g_ch_ok[ch] = false;
        g_ch_front[ch] = 0;
        g_ch_frames[ch] = 0;
    }
    for (uint8_t ch = 0; ch < 8; ch++) {
        if (g_ch_present[ch]) checkChannel(ch);
        Serial.printf("CH%d: present=%d ok=%d\n", ch, g_ch_present[ch], g_ch_ok[ch]);
    }
    Serial.println("test done");
}

static void handleRoot() {
    String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta http-equiv='refresh' content='3'><title>Laser Health</title>"
        "<style>body{font-family:monospace;background:#111;color:#0f0;padding:16px;font-size:16px}"
        ".ok{color:#0f0}.bad{color:#f44}</style></head><body>";
    html += "<h2>多路激光健康检查</h2>";
    html += "<p>TCA9548A: <span class='";
    html += g_tca_ok ? "ok'>OK" : "bad'>FAIL";
    html += "</span></p><pre>";
    char buf[128];
    for (uint8_t i = 0; i < 8; i++) {
        const char* st;
        if (!g_ch_present[i])      st = "无传感器";
        else if (g_ch_ok[i])       st = "正常";
        else                       st = "异常(无有效帧)";
        snprintf(buf, sizeof(buf), "CH%d: %-16s present=%d ok=%d front=%umm frames=%lu\n",
                 i, st, g_ch_present[i], g_ch_ok[i],
                 g_ch_front[i], (unsigned long)g_ch_frames[i]);
        html += buf;
    }
    html += "</pre></body></html>";
    server.send(200, "text/html; charset=utf-8", html);
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== 多路激光健康检查 ===");
    Wire.begin(TOF_I2C_SDA, TOF_I2C_SCL, 400000);
    runTest();
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    server.on("/", handleRoot);
    server.begin();
    Serial.printf("[HTTP] http://%s/\n", WiFi.softAPIP().toString().c_str());
}

void loop() {
    server.handleClient();
    delay(2);
}
