/*
 * tof_test.ino - VL53L5CX 8x8 ToF 测距独立测试固件 (学术可视化)
 *
 * 硬件: ESP32-S3 + VL53L5CX 8x8 ToF (TCA9548A 0x70 CH0)
 * 引脚: SDA=GPIO4, SCL=GPIO5  (与主固件一致)
 * 模式: WiFi AP "robot" / "12345678", 网页 http://192.168.4.1/tof
 *
 * 端点:
 *   GET /tof        学术风动态热力图页面
 *   GET /tof/matrix 8x8 距离/状态 JSON
 *
 * 本文件独立于 firmware/, 不影响主固件代码。
 */
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_VL53L5CX.h>
#include "index_html.h"

/* ==================== 配置 ==================== */
#define WIFI_SSID       "robot"
#define WIFI_PASS       "12345678"

#define TCA9548A_ADDR   0x70
#define LASER_CH        0
#define TOF_I2C_SDA     4
#define TOF_I2C_SCL     5

/* 前方扇形 ROI: 物理前方对应 row4-7 (模组倒装, row0-3 朝下为地面), 中间列 col1-6 */
#define FAN_ROW_START   4
#define FAN_ROW_END     7
#define FAN_COL_START   1
#define FAN_COL_END     6

/* 有效目标状态 (VL53L5CX target_status==5 表示有效) */
#define TOF_VALID_STATUS 5

/* ==================== 全局 ==================== */
static Adafruit_VL53L5CX sensor;
static VL53L5CX_ResultsData results;
static WebServer server(80);

static volatile bool     g_online = false;
static volatile uint16_t g_front  = 0;       /* 前方扇形最小距离 mm, 0=无效 */
static volatile uint32_t g_frames = 0;       /* 累计帧数 */
static volatile float    g_fps    = 0.0f;

/* 8x8 距离/状态快照 (供 HTTP 读取, 加锁保护) */
static portMUX_TYPE      g_mux = portMUX_INITIALIZER_UNLOCKED;
static uint16_t          g_dist[64];
static uint8_t           g_status[64];

/* ==================== I2C 多路复用 ==================== */
static void tcaSelect(uint8_t ch) {
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << ch);
    Wire.endTransmission();
}

/* ==================== 采样任务 ==================== */
static void Task_Laser(void *pv) {
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);
    uint32_t fps_frames = 0;
    uint32_t fps_t0 = millis();

    while (1) {
        vTaskDelayUntil(&last, period);
        if (!g_online) continue;

        tcaSelect(LASER_CH);
        if (!sensor.isDataReady()) continue;
        if (!sensor.getRangingData(&results)) continue;

        uint16_t front = 0;
        uint16_t snap_dist[64];
        uint8_t  snap_status[64];
        for (int i = 0; i < 64; i++) {
            uint8_t st = results.target_status[i];
            uint16_t d = results.distance_mm[i];
            snap_status[i] = st;
            snap_dist[i] = (st == TOF_VALID_STATUS && d > 0) ? d : 0;
        }
        for (int row = FAN_ROW_START; row <= FAN_ROW_END; row++) {
            for (int col = FAN_COL_START; col <= FAN_COL_END; col++) {
                uint16_t d = snap_dist[row * 8 + col];
                if (d == 0) continue;
                if (front == 0 || d < front) front = d;
            }
        }

        portENTER_CRITICAL(&g_mux);
        memcpy((void *)g_dist, snap_dist, sizeof(g_dist));
        memcpy((void *)g_status, snap_status, sizeof(g_status));
        g_front = front;
        portEXIT_CRITICAL(&g_mux);

        g_frames++;
        fps_frames++;
        uint32_t now = millis();
        if (now - fps_t0 >= 1000) {
            g_fps = fps_frames * 1000.0f / (now - fps_t0);
            fps_frames = 0;
            fps_t0 = now;
        }
    }
}

/* ==================== HTTP ==================== */
static void handleRoot() {
    server.sendHeader("Location", "/tof");
    server.send(302, "text/plain", "");
}

static void handleTofPage() {
    server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

static void handleMatrix() {
    uint16_t d[64];
    uint8_t  s[64];
    portENTER_CRITICAL(&g_mux);
    memcpy(d, (const void *)g_dist, sizeof(d));
    memcpy(s, (const void *)g_status, sizeof(s));
    uint16_t front = g_front;
    portEXIT_CRITICAL(&g_mux);

    String json;
    json.reserve(1024);
    json += "{\"online\":";
    json += g_online ? "true" : "false";
    json += ",\"fps\":";
    json += String(g_fps, 1);
    json += ",\"frames\":";
    json += String((uint32_t)g_frames);
    json += ",\"front\":";
    json += String(front);
    json += ",\"d\":[";
    for (int i = 0; i <64; i++) {
        if (i) json += ',';
        json += String(d[i]);
    }
    json += "],\"s\":[";
    for (int i = 0; i < 64; i++) {
        if (i) json += ',';
        json += String(s[i]);
    }
    json += "]}";

    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", json);
}

/* ==================== setup / loop ==================== */
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== VL53L5CX 8x8 ToF 测距测试 ===\n");

    Wire.begin(TOF_I2C_SDA, TOF_I2C_SCL);
    tcaSelect(LASER_CH);
    Serial.println("[TOF] init...");
    if (!sensor.begin(VL53L5CX_DEFAULT_ADDRESS, &Wire, 400000)) {
        Serial.println("[TOF] begin FAILED - 检查接线/TCA9548A 通道");
        g_online = false;
    } else {
        sensor.setResolution(64);
        sensor.setRangingFrequency(15);
        sensor.startRanging();
        g_online = true;
        Serial.println("[TOF] ready (8x8, 15Hz, CH0)");
    }

    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    Serial.printf("[WiFi] AP: %s / %s  IP: %s\n",
                  WIFI_SSID, WIFI_PASS, WiFi.softAPIP().toString().c_str());

    server.on("/", handleRoot);
    server.on("/tof", handleTofPage);
    server.on("/tof/matrix", handleMatrix);
    server.begin();
    Serial.println("[HTTP] http://192.168.4.1/tof");

    xTaskCreatePinnedToCore(Task_Laser, "Laser", 4096, NULL, 2, NULL, 1);
    Serial.println("\n[MAIN] 就绪\n");
}

void loop() {
    server.handleClient();
    delay(2);
}
