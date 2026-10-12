/*
 * imu_wireless.ino - IMU 无线传感器板 (STA模式)
 * 连接 Robot 热点, 静态IP 192.168.4.55
 * HTTP GET / → 返回 JSON 格式 Roll/Pitch/Yaw
 */

#include "config.h"
#include "imu_driver.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

static WebServer server(80);

static void handleRoot() {
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"R\":%.2f,\"P\":%.2f,\"Y\":%.2f}",
             IMU_GetRoll(), IMU_GetPitch(), IMU_GetYaw());
    server.send(200, "application/json", buf);
}

static void Task_IMU(void *pvParameters) {
    while (1) {
        IMU_Update();
        vTaskDelay(pdMS_TO_TICKS(10));  // 高速采样, 不丢帧
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== IMU 无线传感器板 ===\n");

    /* 1. IMU 串口 */
    IMU_Init();

    /* 2. WiFi STA */
    WiFi.config(IMU_STATIC_IP, IPAddress(192,168,4,1), IPAddress(255,255,255,0));
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("[WiFi] 连接 %s ...", WIFI_SSID);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
        delay(500); Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WiFi] 已连接, IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n[WiFi] 连接失败! 重启中...");
        ESP.restart();
    }

    /* 3. HTTP 服务器 */
    server.on("/", handleRoot);
    server.begin();
    Serial.println("[HTTP] 服务器已启动 :80");

    /* 4. IMU 采样任务 */
    xTaskCreate(Task_IMU, "IMU_Samp", 4096, NULL, 2, NULL);
}

void loop() {
    server.handleClient();
    delay(1);
}