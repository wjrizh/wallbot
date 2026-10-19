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

    AlignMeasure_Init();          /* 先恢复参考位 (NVS) */

    Cam_Init();
    if (!Cam_IsOk()) {
        Serial.println("Camera init failed");
    } else {
        Serial.printf("Camera init ok (pid=0x%x %ums jpeg=%d)\n",
                      Cam_SensorPid(), Cam_InitMs(), Cam_JpegOk());
    }

    wifi_connect();

    if (StreamServer_Start()) {
        Serial.println("HTTP servers started (:80 /jpg /align/*, :81 /stream)");
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
            Discovery_Reset();
        }
    }
    Discovery_Poll();
    delay(20);
}
