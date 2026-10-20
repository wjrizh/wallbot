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
#include <string.h>
#include "config.h"
#include "cam_module.h"
#include "discovery.h"
#include "stream_server.h"
#include "align_measure.h"
#include "align_server.h"
#include "stream_server.h"
#include "log_page.h"

#if CAM_AP_MODE
/* AP 模式: 子机自己开热点, 地址固定, 不依赖外部路由器与 DHCP */
static void wifi_connect() {
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    IPAddress ip, gw, mask;
    ip.fromString(AP_IP);
    gw.fromString(AP_GW);
    mask.fromString(AP_MASK);
    bool ok = WiFi.softAPConfig(ip, gw, mask);
    /* 显式指定 OPEN 认证, 不传密码串 (见 config.h 中关于 wpa_ap_get_wpa_ie 崩溃的说明) */
    bool up;
    if (strlen(AP_PASS) >= 8) {
        up = WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, 4);
    } else {
        up = WiFi.softAP(AP_SSID, NULL, AP_CHANNEL, 0, 1);
    }
    delay(300);
    Stream_Log("AP %s cfg=%d up=%d ip=%s", AP_SSID, ok ? 1 : 0, up ? 1 : 0,
               WiFi.softAPIP().toString().c_str());
    if (!up) Stream_Log("ERR softAP failed");
}
#else
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
        Stream_Log("WiFi ok ip=%s", WiFi.localIP().toString().c_str());
    } else {
        Stream_Log("ERR WiFi timeout, retry in loop");
    }
}
#endif

void setup() {
    Serial.begin(115200);
    Serial.println();

    Stream_Log("=== boot ===");
    Stream_Log("chip=%s cores=%d flash=%uMB psram=%uKB",
               ESP.getChipModel(), (int)ESP.getChipCores(),
               (unsigned)(ESP.getFlashChipSize() / (1024 * 1024)),
               (unsigned)(ESP.getPsramSize() / 1024));
    Stream_Log("ssid=%s build=%s %s", WIFI_SSID, __DATE__, __TIME__);

    AlignMeasure_Init();          /* 先恢复参考位 (NVS) */

    Cam_Init();
    if (!Cam_IsOk()) {
        Stream_Log("ERR Camera init failed err=0x%x", Cam_Err());
    } else {
        Stream_Log("Cam ok pid=0x%x %ums jpeg=%d",
                   Cam_SensorPid(), (unsigned)Cam_InitMs(), Cam_JpegOk() ? 1 : 0);
    }

    wifi_connect();

    if (StreamServer_Start()) {
        Stream_Log("HTTP ready :80 /logs /status /view /log /align/*, :81 /stream");
    } else {
        Stream_Log("ERR HTTP server start failed");
    }
}

void loop() {
    static uint32_t last_check = 0;
    uint32_t now = millis();

#if !CAM_AP_MODE
    // STA 模式: WiFi 掉线重连
    if (now - last_check > 3000) {
        last_check = now;
        if (WiFi.status() != WL_CONNECTED) {
            Stream_Log("WiFi lost, reconnecting...");
            WiFi.disconnect();
            WiFi.begin(WIFI_SSID, WIFI_PASS);
            Discovery_Reset();
        }
    }
#else
    (void)last_check;
#endif
    Discovery_Poll();
    delay(20);
}
