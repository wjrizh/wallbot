/*
 * web_server.cpp - WiFi AP + HTTP + WebSocket 二进制协议版
 * 下推帧: AA 55 81 1B [27字节] CRC16
 * 上送帧: AA 55 CMD LEN [PAYLOAD] CRC16
 * 回执帧: AA 55 82 02 [CMD][CODE] CRC16  (CODE: 0=OK 1=参数错 2=CRC错 3=未知命令)
 */

#include "web_server.h"
#include "config.h"
#include "protocol.h"
#include "servo_driver.h"
#include "pump_driver.h"
#include "can_rs02.h"
#include "gait_engine.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <ESPAsyncWebServer.h>
#include <string.h>
#include "auto_align.h"
#include "gripper.h"
#include "imu_driver.h"
#include "laser_tof.h"
#include "test_mode.h"
#include "align_vision.h"

static void onTestTelemetry(const uint8_t* f, uint8_t len);

bool g_imu_online = false;
static uint8_t g_imu_fail_count = 0;
static volatile float g_imu_roll_cached = 0.0f;

/* ==================== UDP 自动发现 ==================== */
/* App 广播 WALLBOT_DISCOVER, 各板回 WALLBOT:<role>:<ip> */
static const uint16_t DISCOVERY_PORT = 48899;
static const char* DISCOVERY_REQ = "WALLBOT_DISCOVER";
static WiFiUDP g_discovery_udp;
static bool g_ws_wifi_ok = false;

static void Task_Discovery(void *pvParameters) {
    (void)pvParameters;
    if (!g_discovery_udp.begin(DISCOVERY_PORT)) {
        Serial.printf("[UDP] begin :%u failed\n", DISCOVERY_PORT);
        vTaskDelete(NULL);
        return;
    }
    Serial.printf("[UDP] discovery listen :%u\n", DISCOVERY_PORT);

    char rx[64];
    for (;;) {
        int size = g_discovery_udp.parsePacket();
        if (size > 0 && size < (int)sizeof(rx)) {
            int len = g_discovery_udp.read(rx, sizeof(rx) - 1);
            if (len > 0) {
                rx[len] = 0;
                if (strncmp(rx, DISCOVERY_REQ, strlen(DISCOVERY_REQ)) == 0) {
                    IPAddress rip = g_discovery_udp.remoteIP();
                    uint16_t rport = g_discovery_udp.remotePort();
                    char reply[48];
                    int n = snprintf(reply, sizeof(reply), "WALLBOT:host:%s",
                                     WiFi.localIP().toString().c_str());
                    g_discovery_udp.beginPacket(rip, rport);
                    g_discovery_udp.write((const uint8_t*)reply, n);
                    g_discovery_udp.endPacket();
                    Serial.printf("[UDP] reply %s -> %s\n", reply, rip.toString().c_str());
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void WebServer_RefreshIMU(void) {
    g_imu_roll_cached = FetchIMURoll();
}

float WebServer_GetIMURoll(void) {
    return g_imu_roll_cached;
}

float FetchIMURoll(void) {
    float roll = IMU_GetRoll();
    /* 有线串口: 由 Task_IMU 周期调用 IMU_Update() 采样, 这里直接取缓存值 */
    if (g_imu_fail_count > 0) g_imu_fail_count = 0;
    g_imu_online = true;
    return roll;
}

static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");

/* CRC16 (与主项目一致) */
static uint16_t crc16(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}


/* ==================== WebSocket 事件处理 ==================== */

static void onWsEvent(AsyncWebSocket *s, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_DATA) {
        if (len < 6) return;
        if (data[0] != 0xAA || data[1] != 0x55) return;

        uint8_t cmd = data[2];
        uint8_t pay_len = data[3];
        if (len < (size_t)(pay_len + 6)) return;

        uint16_t crc_recv = data[4 + pay_len] | ((uint16_t)data[5 + pay_len] << 8);
        uint16_t crc_calc = crc16(data, pay_len + 4);
        if (crc_recv != crc_calc) { Protocol_SendAck(cmd, 2); return; }

        const uint8_t* p = (pay_len > 0) ? &data[4] : NULL;

        uint8_t code = 0;   /* 0=OK 1=参数错误 3=未知命令 */
        bool acked = false; /* 0x18/19/1A 委托 Protocol_ProcessFrame, 内部已回执 */

        switch (cmd) {
            /* 单舵机: id(1) angle×10(2LE) dur×10(2LE) */
            case 0x01: {
                if (pay_len < 5 || !p) { code = 1; break; }
                uint8_t id = p[0];
                uint16_t av = p[1] | ((uint16_t)p[2] << 8);
                uint16_t dv = p[3] | ((uint16_t)p[4] << 8);
                float angle = av / 10.0f;
                float dur = dv / 10.0f;
                if (angle > 180) angle = 180;
                if (dur < 0.1f) dur = 0.5f; if (dur > 10.0f) dur = 10.0f;
                switch (id) {
                    case 1: Servo_SetM1(angle, dur); break;
                    case 2: Servo_SetM2(angle, dur); break;
                    case 3: Servo_SetM3(angle, dur); break;
                    case 9: Servo_SetPB9(angle, dur); break;
                }
                break;
            }
            /* 全部舵机: m1×10(2) m2×10(2) m3×10(2) dur×10(2) */
            case 0x02: {
                if (pay_len < 8 || !p) { code = 1; break; }
                uint16_t m1v = p[0] | ((uint16_t)p[1] << 8);
                uint16_t m2v = p[2] | ((uint16_t)p[3] << 8);
                uint16_t m3v = p[4] | ((uint16_t)p[5] << 8);
                uint16_t dv  = p[6] | ((uint16_t)p[7] << 8);
                float m1 = m1v/10.0f, m2 = m2v/10.0f, m3 = m3v/10.0f, dur = dv/10.0f;
                if (m1>180) m1=180; if (m2>180) m2=180; if (m3>180) m3=180;
                if (dur<0.1f) dur=0.5f; if (dur>10.0f) dur=10.0f;
                Servo_SetM1(m1, dur); Servo_SetM2(m2, dur); Servo_SetM3(m3, dur);
                break;
            }
            /* 泵: id(1) state(1) */
            case 0x03: {
                if (pay_len < 2 || !p) { code = 1; break; }
                if (p[0]==1||p[0]==2) Pump_Set(p[0], p[1]);
                break;
            }
            /* CAN角度: id(1) angle×10(2LE) dur×10(2LE) */
            case 0x04: {
                if (pay_len < 5 || !p) { code = 1; break; }
                uint8_t id = p[0];
                uint16_t av = p[1] | ((uint16_t)p[2] << 8);
                uint16_t dv = p[3] | ((uint16_t)p[4] << 8);
                float angle_deg = av / 10.0f;
                float dur = dv / 10.0f;
                if (dur < 0.5f) dur = 0.5f;
                float rad = angle_deg * PI / 180.0f;
                CAN_RS02_StartTraj(id, rad, dur);
                break;
            }
            /* CAN命令: id(1) subcmd(1) — 0=使能 1=失能 2=设零 */
            case 0x05: {
                if (pay_len < 2 || !p) { code = 1; break; }
                uint8_t id = p[0], sub = p[1];
                if (sub == 0) {
                    /* 使能后保持当前位置, 不归零: 目标=当前反馈角 */
                    float cur = CAN_RS02_GetPosition(id);
                    CAN_RS02_ResetTraj(id, cur);
                    CAN_RS02_Enable(id);
                }
                else if (sub == 1) CAN_RS02_Disable(id);
                else if (sub == 2) { CAN_RS02_SetZero(id); CAN_RS02_ResetTraj(id, 0.0f); }
                break;
            }
            /* 自动纠偏 */
            case 0x09: {
                if (!AutoAlign_IsActive()) AutoAlign_Start();
                break;
            }
            /* 自动纠偏急停 */
            case 0x0A: {
                AutoAlign_Stop();
                break;
            }
            /* 越障 */
            case 0x11: {
                GaitEngine_EnqueueCmd(0x11, NULL, 0);
                break;
            }
            /* 启停: val(1) 0=停 1=启 2=实战 */
            case 0x12: {
                if (pay_len < 1 || !p) { code = 1; break; }
                uint8_t val = p[0];
                uint8_t d[1] = {val};
                GaitEngine_EnqueueCmd(0x12, d, 1);
                break;
            }
            /* 迈步: step_mm(4 floatLE) */
            case 0x13: {
                if (pay_len < 4 || !p) { code = 1; break; }
                float step_mm;
                memcpy(&step_mm, &p[0], 4);
                GaitEngine_EnqueueCmd(0x10, (uint8_t*)&step_mm, 4);
                break;
            }
            /* 掉头 */
            case 0x14: {
                GaitEngine_EnqueueCmd(0x14, NULL, 0);
                break;
            }
            /* 确认下一步 */
            case 0x15: {
                GaitEngine_ConfirmNext();
                break;
            }
            /* ===== 视觉纠偏测试 (无摄像头, 假偏差下发) ===== */
            /* 0x30 开始测试 */
            case 0x30: {
                TestMode_Start();
                break;
            }
            /* 0x31 下发假偏差: phi(4 floatLE) psi(4) ax(4) */
            case 0x31: {
                if (pay_len < 12 || !p) { code = 1; break; }
                float phi, psi, ax;
                memcpy(&phi, &p[0], 4);
                memcpy(&psi, &p[4], 4);
                memcpy(&ax,  &p[8], 4);
                TestMode_SendAdj(phi, psi, ax);
                break;
            }
            /* 0x32 结束测试 */
            case 0x32: {
                TestMode_Stop();
                break;
            }
            /* ===== 真实视觉闭环 (align_vision) ===== */
            /* 0x22 启动: ip[4] gainPct maxIter flags */
            case 0x22: {
                if (pay_len < 4 || !p) { code = 1; break; }
                if (AlignVision_IsActive()) { code = 1; break; }
                uint8_t gain = (pay_len >= 5) ? p[4] : AV_DEFAULT_GAIN_PCT;
                uint8_t iter = (pay_len >= 6) ? p[5] : AV_MAX_ITER;
                uint8_t flags = (pay_len >= 7) ? p[6] : (AV_FLAG_START_CAM | AV_FLAG_STOP_CAM);
                if (!AlignVision_Start(p, gain, iter, flags)) { code = 1; break; }
                break;
            }
            /* 0x23 停止 */
            case 0x23: {
                AlignVision_Stop();
                break;
            }
            /* 0x24 标定: signPhi signPsi signAx betaDeg speedPct */
            case 0x24: {
                if (pay_len < 5 || !p) { code = 1; break; }
                AlignVision_SetTune((int8_t)p[0], (int8_t)p[1], (int8_t)p[2], p[3], p[4]);
                break;
            }
            /* 0x25 单步: which(1) + phi f32 + psi f32 + ax f32 */
            case 0x25: {
                if (pay_len < 13 || !p) { code = 1; break; }
                float phi, psi, ax;
                memcpy(&phi, &p[1], 4);
                memcpy(&psi, &p[5], 4);
                memcpy(&ax,  &p[9], 4);
                if (!AlignVision_Step(p[0], phi, psi, ax)) { code = 1; break; }
                break;
            }
            /* 0x26 视觉侧控制: action(1) + ip[4] */
            case 0x26: {
                if (pay_len < 5 || !p) { code = 1; break; }
                if (!AlignVision_CamCtl(p[0], &p[1])) { code = 1; break; }
                break;
            }
            /* ===== NRF手柄 + 子母协同 → 协议模块 (内部已回执) ===== */
            case 0x18: case 0x19: case 0x1A: case 0x1B: case 0x1D:
            case 0x20: case 0x21: {
                Protocol_ProcessFrame(data, len);
                acked = true;
                break;
            }
            default: {
                code = 3;   /* 未知命令 */
                break;
            }
        }

        if (!acked) Protocol_SendAck(cmd, code);
    }
}

/* ==================== 接口函数 ==================== */

void WebServer_Init(void) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    /* 关掉开机诊断扫描: scanNetworks() 耗时 2-4s, 会推迟 WS 服务就绪.
       DHCP: 从热点获取与热点同网段的地址, 避免静态 IP 与热点网段不匹配 */
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("[WiFi] STA connecting to %s ...\n", WIFI_SSID);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.printf("[WiFi] status=%d (3=connected, 1=no_ssid, 4=connect_failed, 6=disconnected)\n", (int)WiFi.status());
    Serial.printf("[WiFi] STA: %s, IP: %s, GW: %s, MASK: %s\n", WIFI_SSID,
                  WiFi.localIP().toString().c_str(),
                  WiFi.gatewayIP().toString().c_str(),
                  WiFi.subnetMask().toString().c_str());

    /* 注入回执发送实现: 回给所有已连接客户端 */
    Protocol_SetAckSender([](const uint8_t* frame, uint16_t len) {
        if (ws.count() == 0) return;
        ws.binaryAll(const_cast<uint8_t*>(frame), len);
    });

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    TestMode_SetTelemetrySink(onTestTelemetry);

    /* 只保存 WS 通道, 无内部页：网页根路径回应数据 */
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "wallbot-ok");
    });

    /* IMU 调试端点: http://10.32.7.216/imu */
    server.on("/imu", HTTP_GET, [](AsyncWebServerRequest *request) {
        static char dbgbuf[1024];
        int n = IMU_GetDebug(dbgbuf, sizeof(dbgbuf));
        if (n <0) n = 0;
        request->send(200, "text/plain", String(dbgbuf));
    });

    server.begin();
    Serial.println("[Web] HTTP + WS 二进制协议 已启动");

    /* Discovery 任务无条件启动: 未连上时 UDP 收不到包, 不影响;
       连上后即可应答, 避免 WiFi 晚连导致发现能力永久缺失 */
    xTaskCreate(Task_Discovery, "Discovery", 4096, NULL, 1, NULL);
    g_ws_wifi_ok = (WiFi.status() == WL_CONNECTED);
}

/* 由周期任务调用: WiFi 掉线自动重连, 状态变化时打印 */
void WebServer_Tick(void) {
    static uint32_t last_try = 0;
    static uint32_t last_log = 0;
    wl_status_t st = WiFi.status();
    if (st == WL_CONNECTED) {
        if (!g_ws_wifi_ok) {
            g_ws_wifi_ok = true;
            Serial.printf("[WiFi] reconnected, IP: %s\n", WiFi.localIP().toString().c_str());
        }
        return;
    }
    if (g_ws_wifi_ok) {
        g_ws_wifi_ok = false;
        Serial.printf("[WiFi] lost (status=%d), reconnecting\n", (int)st);
    }
    uint32_t now = millis();
    if (now - last_try < 3000) return;
    last_try = now;
    WiFi.reconnect();
    if (now - last_log > 15000) {
        last_log = now;
        Serial.printf("[WiFi] retry connect %s (status=%d)\n", WIFI_SSID, (int)st);
    }
}

void WebServer_PushLaser(void) {
    if (ws.count() == 0) return;

    uint8_t m1 = (uint8_t)(Servo_GetAngle(1) + 0.5f);
    uint8_t m2 = (uint8_t)(Servo_GetAngle(2) + 0.5f);
    uint8_t m3 = (uint8_t)(Servo_GetAngle(3) + 0.5f);
    uint8_t pb9 = (uint8_t)(Servo_GetAngle(9) + 0.5f);
    uint8_t pump = (Pump_GetState(1) ? 1 : 0) | (Pump_GetState(2) ? 2 : 0);

    float rad1 = CAN_RS02_GetPosition(1);
    float rad2 = CAN_RS02_GetPosition(2);

    float imu_roll = g_imu_roll_cached;

    float cal_roll_gnd = Imu_GetGnd();
    float grip_angle = Gripper_GetAngle();
    bool grip_online = Gripper_IsOnline();

    uint8_t payload[32];
    payload[0] = m1;
    payload[1] = m2;
    payload[2] = m3;
    payload[3] = pb9;
    payload[4] = pump;
    memcpy(&payload[5], &rad1, 4);
    memcpy(&payload[9], &rad2, 4);
    memcpy(&payload[13], &imu_roll, 4);
    payload[17] = g_imu_online ? 1 : 0;
    uint16_t laser_mm = Laser_GetFront();
    memcpy(&payload[18], &laser_mm, 2);
    payload[20] = Laser_IsOnline() ? 1 : 0;
    payload[21] = 0;
    memcpy(&payload[22], &cal_roll_gnd, 4);
    payload[26] = (uint8_t)AutoAlign_GetCorrMode();
    memcpy(&payload[27], &grip_angle, 4);
    payload[31] = grip_online ? 1 : 0;

    uint8_t frame[2+1+1+32+2];
    frame[0] = 0xAA; frame[1] = 0x55;
    frame[2] = 0x81; frame[3] = 32;
    memcpy(&frame[4], payload, 32);
    uint16_t crc = crc16(frame, 36);
    frame[36] = crc & 0xFF;
    frame[37] = (crc >> 8) & 0xFF;

    ws.binaryAll(frame, 38);
}

/* 子机轮子运动遥测 'J' -> App (0x86) */
static void onTestTelemetry(const uint8_t* f, uint8_t len) {
    if (ws.count() == 0) return;
    uint8_t frame[2 + 1 + 1 + 32 + 2];
    frame[0] = 0xAA; frame[1] = 0x55;
    frame[2] = 0x86;
    frame[3] = len;
    memcpy(&frame[4], f, len);
    uint16_t crc = crc16(frame, 4 + len);
    frame[4 + len] = crc & 0xFF;
    frame[5 + len] = (crc >> 8) & 0xFF;
    ws.binaryAll(frame, 6 + len);
}

/* 测试状态帧 (0x87): [state][round][last_rc][buf] */
void WebServer_PushTestStatus(void) {
    if (ws.count() == 0) return;
    uint8_t st, rd, rc;
    TestMode_GetStatus(&st, &rd, &rc);
    uint8_t pay[3] = {st, rd, rc};
    uint8_t frame[2 + 1 + 1 + 3 + 2];
    frame[0] = 0xAA; frame[1] = 0x55;
    frame[2] = 0x87;
    frame[3] = 3;
    memcpy(&frame[4], pay, 3);
    uint16_t crc = crc16(frame, 7);
    frame[7] = crc & 0xFF;
    frame[8] = (crc >> 8) & 0xFF;
    ws.binaryAll(frame, 9);
}

void WebServer_SendLog(const char* msg) {
    if (ws.count() == 0) return;
    size_t len = strlen(msg);
    if (len > 200) len = 200;
    uint8_t frame[2 + 1 + 1 + 200 + 2];
    frame[0] = 0xAA; frame[1] = 0x55;
    frame[2] = 0x83;
    frame[3] = (uint8_t)len;
    memcpy(&frame[4], msg, len);
    uint16_t crc = crc16(frame, 4 + len);
    frame[4 + len] = crc & 0xFF;
    frame[5 + len] = (crc >> 8) & 0xFF;
    ws.binaryAll(frame, 6 + len);
}

/* 0x88: 真实视觉闭环状态 */
void WebServer_PushAlignState(void) {
    if (ws.count() == 0) return;
    uint8_t pay[11];
    AlignVision_GetStatus(pay);
    uint8_t frame[2 + 1 + 1 + 11 + 2];
    frame[0] = 0xAA; frame[1] = 0x55;
    frame[2] = 0x88;
    frame[3] = 11;
    memcpy(&frame[4], pay, 11);
    uint16_t crc = crc16(frame, 15);
    frame[15] = crc & 0xFF;
    frame[16] = (crc >> 8) & 0xFF;
    ws.binaryAll(frame, 17);
}

/* 0x89: 视觉原始 JSON */
void WebServer_PushAlignJson(const char* json) {
    if (ws.count() == 0 || !json) return;
    size_t len = strlen(json);
    if (len > 200) len = 200;
    uint8_t frame[2 + 1 + 1 + 200 + 2];
    frame[0] = 0xAA; frame[1] = 0x55;
    frame[2] = 0x89;
    frame[3] = (uint8_t)len;
    memcpy(&frame[4], json, len);
    uint16_t crc = crc16(frame, 4 + len);
    frame[4 + len] = crc & 0xFF;
    frame[5 + len] = (crc >> 8) & 0xFF;
    ws.binaryAll(frame, 6 + len);
}