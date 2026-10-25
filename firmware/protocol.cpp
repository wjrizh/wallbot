/*
 * protocol.cpp - 二进制协议 + 命令分发 (从 web_server.cpp 抽出)
 * 帧格式: AA 55 CMD LEN [PAYLOAD] CRC16_LE
 * 下推帧: AA 55 81 1B [27字节] CRC16
 * 回执帧: AA 55 82 02 [CMD][CODE] CRC16   (CODE: 0=OK 1=参数错 2=CRC错 3=未知命令)
 */

#include "protocol.h"
#include "config.h"
#include "web_server.h"
#include "servo_driver.h"
#include "pump_driver.h"
#include "can_rs02.h"
#include "gait_engine.h"
#include "auto_align.h"
#include "joypad_input.h"
#include "joypad_sender.h"
#include "auto_mode.h"
#include "align_test.h"
#include "gripper.h"
#include "avoid_mode.h"
#include "child_cmd.h"
#include <Arduino.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* ==================== CRC16 ==================== */

uint16_t Protocol_CRC16(const uint8_t* data, uint16_t len) {
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

/* ==================== 帧打包 ==================== */

static void frame_pack(uint8_t cmd, const uint8_t* payload, uint8_t pay_len,
                       uint8_t* out, uint16_t* out_len) {
    out[0] = 0xAA;
    out[1] = 0x55;
    out[2] = cmd;
    out[3] = pay_len;
    if (pay_len > 0) memcpy(&out[4], payload, pay_len);
    uint16_t crc = Protocol_CRC16(out, pay_len + 4);
    out[4 + pay_len] = crc & 0xFF;
    out[5 + pay_len] = (crc >> 8) & 0xFF;
    *out_len = pay_len + 6;
}

/* ==================== 回执帧 (ACK) ==================== */

/* 由 web_server.cpp 注入发送实现 (发往当前 WS 客户端) */
static void (*s_ack_sender)(const uint8_t*, uint16_t) = NULL;

void Protocol_SetAckSender(void (*fn)(const uint8_t*, uint16_t)) {
    s_ack_sender = fn;
}

/* CODE: 0=成功 1=参数错误 2=CRC错误 3=未知命令 */
void Protocol_SendAck(uint8_t cmd, uint8_t code) {
    if (!s_ack_sender) return;
    uint8_t payload[2];
    payload[0] = cmd;
    payload[1] = code;
    uint8_t frame[8];
    uint16_t flen = 0;
    frame_pack(0x82, payload, 2, frame, &flen);
    s_ack_sender(frame, flen);
}

static void send_ack(uint8_t cmd, uint8_t code) {
    Protocol_SendAck(cmd, code);
}

/* ==================== 命令处理 ==================== */

void Protocol_ProcessFrame(const uint8_t* data, uint16_t len) {
    if (len < 6) return;
    if (data[0] != 0xAA || data[1] != 0x55) return;

    uint8_t cmd = data[2];
    uint8_t pay_len = data[3];
    if (len < (uint16_t)(pay_len + 6)) return;

    uint16_t crc_recv = data[4 + pay_len] | ((uint16_t)data[5 + pay_len] << 8);
    uint16_t crc_calc = Protocol_CRC16(data, pay_len + 4);
    if (crc_recv != crc_calc) { send_ack(cmd, 2); return; }

    const uint8_t* p = (pay_len > 0) ? &data[4] : NULL;

    uint8_t code = 0;   /* 0=OK 1=参数错误 3=未知命令 */

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
            Serial.printf("[CMD] 舵机%d -> %.1f° / %.1fs\n", id, angle, dur);
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
            Serial.printf("[CMD] ALL M1=%.1f M2=%.1f M3=%.1f\n", m1, m2, m3);
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
            Serial.printf("[CMD] CAN%d -> %.1f° / %.1fs\n", id, angle_deg, dur);
            break;
        }
        /* CAN命令: id(1) subcmd(1) — 0=使能 1=失能 2=设零 */
        case 0x05: {
            if (pay_len < 2 || !p) { code = 1; break; }
            uint8_t id = p[0], sub = p[1];
            if (sub == 0) { float cur = CAN_RS02_GetPosition(id); CAN_RS02_ResetTraj(id, cur); CAN_RS02_Enable(id); }
            else if (sub == 1) CAN_RS02_Disable(id);
            else if (sub == 2) { CAN_RS02_SetZero(id); CAN_RS02_ResetTraj(id, 0.0f); }
            const char* names[] = {"使能","失能","设零"};
            Serial.printf("[CMD] CAN%d %s\n", id, sub<3 ? names[sub] : "?");
            break;
        }
        /* 自动纠偏 */
        case 0x09: {
            if (!AutoAlign_IsActive()) {
                AutoAlign_Start();
                Serial.println("[CMD] 自动纠偏启动");
            }
            break;
        }
        /* 自动纠偏急停 */
        case 0x0A: {
            AutoAlign_Stop();
            Serial.println("[CMD] 自动纠偏急停");
            break;
        }
        /* 越障 */
        case 0x11: {
            GaitEngine_EnqueueCmd(0x11, NULL, 0);
            Serial.println("[CMD] 越障");
            break;
        }
        /* 启停 */
        case 0x12: {
            if (pay_len < 1 || !p) { code = 1; break; }
            uint8_t val = p[0];
            uint8_t data[1] = {val};
            GaitEngine_EnqueueCmd(0x12, data, 1);
            Serial.printf("[CMD] 启停 val=%d\n", val);
            break;
        }
        /* 迈步 */
        case 0x13: {
            if (pay_len < 4 || !p) { code = 1; break; }
            float step_mm;
            memcpy(&step_mm, &p[0], 4);
            GaitEngine_EnqueueCmd(0x10, (uint8_t*)&step_mm, 4);
            Serial.printf("[CMD] 迈步 %.0f mm\n", step_mm);
            break;
        }
        /* 掉头 */
        case 0x14: {
            GaitEngine_EnqueueCmd(0x14, NULL, 0);
            Serial.println("[CMD] 掉头");
            break;
        }
        /* 确认下一步 */
        case 0x15: {
            GaitEngine_ConfirmNext();
            Serial.println("[CMD] 确认下一步");
            break;
        }

        /* 0x1C: 激光自主避障行走 开关, payload: [0=停 1=开] */
        case 0x1C: {
            if (pay_len < 1 || !p) { code = 1; break; }
            if (p[0]) Avoid_Start();
            else       Avoid_Stop();
            Serial.printf("[CMD] 避障行走 %s\n", p[0] ? "开" : "关");
            break;
        }

        /* ==== NRF 手柄/协同 (新增) ==== */

        /* 0x18: 虚拟手柄控制, payload: LH(1) LV(1) RH(1) RV(1) KEY(1) */
        case 0x18: {
            if (pay_len < 5 || !p) { code = 1; break; }
            int8_t lh = (int8_t)p[0];
            int8_t lv = (int8_t)p[1];
            int8_t rh = (int8_t)p[2];
            int8_t rv = (int8_t)p[3];
            uint8_t key = p[4];
            Joypad_SetAxis(JOYPAD_CH_LH, lh);
            Joypad_SetAxis(JOYPAD_CH_LV, lv);
            Joypad_SetAxis(JOYPAD_CH_RH, rh);
            Joypad_SetAxis(JOYPAD_CH_RV, rv);
            if (key != 0) Joypad_PressKey(key);
            break;
        }
        /* 0x19 已移至 web_server (带 K230 ip[4] payload, 注入 CoopVision) */
        /* 0x1D: 协同中止 */
        case 0x1D: {
            AutoMode_Abort();
            Serial.println("[CMD] 协同中止");
            break;
        }
        /* 0x1B: 夹持器下发角度, payload: angle×10 (2LE), 0-180 */
        case 0x1B: {
            if (pay_len < 2 || !p) { code = 1; break; }
            uint16_t av = p[0] | ((uint16_t)p[1] << 8);
            float deg = av / 10.0f;
            if (deg > 180.0f) deg = 180.0f;
            if (!Gripper_SetTarget(deg)) { code = 1; break; }
            break;
        }
        /* 0x20: 子机运动帧直通, payload: speed(1) spin(1) key(1) */
        case 0x20: {
            if (pay_len < 3 || !p) { code = 1; break; }
            if (AutoMode_IsActive() || AlignTest_IsActive()) { code = 1; break; } /* 协同/纠偏测试中拒绝 */
            ChildCmd_Set((int8_t)p[0], (int8_t)p[1], p[2]);
            Serial.printf("[CMD] 子机直通 speed=%d spin=%d key=%u\n",
                          (int)(int8_t)p[0], (int)(int8_t)p[1], p[2]);
            break;
        }
        /* 0x21: 清空子机直通队列并发停止帧 */
        case 0x21: {
            ChildCmd_Clear();
            ChildCmd_Set(0, 0, 0);
            Serial.println("[CMD] 子机直通清空");
            break;
        }
        default: {
            code = 3;   /* 未知命令 */
            break;
        }
    }

    send_ack(cmd, code);
}

/* ==================== IMU 自动标定 ==================== */

static float g_imu_gnd = 0.0f;
static bool  g_imu_calibrated = false;

void Imu_SetGnd(float roll_deg) { g_imu_gnd = roll_deg; g_imu_calibrated = true; }
float Imu_GetGnd(void) { return g_imu_gnd; }
bool  Imu_IsCalibrated(void) { return g_imu_calibrated; }

void Imu_CalibrateOnce(void) {
    if (g_imu_calibrated) return;

    const int SAMPLE_COUNT = 15;
    float buf[SAMPLE_COUNT];
    int n = 0;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        vTaskDelay(pdMS_TO_TICKS(200));
        buf[n++] = FetchIMURoll();
    }
    for (int i = 0; i < n - 1; i++)
        for (int j = 0; j < n - 1 - i; j++)
            if (buf[j] > buf[j + 1]) { float t = buf[j]; buf[j] = buf[j + 1]; buf[j + 1] = t; }

    float sum = 0.0f; int cnt = 0;
    for (int i = 1; i < n - 1; i++) { sum += buf[i]; cnt++; }
    g_imu_gnd = (cnt > 0) ? (sum / cnt) : 0.0f;
    g_imu_calibrated = true;

    Serial.printf("[标定] IMU 基准采样完成: gnd=%.2f (采样%d, 去极平均)\n", g_imu_gnd, n);
}

/* ==================== 初始化 ==================== */

void Protocol_Init(void) {
    ChildCmd_Init();
    Serial.println("[PROTO] 协议模块初始化完成");
}