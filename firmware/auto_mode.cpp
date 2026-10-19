#include "auto_mode.h"
#include "nrf_sender.h"
#include "servo_driver.h"
#include "child_cmd.h"
#include "config.h"
#include <Arduino.h>
#include <string.h>

/* ===== 子母协同协议 (4 字节 ASCII, 母机侧) ===== */
static const uint8_t FRAME_COOP[4] = {'C','O','O','P'};  /* 母->子: 开夹持器130到位 */
static const uint8_t FRAME_GOON[4] = {'G','O','O','N'};  /* 母->子: 抬起160放行 */
static const uint8_t FRAME_LOCK[4] = {'L','O','C','K'};  /* 母->子: 预夹+夹紧完成 */
static const uint8_t FRAME_REDY[4] = {'R','E','D','Y'};  /* 子->母: 就绪 */
static const uint8_t FRAME_DONE[4] = {'D','O','N','E'};  /* 子->母: 工作完成 */
static const uint8_t FRAME_ACKD[4] = {'A','C','K','D'};  /* 子->母: 确认 */
static const uint8_t FRAME_STOP[4] = {'S','T','O','P'};  /* 母->子: 协同中止 */

typedef enum {
    COOP_IDLE = 0,
    COOP_PB9_DOWN,
    COOP_GRIPPER_OPEN,
    COOP_SEND_COOP,
    COOP_WAIT_REDY,
    COOP_PB9_UP_1,
    COOP_SEND_GOON,
    COOP_WAIT_DONE,
    COOP_PB9_DOWN_2,
    COOP_PRE_GRIP,
    COOP_GRIPPER_CLOSE,
    COOP_SEND_LOCK,
    COOP_WAIT_ACKD,
    COOP_PB9_UP_2,
    COOP_SEND_STOP,
} CoopState;

static CoopState g_cs = COOP_IDLE;
static uint32_t g_delay_deadline = 0;

/* 待发送帧 */
static bool g_wants_send = false;
static uint8_t g_send_buf[4] = {0};
static uint8_t g_send_len = 0;

/* 接收滑动窗口 */
static uint8_t g_rx_buf[8] = {0};
static uint8_t g_rx_idx = 0;

// 夹持器轮询状态
static bool g_gripper_polling = false;
static uint32_t g_gripper_timeout = 0;

/* === 子机 IMU 数据 === */
static float g_child_pitch = 0.0f;
static float g_child_roll = 0.0f;
static bool g_child_imu_updated = false;

float AutoMode_GetChildPitch(void) { return g_child_pitch; }
float AutoMode_GetChildRoll(void) { return g_child_roll; }
bool AutoMode_IsChildIMUUpdated(void) { return g_child_imu_updated; }
void AutoMode_ClearChildIMUFlag(void) { g_child_imu_updated = false; }

static bool poll_gripper(float target) {
    if (!g_gripper_polling) {
        Servo_SetGrip(target, 1.5f);
        g_gripper_polling = true;
        g_gripper_timeout = millis() + 2000;
        Serial.printf("[COOP] 夹持器 -> %.1f°\n", target);
        return false;
    }
    if (millis() >= g_gripper_timeout) {
        g_gripper_polling = false;
        Serial.printf("[COOP] 夹持器到位: %.1f°\n", target);
        return true;
    }
    return false;
}

/* 请求发送一帧 (4 字节) */
static void request_send(const uint8_t* frame) {
    memcpy(g_send_buf, frame, 4);
    g_send_len = 4;
    g_wants_send = true;
}

void AutoMode_Init(void) { g_cs = COOP_IDLE; }
bool AutoMode_IsActive(void) { return g_cs != COOP_IDLE; }

void AutoMode_StartCoop(void) {
    Serial.println("[COOP] 完整协同: PB9 160->70");
    Servo_SetPB9(70.0f, 2.0f);
    g_delay_deadline = millis() + 3000;
    g_cs = COOP_PB9_DOWN;
}

/* 协同中止: PB9/夹持器停在当前位置(不锁死), 发STOP通知子机, 再回IDLE */
void AutoMode_Abort(void) {
    if (g_cs == COOP_IDLE) return;
    Serial.println("[COOP] 人工介入, 立即退出协同");
    /* 吸盘/夹持器: 冻结在当前角度, 停止插值且不改 PWM, 不参与接管后的控制 */
    Servo_HoldPB9();
    Servo_HoldGrip();
    g_gripper_polling = false;
    /* 丢弃未发出的子机直通指令, 避免接管后继续驱动子机 */
    ChildCmd_Clear();
    /* 通知子机中止: 帧先入队, 由发送任务下一次循环发出 */
    request_send(FRAME_STOP);
    /* 立即回 IDLE, 不等 STOP 帧发送结果, 保证子机控制通道立刻可用 */
    g_cs = COOP_IDLE;
    NRF_StopListening();
}

bool AutoMode_WantsSendFrame(uint8_t* out_buf, uint8_t* out_len) {
    if (g_wants_send) {
        memcpy(out_buf, g_send_buf, g_send_len);
        *out_len = g_send_len;
        g_wants_send = false;
        return true;
    }
    return false;
}

void AutoMode_OnFrameSent(void) {
    NRF_StartListening();
    g_rx_idx = 0;
    memset(g_rx_buf, 0, 8);
    /* 帧发送完成, 推进 SEND 状态到对应等待态 */
    if (g_cs == COOP_SEND_COOP)      g_cs = COOP_WAIT_REDY;
    else if (g_cs == COOP_SEND_GOON) g_cs = COOP_WAIT_DONE;
    else if (g_cs == COOP_SEND_LOCK) g_cs = COOP_WAIT_ACKD;
    else if (g_cs == COOP_SEND_STOP) g_cs = COOP_IDLE;
}

/* 收帧: 命中返回 true, 并把匹配帧拷到 match_out */
static bool recv_match(const uint8_t* target) {
    uint8_t len = 0;
    uint8_t tmp[32];
    while (NRF_ReceiveFrame(tmp, sizeof(tmp), &len)) {
        for (uint8_t i = 0; i <len; i++) {
            g_rx_buf[g_rx_idx] = tmp[i]; g_rx_idx++;
            if (g_rx_idx >= 4) {
                if (memcmp(g_rx_buf, target, 4) == 0) {
                    g_rx_idx = 0; memset(g_rx_buf, 0, 8);
                    return true;
                }
                memmove(g_rx_buf, g_rx_buf+1, 3); g_rx_idx = 3;
            }
        }
    }
    return false;
}

/* 收帧, 同时解析子机 IMU 帧 */
static bool recv_match_or_imu(const uint8_t* target) {
    uint8_t len = 0;
    uint8_t tmp[32];
    while (NRF_ReceiveFrame(tmp, sizeof(tmp), &len)) {
        if (len >= 9 && tmp[0] == 'I') {
            memcpy(&g_child_pitch, &tmp[1], 4);
            memcpy(&g_child_roll, &tmp[5], 4);
            g_child_imu_updated = true;
            continue;
        }
        for (uint8_t i = 0; i <len; i++) {
            g_rx_buf[g_rx_idx] = tmp[i]; g_rx_idx++;
            if (g_rx_idx >= 4) {
                if (memcmp(g_rx_buf, target, 4) == 0) {
                    g_rx_idx = 0; memset(g_rx_buf, 0, 8);
                    return true;
                }
                memmove(g_rx_buf, g_rx_buf+1, 3); g_rx_idx = 3;
            }
        }
    }
    return false;
}

void AutoMode_Tick(void) {
    switch (g_cs) {
    case COOP_PB9_DOWN:
        if (millis() > g_delay_deadline) {
            Serial.println("[COOP] PB9到位, 打开夹持器");
            g_gripper_polling = false;
            g_cs = COOP_GRIPPER_OPEN;
        }
        break;
    case COOP_GRIPPER_OPEN:
        if (poll_gripper(130.0)) {
            Serial.println("[COOP] 夹持器到位, 发 COOP");
            request_send(FRAME_COOP);
            g_cs = COOP_SEND_COOP;
        }
        break;
    case COOP_SEND_COOP:
        break;
    case COOP_WAIT_REDY:
        if (recv_match(FRAME_REDY)) {
            Serial.println("[COOP] 收到 REDY, PB9 70->160");
            NRF_StopListening();
            Servo_SetPB9(160.0f, 2.0f);
            g_delay_deadline = millis() + 3000;
            g_cs = COOP_PB9_UP_1;
        }
        break;
    case COOP_PB9_UP_1:
        if (millis() > g_delay_deadline) {
            Serial.println("[COOP] 抬起完成, 发 GOON");
            request_send(FRAME_GOON);
            g_cs = COOP_SEND_GOON;
        }
        break;
    case COOP_SEND_GOON:
        break;
    case COOP_WAIT_DONE:
        if (recv_match_or_imu(FRAME_DONE)) {
            Serial.println("[COOP] 收到 DONE, PB9 160->95");
            NRF_StopListening();
            Servo_SetPB9(95.0f, 2.0f);
            g_delay_deadline = millis() + 3000;
            g_cs = COOP_PB9_DOWN_2;
        }
        break;
    case COOP_PB9_DOWN_2:
        if (millis() > g_delay_deadline) {
            Serial.println("[COOP] PB9下压完成, 预夹紧40°");
            g_gripper_polling = false;
            g_cs = COOP_PRE_GRIP;
        }
        break;
    case COOP_PRE_GRIP:
        if (poll_gripper(40.0)) {
            Serial.println("[COOP] 预夹紧到位, 发 LOCK");
            request_send(FRAME_LOCK);
            g_cs = COOP_SEND_LOCK;
        }
        break;
    case COOP_GRIPPER_CLOSE:
        if (poll_gripper(0.0)) {
            Serial.println("[COOP] 夹紧完成, PB9 80->160");
            g_gripper_polling = false;
            Servo_SetPB9(160.0f, 2.0f);
            g_delay_deadline = millis() + 3000;
            g_cs = COOP_PB9_UP_2;
        }
        break;
    case COOP_SEND_LOCK:
        break;
    case COOP_SEND_STOP:
        /* STOP 已发出 (或超时未发出), 确保退出协同 */
        g_cs = COOP_IDLE;
        break;
    case COOP_WAIT_ACKD:
        if (recv_match(FRAME_ACKD)) {
            Serial.println("[COOP] 收到 ACKD, 夹持器回0°夹紧");
            NRF_StopListening();
            g_gripper_polling = false;
            g_cs = COOP_GRIPPER_CLOSE;
        }
        break;
    case COOP_PB9_UP_2:
        if (millis() > g_delay_deadline) {
            g_cs = COOP_IDLE;
            Serial.println("[COOP] 协同完成");
        }
        break;
    default: break;
    }
}