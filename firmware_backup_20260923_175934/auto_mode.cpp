#include "auto_mode.h"
#include "nrf_sender.h"
#include "servo_driver.h"
#include "config.h"
#include <Arduino.h>
#include <string.h>

typedef enum {
    COOP_IDLE = 0,
    COOP_PB9_DOWN,
    COOP_GRIPPER_OPEN,
    COOP_SEND_KEY10,
    COOP_WAIT_FAN_OK,
    COOP_WAIT_FAN_DELAY,
    COOP_PB9_UP_1,
    COOP_WAIT_DONE,
    COOP_PB9_DOWN_2,
    COOP_PRE_GRIP,
    COOP_SEND_KEY6,
    COOP_GRIPPER_CLOSE,
    COOP_PB9_UP_2,
} CoopState;

static CoopState g_cs = COOP_IDLE;
static bool g_is_coop = false;
static bool g_wants_key = false;
static uint8_t g_out_key = 0;
static uint32_t g_delay_deadline = 0;

static uint8_t g_rx_buf[8] = {0};
static uint8_t g_rx_idx = 0;

// 夹持器轮询状态
static bool g_gripper_polling = false;
static float g_gripper_target = 0.0;
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
        g_gripper_target = target;
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

void AutoMode_Init(void) { g_cs = COOP_IDLE; }
bool AutoMode_IsActive(void) { return g_cs != COOP_IDLE; }

void AutoMode_StartCoop(void) {
    Serial.println("[COOP] 完整协同: PB9 160->70");
    Servo_SetPB9(70.0f, 2.0f);
    g_delay_deadline = millis() + 3000;
    g_cs = COOP_PB9_DOWN;
    g_is_coop = true;
}

void AutoMode_StartKey10(void) {
    Serial.println("[AUTO] 发KEY=10, 死等DONE");
    g_gripper_polling = false;
    g_out_key = 10; g_wants_key = true;
    g_cs = COOP_SEND_KEY10; g_is_coop = false;
}

bool AutoMode_WantsSendKey(uint8_t* out_key) {
    if (g_wants_key) { *out_key = g_out_key; g_wants_key = false; return true; }
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
            Serial.println("[COOP] 夹持器到位, 发KEY=10");
            g_out_key = 10; g_wants_key = true;
            g_cs = COOP_SEND_KEY10;
        }
        break;
    case COOP_SEND_KEY10:
        break;
    case COOP_WAIT_FAN_OK: {
        uint8_t len = 0;
        uint8_t tmp[32];
        if (NRF_ReceiveFrame(tmp, sizeof(tmp), &len)) {
            for (uint8_t i = 0; i < len; i++) {
                g_rx_buf[g_rx_idx] = tmp[i]; g_rx_idx++;
                if (g_rx_idx >= 6) {
                    if (memcmp(g_rx_buf, "FAN_OK", 6) == 0) {
                        g_rx_idx = 0; memset(g_rx_buf, 0, 8);
                        if (g_is_coop) {
                            NRF_StopListening();
                            Serial.println("[COOP] 收到FAN_OK, 等待2秒...");
                            g_delay_deadline = millis() + 2000;
                            g_cs = COOP_WAIT_FAN_DELAY;
                        } else {
                            Serial.println("[AUTO] 收到FAN_OK, 等DONE");
                            g_cs = COOP_WAIT_DONE;
                        }
                        break;
                    }
                    memmove(g_rx_buf, g_rx_buf+1, 5); g_rx_idx = 5;
                }
            }
        }
        break;
    }
    case COOP_WAIT_FAN_DELAY:
        if (millis() > g_delay_deadline) {
            Serial.println("[COOP] 2秒到, PB9 70->160");
            Servo_SetPB9(160.0f, 2.0f);
            g_delay_deadline = millis() + 3000;
            g_cs = COOP_PB9_UP_1;
        }
        break;
    case COOP_PB9_UP_1:
        if (millis() > g_delay_deadline) {
            /* PB9已复位，发OK_GO给STM32让它开始下行 */
            NRF_StopListening();
            uint8_t okgo[] = {'O','K','_','G','O'};
            NRF_SendFrame(okgo, 5);
            Serial.println("[COOP] 发送OK_GO");
            NRF_StartListening();
            g_rx_idx = 0; memset(g_rx_buf, 0, 8);
            g_cs = COOP_WAIT_DONE;
        }
        break;
    case COOP_WAIT_DONE: {
        uint8_t len = 0;
        uint8_t tmp[32];
        if (NRF_ReceiveFrame(tmp, sizeof(tmp), &len)) {
            /* === 子机 IMU 帧 === */
            if (len >= 9 && tmp[0] == 'I') {
                memcpy(&g_child_pitch, &tmp[1], 4);
                memcpy(&g_child_roll, &tmp[5], 4);
                g_child_imu_updated = true;
                break;
            }
            for (uint8_t i = 0; i < len; i++) {
                g_rx_buf[g_rx_idx] = tmp[i]; g_rx_idx++;
                if (g_rx_idx >= 4) {
                    if (memcmp(g_rx_buf, "DONE", 4) == 0) {
                        g_rx_idx = 0; memset(g_rx_buf, 0, 8);
                        NRF_StopListening();
                        if (g_is_coop) {
                            Serial.println("[COOP] 收到DONE, PB9 160->80");
                            Servo_SetPB9(80.0f, 2.0f);
                            g_delay_deadline = millis() + 3000;
                            g_cs = COOP_PB9_DOWN_2;
                        } else {
                            Serial.println("[AUTO] 收到DONE, 发KEY=6");
                            g_out_key = 6; g_wants_key = true;
                            g_cs = COOP_SEND_KEY6;
                        }
                        break;
                    }
if (g_rx_buf[0] == 'D' && g_rx_buf[1] != 'O') {
                        g_rx_idx = 0; memset(g_rx_buf, 0, 8);
                        NRF_StopListening();
                        if (g_is_coop) {
                            Serial.println("[COOP] 收到D(单字节), PB9 160->80");
                            Servo_SetPB9(80.0f, 2.0f);
                            g_delay_deadline = millis() + 3000;
                            g_cs = COOP_PB9_DOWN_2;
                        } else {
                            Serial.println("[AUTO] 收到D, 发KEY=6");
                            g_out_key = 6; g_wants_key = true;
                            g_cs = COOP_SEND_KEY6;
                        }
                        break;
                    }
                    memmove(g_rx_buf, g_rx_buf+1, 3); g_rx_idx = 3;
                }
            }
        }
        break;
    }
    case COOP_PB9_DOWN_2:
        if (millis() > g_delay_deadline) {
            Serial.println("[COOP] PB9下压完成, 预夹紧40°");
            g_gripper_polling = false;
            g_cs = COOP_PRE_GRIP;
        }
        break;
    case COOP_PRE_GRIP:
        if (poll_gripper(40.0)) {
            Serial.println("[COOP] 预夹紧到位, 发KEY=6");
            g_out_key = 6; g_wants_key = true;
            g_cs = COOP_SEND_KEY6;
        }
        break;
    case COOP_SEND_KEY6:
        break;
    case COOP_GRIPPER_CLOSE:
        if (millis() < g_delay_deadline) break;
        if (poll_gripper(0.0)) {
            Serial.println("[COOP] 夹持器到位, PB9 80->160");
            Servo_SetPB9(160.0f, 2.0f);
            g_delay_deadline = millis() + 3000;
            g_cs = COOP_PB9_UP_2;
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

void AutoMode_OnSent(uint8_t key) {
    if (key == 10 && g_cs == COOP_SEND_KEY10) {
        Serial.println("[COOP] KEY=10已发, 切RX等FAN_OK");
        NRF_StartListening();
        g_rx_idx = 0;
        memset(g_rx_buf, 0, 8);
        g_cs = COOP_WAIT_FAN_OK;
    } else if (key == 6 && g_cs == COOP_SEND_KEY6) {
        if (g_is_coop) {
            g_gripper_polling = false;
            g_delay_deadline = millis() + 2000;
            g_cs = COOP_GRIPPER_CLOSE;
        } else {
            g_cs = COOP_IDLE;
            Serial.println("[AUTO] 简化自动完成");
        }
    }
}