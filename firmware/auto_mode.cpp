#include "auto_mode.h"
#include "nrf_sender.h"
#include "servo_driver.h"
#include "child_cmd.h"
#include "coop_vision.h"
#include "align_test.h"
#include "web_server.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

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
    COOP_VISION_REF,       /* 记参考 (REDY① 后, PB9 抬起) */
    COOP_SEND_GOON,
    COOP_WAIT_DONE,        /* 等 REDY② (子机作业完成, 开始纠偏) */
    COOP_VISION,           /* 等 /align/once 结果, 算命令 */
    COOP_VISION_WAIT,      /* 等 REDY (子机走完一步) */
    COOP_VISION_WAIT_DONE, /* 等 DONE 前的 GOON 送达子机 */
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
static uint32_t g_waitDeadline = 0;   /* 各等待态统一超时 */

/* 视觉纠偏计数 */
static uint16_t g_cvIter = 0;
static uint8_t  g_cvAlignedHits = 0;
static uint8_t  g_refRetry = 0, g_measRetry = 0;

/* 待发送帧 (最长 ADJ,-10.0,-10.0,-30.0 = 21 字节) */
static bool g_wants_send = false;
static uint8_t g_send_buf[32] = {0};
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

/* 请求发送纠偏命令帧 (变长: ADJ,phi,psi,ax) */
static void request_send_adj(float phi, float psi, float ax) {
    int n = snprintf((char*)g_send_buf, sizeof(g_send_buf),
                     "ADJ,%.1f,%.1f,%.1f", (double)phi, (double)psi, (double)ax);
    if (n < 0) n = 0;
    if (n > (int)sizeof(g_send_buf)) n = (int)sizeof(g_send_buf);
    g_send_len = (uint8_t)n;
    g_wants_send = true;
}

void AutoMode_Init(void) { g_cs = COOP_IDLE; }
bool AutoMode_IsActive(void) { return g_cs != COOP_IDLE; }

void AutoMode_StartCoop(void) {
    if (g_cs != COOP_IDLE) {
        Serial.printf("[COOP] 已在协同中 (state=%u), 拒绝\n", (unsigned)g_cs);
        WebServer_SendLog("[COOP] 已在协同中, 拒绝再次启动");
        return;
    }
    if (AlignTest_IsActive()) {
        Serial.println("[COOP] align-test active, reject coop");
        WebServer_SendLog("[COOP] align-test active, reject");
        return;
    }
    CoopVision_Reset();
    g_cvIter = 0;
    g_cvAlignedHits = 0;
    g_refRetry = 0;
    g_measRetry = 0;
    Serial.println("[COOP] 完整协同: PB9 160->60");
    WebServer_SendLog("[COOP] 协同启动");
    Servo_SetPB9(60.0f, 2.0f);
    g_delay_deadline = millis() + 3000;
    g_cs = COOP_PB9_DOWN;
}

/* 协同中止: PB9/夹持器停在当前位置(不锁死), 发STOP通知子机, 再回IDLE */
void AutoMode_Abort(void) {
    if (g_cs == COOP_IDLE) return;
    Serial.println("[COOP] 人工介入, 立即退出协同");
    WebServer_SendLog("[COOP] 协同中止");
    /* 吸盘/夹持器: 冻结在当前角度, 停止插值且不改 PWM, 不参与接管后的控制 */
    Servo_HoldPB9();
    Servo_HoldGrip();
    g_gripper_polling = false;
    /* 丢弃未发出的子机直通指令, 避免接管后继续驱动子机 */
    ChildCmd_Clear();
    /* 丢弃视觉请求与过期测量结果 */
    CoopVision_Reset();
    g_cvIter = 0;
    g_cvAlignedHits = 0;
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
    /* 帧发送完成, 推进 SEND 状态到对应等待态, 并设等待超时 */
    if (g_cs == COOP_SEND_COOP) {
        g_cs = COOP_WAIT_REDY;
        g_waitDeadline = millis() + 10000;    /* 子机开机+风机 ~5s */
    } else if (g_cs == COOP_SEND_GOON) {
        g_cs = COOP_WAIT_DONE;
        g_waitDeadline = millis() + 120000;   /* 子机整个作业流程 */
    } else if (g_cs == COOP_SEND_LOCK) {
        g_cs = COOP_WAIT_ACKD;
        g_waitDeadline = millis() + 10000;
    } else if (g_cs == COOP_SEND_STOP) {
        g_cs = COOP_IDLE;
    }
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
            Serial.println("[COOP] 收到 REDY①, PB9 60->160");
            NRF_StopListening();
            Servo_SetPB9(160.0f, 2.0f);
            g_delay_deadline = millis() + 3000;
            g_cs = COOP_PB9_UP_1;
        } else if (millis() > g_waitDeadline) {
            Serial.println("[COOP] 等 REDY① 超时, 中止");
            AutoMode_Abort();
        }
        break;
    case COOP_PB9_UP_1:
        if (millis() > g_delay_deadline) {
            Serial.println("[COOP] 抬起到位, 开始记参考");
            CoopVision_RequestRef();
            g_refRetry = 0;
            g_waitDeadline = millis() + 15000;
            g_cs = COOP_VISION_REF;
        }
        break;
    case COOP_VISION_REF: {
        CoopVisionState cv = CoopVision_GetState();
        if (cv == CV_REF_DONE) {
            Serial.println("[COOP] 参考位完成, 发 GOON");
            WebServer_SendLog("[COOP] 参考位完成");
            request_send(FRAME_GOON);
            g_cs = COOP_SEND_GOON;
        } else if (cv == CV_REF_FAIL) {
            g_refRetry++;
            if (g_refRetry >= 3) {
                Serial.println("[COOP] 记参考连续失败, 中止协同");
                WebServer_SendLog("[COOP] 记参考连续失败, 中止");
                AutoMode_Abort();
            } else {
                Serial.printf("[COOP] 记参考失败, 重试 %u/3\n", (unsigned)g_refRetry);
                CoopVision_RequestRef();
            }
        } else if (millis() > g_waitDeadline) {
            Serial.println("[COOP] 记参考超时, 中止");
            AutoMode_Abort();
        }
        break;
    }
    case COOP_SEND_GOON:
        break;
    case COOP_WAIT_DONE:
        if (recv_match_or_imu(FRAME_REDY)) {
            Serial.println("[COOP] 收 REDY②, 开始视觉纠偏");
            WebServer_SendLog("[COOP] 子机作业完成, 开始纠偏");
            g_cvIter = 0;
            g_cvAlignedHits = 0;
            g_measRetry = 0;
            CoopVision_RequestMeasure();
            g_waitDeadline = millis() + 15000;
            g_cs = COOP_VISION;
        } else if (millis() > g_waitDeadline) {
            Serial.println("[COOP] 等子机作业(REDY②)超时, 中止");
            AutoMode_Abort();
        }
        break;
    case COOP_VISION: {
        CoopVisionState cv = CoopVision_GetState();
        if (cv == CV_MEAS_DONE) {
            float phi, psi, ax;
            bool aligned;
            CoopVision_GetDev(&phi, &psi, &ax, &aligned);
            g_cvIter++;
            Serial.printf("[CV] it=%u meas phi=%.2f psi=%.2f ax=%.1f aligned=%d\n",
                          (unsigned)g_cvIter, (double)phi, (double)psi, (double)ax,
                          aligned ? 1 : 0);
            char lb[96];
            snprintf(lb, sizeof(lb), "[CV] it=%u phi=%.2f psi=%.2f ax=%.1f%s",
                     (unsigned)g_cvIter, (double)phi, (double)psi, (double)ax,
                     aligned ? " 已对齐" : "");
            WebServer_SendLog(lb);
            /* 轮次安全阀: 正常终止只靠收敛判据, 这里防振荡/不收敛死循环 */
            if (g_cvIter >= AV_SAFETY_MAX_ITER) {
                Serial.println("[CV] 达轮次安全上限, 中止");
                WebServer_SendLog("[CV] 达轮次安全上限, 中止");
                AutoMode_Abort();
                break;
            }
            if (aligned) {
                g_cvAlignedHits++;
                if (g_cvAlignedHits >= AV_ALIGNED_HITS) {
                    Serial.printf("[CV] 收敛, 共 %u 轮\n", (unsigned)g_cvIter);
                    WebServer_SendLog("[CV] 视觉收敛, 进入夹紧");
                    request_send(FRAME_GOON);
                    g_waitDeadline = millis() + 10000;
                    g_cs = COOP_VISION_WAIT_DONE;
                } else {
                    Serial.printf("[CV] aligned hit %u/%u, 复测\n",
                                  (unsigned)g_cvAlignedHits, (unsigned)AV_ALIGNED_HITS);
                    CoopVision_RequestMeasure();
                    g_waitDeadline = millis() + 15000;
                }
            } else {
                g_cvAlignedHits = 0;
                float cPhi, cPsi, cAx;
                CoopVision_ComputeCmd(phi, psi, ax, &cPsi, &cPhi, &cAx);
                Serial.printf("[CV] it=%u cmd phi=%.2f psi=%.2f ax=%.1f\n",
                              (unsigned)g_cvIter, (double)cPhi, (double)cPsi, (double)cAx);
                if (cPhi == 0.0f && cPsi == 0.0f && cAx == 0.0f) {
                    /* 全死区但未 aligned: 按收敛处理 */
                    Serial.println("[CV] 命令全死区, 视为收敛");
                    WebServer_SendLog("[CV] 命令全死区, 视为收敛");
                    request_send(FRAME_GOON);
                    g_waitDeadline = millis() + 10000;
                    g_cs = COOP_VISION_WAIT_DONE;
                } else {
                    request_send_adj(cPhi, cPsi, cAx);
                    g_waitDeadline = millis() + 70000;   /* 子机单步最长 */
                    g_cs = COOP_VISION_WAIT;
                }
            }
        } else if (cv == CV_MEAS_FAIL) {
            g_measRetry++;
            if (g_measRetry >= 3) {
                Serial.println("[CV] 测量连续失败, 中止");
                WebServer_SendLog("[CV] 测量连续失败, 中止");
                AutoMode_Abort();
            } else {
                Serial.printf("[CV] 测量失败, 重试 %u/3\n", (unsigned)g_measRetry);
                CoopVision_RequestMeasure();
            }
        } else if (millis() > g_waitDeadline) {
            Serial.println("[CV] 测量超时, 中止");
            AutoMode_Abort();
        }
        break;
    }
    case COOP_VISION_WAIT:
        if (recv_match_or_imu(FRAME_REDY)) {
            Serial.printf("[CV] it=%u 子机走完, 复测\n", (unsigned)g_cvIter);
            CoopVision_RequestMeasure();
            g_waitDeadline = millis() + 15000;
            g_cs = COOP_VISION;
        } else if (millis() > g_waitDeadline) {
            Serial.println("[CV] 子机步超时, 中止");
            AutoMode_Abort();
        }
        break;
    case COOP_VISION_WAIT_DONE:
        if (recv_match(FRAME_DONE)) {
            Serial.println("[COOP] 收 DONE, PB9 160->80");
            NRF_StopListening();
            Servo_SetPB9(80.0f, 2.0f);
            g_delay_deadline = millis() + 3000;
            g_cs = COOP_PB9_DOWN_2;
        } else if (millis() > g_waitDeadline) {
            Serial.println("[COOP] DONE 超时, 中止");
            AutoMode_Abort();
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
        } else if (millis() > g_waitDeadline) {
            Serial.println("[COOP] 等 ACKD 超时, 中止");
            AutoMode_Abort();
        }
        break;
    case COOP_PB9_UP_2:
        if (millis() > g_delay_deadline) {
            g_cs = COOP_IDLE;
            Serial.println("[COOP] 协同完成");
            WebServer_SendLog("[COOP] 协同完成");
        }
        break;
    default: break;
    }
}

/* ==================== 0x88 状态帧填充 ==================== */

void AutoMode_FillVisionStatus(uint8_t out[11]) {
    float phi = 0.0f, psi = 0.0f, ax = 0.0f;
    bool aligned = false;
    CoopVision_GetDev(&phi, &psi, &ax, &aligned);
    uint8_t st;
    switch (g_cs) {
    case COOP_VISION_REF:
    case COOP_VISION:
    case COOP_VISION_WAIT:
    case COOP_VISION_WAIT_DONE:
        st = (uint8_t)CoopVision_GetState();
        break;
    case COOP_IDLE:
        st = (uint8_t)CV_IDLE;
        break;
    default:
        st = 200;   /* 协同进行中(非视觉阶段) */
        break;
    }
    out[0] = st;
    out[1] = (uint8_t)g_cvIter;
    out[2] = 0;
    int16_t p = (int16_t)lroundf(phi * 100.0f);
    int16_t q = (int16_t)lroundf(psi * 100.0f);
    int16_t a = (int16_t)lroundf(ax);
    memcpy(&out[3], &p, 2);
    memcpy(&out[5], &q, 2);
    memcpy(&out[7], &a, 2);
    out[9] = aligned ? 1 : 0;
    out[10] = (uint8_t)AV_SAFETY_MAX_ITER;
}