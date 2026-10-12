/*
 * imu_driver.cpp - JY62 串口驱动 (带一阶低通滤波 + 调试统计)
 * 协议: 0x55 TYPE DATA1L DATA1H ... DATA4L DATA4H SUM
 * 角度帧 0x53: Roll Pitch Yaw 版本号
 * 滤波: filtered = filtered*(1-α) + raw*α
 * 接线: 母机 GPIO40(RX) <- JY62 TX, GPIO39(TX) -> JY62 RX
 */

#include "imu_driver.h"
#include "config.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <stdio.h>
#include <string.h>

static HardwareSerial IMUSerial(1);

static float g_roll = 0.0f, g_roll_filt = 0.0f;
static float g_pitch = 0.0f, g_pitch_filt = 0.0f;
static float g_yaw = 0.0f, g_yaw_filt = 0.0f;
static bool g_has_data = false;

static uint8_t rx_buf[64];
static uint8_t rx_idx = 0;

/* ===== 调试统计 ===== */
static volatile uint32_t dbg_bytes = 0;       /* 串口累计收到字节数 */
static volatile uint32_t dbg_updates = 0;     /* IMU_Update() 调用次数 */
static volatile uint32_t dbg_frames = 0;      /* 校验通过的总帧数(任意类型) */
static volatile uint32_t dbg_ang_frames = 0;  /* 0x53 角度帧数 */
static volatile uint32_t dbg_sum_fail = 0;    /* 校验和失败帧数 */
static volatile uint32_t dbg_last_ms = 0;     /* 最近收到有效角度帧的时刻 */
static uint8_t dbg_last_frame[11];            /* 最近一个完整帧 */
static bool dbg_has_frame = false;
static char dbg_rx_tail[64];                  /* 最近收到的原始字节 hex */
static uint8_t dbg_rx_tail_len = 0;

static uint8_t calc_sum(uint8_t* data, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i <len; i++) sum += data[i];
    return sum & 0xFF;
}

void IMU_Init(void) {
    IMUSerial.begin(IMU_BAUD, SERIAL_8N1, IMU_RX_PIN, IMU_TX_PIN);
    Serial.printf("[IMU] JY62 串口初始化完成 (baud=%d, RX=GPIO%d, TX=GPIO%d)\n",
                  IMU_BAUD, IMU_RX_PIN, IMU_TX_PIN);
}

bool IMU_Update(void) {
    bool updated = false;
    dbg_updates++;

    while (IMUSerial.available()) {
        uint8_t b = IMUSerial.read();
        dbg_bytes++;

        /* 记录最近 32 字节原始 hex (环形) */
        if (dbg_rx_tail_len < 32) {
            dbg_rx_tail[dbg_rx_tail_len * 3] = "0123456789ABCDEF"[b >> 4];
            dbg_rx_tail[dbg_rx_tail_len * 3 + 1] = "0123456789ABCDEF"[b & 0x0F];
            dbg_rx_tail[dbg_rx_tail_len * 3 + 2] = ' ';
            dbg_rx_tail_len++;
            dbg_rx_tail[dbg_rx_tail_len * 3] = '\0';
        } else {
            memmove(dbg_rx_tail, dbg_rx_tail + 3, 31 * 3);
            dbg_rx_tail[31 * 3] = "0123456789ABCDEF"[b >> 4];
            dbg_rx_tail[31 * 3 + 1] = "0123456789ABCDEF"[b & 0x0F];
            dbg_rx_tail[31 * 3 + 2] = ' ';
            dbg_rx_tail[32 * 3] = '\0';
        }

        if (b == 0x55) {
            rx_idx = 0;
            rx_buf[rx_idx++] = b;
        } else if (rx_idx > 0) {
            rx_buf[rx_idx++] = b;

            if (rx_idx == 11) {
                uint8_t sum = calc_sum(rx_buf, 10);
                if (sum == rx_buf[10]) {
                    dbg_frames++;
                    memcpy(dbg_last_frame, rx_buf, 11);
                    dbg_has_frame = true;
                    uint8_t type = rx_buf[1];

                    if (type == 0x53) {
                        int16_t roll_raw  = (int16_t)rx_buf[3] << 8 | rx_buf[2];
                        int16_t pitch_raw = (int16_t)rx_buf[5] << 8 | rx_buf[4];
                        int16_t yaw_raw   = (int16_t)rx_buf[7] << 8 | rx_buf[6];

                        float r = (float)roll_raw / 32768.0f * 180.0f;
                        float p = (float)pitch_raw / 32768.0f * 180.0f;
                        float y = (float)yaw_raw / 32768.0f * 180.0f;

                        if (!g_has_data) {
                            g_roll_filt = r; g_pitch_filt = p; g_yaw_filt = y;
                            g_has_data = true;
                        } else {
                            g_roll_filt = g_roll_filt * (1.0f - IMU_FILTER_ALPHA) + r * IMU_FILTER_ALPHA;
                            g_pitch_filt = g_pitch_filt * (1.0f - IMU_FILTER_ALPHA) + p * IMU_FILTER_ALPHA;
                            g_yaw_filt = g_yaw_filt * (1.0f - IMU_FILTER_ALPHA) + y * IMU_FILTER_ALPHA;
                        }
                        g_roll = r; g_pitch = p; g_yaw = y;
                        dbg_ang_frames++;
                        dbg_last_ms = millis();
                        updated = true;
                    }
                } else {
                    dbg_sum_fail++;
                }
                rx_idx = 0;
            }
        }
    }
    return updated;
}

float IMU_GetRoll(void)  { return g_roll_filt; }
float IMU_GetPitch(void) { return g_pitch_filt; }
float IMU_GetYaw(void)   { return g_yaw_filt; }

int IMU_GetDebug(char* buf, unsigned int n) {
    if (!buf || n == 0) return 0;
    uint32_t now = millis();
    uint32_t age = dbg_last_ms ? (now - dbg_last_ms) : 0;

    char hex[11 * 3 + 1];
    hex[0] = '\0';
    if (dbg_has_frame) {
        int k = 0;
        for (int i = 0; i < 11; i++) {
            hex[k++] = "0123456789ABCDEF"[dbg_last_frame[i] >> 4];
            hex[k++] = "0123456789ABCDEF"[dbg_last_frame[i] & 0x0F];
            hex[k++] = ' ';
        }
        hex[k] = '\0';
    }

    int written = snprintf(buf, n,
        "IMU DEBUG\n"
        "init: RX=GPIO%d TX=GPIO%d baud=%d alpha=%.2f\n"
        "updates=%lu\n"
        "bytes=%lu\n"
        "frames_ok=%lu\n"
        "ang_frames_0x53=%lu\n"
        "sum_fail=%lu\n"
        "last_frame_age_ms=%lu\n"
        "has_data=%d\n"
        "raw: roll=%.3f pitch=%.3f yaw=%.3f\n"
        "filt: roll=%.3f pitch=%.3f yaw=%.3f\n"
        "last_frame_hex: %s\n"
        "rx_tail_hex: %s\n"
        "verdict: %s\n",
        IMU_RX_PIN, IMU_TX_PIN, IMU_BAUD, IMU_FILTER_ALPHA,
        (unsigned long)dbg_updates,
        (unsigned long)dbg_bytes,
        (unsigned long)dbg_frames,
        (unsigned long)dbg_ang_frames,
        (unsigned long)dbg_sum_fail,
        (unsigned long)age,
        g_has_data ? 1 : 0,
        g_roll, g_pitch, g_yaw,
        g_roll_filt, g_pitch_filt, g_yaw_filt,
        hex,
        dbg_rx_tail_len ? dbg_rx_tail : "(none)",
        (!g_has_data && dbg_bytes == 0) ? "NO_SERIAL_DATA(check wiring/baud/pins)" :
        (!g_has_data && dbg_bytes > 0 && dbg_frames == 0) ? "GARBAGE_BYTES(no valid frame: baud/protocol mismatch)" :
        (!g_has_data && dbg_frames > 0 && dbg_ang_frames == 0) ? "FRAMES_OK_BUT_NO_0x53(wrong frame type)" :
        (g_has_data && age > 2000) ? "STALE(last 0x53 >2s ago)" :
        (g_has_data && g_roll == 0.0f && g_pitch == 0.0f && g_yaw == 0.0f) ? "ALL_ZERO(sensor not streaming angles)" :
        "OK");
    return written;
}