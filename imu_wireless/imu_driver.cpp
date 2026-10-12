/*
 * imu_driver.cpp - JY62 串口驱动 (带一阶低通滤波)
 * 协议: 0x55 TYPE DATA1L DATA1H ... DATA4L DATA4H SUM
 * 角度帧 0x53: Roll Pitch Yaw 版本号
 * 滤波: filtered = filtered*(1-α) + raw*α
 */

#include "imu_driver.h"
#include "config.h"
#include <Arduino.h>
#include <HardwareSerial.h>

static HardwareSerial IMUSerial(1);

static float g_roll = 0.0f, g_roll_filt = 0.0f;
static float g_pitch = 0.0f, g_pitch_filt = 0.0f;
static float g_yaw = 0.0f, g_yaw_filt = 0.0f;
static bool g_has_data = false;

static uint8_t rx_buf[64];
static uint8_t rx_idx = 0;

static uint8_t calc_sum(uint8_t* data, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) sum += data[i];
    return sum & 0xFF;
}

void IMU_Init(void) {
    IMUSerial.begin(IMU_BAUD, SERIAL_8N1, IMU_RX_PIN, IMU_TX_PIN);
    Serial.println("[IMU] JY62 串口初始化完成 (115200bps, 100Hz)");
}

bool IMU_Update(void) {
    bool updated = false;
    
    while (IMUSerial.available()) {
        uint8_t b = IMUSerial.read();
        
        if (b == 0x55) {
            rx_idx = 0;
            rx_buf[rx_idx++] = b;
        } else if (rx_idx > 0) {
            rx_buf[rx_idx++] = b;
            
            if (rx_idx == 11) {
                uint8_t sum = calc_sum(rx_buf, 10);
                if (sum == rx_buf[10]) {
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
                        updated = true;
                    }
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