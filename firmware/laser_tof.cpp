/*
 * laser_tof.cpp - VL53L5CX 8x8 测距 (TCA9548A CH0)
 * 前方扇形 = row 6~7, col 1~6
 * 取扇形区所有有效区的最小值作为障碍距离
 */
#include "laser_tof.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L5CX.h>

#define TCA9548A_ADDR   0x70
#define LASER_CH        0
#define TOF_I2C_SDA     4
#define TOF_I2C_SCL     5

#define FAN_ROW_START   6
#define FAN_ROW_END     7
#define FAN_COL_START   1
#define FAN_COL_END     6

static Adafruit_VL53L5CX sensor;
static VL53L5CX_ResultsData results;
static bool g_online = false;
static volatile uint16_t g_front = 0;

static void tcaSelect(uint8_t ch) {
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << ch);
    Wire.endTransmission();
}

void Laser_Init(void) {
    Wire.begin(TOF_I2C_SDA, TOF_I2C_SCL);
    tcaSelect(LASER_CH);
    Serial.println("[TOF] init...");
    if (!sensor.begin(VL53L5CX_DEFAULT_ADDRESS, &Wire, 400000)) {
        Serial.println("[TOF] begin FAILED");
        g_online = false;
        return;
    }
    sensor.setResolution(64);
    sensor.setRangingFrequency(15);
    sensor.startRanging();
    g_online = true;
    Serial.println("[TOF] ready (CH0, fan +-15deg, row0-3)");
}

bool Laser_IsOnline(void) { return g_online; }
uint16_t Laser_GetFront(void) { return g_front; }

void Laser_Tick(void) {
    if (!g_online) return;
    tcaSelect(LASER_CH);
    if (!sensor.isDataReady()) return;
    if (!sensor.getRangingData(&results)) return;

    uint16_t front = 0;
    for (int row = FAN_ROW_START; row <= FAN_ROW_END; row++) {
        for (int col = FAN_COL_START; col <= FAN_COL_END; col++) {
            int idx = row * 8 + col;
            if (results.target_status[idx] != 5) continue;
            uint16_t d = results.distance_mm[idx];
            if (d == 0) continue;
            if (front == 0 || d < front) front = d;
        }
    }
    g_front = front;
}
