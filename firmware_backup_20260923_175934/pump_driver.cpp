/*
 * pump_driver.cpp - 双泵 GPIO 控制 (独立标定项目)
 * 泵1: GPIO 8, 泵2: GPIO 11, HIGH=吸, LOW=松
 */

#include "pump_driver.h"
#include "config.h"
#include <Arduino.h>

static uint8_t pump_state[3] = {0, 0, 0}; // 1-indexed

void Pump_Init(void) {
    pinMode(PUMP1_PIN, OUTPUT);
    pinMode(PUMP2_PIN, OUTPUT);
    digitalWrite(PUMP1_PIN, PUMP_OFF);
    digitalWrite(PUMP2_PIN, PUMP_OFF);
    pump_state[1] = 0;
    pump_state[2] = 0;
    Serial.println("[Pump] 双泵初始化完成 (松开)");
}

void Pump_Set(uint8_t id, uint8_t state) {
    uint8_t pin;
    if (id == 1) pin = PUMP1_PIN;
    else if (id == 2) pin = PUMP2_PIN;
    else return;

    digitalWrite(pin, state ? PUMP_ON : PUMP_OFF);
    pump_state[id] = state ? 1 : 0;
    Serial.printf("[Pump] 泵%d → %s\n", id, state ? "吸附" : "松开");
}

uint8_t Pump_GetState(uint8_t id) {
    if (id == 1 || id == 2) return pump_state[id];
    return 0;
}