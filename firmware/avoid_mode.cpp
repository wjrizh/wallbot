/*
 * avoid_mode.cpp - 激光自主避障行走
 * 开启 -> 3s采集(去极值取平均) -> 决策(步距/越障) -> 执行 -> 等1s -> 循环
 * 停止: SAMPLING/WAIT 立即停; EXEC 中急停(gait_estop)
 */
#include "avoid_mode.h"
#include "config.h"
#include "laser_tof.h"
#include "gait_engine.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define SAMPLE_MAX  128

static volatile AvoidState g_state = AVOID_OFF;
static volatile uint16_t g_last_dist = 0;
static SemaphoreHandle_t g_mtx = NULL;

static void Avoid_Log(const char* m){ Serial.print("[AVOID] "); Serial.println(m); }

static int cmp_u16(const void* a, const void* b){
    return (int)(*(const uint16_t*)a) - (int)(*(const uint16_t*)b);
}

/* 采集 3s, 返回滤波距离 (mm), 0=无有效数据. 可被 Stop 打断, 返回 0xFFFF 表示被打断 */
static uint16_t sample_filter(void){
    static uint16_t buf[SAMPLE_MAX];
    int n = 0;
    uint32_t start = millis();
    uint32_t last = 0;
    while (millis() - start <AVOID_SAMPLE_MS) {
        if (g_state != AVOID_SAMPLING) return 0xFFFF;
        uint32_t now = millis();
        if (now - last >= (1000 / AVOID_SAMPLE_HZ)) {
            last = now;
            uint16_t d = Laser_GetFront();
            if (d > 0 && n <SAMPLE_MAX) buf[n++] = d;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (n < (2*AVOID_TRIM + 1)) {
        if (n == 0) return 0;
        /* 样本太少, 直接平均 */
        uint32_t s = 0; for (int i=0;i<n;i++) s += buf[i];
        return (uint16_t)(s / n);
    }
    qsort(buf, n, sizeof(uint16_t), cmp_u16);
    int lo = AVOID_TRIM, hi = n - AVOID_TRIM;
    uint32_t s = 0; int c = 0;
    for (int i = lo; i < hi; i++){ s += buf[i]; c++; }
    return c ? (uint16_t)(s / c) : 0;
}

static float step_from_dist(uint16_t d){
    if (d >= AVOID_STEP_FAR_MM) return AVOID_STEP_MAX_MM;
    if (d <= AVOID_STOP_MM)     return AVOID_STEP_MIN_MM;
    float t = (float)(d - AVOID_STOP_MM) / (float)(AVOID_STEP_FAR_MM - AVOID_STOP_MM);
    return AVOID_STEP_MIN_MM + t * (AVOID_STEP_MAX_MM - AVOID_STEP_MIN_MM);
}

static void Task_Avoid(void* pv){
    while (1) {
        if (g_state != AVOID_SAMPLING) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

        uint16_t d = sample_filter();
        if (d == 0xFFFF) { Avoid_Log("采样被打断"); continue; }
        g_last_dist = d;
        Serial.printf("[AVOID] 滤波距离=%umm\n", d);

        if (d == 0) {
            Avoid_Log("无有效目标, 重采");
            continue;
        }
        if (g_state != AVOID_SAMPLING) continue;

        g_state = AVOID_EXEC;
        if (d < AVOID_STOP_MM) {
            Serial.printf("[AVOID] %umm<%d 越障\n", d, AVOID_STOP_MM);
            GaitEngine_Rotate();
        } else {
            float st = step_from_dist(d);
            Serial.printf("[AVOID] %umm 步距=%.0fmm\n", d, st);
            GaitEngine_StepWalk(st);
        }

        if (g_state != AVOID_EXEC) continue;
        g_state = AVOID_WAIT;
        uint32_t w = millis();
        while (millis() - w < AVOID_WAIT_MS) {
            if (g_state != AVOID_WAIT) break;
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        if (g_state == AVOID_WAIT) g_state = AVOID_SAMPLING;
    }
}

void Avoid_Init(void){
    g_mtx = xSemaphoreCreateMutex();
    xTaskCreate(Task_Avoid, "Avoid", 8192, NULL, 3, NULL);
    Serial.println("[AVOID] init");
}

void Avoid_Start(void){
    if (g_state != AVOID_OFF) return;
    gait_estop = false;
    g_state = AVOID_SAMPLING;
    Serial.println("[AVOID] start");
}

void Avoid_Stop(void){
    if (g_state == AVOID_OFF) return;
    if (g_state == AVOID_EXEC) {
        gait_estop = true;
        Serial.println("[AVOID] EXEC急停");
    }
    g_state = AVOID_OFF;
    Serial.println("[AVOID] stop");
}

bool Avoid_IsActive(void){ return g_state != AVOID_OFF; }
uint8_t Avoid_GetState(void){ return (uint8_t)g_state; }
uint16_t Avoid_GetDistance(void){ return g_last_dist; }
