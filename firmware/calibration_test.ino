/*
 * calibration_test.ino - 激光标定测试工具 + NRF子母协同
 */

#include "config.h"
#include "can_rs02.h"
#include "servo_driver.h"
#include "pump_driver.h"
#include "gait_engine.h"
#include "auto_align.h"
#include "protocol.h"         // 新增
#include "joypad_sender.h"    // 新增
#include "auto_mode.h"        // 新增
#include "gripper.h"
#include "web_server.h"
#include "laser_tof.h"
#include "avoid_mode.h"
#include "imu_driver.h"
#include <Arduino.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* ==================== 激光推送任务 (200ms) ==================== */

static void Task_PushLaser(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(200);

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        WebServer_PushLaser();
    }
}

/* ==================== 激光 ToF 刷新任务 (20ms) ==================== */

static void Task_Laser(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(20);

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        Laser_Tick();
    }
}

/* ==================== IMU 后台刷新任务 (250ms) ==================== */

static void Task_IMU(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(250);

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        IMU_Update();
        WebServer_RefreshIMU();
    }
}

/* ==================== 夹持器任务 (HTTP, 100ms 轮询) ==================== */

static void Task_Gripper(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(100);

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        Gripper_Tick();
    }
}

/* ==================== Arduino Setup ==================== */

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== 激光标定测试工具 + NRF子母协同 ===\n");

    /* 1. CAN RS02 */
    CAN_RS02_Init();

    /* ★ Web 服务器 (必须在所有FreeRTOS任务之前初始化) */
    WebServer_Init();

    /* 3. 舵机 PWM */
    Servo_Init();
    Serial.printf("[Servo] 上电姿态: M1=%.1f M2=%.1f M3=%.1f PB9=%.1f\n",
                  Servo_GetAngle(1), Servo_GetAngle(2),
                  Servo_GetAngle(3), Servo_GetAngle(9));

    /* 4. 真空泵 */
    Pump_Init();

    /* 4.4 激光 ToF + 避障 */
    Laser_Init();
    Avoid_Init();
    IMU_Init();

    /* 4.5 步态引擎 */
    GaitEngine_Init();

    /* 4.7 自动纠偏 */
    AutoAlign_Init();

    /* 4.8 协议模块 */
    Protocol_Init();

    /* 4.9 NRF + 虚拟手柄 (子母协同时启用) */
    JoypadSender_Init();
    AutoMode_Init();
xTaskCreate(JoypadSender_Task, "JoypadSend", 4096, NULL, 2, NULL);

    /* 6. 推送任务 */
    xTaskCreate(Task_PushLaser, "PushLaser", 4096, NULL, 1, NULL);

    /* 6.4 激光 ToF 刷新 */
    xTaskCreate(Task_Laser, "Laser", 4096, NULL, 2, NULL);

    /* 6.5 IMU 后台刷新 (独立任务, 不阻塞推送) */
    xTaskCreate(Task_IMU, "IMU", 4096, NULL, 2, NULL);

    /* 7. 夹持器任务 */
    Gripper_Init();
    xTaskCreate(Task_Gripper, "Gripper", 4096, NULL, 2, NULL);

    Serial.println("\n[MAIN] 全部就绪");
    Serial.print  ("[MAIN] 连接 WiFi: ");
    Serial.print  (WIFI_SSID);
    Serial.print  (" / ");
    Serial.print  (WIFI_PASSWORD);
    Serial.print  ("\n[MAIN] 浏览器打开: http://");
    Serial.print  (WiFi.localIP().toString());
    Serial.println("/\n");
}

/* ==================== Arduino Loop ==================== */

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}