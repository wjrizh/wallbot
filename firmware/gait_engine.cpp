/*
 * gait_engine.cpp - 裁剪版步态引擎 (移植自主项目)
 * 保留: WalkSequence (去RS02修正) + RotSequence
 * 去掉: RS02修正、雷达巡航、弹窗确认、ReturnToBase
 */

#include "gait_engine.h"
#include "config.h"
#include "protocol.h"
#include "servo_driver.h"
#include "pump_driver.h"
#include "can_rs02.h"
#include "kinematics.h"
#include "rs02_correct.h"
#include "auto_align.h"
#include "imu_driver.h"
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <math.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

volatile bool gait_estop = false;

static volatile bool    g_auto_enabled = false;
static volatile GaitMode g_auto_mode = GAIT_MODE_OFF;
static volatile GaitState g_auto_state = GAIT_IDLE;
static volatile uint32_t g_combat_steps = 0;
static volatile uint8_t  g_suspended_foot = 0;

static uint32_t state_timer = 0;

typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint8_t data[16];
} GaitCmd_t;

static QueueHandle_t xGaitCmdQueue = NULL;

/* ==================== 内部函数声明 ==================== */

static void Gait_SafeDelay(uint32_t ms);
static bool Gait_ExecutePump(uint8_t id, uint8_t state);
static void Gait_MoveFoot(float x1, float y1, float x2, float y2, uint16_t time_ms, bool is_front_move);
static void Gait_WalkSequence(float step_mm);
static void Gait_RotSequence(void);
static void Gait_UTurnSequence(void);
static void Task_GaitEngine(void *pvParameters);

static void Gait_Log(const char* msg) {
    Serial.print("[GAIT] ");
    Serial.println(msg);
}

/* ==================== 内部函数实现 ==================== */

static void Gait_SafeDelay(uint32_t ms) {
    uint32_t start = millis();
    while (millis() - start < ms) {
        if (gait_estop) return;
        TickType_t ticks = pdMS_TO_TICKS(20);
        vTaskDelay(ticks > 0 ? ticks : 1);
    }
}

static bool Gait_ExecutePump(uint8_t id, uint8_t state) {
    Pump_Set(id, state);

    uint32_t start = millis();
    uint8_t consecutive = 0;

    while (millis() - start < GAIT_PUMP_TIMEOUT_MS) {
        if (gait_estop) return false;

        uint8_t cur = Pump_GetState(id);
        if (cur == state) {
            consecutive++;
            if (consecutive >= 3) return true;
        } else {
            consecutive = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    Gait_Log("[错误] 吸盘动作执行超时(5s), 强制急停!");
    gait_estop = true;
    Pump_Set(1, 1);
    Pump_Set(2, 1);
    return false;
}

static void Gait_MoveFoot(float x1, float y1, float x2, float y2, uint16_t time_ms, bool is_front_move) {
    uint16_t steps = time_ms / GAIT_STEP_DELAY_MS;
    if (steps < 1) steps = 1;

    TickType_t step_ticks = pdMS_TO_TICKS(GAIT_STEP_DELAY_MS);
    if (step_ticks == 0) step_ticks = 1;

    for (uint16_t i = 0; i <= steps; i++) {
        if (gait_estop) return;

        float t = (float)i / (float)steps;
        float s = t * t * t * (10.0f - 15.0f * t + 6.0f * t * t);
        float cur_x = x1 + s * (x2 - x1);
        float cur_y = y1 + s * (y2 - y1);

        float m1, m2, m3, fixX, fixY;
        Kinematics_IK(cur_x, cur_y, is_front_move, &m1, &m2, &m3, &fixX, &fixY);

        Servo_SetM1(m1, 0.0f);
        Servo_SetM2(m2, 0.0f);
        Servo_SetM3(m3, 0.0f);

        vTaskDelay(step_ticks);
    }
}

/* ===== 步态序列（舵机直驱 + IMU纠偏融合版） ===== */

static float g_gait_aa_theta = 0.0f;  // 步态内 RS02 纠偏累积量

static void Gait_WalkSequence(float step_mm) {
    gait_estop = false;
    if (step_mm <= 0) return;

    Gait_Log("[步态] 执行迈步...");
    Gait_SafeDelay(100); if (gait_estop) return;

    float imu_gnd = 0.0f;
    float roll_start = 0.0f;
    float error = 0.0f;
    float abs_err = 0.0f;

#ifndef GAIT_SKIP_IMU
    /* ===== 首次行走自动标定: 停 3s 采样记录 IMU 地面基准 ===== */
    if (!Imu_IsCalibrated()) {
        Gait_Log("[步态] 首次行走, 采样标定 3s...");
        Imu_CalibrateOnce();
    }
    imu_gnd = Imu_GetGnd();

    /* ===== 步骤0：双脚着地读一次IMU (有线串口) ===== */
    roll_start = IMU_GetRoll();

    error = roll_start - imu_gnd;
    abs_err = fabsf(error);
    Serial.printf("[GAIT] 地面IMU: roll=%.2f gnd=%.2f err=%.2f°\n",
                  roll_start, imu_gnd, error);
#else
    Gait_Log("[步态] 已跳过IMU (GAIT_SKIP_IMU)");
#endif

    /* ===== 情况A：偏差>4°，调用自动纠偏，结束后返回 ===== */
    if (abs_err > 4.0f) {
        Gait_Log("[步态] 偏差>4°, 启动自动纠偏...");
        AutoAlign_Start();

        /* 等待纠偏完成，只响应急停 */
        while (AutoAlign_IsActive()) {
            if (gait_estop) { AutoAlign_Stop(); return; }
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        Gait_Log("[步态] 纠偏完成, 本次迈步取消");
        return;
    }

    /* ===== 情况B：偏差≤4°，正常步态 ===== */

    /* 计算落地目标 (两髋水平距 = 站立距 - step, 对称等高) */
    float arch_m1, arch_m2, arch_m3;
    Kinematics_SolveStep(step_mm, &arch_m1, &arch_m2, &arch_m3);
    Serial.printf("[GAIT] step=%.0f → 落地 M1=%.1f M2=%.1f M3=%.1f\n",
                  step_mm, arch_m1, arch_m2, arch_m3);

    /* 1. 后腿：抬起 → RS02纠偏 → 拱起放下 */
    if (!Gait_ExecutePump(2, 0)) return;
    Gait_SafeDelay(1000); if (gait_estop) return;

    Gait_Log("[步态] 后腿→纠偏位(150,130,50)");
    Servo_SetM1(150.0f, CALIB_AIR_MOVE_DUR);
    Servo_SetM2(130.0f, CALIB_AIR_MOVE_DUR);
    Servo_SetM3(50.0f, CALIB_AIR_MOVE_DUR);
    Gait_SafeDelay((uint32_t)(CALIB_AIR_MOVE_DUR * 1000) + 500);
    if (gait_estop) return;

    /* 后腿抬起 → 用初始error做一次IMU纠偏（仅当>1°） */
    if (abs_err > 1.0f) {
        float step = AA_STEP_BASE + (abs_err / AA_K_SYS) * AA_STEP_P;
        if (step > AA_STEP_MAX) step = AA_STEP_MAX;
        if (step <AA_STEP_MIN) step = AA_STEP_MIN;
        float sign = (error > 0.0f) ? 1.0f : -1.0f;
        g_gait_aa_theta = sign * step;
        if (g_gait_aa_theta > 0.10f) g_gait_aa_theta = 0.10f;
        if (g_gait_aa_theta < -0.10f) g_gait_aa_theta = -0.10f;
        Serial.printf("[GAIT] 后腿IMU纠偏: err=%.2f° RS02→%.4f rad\n",
                      error, g_gait_aa_theta);
        RS02_DualCorrect(1, g_gait_aa_theta, 2, g_gait_aa_theta, RS02_CORRECT_TIMEOUT_MS);
        if (gait_estop) return;
    } else {
        Gait_Log("[步态] 偏差≤1°, 跳过后腿纠偏");
    }
    if (!Gait_ExecutePump(2, 1)) return;
    Gait_Log("[步态] 后腿→远伸过渡");
    Servo_SetM1(arch_m1, CALIB_AIR_MOVE_DUR);
    Servo_SetM2(arch_m2, CALIB_AIR_MOVE_DUR);
    Servo_SetM3(arch_m3, CALIB_AIR_MOVE_DUR);
    Gait_SafeDelay((uint32_t)(CALIB_AIR_MOVE_DUR * 1000) + 500);
    if (gait_estop) return;

    Gait_Log("[步态] 后腿→落地");
    Servo_SetM1(arch_m1, CALIB_FAST_MOVE_DUR);
    Servo_SetM2(arch_m2+10.0f, CALIB_FAST_MOVE_DUR);
    Servo_SetM3(arch_m3, CALIB_FAST_MOVE_DUR);
    Gait_SafeDelay(CALIB_FAST_WAIT_MS);
    if (gait_estop) return;

    Gait_Log("[步态] 后腿→二次落地");
    Servo_SetM1(arch_m1, CALIB_FAST_MOVE_DUR);
    Servo_SetM2(arch_m2, CALIB_FAST_MOVE_DUR);
    Servo_SetM3(arch_m3, CALIB_FAST_MOVE_DUR);
    Gait_SafeDelay(CALIB_FAST_WAIT_MS);
    if (gait_estop) return;

    /* 2. 前腿：抬起 → RS02归零 → 放下 */
    if (!Gait_ExecutePump(1, 0)) return;
    Gait_SafeDelay(1000); if (gait_estop) return;

    Gait_Log("[步态] 前腿→纠偏位(60,120,115)");
    Servo_SetM1(60.0f, CALIB_AIR_MOVE_DUR);
    Servo_SetM2(120.0f, CALIB_AIR_MOVE_DUR);
    Servo_SetM3(115.0f, CALIB_AIR_MOVE_DUR);
    Gait_SafeDelay((uint32_t)(CALIB_AIR_MOVE_DUR * 1000) + 500);
    if (gait_estop) return;

    /* 前腿抬起 → RS02归零 */
    Gait_Log("[步态] 前腿抬起, RS02归零→0 rad");
    RS02_DualCorrect(1, 0.0f, 2, 0.0f, RS02_CORRECT_TIMEOUT_MS);
    if (gait_estop) return;
    g_gait_aa_theta = 0.0f;
    if (!Gait_ExecutePump(1, 1)) return;

Gait_Log("[步态] 前腿→远伸过渡(50,100,35)");
Servo_SetM1(60.0f, CALIB_AIR_MOVE_DUR);
Servo_SetM2(100.0f, CALIB_AIR_MOVE_DUR);
Servo_SetM3(35.0f, CALIB_AIR_MOVE_DUR);
Gait_SafeDelay((uint32_t)(CALIB_AIR_MOVE_DUR * 1000) + 500);
if (gait_estop) return;

Gait_Log("[步态] 前腿→复位(50,110,35)");
Servo_SetM1(60.0f, CALIB_FAST_MOVE_DUR);
Servo_SetM2(110.0f, CALIB_FAST_MOVE_DUR);
Servo_SetM3(35.0f, CALIB_FAST_MOVE_DUR);
Gait_SafeDelay(CALIB_FAST_WAIT_MS);
if (gait_estop) return;

Gait_Log("[完成] 迈步完毕");
}

static void Gait_RotSequence(void) {
    gait_estop = false;

#ifndef GAIT_SKIP_IMU
    /* ===== 首次越障自动标定: 停 3s 采样记录 IMU 地面基准 ===== */
    if (!Imu_IsCalibrated()) {
        Gait_Log("[越障] 首次越障, 采样标定 3s...");
        Imu_CalibrateOnce();
    }

    /* ===== 偏差>4°: 先做自动纠偏, 本次越障取消 ===== */
    {
        float gnd = Imu_GetGnd();
        float err0 = fabsf(IMU_GetRoll() - gnd);
        if (err0 > 4.0f) {
            Gait_Log("[越障] 偏差>4°, 启动自动纠偏...");
            AutoAlign_Start();
            while (AutoAlign_IsActive()) {
                if (gait_estop) { AutoAlign_Stop(); return; }
                vTaskDelay(pdMS_TO_TICKS(200));
            }
            Gait_Log("[越障] 纠偏完成, 本次越障取消");
            return;
        }
    }
#endif

    float initial_rad1 = CAN_RS02_GetPosition(1);
    float initial_rad2 = CAN_RS02_GetPosition(2);

    float rot_dur = GAIT_ROT_TIME_MS / 1000.0f;

    Gait_Log("[越障] 执行越障旋转...");

    /* ===== 后腿抬起 + 双电机 +180° ===== */
    if (!Gait_ExecutePump(2, 0)) return;
    Gait_SafeDelay(1000); if (gait_estop) return;

    /* 后腿抬起 → 读 IMU 基准偏差, 1~5° 内算一次纠偏量 */
    float rot_corr = 0.0f;
    {
        float imu_gnd = Imu_GetGnd();
        float roll_now = IMU_GetRoll();
        float err = roll_now - imu_gnd;
        float abs_e = fabsf(err);
        if (abs_e > 1.0f && abs_e <= 3.0f) {
            float step = AA_STEP_BASE + (abs_e / AA_K_SYS) * AA_STEP_P;
            if (step > AA_STEP_MAX) step = AA_STEP_MAX;
            if (step < AA_STEP_MIN) step = AA_STEP_MIN;
            float sign = (err > 0.0f) ? 1.0f : -1.0f;
            rot_corr = sign * step;
            if (rot_corr > 0.10f) rot_corr = 0.10f;
            if (rot_corr < -0.10f) rot_corr = -0.10f;
        }
        Serial.printf("[越障] IMU: roll=%.2f gnd=%.2f err=%.2f° 纠偏=%.4f rad\n",
                      roll_now, imu_gnd, err, rot_corr);
    }

    Gait_Log("[越障] 后腿抬起 M1→150°");
    Servo_SetM1(150.0f, 4.0f);
    Gait_SafeDelay(GAIT_MOVE_TIME_MS+1000); if (gait_estop) return;

    Gait_Log("[越障] 双电机 +180°+纠偏");
    CAN_RS02_StartTraj(1, initial_rad1 + PI + rot_corr, rot_dur);
    CAN_RS02_StartTraj(2, initial_rad2 + PI + rot_corr, rot_dur);
    Gait_SafeDelay(GAIT_ROT_TIME_MS + 500); if (gait_estop) return;

    Gait_Log("[越障] 修正双RS02到+180°+纠偏...");
    RS02_DualCorrect(1, initial_rad1 + PI + rot_corr, 2, initial_rad2 + PI + rot_corr, RS02_CORRECT_TIMEOUT_MS);
    if (gait_estop) return;

    Gait_Log("[越障] 后腿→远伸过渡(50,100,35)");
    Servo_SetM1(60.0f, 4.0f);
    Servo_SetM2(90.0f, 4.0f);
    Servo_SetM3(35.0f, 4.0f);
    Gait_SafeDelay(GAIT_MOVE_TIME_MS+1000); if (gait_estop) return;

    Gait_Log("[越障] 后腿复位(50,110,35)");
    Servo_SetM1(60.0f, CALIB_FAST_MOVE_DUR);
    Servo_SetM2(110.0f, CALIB_FAST_MOVE_DUR);
    Servo_SetM3(35.0f, CALIB_FAST_MOVE_DUR);
    Gait_SafeDelay(CALIB_FAST_WAIT_MS); if (gait_estop) return;

    if (!Gait_ExecutePump(2, 1)) return;
    Gait_SafeDelay(1000); if (gait_estop) return;

    /* ===== 前腿抬起 + 双电机 -180° ===== */
    if (!Gait_ExecutePump(1, 0)) return;
    Gait_SafeDelay(1000); if (gait_estop) return;

    Gait_Log("[越障] 前腿抬起 M3→115°");
    Servo_SetM3(115.0f, 4.0f);
    Gait_SafeDelay(GAIT_MOVE_TIME_MS+1000); if (gait_estop) return;

    Gait_Log("[越障] 双电机 -180°");
    CAN_RS02_StartTraj(1, initial_rad1, rot_dur);
    CAN_RS02_StartTraj(2, initial_rad2, rot_dur);
    Gait_SafeDelay(GAIT_ROT_TIME_MS + 500); if (gait_estop) return;

    Gait_Log("[越障] 修正双RS02到init...");
    RS02_DualCorrect(1, initial_rad1, 2, initial_rad2, RS02_CORRECT_TIMEOUT_MS);
    if (gait_estop) return;

    Gait_Log("[越障] 前腿→远伸过渡(50,100,35)");
    Servo_SetM1(60.0f, 4.0f);
    Servo_SetM2(90.0f, 4.0f);
    Servo_SetM3(35.0f, 4.0f);
    Gait_SafeDelay(GAIT_MOVE_TIME_MS+1000); if (gait_estop) return;

    Gait_Log("[越障] 前腿复位(50,110,35)");
    Servo_SetM1(60.0f, CALIB_FAST_MOVE_DUR);    
    Servo_SetM2(110.0f, CALIB_FAST_MOVE_DUR);
    Servo_SetM3(35.0f, CALIB_FAST_MOVE_DUR);
    Gait_SafeDelay(CALIB_FAST_WAIT_MS); if (gait_estop) return;

    if (!Gait_ExecutePump(1, 1)) return;
    Gait_SafeDelay(1000); if (gait_estop) return;

    Gait_Log("[完成] 越障完毕");
}

static void Gait_UTurnSequence(void) {
    gait_estop = false;

    float initial_rad2 = CAN_RS02_GetPosition(2);
    float rot_dur = GAIT_ROT_TIME_MS / 1000.0f;

    Gait_Log("[掉头] 执行掉头...");

    /* ===== 前腿抬起 + 电机2 +180° ===== */
    if (!Gait_ExecutePump(1, 0)) return;
    Gait_SafeDelay(1000); if (gait_estop) return;

    Gait_Log("[掉头] 前腿抬起 M3→115°");
    Servo_SetM3(115.0f, 4.0f);
    Gait_SafeDelay(GAIT_MOVE_TIME_MS + 1000); if (gait_estop) return;

    Gait_Log("[掉头] 电机2 +180°");
    CAN_RS02_StartTraj(2, initial_rad2 + PI, rot_dur);
    Gait_SafeDelay(GAIT_ROT_TIME_MS + 500); if (gait_estop) return;

    Gait_Log("[掉头] 修正电机2到+180°...");
    RS02_DualCorrect(2, initial_rad2 + PI, 2, initial_rad2 + PI, RS02_CORRECT_TIMEOUT_MS);
    if (gait_estop) return;

    if (!Gait_ExecutePump(1, 1)) return;
    Gait_Log("[掉头] 前腿→远伸过渡(50,100,35)");
    Servo_SetM1(60.0f, 4.0f);
    Servo_SetM2(90.0f, 4.0f);
    Servo_SetM3(35.0f, 4.0f);
    Gait_SafeDelay(GAIT_MOVE_TIME_MS+2000); if (gait_estop) return;

    Gait_Log("[掉头] 前腿复位(50,110,35)");
    Servo_SetM1(60.0f, CALIB_FAST_MOVE_DUR);
    Servo_SetM2(110.0f, CALIB_FAST_MOVE_DUR);
    Servo_SetM3(35.0f, CALIB_FAST_MOVE_DUR);
    Gait_SafeDelay(CALIB_FAST_WAIT_MS); if (gait_estop) return;
    Gait_SafeDelay(1000); if (gait_estop) return;

    /* ===== 后腿抬起 + 电机2 -180° ===== */
    if (!Gait_ExecutePump(2, 0)) return;
    Gait_SafeDelay(1000); if (gait_estop) return;

    Gait_Log("[掉头] 后腿→纠偏位(140,130,50)");
    Servo_SetM1(150.0f, CALIB_AIR_MOVE_DUR);
    Servo_SetM2(130.0f, CALIB_AIR_MOVE_DUR);
    Servo_SetM3(50.0f, CALIB_AIR_MOVE_DUR);
    Gait_SafeDelay((uint32_t)(CALIB_AIR_MOVE_DUR * 1000) + 500);
    if (gait_estop) return;

    Gait_Log("[掉头] 电机2 -180°");
    CAN_RS02_StartTraj(2, initial_rad2, rot_dur);
    Gait_SafeDelay(GAIT_ROT_TIME_MS + 500); if (gait_estop) return;

    Gait_Log("[掉头] 修正电机2归位...");
    RS02_DualCorrect(2, initial_rad2, 2, initial_rad2, RS02_CORRECT_TIMEOUT_MS);
    if (gait_estop) return;

    if (!Gait_ExecutePump(2, 1)) return;
    Gait_Log("[掉头] 后腿→远伸过渡(50,100,35)");
    Servo_SetM1(60.0f, 4.0f);
    Servo_SetM2(120.0f, 4.0f);
    Servo_SetM3(35.0f, 4.0f);
    Gait_SafeDelay(GAIT_MOVE_TIME_MS+2000);
    if (gait_estop) return;

    Gait_Log("[掉头] 后腿复位(50,110,35)");
    Servo_SetM1(60.0f, CALIB_FAST_MOVE_DUR);
    Servo_SetM2(110.0f, CALIB_FAST_MOVE_DUR);
    Servo_SetM3(35.0f, CALIB_FAST_MOVE_DUR);
    Gait_SafeDelay(CALIB_FAST_WAIT_MS);
    if (gait_estop) return;
    Gait_Log("[完成] 掉头完毕");
}

/* ==================== 接口函数 ==================== */

void GaitEngine_Init(void) {
    g_auto_enabled = false;
    g_auto_mode = GAIT_MODE_OFF;
    g_auto_state = GAIT_IDLE;
    gait_estop = false;
    g_combat_steps = 0;
    g_suspended_foot = 0;

    xGaitCmdQueue = xQueueCreate(10, sizeof(GaitCmd_t));

    xTaskCreate(Task_GaitEngine, "GaitEng", 8192, NULL, 5, NULL);

    Serial.println("[GAIT] 步态引擎初始化完成");
}

void GaitEngine_Start(void) {
    g_auto_enabled = true;
    g_auto_mode = GAIT_MODE_NORMAL;
    g_auto_state = GAIT_IDLE;
    g_combat_steps = 0;
    gait_estop = false;
}

void GaitEngine_CombatStart(void) {
    g_auto_enabled = true;
    g_auto_mode = GAIT_MODE_COMBAT;
    g_auto_state = GAIT_IDLE;
    g_combat_steps = 0;
    gait_estop = false;
}

void GaitEngine_Stop(void) {
    g_auto_enabled = false;
    g_auto_state = GAIT_IDLE;
    g_auto_mode = GAIT_MODE_OFF;
    gait_estop = true;
    Pump_Set(1, 1);
    Pump_Set(2, 1);
}

void GaitEngine_ConfirmNext(void) {
    if (g_auto_state == GAIT_WAITING || g_suspended_foot != 0) {
        g_suspended_foot = 0;
        if (g_auto_state == GAIT_WAITING) {
            g_auto_state = GAIT_DEBOUNCE;
        }
    }
}

void GaitEngine_StepWalk(float step_mm) {
    Gait_WalkSequence(step_mm);

    if (!gait_estop && g_auto_enabled && g_auto_state == GAIT_EXECUTING) {
        Gait_SafeDelay(300);
        if (g_auto_mode == GAIT_MODE_COMBAT) {
            g_combat_steps++;
            g_auto_state = GAIT_DEBOUNCE;
        } else {
            g_auto_state = GAIT_WAITING;
        }
    }
}

void GaitEngine_Rotate(void) {
    Gait_RotSequence();

    if (!gait_estop && g_auto_enabled && g_auto_state == GAIT_EXECUTING) {
        Gait_SafeDelay(300);
        if (g_auto_mode == GAIT_MODE_COMBAT) {
            g_combat_steps++;
            g_auto_state = GAIT_DEBOUNCE;
        } else {
            g_auto_state = GAIT_WAITING;
        }
    }
}

void GaitEngine_UTurn(void) {
    Gait_UTurnSequence();

    if (!gait_estop && g_auto_enabled && g_auto_state == GAIT_EXECUTING) {
        Gait_SafeDelay(300);
        if (g_auto_mode == GAIT_MODE_COMBAT) {
            g_combat_steps++;
            g_auto_state = GAIT_DEBOUNCE;
        } else {
            g_auto_state = GAIT_WAITING;
        }
    }
}

uint8_t GaitEngine_GetState(void)  { return (uint8_t)g_auto_state; }
uint8_t GaitEngine_GetMode(void)   { return (uint8_t)g_auto_mode; }
uint32_t GaitEngine_GetCombatSteps(void) { return g_combat_steps; }
uint8_t GaitEngine_GetSuspendedFoot(void) { return g_suspended_foot; }

void GaitEngine_EnqueueCmd(uint8_t cmd, const uint8_t* data, uint8_t len) {
    if (!xGaitCmdQueue) return;
    GaitCmd_t gcmd;
    gcmd.cmd = cmd;
    gcmd.len = (len > 16) ? 16 : len;
    if (data && len > 0) {
        memcpy(gcmd.data, data, gcmd.len);
    }
    if (xQueueSend(xGaitCmdQueue, &gcmd, 0) != pdTRUE) {
        Serial.println("[错误] 步态指令队列已满!");
    }
}

/* ==================== FreeRTOS 任务 ==================== */

static void Task_GaitEngine(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(20);
    GaitCmd_t cmd;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        while (xQueueReceive(xGaitCmdQueue, &cmd, 0) == pdTRUE) {
            switch (cmd.cmd) {
                case 0x10:
                    GaitEngine_StepWalk(*(float*)cmd.data);
                    break;
                case 0x11:
                    GaitEngine_Rotate();
                    break;
                case 0x12: {
                    uint8_t val = cmd.data[0];
                    if (val == 0) GaitEngine_Stop();
                    else if (val == 2) GaitEngine_CombatStart();
                    else GaitEngine_Start();
                    break;
                }
                case 0x14:
                    GaitEngine_UTurn();
                    break;
                case 0x15:
                    GaitEngine_ConfirmNext();
                    break;
            }
        }
    }
}