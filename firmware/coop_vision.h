/* coop_vision.h - 协同内视觉纠偏后端 (K230 HTTP)
 *
 * 从 align_vision.cpp 精简而来: 去掉 TestMode 与独立收敛状态机,
 * 只做「请求 -> HTTP 测量 -> 缓存结果」的后端; 收敛/轮次/命令决策
 * 全部在 auto_mode 的协同状态机里 (COOP_VISION_* 状态)。
 *
 * 线程模型: 请求接口由 auto_mode(摇杆任务)调用, 只置标志+置 BUSY;
 * HTTP 阻塞读在本模块自己的 FreeRTOS 任务里, 结果经 volatile 状态
 * + GetDev 交回。0x89 原始 JSON 在任务内直接推送。
 */
#ifndef COOP_VISION_H
#define COOP_VISION_H

#include <stdint.h>
#include <stdbool.h>

/* 0x88 payload[0]: 0-6 为 CV 状态; auto_mode 侧另有 200=协同进行中(非视觉) */
typedef enum {
    CV_IDLE = 0,
    CV_REF_BUSY,      /* 1 正在 GET /align/ref */
    CV_MEAS_BUSY,     /* 2 正在 GET /align/once */
    CV_REF_DONE,      /* 3 */
    CV_REF_FAIL,      /* 4 */
    CV_MEAS_DONE,     /* 5 */
    CV_MEAS_FAIL,     /* 6 */
} CoopVisionState;

void CoopVision_Init(void);
void CoopVision_Task(void* pvParameters);

/* 设置 K230 地址 (0x19 协同启动时由 App 注入, NVS 持久化, 重启不失) */
void CoopVision_SetBase(const uint8_t ip[4]);

/* 请求接口: 同步置 BUSY, 调用方轮询 GetState 取结果 */
void CoopVision_RequestRef(void);
void CoopVision_RequestMeasure(void);

/* 中止/新协同开始时清状态与未决请求, 丢弃过期结果 */
void CoopVision_Reset(void);

CoopVisionState CoopVision_GetState(void);
void CoopVision_GetDev(float* phi, float* psi, float* ax, bool* aligned);

/* 符号/速度标定 (NVS wallbot-cv 持久化, 首次从旧 wallbot-av 迁移) */
void CoopVision_SetTune(int8_t signPhi, int8_t signPsi, int8_t signAx,
                        uint8_t betaDeg, uint8_t speedPct);
void CoopVision_GetTune(int8_t* signPhi, int8_t* signPsi, int8_t* signAx,
                        uint8_t* betaDeg, uint8_t* speedPct);

/* 测量偏差 -> 子机命令: 符号×增益×限幅 + 容差清零 + 最小抬升
 * (决策(收敛/死区终判)在 auto_mode, 本函数只做数值变换) */
void CoopVision_ComputeCmd(float phi, float psi, float ax,
                           float* cPsi, float* cPhi, float* cAx);

#endif /* COOP_VISION_H */
