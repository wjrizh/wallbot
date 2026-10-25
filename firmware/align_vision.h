/* align_vision.h - 真实视觉纠偏闭环 (K230 /align/* -> TestMode -> 子机)
 *
 * 与 align_bridge 的区别:
 *   align_bridge  : 拉 /align/get (只读缓存), 速度闭环, 经 ChildCmd 直通
 *   align_vision  : 拉 /align/once (触发新测量), 位置级闭环,
 *                   经 TestMode 的 TSTA/ADJ/TSTB/DONE 握手下发, 子机执行
 *                   Adj_Execute(phi,psi,ax) 后回 DONE。
 *
 * NRF 收发时序完全复用 test_mode + nrf_sender, 本模块不碰 RF24。
 *
 * 触发: App WS 0x22 -> AlignVision_Start(ip, gain, maxIter, flags)
 * 停止: App WS 0x23 -> AlignVision_Stop()
 * 参数: App WS 0x24 -> AlignVision_SetTune(signPhi, signPsi, signAx, betaDeg, speedPct)
 * 单步: App WS 0x25 -> AlignVision_Step(which, phi, psi, ax)
 * 视觉: App WS 0x26 -> AlignVision_CamCtl(action)
 */
#ifndef ALIGN_VISION_H
#define ALIGN_VISION_H

#include <stdint.h>
#include <stdbool.h>

/* ---- 状态 (App 0x88 payload[0]) ---- */
enum {
    AV_ST_IDLE = 0,
    AV_ST_CAM_START,      /* 1 请求 K230 /align/start */
    AV_ST_TEST_START,     /* 2 发 TSTA, 等 TSTB */
    AV_ST_WAIT_READY,     /* 3 */
    AV_ST_MEASURE,        /* 4 拉 /align/once */
    AV_ST_SEND_ADJ,       /* 5 下发 ADJ 帧 */
    AV_ST_WAIT_DONE,      /* 6 等子机 DONE (round++) */
    AV_ST_CONVERGED,      /* 7 收敛/单步完成 */
    AV_ST_TIMEOUT,        /* 8 */
    AV_ST_FAIL_CAM,       /* 9 视觉侧失败 */
    AV_ST_FAIL_TEST,      /* 10 子机侧失败 */
    AV_ST_STOPPED         /* 11 用户停止 */
};

/* ---- 结束原因 (App 0x88 payload[7]) ---- */
enum {
    AV_RC_NONE = 0,
    AV_RC_NO_REF,          /* 1 K230 未记参考位 */
    AV_RC_NOT_RUNNING,     /* 2 K230 未「开始」 */
    AV_RC_NO_TAG,          /* 3 */
    AV_RC_NEED_TWO_TAGS,   /* 4 */
    AV_RC_HTTP_FAIL,       /* 5 */
    AV_RC_ITER_MAX,        /* 6 */
    AV_RC_TOTAL_TIMEOUT,   /* 7 */
    AV_RC_FAIL_TEST,       /* 8 子机 TSTB/DONE 超时 */
    AV_RC_USER_STOP,       /* 9 */
    AV_RC_DONE            /* 10 */
};

/* ---- Start flags (0x22 payload[6]) ---- */
#define AV_FLAG_DRY_RUN      0x01   /* 只测量+打印, 不下发 ADJ */
#define AV_FLAG_START_CAM    0x02   /* 自动打 K230 /align/start */
#define AV_FLAG_STOP_CAM     0x04   /* 结束时打 /align/stop */
#define AV_FLAG_SINGLE_STEP  0x08   /* 单步: 跑完一轮就结束 */

/* ---- Step 自由度 (0x25 payload[0]) ---- */
enum {
    AV_STEP_PSI = 1,   /* 只自旋 (psi) */
    AV_STEP_PHI = 2,   /* 只直行 (phi) */
    AV_STEP_AX  = 3,   /* 只斜行 (ax) */
    AV_STEP_ALL = 4    /* 全量 */
};

/* ---- CamCtl action (0x26 payload[0]) ---- */
enum {
    AV_CAM_STATUS = 0,
    AV_CAM_START  = 1,
    AV_CAM_STOP   = 2,
    AV_CAM_REF    = 3,   /* 记参考位 */
    AV_CAM_ONCE   = 4,   /* 点测一次 */
    AV_CAM_CLRREF = 5    /* 清参考位 */
};

void AlignVision_Init(void);

/* FreeRTOS 任务体 */
void AlignVision_Task(void* pvParameters);

/* 启动: ip 4 字节; gainPct 10..100 (0.1~1.0); maxIter 1..50 (轮次安全上限); flags 见 AV_FLAG_* */
bool AlignVision_Start(const uint8_t ip[4], uint8_t gainPct, uint8_t maxIter, uint8_t flags);

/* 停止: 发 TSTE 并回 IDLE */
void AlignVision_Stop(void);

bool AlignVision_IsActive(void);

/* 符号/速度标定: sign* 取 -1/+1; betaDeg 10~40; speedPct 20~100 (子机 ADJ_SPEED_REV 的百分比) */
void AlignVision_SetTune(int8_t signPhi, int8_t signPsi, int8_t signAx,
                         uint8_t betaDeg, uint8_t speedPct);
void AlignVision_GetTune(int8_t* signPhi, int8_t* signPsi, int8_t* signAx,
                         uint8_t* betaDeg, uint8_t* speedPct);

/* 单步标定: which = AV_STEP_*; 未选中分量强制 0 */
bool AlignVision_Step(uint8_t which, float phi, float psi, float ax);

/* 视觉侧控制: action = AV_CAM_*; 直接打 HTTP */
bool AlignVision_CamCtl(uint8_t action, const uint8_t ip[4]);

/* 0x88 payload: [state][iter][rc][phi i16][psi i16][ax i16][aligned][reason] = 11 字节 */
#define AV_STATUS_LEN 11
void AlignVision_GetStatus(uint8_t out[AV_STATUS_LEN]);

/* 最近一次 /align/once 原始 JSON (供 0x89 推送) */
const char* AlignVision_GetLastJson(void);

#endif /* ALIGN_VISION_H */
