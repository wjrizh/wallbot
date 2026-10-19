/* cam_module.h - 摄像头初始化 / 检测模式切换 / 诊断信息 */
#ifndef WALLBOT_CAM_MODULE_H
#define WALLBOT_CAM_MODULE_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_camera.h"

void        Cam_Init(void);          /* 初始化相机, 失败不阻塞启动 */
bool        Cam_IsOk(void);
int         Cam_Err(void);           /* esp_camera_init 的返回码 */
int         Cam_Err2(void);          /* 二次探测错误码 */
bool        Cam_JpegOk(void);
int         Cam_SensorPid(void);
uint32_t    Cam_InitMs(void);        /* 初始化耗时 */

/* 检测模式: 只切分辨率 (JPEG 保持不变, 由驱动官方支持的运行时切换)。
 * 检测侧拿到 QVGA JPEG 后自行解码为灰度再跑 AprilTag。
 * 期间 /stream 照常工作 (画面短暂变小), 用完必须 Exit 配对调用。 */
bool        Cam_EnterDetectMode(void);
void        Cam_ExitDetectMode(void);
bool        Cam_InDetectMode(void);

/* 抓一帧并丢弃 (模式切换后等 AEC/AGC 稳定用) */
void        Cam_FlushFrames(int n, int delay_ms);

#endif /* WALLBOT_CAM_MODULE_H */
