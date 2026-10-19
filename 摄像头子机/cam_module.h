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

/* 检测模式: 切到灰度低分辨率跑 AprilTag 检测, 用完必须 Exit 配对调用。
 * 期间 /stream 仍可用 (灰度帧走软编码), 但帧率下降, 属正常现象。 */
bool        Cam_EnterDetectMode(void);
void        Cam_ExitDetectMode(void);
bool        Cam_InDetectMode(void);

/* 抓一帧并丢弃 (模式切换后等 AEC/AGC 稳定用) */
void        Cam_FlushFrames(int n, int delay_ms);

#endif /* WALLBOT_CAM_MODULE_H */
