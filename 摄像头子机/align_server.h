/* align_server.h - 视觉纠偏 HTTP API (挂在 :80 控制服务上)
 *
 * GET /align/ref          取后台共识位姿记为参考位 (放子机前、夹爪对中时调用)
 * GET /align/ref?clear=1  清除参考位
 * GET /align/get          读后台缓存结果秒回 (闭环主接口: phi/psi/axial/arc/aligned)
 * GET /align/status       参考位 + 上次测量 + 配置概览 (纯文本)
 *
 * 架构: /align/get 不再自己取帧+检测, 只续租并读缓存;
 *       由 align_worker_task 后台取帧检测, 帧优先由 :81/stream 旁路投喂。
 */
#ifndef WALLBOT_ALIGN_SERVER_H
#define WALLBOT_ALIGN_SERVER_H

#include "esp_http_server.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdint.h>
#include <stddef.h>

/* 检测互斥锁: TagDetector 内部使用全局静态工作区 g_sc, 不可并发。
 * 后台检测任务与 /view (:81) 在不同任务上, 必须共用这把锁。 */
extern SemaphoreHandle_t g_measure_lock;

void AlignServer_Register(httpd_handle_t control_httpd);

/* 供 :81/stream 推流循环顺手投喂最新 JPEG 帧 (非阻塞, 仅当后台检测需要帧时拷贝) */
void AlignServer_FeedFrame(const uint8_t *jpg, size_t len, int w, int h);

#endif /* WALLBOT_ALIGN_SERVER_H */
