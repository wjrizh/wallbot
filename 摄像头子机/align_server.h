/* align_server.h - 视觉纠偏 HTTP API (挂在 :80 控制服务上)
 *
 * GET /align/ref          点测共识位姿记为参考位 (放子机前、夹爪对中时调用)
 * GET /align/ref?clear=1  清除参考位
 * GET /align/get          只读上次缓存, 不续租、不触发检测
 * GET /align/once         点测一次: 激活检测→等 ALIGN_FRAMES 共识→释租
 * GET /align/status       参考位 + 上次测量 + 配置概览 (纯文本)
 *
 * 架构: 平时 worker 休眠; /align/once 与 /align/ref 临时续租做共识;
 *       /align/get 只读缓存; 帧优先由 :81/stream 旁路投喂。
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

/* /align/once|/align/ref 续租是否仍有效: :81/stream 据此降频让出相机 */
bool AlignServer_LeaseActive(void);

#endif /* WALLBOT_ALIGN_SERVER_H */
