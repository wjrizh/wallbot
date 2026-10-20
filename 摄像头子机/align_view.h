/* align_view.h - 检测可视化流 (:80/view MJPEG)
 *
 * 输出灰度检测帧, 并在帧上叠加: 候选框/角点/连通域数/丢弃原因/汉明距离。
 * 用途: 不再依赖串口, 直接在 App 里看到"检测到了什么"。
 */
#ifndef WALLBOT_ALIGN_VIEW_H
#define WALLBOT_ALIGN_VIEW_H

#include <stdbool.h>
#include "esp_http_server.h"

/* 把 /view 路由注册到控制服务 (端口 80) */
void AlignView_Register(httpd_handle_t httpd);

#endif /* WALLBOT_ALIGN_VIEW_H */
