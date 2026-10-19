/* align_server.h - 视觉纠偏 HTTP API (挂在 :80 控制服务上)
 *
 * GET /align/ref          连拍投票检测, 将当前位姿记录为参考位 (放子机前、夹爪对中时调用)
 * GET /align/ref?clear=1  清除参考位
 * GET /align/get          测量当前偏差 -> JSON (闭环主接口: phi/psi/axial/arc/aligned)
 * GET /align/status       参考位 + 上次测量 + 配置概览 (纯文本)
 */
#ifndef WALLBOT_ALIGN_SERVER_H
#define WALLBOT_ALIGN_SERVER_H

#include "esp_http_server.h"

void AlignServer_Register(httpd_handle_t control_httpd);

#endif /* WALLBOT_ALIGN_SERVER_H */
