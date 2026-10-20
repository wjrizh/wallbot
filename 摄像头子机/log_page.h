/* log_page.h - 内置网页日志页 (浏览器直接看, 不依赖 App) */
#ifndef WALLBOT_LOG_PAGE_H
#define WALLBOT_LOG_PAGE_H

#include "esp_http_server.h"

void LogPage_Register(httpd_handle_t httpd);

#endif /* WALLBOT_LOG_PAGE_H */
