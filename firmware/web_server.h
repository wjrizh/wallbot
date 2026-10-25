#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <stdint.h>

void WebServer_Init(void);
void WebServer_Tick(void);
void WebServer_PushLaser(void);
void WebServer_RefreshIMU(void);
float WebServer_GetIMURoll(void);
float FetchIMURoll(void);
void WebServer_SendLog(const char* msg);
/* 协同视觉纠偏: 推送 0x88 状态帧 与 0x89 视觉 JSON */
void WebServer_PushAlignState(void);
void WebServer_PushAlignJson(const char* json);

#endif