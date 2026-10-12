#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <stdint.h>

void WebServer_Init(void);
void WebServer_PushLaser(void);
void WebServer_RefreshIMU(void);
float WebServer_GetIMURoll(void);
float FetchIMURoll(void);
void WebServer_SendLog(const char* msg);

#endif