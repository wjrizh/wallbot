/* discovery.h - UDP 自动发现 (母机/App 广播询问, 本机应答角色+IP) */
#ifndef WALLBOT_DISCOVERY_H
#define WALLBOT_DISCOVERY_H

void Discovery_Poll(void);              /* 主循环轮询, 应答发现请求 */
void Discovery_Reset(void);             /* WiFi 重连后重置 socket */

#endif /* WALLBOT_DISCOVERY_H */
