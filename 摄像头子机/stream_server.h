/* stream_server.h - MJPEG 流服务 + 截图/帧率/诊断接口
 *
 * :81/stream  multipart/x-mixed-replace MJPEG 实时流
 * :80/jpg     单帧 JPEG
 * :80/fps     当前流帧率
 * :80/        诊断信息 (cam_init/pid/psram/heap)
 */
#ifndef WALLBOT_STREAM_SERVER_H
#define WALLBOT_STREAM_SERVER_H

#include <stdbool.h>

bool StreamServer_Start(void);   /* 启动 :80 控制服务 + :81 流服务 */

#endif /* WALLBOT_STREAM_SERVER_H */
