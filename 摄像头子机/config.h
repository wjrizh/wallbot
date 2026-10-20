/* config.h - 摄像头子机全部可调参数 (集中一处, 上机调参只改这里)
 *
 * 模块划分:
 *   摄像头子机.ino  主入口: 初始化/看门狗/模块编排
 *   cam_module      摄像头初始化 + 测检模式切换 + 诊断计数
 *   discovery       UDP 自动发现 (端口 48899)
 *   stream_server   MJPEG 流 (:81/stream) + :80 的 /jpg /fps / 诊断
 *   tag_detector    板端 AprilTag 36h11 检测 (灰度帧 -> 角点/ID)
 *   align_measure   视觉纠偏测量 (参考位 + 偏差解算, 移植 tag_utils.py)
 *   align_server    纠偏 HTTP API (/align/*), 母机/App 闭环调用的接口
 */
#ifndef WALLBOT_CAM_CONFIG_H
#define WALLBOT_CAM_CONFIG_H

#include "esp_camera.h"   /* FRAMESIZE_*/

/* ---- WiFi: 子机自开热点 (AP 模式) ----
 * 不再依赖外部热点与 DHCP: 地址固定, 平板/手机直连后打开 http://192.168.4.1/ 即可。
 * 若需回到 STA 模式 (与母机同一热点), 把 CAM_AP_MODE 设为 0。 */
#define CAM_AP_MODE   1
#define WIFI_SSID     "robot"          /* STA 模式的上级热点 (CAM_AP_MODE=0 时生效) */
#define WIFI_PASS     "12345678"
#define AP_SSID       "wallbot-cam"    /* 子机自己开的热点名 */
/* 开放热点 (无密码): ESP32 Arduino 3.3.11 的 AP+WPA2 在 wpa_ap_get_wpa_ie
 * 有已知崩溃 (LoadProhibited/LoadStoreAlignment), 客户端一探测就重启。
 * 调试场景无需加密, 置空即走 WIFI_AUTH_OPEN, 彻底绕开该缺陷。 */
#define AP_PASS       ""
#define AP_IP         "192.168.4.1"
#define AP_GW         "192.168.4.1"
#define AP_MASK       "255.255.255.0"
#define AP_CHANNEL    6

/* ---- UDP 自动发现 ---- */
#define DISCOVERY_PORT        48899
#define DISCOVERY_REQ         "WALLBOT_DISCOVER"
#define DEVICE_ROLE           "camchild"

/* ---- 摄像头 ---- */
/* 流媒体分辨率/画质 (MJPEG 硬编码)
 * 固定 VGA 不再运行时切换: 切 framesize 会重置 GC2145 寄存器与 GDMA 链,
 * 正在推流的帧被截断(丢 EOI), WebView 的 MJPEG 解码器会定格,
 * 且画面尺寸跳变本身就不满足"检测时画面不变"的要求。
 * 检测侧用 esp_jpeg_decode 的 1/2 缩放直接从 VGA 解出 320x240。 */
#define STREAM_FRAMESIZE      FRAMESIZE_VGA
#define STREAM_JPEG_QUALITY   12
#define STREAM_FB_COUNT       3      /* 3 缓冲: 推流与后台检测并发取帧时不至于 DMA 停摆 */
/* ---- 检测分辨率 (固定, 不再是运行时的另一种相机模式) ----
 * VGA 帧经 JPEG_IMAGE_SCALE_1_2 解码后正好是 320x240,
 * tag_detector 的像素参数(EDGE_PX_MIN/ADAPT_WIN)全部沿用不变。 */
#define ALIGN_DETECT_W          320
#define ALIGN_DETECT_H          240

/* ---- AprilTag / 视觉纠偏几何参数 (与 code/tag_utils.py 保持一致) ---- */
#define TAG_REAL_MM       50.0f  /* 标签黑框真实边长(mm), 打印后实测填入 */
#define R_TAG_MM          60.0f  /* 标签黑框中心平面到管道轴线的垂直距离(mm), 贴好后必须实测 */
#define PIPE_RADIUS_MM    60.0f  /* 管道内半径(mm), 仅用于把 phi 换算成沿管壁弧长 */

#define TAG_ID            0      /* 使用的标签 ID */

/* ---- 检测器参数 ---- */
#define DET_EDGE_PX_MIN   12     /* 黑框边长像素下限; 12px 支持放远约 2 倍距离 */
#define DET_EDGE_PX_MAX   200    /* 标签黑框边长像素上限 */
#define DET_MAX_HAMMING   1      /* 解码允许的最大汉明距离 */
#define DET_CONTRAST_MIN  25.0f  /* 黑白电平最小对比度 (抗全黑/全白误检) */
#define DET_ADAPT_WIN     45     /* 自适应阈值窗口(奇数, 像素)。
                                  * 须大于最大标签的黑框厚度(edge_max/8*1=25px):
                                  * 窗口大->厚黑框核心也判暗(或成闭合厚环带), 不断裂 */
#define DET_ADAPT_BIAS    25     /* 判黑偏置: gray < mean - bias 视为暗。
                                  * 取 ~2.5x 噪声sigma: 噪声误判率<1%, 背景不絮凝;
                                  * 黑框会变细环带(连续等值线,不断裂), 由填洞补实心 */

/* ---- 纠偏闭环参数 (执行文档 6.3 节: ANGLE_TOL=4°, 3 帧一致才采信) ---- */
#define ALIGN_FRAMES        3      /* 每次测量连拍帧数 (识别要连续一致) */
#define ALIGN_THETA_SPREAD  1.5f   /* 多帧 theta 一致性阈值 (deg) */
#define ALIGN_UV_SPREAD     3.0f   /* 多帧中心一致性阈值 (px) */
#define ALIGN_ANGLE_TOL     4.0f   /* 夹取判据: |phi|,|psi| <= 4° (夹爪 5° 留 1° 裕量) */
#define ALIGN_AXIAL_TOL_MM  20.0f  /* 轴向进/退偏差容差(mm), 夹爪开口容差相关 */

/* ---- NVS 存储命名空间 (参考位掉电保存) ---- */
#define ALIGN_NVS_NS        "wallbot-cam"

#endif /* WALLBOT_CAM_CONFIG_H */
