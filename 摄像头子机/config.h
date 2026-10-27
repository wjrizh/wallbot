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
 * 固定 SVGA, 运行时绝不切 framesize: 切分辨率会把正在推的 MJPEG 打断。
 * 检测: JPEG 解成 800x600 灰度; 连通域与采码均在 800x600 全分辨率跑 (Phase A1)。
 * 22mm @ ~30cm、~65° 镜头, 800 宽约 46px 黑框, 接近 5.8px/module。 */
#define STREAM_FRAMESIZE      FRAMESIZE_SVGA
/* ESP JPEG quality: 数字越小画质越高。8 比 12 更利格子解码, 略增带宽/解码耗时 (Phase A2) */
#define STREAM_JPEG_QUALITY   8
#define STREAM_FB_COUNT       3
#define ALIGN_DETECT_W          800
#define ALIGN_DETECT_H          600

/* ---- 路标模式 ----
 * LANDMARK_DUAL_TAG: 两孔位各贴不同 ID 的 22mm tag36h11 (当前默认)
 *   两中心中点 -> u,v; 小ID->大ID 连线 -> theta (消 180); 中心距 -> edge
 * LANDMARK_CHASSIS / LANDMARK_APRILTAG: 备份 */
#define LANDMARK_APRILTAG  0
#define LANDMARK_CHASSIS   1
#define LANDMARK_DUAL_TAG  2
#define LANDMARK_MODE      LANDMARK_DUAL_TAG

#define TAG_REAL_MM         15.0f  /* 单枚码黑框边长(mm); 双码模式不用它做比例尺 */
#define DUAL_TAG_SEP_MM     40.0f  /* ★两码中心实测距(mm), 卡尺量完必须改 */
#define CHASSIS_HOLE_SEP_MM 45.0f  /* 双孔备份: 两腰形孔中心距(mm) */
#if LANDMARK_MODE == LANDMARK_DUAL_TAG
#  define LANDMARK_MM  DUAL_TAG_SEP_MM
#elif LANDMARK_MODE == LANDMARK_CHASSIS
#  define LANDMARK_MM  CHASSIS_HOLE_SEP_MM
#else
#  define LANDMARK_MM  TAG_REAL_MM
#endif

#define R_TAG_MM          395.0f  /* 路标中心(双码中点)到管道轴线的垂直距离(mm), 贴好后必须实测 */
#define PIPE_RADIUS_MM    250.0f  /* 管道内半径(mm), 仅用于把 phi 换算成沿管壁弧长 */

#define TAG_ID_A          0      /* 双码第一枚 ID (须与 TAG_ID_B 不同) */
#define TAG_ID_B          1      /* 双码第二枚 ID */
#define TAG_ID            TAG_ID_A  /* 兼容旧单码路径 */

/* ---- 检测器参数 ---- */
#define DET_EDGE_PX_MIN   12     /* 全分辨率连通域: 25cm 约 33px 黑框, 12 留余量 */
#define DET_EDGE_PX_MAX   280    /* 近距全分辨率上限 (原半分辨 200 的约 1.4x) */
#define DET_MAX_HAMMING   1      /* 解码允许的最大汉明距离 */
#define DET_CONTRAST_MIN  25.0f  /* 黑白电平最小对比度 (抗全黑/全白误检) */
#define DET_ADAPT_WIN     45     /* 自适应阈值窗口(奇数, 像素)。
                                  * 须大于最大标签的黑框厚度(edge_max/8*1=25px):
                                  * 窗口大->厚黑框核心也判暗(或成闭合厚环带), 不断裂 */
#define DET_ADAPT_BIAS    25     /* 判黑偏置: gray < mean - bias 视为暗。
                                  * 取 ~2.5x 噪声sigma: 噪声误判率<1%, 背景不絮凝;
                                  * 黑框会变细环带(连续等值线,不断裂), 由填洞补实心 */

/* ---- 纠偏闭环参数 ----
 * 双码方向稳定; Phase B: 2 帧共识略放宽, 手持间歇失败时更快出结果。 */
#define ALIGN_FRAMES        2
#define ALIGN_THETA_SPREAD  12.0f
#define ALIGN_UV_SPREAD     24.0f
#define ALIGN_ANGLE_TOL     4.0f   /* 夹取判据: |phi|,|psi| <= 4° (夹爪 5° 留 1° 裕量) */
#define ALIGN_AXIAL_TOL_MM  20.0f  /* 轴向进/退偏差容差(mm), 夹爪开口容差相关 */

/* Phase B soft-hold: when one of two tags briefly drops during consensus,
 * reuse last good dual observation for that slot.
 * TTL ~2500ms ≈ 1–2 detect cycles at current ~1–2s/frame.
 * Timestamp refreshes only on a REAL dual success, never on hold reuse
 * (so geometry is not invented forever). */
#define DUAL_SOFT_HOLD_MS   2500

/* ---- NVS 存储命名空间 (参考位掉电保存) ---- */
#define ALIGN_NVS_NS        "wallbot-cam"

#endif /* WALLBOT_CAM_CONFIG_H */
