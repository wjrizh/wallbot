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

/* ---- WiFi (与母机/平板同一热点) ---- */
#define WIFI_SSID     "robot"
#define WIFI_PASS     "12345678"
/* NOTE: DHCP 获取与平板热点同网段地址, 具体 IP 由 App 通过 UDP 发现获得 */

/* ---- UDP 自动发现 ---- */
#define DISCOVERY_PORT        48899
#define DISCOVERY_REQ         "WALLBOT_DISCOVER"
#define DEVICE_ROLE           "camchild"

/* ---- 摄像头 ---- */
/* 流媒体分辨率/画质 (MJPEG 硬编码) */
#define STREAM_FRAMESIZE      FRAMESIZE_HD
#define STREAM_JPEG_QUALITY   11
/* ---- 检测分辨率 (检测模式只切分辨率, 保持 JPEG; 板上解码为灰度后检测) ---- */
#define ALIGN_DETECT_FRAMESIZE  FRAMESIZE_QVGA
#define ALIGN_DETECT_W          320   /* FRAMESIZE_QVGA 对应的宽 */
#define ALIGN_DETECT_H          240   /* FRAMESIZE_QVGA 对应的高 */

/* ---- AprilTag / 视觉纠偏几何参数 (与 code/tag_utils.py 保持一致) ---- */
#define TAG_REAL_MM       50.0f  /* 标签黑框真实边长(mm), 打印后实测填入 */
#define R_TAG_MM          60.0f  /* 标签黑框中心平面到管道轴线的垂直距离(mm), 贴好后必须实测 */
#define PIPE_RADIUS_MM    60.0f  /* 管道内半径(mm), 仅用于把 phi 换算成沿管壁弧长 */

#define TAG_ID            0      /* 使用的标签 ID */

/* ---- 检测器参数 ---- */
#define DET_EDGE_PX_MIN   24     /* 标签黑框边长像素下限 (QVGA 下的预期范围) */
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
