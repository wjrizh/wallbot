/* tag_detector.h - 板端 AprilTag 36h11 检测 (纯算法, 输入灰度帧)
 *
 * 流程: 自适应阈值 -> 连通域(黑框) -> 主轴对角极值取角点 -> 边缘直线拟合
 *       亚像素角点 -> 单应矩阵 -> 采样 36 数据位 -> 码表匹配(容 1 bit 错)
 * 解码约定与官方 AprilRobotics/apriltag 一致 (见 tag36h11_table.h)。
 */
#ifndef WALLBOT_TAG_DETECTOR_H
#define WALLBOT_TAG_DETECTOR_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool  ok;
    int   id;                 /* 解码出的标签 ID */
    int   hamming;            /* 与码表的汉明距离 (0 = 完全一致) */
    float corners[4][2];      /* 图像坐标, c0左上 c1右上 c2右下 c3左下 (标签自身朝向) */
    float u, v;               /* 中心 (像素) */
    float theta;              /* c0->c1 边方向角 (弧度), 与 tag_utils.py 的 theta 一致 */
    float edge_a;             /* 轴向两边平均边长 (像素), 比例尺基准 */
    float edge_p;             /* 周向两边平均边长 (像素) */
    float margin;             /* 平均判决裕度 (灰度) */
    float contrast;           /* 白黑电平差 */
} TagObservation;

/* 在灰度图中检测 TAG_ID 标签; 找到返回 true 并填充 out */
bool TagDetector_Detect(const uint8_t *gray, int w, int h, int stride,
                        TagObservation *out);

#endif /* WALLBOT_TAG_DETECTOR_H */
