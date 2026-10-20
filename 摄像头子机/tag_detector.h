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

/* 诊断信息: 检测失败时也能看出卡在哪一步 (供 /align/view 叠加显示) */
typedef struct {
    int  ncomps;          /* 连通域数量 */
    int  ncand;           /* 通过面积初筛的候选数 */
    int  cand_bbox[4];    /* 最大候选包围盒 x,y,w,h (无候选时全 0) */
    int  cand_area;
    float cand_fill;      /* 面积/包围盒面积 */
    int  drop_small;      /* 因太小被丢弃的候选数 */
    int  drop_large;      /* 因太大被丢弃 */
    int  drop_fill;       /* 因填充率不符被丢弃 */
    int  drop_aspect;     /* 因长宽比不符被丢弃 */
    int  decode_tried;    /* 实际尝试解码的候选数 */
    int  best_hamming;    /* 尝试过的最优汉明距离 (255=未尝试) */
    float best_contrast;  /* 最优候选的对比度 */
    float best_margin;
    int  stage;           /* 失败阶段: 0=ok 1=无连通域 2=候选全被筛掉 3=解码失败 */
} TagDiag;

/* 在灰度图中检测 TAG_ID 标签; 找到返回 true 并填充 out。
 * diag 可为 NULL; 非 NULL 时无论成功失败都填充诊断信息。 */
bool TagDetector_Detect(const uint8_t *gray, int w, int h, int stride,
                        TagObservation *out);

bool TagDetector_DetectEx(const uint8_t *gray, int w, int h, int stride,
                          TagObservation *out, TagDiag *diag);

#endif /* WALLBOT_TAG_DETECTOR_H */
