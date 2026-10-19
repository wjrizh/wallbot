/* align_measure.h - 视觉纠偏测量 (参考位管理 + 偏差解算)
 *
 * 测量模型移植自 视觉/code/tag_utils.py (俯视安装 v2):
 *   - 相机装夹爪正上方垂直向下拍, 标签贴小车顶面, c0->c1 边平行管道轴线
 *   - phi (周向滚转角): 标签中心垂直于管道方向的位移反解, phi = asin(perp/R_TAG_MM)
 *   - psi (机身歪头角): 面内角差 theta - theta0
 *   - axial (轴向进/退): 中心沿管道方向的位移
 *   - arc: 周向纠偏应走的弧长 = R_PIPE * phi
 */
#ifndef WALLBOT_ALIGN_MEASURE_H
#define WALLBOT_ALIGN_MEASURE_H

#include <stdint.h>
#include <stdbool.h>
#include "tag_detector.h"

/* 参考位 (放子机前、夹爪对中时拍) */
typedef struct {
    bool  valid;
    float u;        /* 参考中心 (像素) */
    float v;
    float theta;    /* 参考面内角 (弧度) = 图像里的管道轴线方向 */
    float edge_a;   /* 参考轴向边长 (像素) */
} AlignRefPose;

/* 偏差报告 (一次测量) */
typedef struct {
    float phi_deg;        /* 周向滚转角 (deg), 主判据 */
    float psi_deg;        /* 机身航向/歪头角 (deg) */
    float phi_shape_deg;  /* 边长收缩反解的滚转角 (无符号, 交叉验证) */
    float axial_mm;       /* 沿管道方向位移 (mm), 正负=进/退 (上机确认) */
    float perp_mm;        /* 垂直管道方向位移 (mm) */
    float arc_mm;         /* 周向纠偏应走弧长 (mm), 带方向 */
    float mm_per_px;      /* 当前帧比例尺 */
    bool  aligned;        /* |phi|,|psi| <= ALIGN_ANGLE_TOL 且 |axial| <= 轴向容差 */
} Deviation;

void             AlignMeasure_Init(void);      /* 从 NVS 加载参考位 */
const AlignRefPose *AlignMeasure_GetRef(void);
bool             AlignMeasure_SetRef(const TagObservation *obs);  /* 保存并写 NVS */
void             AlignMeasure_ClearRef(void);  /* 清除参考位 */
bool             AlignMeasure_Compute(const TagObservation *obs, Deviation *dev);

#endif /* WALLBOT_ALIGN_MEASURE_H */
