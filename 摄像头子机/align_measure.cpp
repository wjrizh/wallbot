/* align_measure.cpp - 视觉纠偏测量 (参考位管理 + 偏差解算)
 * 数学与 视觉/code/tag_utils.py 的 delta_report/arc_mm_for 一一对应。
 */
#include <Arduino.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>
#include "config.h"
#include "align_measure.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static AlignRefPose g_ref;
static Preferences g_nvs;

static float wrap_deg(float d) { return fmodf(d + 180.0f, 360.0f) - 180.0f; }

void AlignMeasure_Init(void) {
    memset(&g_ref, 0, sizeof(g_ref));
    g_nvs.begin(ALIGN_NVS_NS, true);
    g_ref.valid = g_nvs.getBool("valid", false);
    if (g_ref.valid) {
        g_ref.u      = g_nvs.getFloat("u", 0);
        g_ref.v      = g_nvs.getFloat("v", 0);
        g_ref.theta  = g_nvs.getFloat("th", 0);
        g_ref.edge_a = g_nvs.getFloat("ea", 0);
        Serial.printf("[ALIGN] 参考位已从NVS恢复: u=%.1f v=%.1f theta=%.3fdeg edge=%.1fpx\n",
                      g_ref.u, g_ref.v, g_ref.theta * 180.0f / (float)M_PI, g_ref.edge_a);
    }
    g_nvs.end();
}

const AlignRefPose *AlignMeasure_GetRef(void) { return &g_ref; }

bool AlignMeasure_SetRef(const TagObservation *obs) {
    if (!obs || !obs->ok) return false;
    g_ref.valid = true;
    g_ref.u = obs->u;
    g_ref.v = obs->v;
    g_ref.theta = obs->theta;
    g_ref.edge_a = obs->edge_a;

    g_nvs.begin(ALIGN_NVS_NS, false);
    g_nvs.putBool("valid", true);
    g_nvs.putFloat("u", g_ref.u);
    g_nvs.putFloat("v", g_ref.v);
    g_nvs.putFloat("th", g_ref.theta);
    g_nvs.putFloat("ea", g_ref.edge_a);
    g_nvs.end();
    return true;
}

void AlignMeasure_ClearRef(void) {
    g_ref.valid = false;
    g_nvs.begin(ALIGN_NVS_NS, false);
    g_nvs.putBool("valid", false);
    g_nvs.end();
}

bool AlignMeasure_Compute(const TagObservation *obs, Deviation *dev) {
    if (!obs || !obs->ok || !g_ref.valid || !dev) return false;

    /* 管道轴线方向由参考位 theta 定义, 自动补偿相机安装旋转 */
    float th = g_ref.theta;
    float dir_ax_x = cosf(th),  dir_ax_y = sinf(th);   /* 图像里管道轴线方向 */
    float dir_pp_x = -sinf(th), dir_pp_y = cosf(th);   /* 垂直管道方向 */

    float scale = TAG_REAL_MM / (obs->edge_a > 1e-3f ? obs->edge_a : 1e-3f);
    float du = obs->u - g_ref.u;
    float dv = obs->v - g_ref.v;
    float axial = (du * dir_ax_x + dv * dir_ax_y) * scale;
    float perp  = (du * dir_pp_x + dv * dir_pp_y) * scale;

    float perp_ratio = perp / R_TAG_MM;
    if (perp_ratio > 1.0f) perp_ratio = 1.0f;
    if (perp_ratio < -1.0f) perp_ratio = -1.0f;
    float phi = asinf(perp_ratio) * 180.0f / (float)M_PI;
    float psi = wrap_deg((obs->theta - g_ref.theta) * 180.0f / (float)M_PI);

    float ratio = obs->edge_p / (obs->edge_a > 1e-3f ? obs->edge_a : 1e-3f);
    if (ratio > 1.0f) ratio = 1.0f;
    if (ratio < -1.0f) ratio = -1.0f;
    float phi_shape = acosf(ratio) * 180.0f / (float)M_PI;

    dev->phi_deg = phi;
    dev->psi_deg = psi;
    dev->phi_shape_deg = phi_shape;
    dev->axial_mm = axial;
    dev->perp_mm = perp;
    dev->arc_mm = phi * (float)M_PI / 180.0f * PIPE_RADIUS_MM;
    dev->mm_per_px = scale;
    dev->aligned = (fabsf(phi) <= ALIGN_ANGLE_TOL &&
                    fabsf(psi) <= ALIGN_ANGLE_TOL &&
                    fabsf(axial) <= ALIGN_AXIAL_TOL_MM);
    return true;
}
