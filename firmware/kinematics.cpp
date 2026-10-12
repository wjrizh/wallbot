/*
 * kinematics.cpp - 3-DOF 运动学解算 (移植自主项目)
 * 几何模型: L1=220mm 前臂, L2=220mm 后臂, H=135mm 腿高
 * 刚度补偿: 当前全为零
 */

#include "kinematics.h"
#include "config.h"
#include <math.h>

#define PI 3.14159265358979323846f

float g_k1_comp = 0.0f;
float g_k2_comp = 0.0f;
float g_k3_comp = 0.0f;
float g_kt_comp = 0.0f;

static void clamp_reach(float* X, float* Y) {
    float L1 = KIN_L1, L2 = KIN_L2;
    float max_reach = (L1 + L2) * 0.9999f;
    float D2 = (*X) * (*X) + (*Y) * (*Y);
    if (D2 > max_reach * max_reach) {
        float D = sqrtf(D2);
        float scale = max_reach / D;
        *X *= scale;
        *Y *= scale;
    }
}

static float safe_acos(float val) {
    if (val > 1.0f) val = 1.0f;
    if (val < -1.0f) val = -1.0f;
    return acosf(val);
}

static float clamp_angle(float a) {
    if (a < 0.0f) return 0.0f;
    if (a > 180.0f) return 180.0f;
    return a;
}

void Kinematics_IK(float X, float Y, bool is_front_move,
                   float* m1, float* m2, float* m3,
                   float* fixX, float* fixY) {
    if (!m1 || !m2 || !m3 || !fixX || !fixY) return;

    float L1 = KIN_L1, L2 = KIN_L2;

    *fixX = X; *fixY = Y;
    clamp_reach(fixX, fixY);
    float D2 = (*fixX) * (*fixX) + (*fixY) * (*fixY);
    float D = sqrtf(D2);

    float cos_alpha = (L1*L1 + L2*L2 - D2) / (2.0f * L1 * L2);
    float alpha = safe_acos(cos_alpha) * 180.0f / PI;
    
    float cos_ang1 = (L1*L1 + D2 - L2*L2) / (2.0f * L1 * D);
    float ang1 = safe_acos(cos_ang1) * 180.0f / PI;

    float cos_ang3 = (L2*L2 + D2 - L1*L1) / (2.0f * L2 * D);
    float ang3 = safe_acos(cos_ang3) * 180.0f / PI;

    float theta_D = atan2f(*fixY, *fixX) * 180.0f / PI;

    float math_m1, math_m3;
    if (!is_front_move) {
        math_m1 = theta_D + ang1;
        math_m3 = ang3 - theta_D;
    } else {
        math_m3 = theta_D + ang3;
        math_m1 = ang1 - theta_D;
    }
    float math_m2 = 220.0f - alpha;

    float comp_m1 = 0.0f, comp_m2 = 0.0f, comp_m3 = 0.0f;
    if (*fixY > 0.0f) {
        float lift_ratio = *fixY / 10.0f;
        float torque_factor = (*fixX / 300.0f) * g_kt_comp;
        float dynamic_K2 = g_k2_comp + torque_factor;
        float fade_out = (*fixY <= 10.0f) ? (*fixY / 10.0f) : 1.0f;

        if (is_front_move) {
            comp_m3 = -(g_k3_comp * lift_ratio * fade_out);
            comp_m2 = (dynamic_K2 * lift_ratio * fade_out);
        } else {
            comp_m1 = (g_k1_comp * lift_ratio * fade_out);
            comp_m2 = (dynamic_K2 * lift_ratio * fade_out);
        }
    }

    float base_math_m1 = 20.0f;
    float base_math_m3 = 20.0f;
    float base_math_m2 = 80.0f;

    float joint_m1 = KIN_BASE_M1 + (math_m1 - base_math_m1 + comp_m1) * 2.0f;
    float joint_m3 = KIN_BASE_M3 + (math_m3 - base_math_m3 + comp_m3) * 2.0f;
    float joint_m2 = KIN_BASE_M2 + (math_m2 - base_math_m2 + comp_m2);

    *m1 = clamp_angle(joint_m1);
    *m2 = clamp_angle(joint_m2);
    *m3 = clamp_angle(joint_m3);
}

void Kinematics_FK(float m1, float m2, float m3, bool anchor_back,
                   float* footX, float* footY) {
    if (!footX || !footY) return;

    float L1 = KIN_L1, L2 = KIN_L2;

    float math_m1 = 20.0f + (m1 - KIN_BASE_M1) / 2.0f;
    float math_m3 = 20.0f + (m3 - KIN_BASE_M3) / 2.0f;
    float math_m2 = m2;

    float alpha = 220.0f - math_m2;
    float alpha_rad = alpha * PI / 180.0f;

    float D = sqrtf(L1*L1 + L2*L2 - 2.0f*L1*L2*cosf(alpha_rad));

    if (anchor_back) {
        float cos_ang3 = (L2*L2 + D*D - L1*L1) / (2.0f * L2 * D);
        float ang3 = safe_acos(cos_ang3);
        float theta_D_rad = (math_m3 * PI / 180.0f) - ang3;
        *footX = D * cosf(theta_D_rad);
        *footY = D * sinf(theta_D_rad);
    } else {
        float cos_ang1 = (L1*L1 + D*D - L2*L2) / (2.0f * L1 * D);
        float ang1 = safe_acos(cos_ang1);
        float theta_D_rad = (math_m1 * PI / 180.0f) - ang1;
        *footX = D * cosf(theta_D_rad);
        *footY = D * sinf(theta_D_rad);
    }
}

float Kinematics_GetSpan(float m2) {
    float L1 = KIN_L1, L2 = KIN_L2;
    float alpha = 220.0f - m2;
    float alpha_rad = alpha * PI / 180.0f;
    return sqrtf(L1*L1 + L2*L2 - 2.0f*L1*L2*cosf(alpha_rad));
}

float Kinematics_GetHoverLimit(void) {
    float max_safe = (KIN_L1 + KIN_L2) * 0.99f;
    float H = KIN_LIFT_H;
    if (max_safe <= H) return 0.0f;
    return sqrtf(max_safe * max_safe - H * H);
}

/* ==================== 尺蠖几何 (对称等高) ==================== */

float Kinematics_StandSpan(void) {
    float th = KIN_STAND_THETA * PI / 180.0f;
    return 2.0f * KIN_ARM_LEN * cosf(th);
}

void Kinematics_SolveSpan(float X, float* m1, float* m2, float* m3) {
    float L2span = 2.0f * KIN_ARM_LEN;
    if (X < 1.0f) X = 1.0f;
    if (X > L2span * 0.9999f) X = L2span * 0.9999f;

    float theta = acosf(X / L2span) * 180.0f / PI;
    float phi   = 180.0f - 2.0f * theta;

    *m1 = KIN_RATIO_M1 * theta + KIN_ZERO_M1;
    *m3 = KIN_RATIO_M3 * theta + KIN_ZERO_M3;
    *m2 = KIN_ZERO_M2 + (180.0f - phi) / KIN_RATIO_M2;

    if (*m1 < 0.0f)   *m1 = 0.0f;   if (*m1 > 180.0f) *m1 = 180.0f;
    if (*m2 < 0.0f)   *m2 = 0.0f;   if (*m2 > 180.0f) *m2 = 180.0f;
    if (*m3 < 0.0f)   *m3 = 0.0f;   if (*m3 > 180.0f) *m3 = 180.0f;
}

void Kinematics_SolveStep(float step_mm, float* m1, float* m2, float* m3) {
    float X = Kinematics_StandSpan() - step_mm;
    Kinematics_SolveSpan(X, m1, m2, m3);
}