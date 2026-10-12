#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>
#include <stdbool.h>

void Kinematics_IK(float X, float Y, bool is_front_move,
                   float* m1, float* m2, float* m3,
                   float* fixX, float* fixY);

void Kinematics_FK(float m1, float m2, float m3, bool anchor_back,
                   float* footX, float* footY);

float Kinematics_GetSpan(float m2);
float Kinematics_GetHoverLimit(void);

/* ===== 尺蠖几何求解 (对称等高约束) ===== */
float Kinematics_StandSpan(void);
void  Kinematics_SolveSpan(float X, float* m1, float* m2, float* m3);
void  Kinematics_SolveStep(float step_mm, float* m1, float* m2, float* m3);

#endif