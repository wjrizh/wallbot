/* adj_plan.h - 视觉纠偏运动规划, 逐字移植自主固件 main.c
 * 常数与主固件完全一致(含占位待标定值) */
#ifndef ADJ_PLAN_H
#define ADJ_PLAN_H

#include <math.h>

#define ADJ_PI              3.14159265f
#define ADJ_SPEED_REV       30.0f
#define ADJ_MM_PER_MOTORREV 15.0f
#define ADJ_TRACK_MM        100.0f
#define ADJ_PIPE_R_MM       60.0f
#define ADJ_DIAG_BETA_DEG   30.0f
#define ADJ_MV_SIGN         (+1)
#define ADJ_SPIN_SIGN       (+1)
#define ADJ_AXIAL_SIGN      (+1)
#define ADJ_TOL_PHI         0.5f
#define ADJ_TOL_PSI         0.5f
#define ADJ_TOL_AX          2.0f
#define ADJ_MAX_PHI         10.0f
#define ADJ_MAX_PSI         10.0f
#define ADJ_MAX_AX          30.0f
#define ADJ_SETTLE_MS       150
#define ADJ_SLIP_DIFF_MM    8.0f
#define ADJ_ST_AXIAL        1
#define ADJ_ST_SPIN         2
#define ADJ_ST_MOVE         3
#define ADJ_ST_SETTLE       4
#define ADJ_TEL_PERIOD_MS   500u

typedef struct { uint16_t a_old, b_old; float a_mm, b_mm; } ADJ_ODOM_T;

static float Adj_Clamp(float v, float lim){
  if(v >  lim) return  lim;
  if(v < -lim) return -lim;
  return v;
}

static void Adj_OdomStart(ADJ_ODOM_T *o){
  o->a_old = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
  o->b_old = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  o->a_mm = 0.0f; o->b_mm = 0.0f;
}

static void Adj_OdomUpdate(ADJ_ODOM_T *o){
  int16_t da = (int16_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim2) - o->a_old);
  int16_t db = (int16_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim3) - o->b_old);
  o->a_old = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
  o->b_old = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  o->a_mm += (float)da / 7.0f * ADJ_MM_PER_MOTORREV;
  o->b_mm += (float)db / 7.0f * ADJ_MM_PER_MOTORREV;
}

static uint8_t Adj_Settle(void){
  uint32_t t0 = HAL_GetTick();
  motor_STOP();
  while(HAL_GetTick() - t0 < ADJ_SETTLE_MS){ HAL_Delay(10); }
  return 0;
}

static uint8_t Adj_Move(float dist_mm){
  ADJ_ODOM_T od;
  float target = fabsf(dist_mm);
  float v = ADJ_SPEED_REV;
  uint32_t t_to = (uint32_t)(target / (ADJ_SPEED_REV * ADJ_MM_PER_MOTORREV) * 1500.0f) + 800u;
  uint32_t t0 = HAL_GetTick();
  if(target < 0.5f) return 0;
  Adj_OdomStart(&od);
  {
    uint32_t t_tel = 0;
    int8_t   dir = (dist_mm > 0) ? 1 : -1;
    while(1){
      if(dist_mm > 0){ PID_MA(+v); PID_MB(-v); }
      else           { PID_MA(-v); PID_MB(+v); }
      HAL_Delay(10);
      Adj_OdomUpdate(&od);
      {
        float done = (fabsf(od.a_mm) + fabsf(od.b_mm)) * 0.5f;
        if(HAL_GetTick() - t_tel >= ADJ_TEL_PERIOD_MS){
          uint8_t pct = (target > 0.01f) ? (uint8_t)(done * 100.0f / target) : 0;
          if(pct > 100) pct = 100;
          t_tel = HAL_GetTick();
          send_telem(ADJ_ST_MOVE, dir, (int16_t)od.a_mm, (int16_t)od.b_mm, (int16_t)target, pct, 0);
        }
        if(done >= target) break;
      }
      if(fabsf(fabsf(od.a_mm) - fabsf(od.b_mm)) > ADJ_SLIP_DIFF_MM){
        motor_STOP();
        send_telem(ADJ_ST_MOVE, dir, (int16_t)od.a_mm, (int16_t)od.b_mm, (int16_t)target, 100, 3);
        printf("ADJ move slip A=%d B=%d\r\n", (int)od.a_mm, (int)od.b_mm);
        return 2;
      }
      if(HAL_GetTick() - t0 > t_to){
        motor_STOP();
        send_telem(ADJ_ST_MOVE, dir, (int16_t)od.a_mm, (int16_t)od.b_mm, (int16_t)target, 100, 3);
        printf("ADJ move timeout\r\n");
        return 2;
      }
    }
    send_telem(ADJ_ST_MOVE, dir, (int16_t)od.a_mm, (int16_t)od.b_mm, (int16_t)target, 100, 1);
  }
  motor_STOP();
  printf("ADJ move %dmm ok (A=%d B=%d)\r\n", (int)target, (int)od.a_mm, (int)od.b_mm);
  return 0;
}

static uint8_t Adj_Spin(float theta_deg){
  ADJ_ODOM_T od;
  float arc = fabsf(theta_deg) * ADJ_PI / 180.0f * (ADJ_TRACK_MM * 0.5f);
  float v = ADJ_SPEED_REV;
  uint32_t t_to = (uint32_t)(arc / (ADJ_SPEED_REV * ADJ_MM_PER_MOTORREV) * 1500.0f) + 800u;
  uint32_t t0 = HAL_GetTick();
  if(arc < 0.5f) return 0;
  Adj_OdomStart(&od);
  {
    uint32_t t_tel = 0;
    int8_t   dir = (theta_deg * (float)ADJ_SPIN_SIGN > 0) ? 1 : -1;
    while(1){
      if(theta_deg * (float)ADJ_SPIN_SIGN > 0){ PID_MA(-v); PID_MB(-v); }
      else                                     { PID_MA(+v); PID_MB(+v); }
      HAL_Delay(10);
      Adj_OdomUpdate(&od);
      {
        float done = (fabsf(od.a_mm) + fabsf(od.b_mm)) * 0.5f;
        if(HAL_GetTick() - t_tel >= ADJ_TEL_PERIOD_MS){
          uint8_t pct = (arc > 0.01f) ? (uint8_t)(done * 100.0f / arc) : 0;
          if(pct > 100) pct = 100;
          t_tel = HAL_GetTick();
          send_telem(ADJ_ST_SPIN, dir, (int16_t)od.a_mm, (int16_t)od.b_mm, (int16_t)arc, pct, 0);
        }
        if(done >= arc) break;
      }
      if(HAL_GetTick() - t0 > t_to){
        motor_STOP();
        send_telem(ADJ_ST_SPIN, dir, (int16_t)od.a_mm, (int16_t)od.b_mm, (int16_t)arc, 100, 3);
        printf("ADJ spin timeout\r\n");
        return 2;
      }
    }
    send_telem(ADJ_ST_SPIN, dir, (int16_t)od.a_mm, (int16_t)od.b_mm, (int16_t)arc, 100, 1);
  }
  motor_STOP();
  printf("ADJ spin %ddeg ok\r\n", (int)theta_deg);
  return 0;
}

static uint8_t Adj_Axial(float ax_mm){
  float beta = ADJ_DIAG_BETA_DEG * ADJ_PI / 180.0f;
  float L = fabsf(ax_mm) / sinf(beta);
  float d = fabsf(ax_mm) / tanf(beta);
  float tilt = (ax_mm > 0 ? 1.0f : -1.0f) * (float)ADJ_AXIAL_SIGN;
  uint8_t rc;
  printf("ADJ axial L=%dmm d=%dmm\r\n", (int)L, (int)d);
  rc = Adj_Spin( tilt * ADJ_DIAG_BETA_DEG);  if(rc) return rc;
  Adj_Settle();
  rc = Adj_Move( +L );                       if(rc) return rc;
  Adj_Settle();
  rc = Adj_Spin( -tilt * ADJ_DIAG_BETA_DEG); if(rc) return rc;
  Adj_Settle();
  rc = Adj_Move( -d );                       if(rc) return rc;
  return 0;
}

static uint8_t Adj_Execute(float phi_deg, float psi_deg, float ax_mm){
  float phi = Adj_Clamp(phi_deg, ADJ_MAX_PHI);
  float psi = Adj_Clamp(psi_deg, ADJ_MAX_PSI);
  float ax  = Adj_Clamp(ax_mm,  ADJ_MAX_AX);
  uint8_t rc = 0;
  Clear_All_PID();
  if(fabsf(phi) < ADJ_TOL_PHI) phi = 0;
  if(fabsf(psi) < ADJ_TOL_PSI) psi = 0;
  if(fabsf(ax)  < ADJ_TOL_AX)  ax  = 0;
  printf("ADJ start phi=%d psi=%d ax=%d\r\n", (int)phi, (int)psi, (int)ax);
  if(ax != 0){ rc = Adj_Axial(ax); if(rc) goto out; }
  Adj_Settle();
  if(psi != 0){ rc = Adj_Spin(-psi * (float)ADJ_SPIN_SIGN); if(rc) goto out; }
  Adj_Settle();
  if(phi != 0){ rc = Adj_Move(phi * ADJ_PI / 180.0f * ADJ_PIPE_R_MM * (float)ADJ_MV_SIGN); if(rc) goto out; }
out:
  motor_STOP();
  printf("ADJ finish rc=%u\r\n", rc);
  return rc;
}

#endif /* ADJ_PLAN_H */
