/* ==========================================================================
 * 01_child_main.c  --  child STM32 main
 * ========================================================================== */

/* --------------------------------------------------------------------------
 * FILE: firmware/pabi_F103rct6/Core/Src/main.c
 * -------------------------------------------------------------------------- */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "flash.h"
#include <math.h>            //???(atan2f/sqrtf)
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>             /* atoi: 树莓派PWM设定指令 */
//GIS? 552g 60% 7.6V ok ??500g?472g ????80? ?? ?
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
__STATIC_INLINE uint32_t GXT_SYSTICK_IsActiveCounterFlag(void)
{
  return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}
static uint32_t getCurrentMicros(void)
{
  /* Ensure COUNTFLAG is reset by reading SysTick control and status register */
  GXT_SYSTICK_IsActiveCounterFlag();
  uint32_t m = HAL_GetTick();
  const uint32_t tms = SysTick->LOAD + 1;
  __IO uint32_t u = tms - SysTick->VAL;
  if (GXT_SYSTICK_IsActiveCounterFlag()) {
    m = HAL_GetTick();
    u = tms - SysTick->VAL;
  }
  return (m * 1000 + (u * 1000) / tms);
}
//??????us
uint32_t micros(void)
{
  return getCurrentMicros();
}

void Delay_micros(uint16_t otime)
{
  uint32_t ODtime = micros();
	while((micros()-ODtime) <= otime){
	}
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



uint32_t TIMEA_MEM_old = 0;
uint32_t TIMEA_MEM_new = 0;
uint32_t TEMP_TIME_A = 0;

uint16_t MA_COUNTER_new = 0;
uint16_t MA_COUNTER_old= 0;
// A轮 TIM2  减速比 1:380，测速单位是电机轴转/s，max约250转/s
float MA_V_GET(void){
  int TEMP_COUNTER = 0;
  float V_A = 0;
  const float dt = 0.01f;  /* 固定10ms控制周期 */
  
  MA_COUNTER_new = __HAL_TIM_GET_COUNTER(&htim2);
  TEMP_COUNTER = MA_COUNTER_new - MA_COUNTER_old;
  
  if(((TEMP_COUNTER) > 10000) || ((TEMP_COUNTER) < -10000))
  {
    if((TEMP_COUNTER) > 10000){
      V_A = ((float)(MA_COUNTER_new - (MA_COUNTER_old + 65535)) / 7.0f) / dt;
    }
    else{
      V_A = ((float)(MA_COUNTER_new - (MA_COUNTER_old - 65535)) / 7.0f) / dt;
    }
  }
  else
  {
    V_A = ((float)(TEMP_COUNTER) / 7.0f) / dt;
  }
  
  MA_COUNTER_old = MA_COUNTER_new;
  return V_A;
}




uint32_t TIMEB_MEM_old = 0;
uint32_t TIMEB_MEM_new = 0;
uint32_t TEMP_TIME_B = 0;

uint16_t MB_COUNTER_new = 0;
uint16_t MB_COUNTER_old= 0;
// B轮 TIM3  减速比 1:380，测速单位是电机轴转/s，max约250转/s
float MB_V_GET(void){
  int TEMP_COUNTER = 0;
  float V_B = 0;
  const float dt = 0.01f;  /* 固定10ms控制周期 */
  
  MB_COUNTER_new = __HAL_TIM_GET_COUNTER(&htim3);
  TEMP_COUNTER = MB_COUNTER_new - MB_COUNTER_old;
  
  if(((TEMP_COUNTER) > 10000) || ((TEMP_COUNTER) < -10000))
  {
    if((TEMP_COUNTER) > 10000){
      V_B = ((float)(MB_COUNTER_new - (MB_COUNTER_old + 65535)) / 7.0f) / dt;
    }
    else{
      V_B = ((float)(MB_COUNTER_new - (MB_COUNTER_old - 65535)) / 7.0f) / dt;
    }
  }
  else
  {
    V_B = ((float)(TEMP_COUNTER) / 7.0f) / dt;
  }
  
  MB_COUNTER_old = MB_COUNTER_new;
  return V_B;
}


float err_MB = 0;
int PIDNUM_MB = 0;//(0~4199)
float g_wheel_va = 0;
float g_wheel_vb = 0;
uint8_t g_wheels_launched = 0;
void PID_MB(float V_Goat){
  float kp = 18;
  float ki = 90;
  float V_now = 0;
  float PIDNUM = 0;
  float err = 0;

  /* ????????180(????): ??????????????, ???? */
  V_Goat = -V_Goat;
  
  V_now = MB_V_GET();
  g_wheel_vb = V_now;
  
  if(V_Goat < 0) err = -V_Goat + V_now;
  else err = V_Goat - V_now;
  
  float dt = 0.01f;  /* 固定10ms控制周期 */
  float P_term = kp * err;
  float I_inc = err * ki * dt;
  
  if ((P_term + err_MB > 4199 && I_inc > 0) || (P_term + err_MB < 0 && I_inc < 0)) {
  } else {
      err_MB += I_inc;
  }
  
  if(err_MB > 4000) err_MB = 4000;
  else if(err_MB < -4000) err_MB = -4000;
  
  PIDNUM = P_term + err_MB;
  PIDNUM_MB = (int)PIDNUM;
  
  if(PIDNUM_MB > 4199) PIDNUM_MB = 4199;
  else if(PIDNUM_MB < 0) PIDNUM_MB = 0;
  
  if(V_Goat < 0){
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,PIDNUM_MB);
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
  }
  else if (V_Goat > 0){
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,PIDNUM_MB);
  }
  else{
    MB_STOP();
  }
}


void MA_STOP(void){
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,4199);
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,4199);
}

void MB_STOP(void){
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,4199);
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,4199);
}

void motor_STOP(void){
  MA_STOP();
  MB_STOP();
}


float err_MA = 0;
int PIDNUM_MA = 0;//(0~4199)
void PID_MA(float V_Goat){
  float kp = 18;
  float ki = 90;
  float V_now = 0;
  float PIDNUM = 0;
  float err = 0;

  /* ????????180(????): ??????????????, ???? */
  V_Goat = -V_Goat;
  
  V_now = MA_V_GET();
  g_wheel_va = V_now;
  
  if(V_Goat < 0) err = -V_Goat + V_now;
  else err = V_Goat - V_now;
  
  float dt = 0.01f;  /* 固定10ms控制周期 */
  float P_term = kp * err;
  float I_inc = err * ki * dt;
  
  if ((P_term + err_MA > 4199 && I_inc > 0) || (P_term + err_MA < 0 && I_inc < 0)) {
  } else {
      err_MA += I_inc;
  }
  
  if(err_MA > 4000) err_MA = 4000;
  else if(err_MA < -4000) err_MA = -4000;
  
  PIDNUM = P_term + err_MA;
  PIDNUM_MA = (int)PIDNUM;
  
  if(PIDNUM_MA > 4199) PIDNUM_MA = 4199;
  else if(PIDNUM_MA < 0) PIDNUM_MA = 0;
  
  if(V_Goat < 0){
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,PIDNUM_MA);
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
  }
  else if(V_Goat > 0){
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,PIDNUM_MA);
  }
  else{
    MA_STOP();
  }
}

int M_nomal_V = 50;// -250 ~ 250; ??2?????????????
float pitch = 0;
float roll = 0;
float yaw = 0;
float pitch_Compose = 0;
float roll_Compose = 0;
extern float g_dmp_quat[4];              //DMP?(w,x,y,z), inv_mpu.c
float qref[4] = {1.0f,0.0f,0.0f,0.0f};   //???(Compose_IMU?)
//pi
//??(qref)????(): 0=??; ?, ???
float IMU_Twist(void)
{
  float qw=g_dmp_quat[0], qx=g_dmp_quat[1], qy=g_dmp_quat[2], qz=g_dmp_quat[3];
  float rw=qref[0], rx=-qref[1], ry=-qref[2], rz=-qref[3];   //qref?
  float w  = qw*rw - qx*rx - qy*ry - qz*rz;
  float vx = qw*rx + qx*rw + qy*rz - qz*ry;
  float vy = qw*ry - qx*rz + qy*rw + qz*rx;
  float vz = qw*rz + qx*ry - qy*rx + qz*rw;
  return 2.0f*atan2f(vz, sqrtf(w*w+vx*vx+vy*vy))*57.2958f;
}

float err_IMU = 0;
/* 1=当前: 上下都 pitch; 上来±20, A=B=100(末5°降到40)
   0=原来: 上来 Twist, ±60夹到[20,120], A=80 B=75; 下去仍 pitch */
#define CLIMB_USE_PITCH 1
void PID_IMU(float u_roll , int M_V_nomal)
{
  float err = 0;
  float kp = 10;
  float ki = 1.5f;
  float PIDIMU = 0;

  static float last_PIDIMU = 0;
  static uint8_t imu_fail_cnt = 0;
  static uint32_t last_imu_time = 0;

  if(mpu_dmp_get_data(&pitch, &roll, &yaw) == 0) {
      imu_fail_cnt = 0;

      uint32_t current_time = micros();
      float dt = (current_time - last_imu_time) / 1000000.0f;
      if(dt > 0.05f || dt <= 0.0f) dt = 0.01f;
      last_imu_time = current_time;

      /* 下去始终 pitch; 上来由 CLIMB_USE_PITCH 切换 */
#if CLIMB_USE_PITCH
      err = u_roll - (pitch - pitch_Compose);
#else
      if (M_V_nomal >= 0) {
        err = -IMU_Twist();
      } else {
        err = u_roll - (pitch - pitch_Compose);
      }
#endif
      err_IMU = err*ki*dt + err_IMU;

      if(err_IMU > 25){
        err_IMU = 25;
      }
      else if(err_IMU < -25){
        err_IMU = -25;
      }

      PIDIMU = kp*err + err_IMU;

      /* 行进时差速限幅; 最后5°(-15档)只允许±10 */
      float base = (M_V_nomal >= 0 ? (float)M_V_nomal : (float)(-M_V_nomal));
      float lim;
      if (base < 5.0f) {
        lim = 45.0f;
      } else if (M_V_nomal == -15) {
        lim = 10.0f;
      } else if (M_V_nomal < 0) {
        lim = 40.0f;
        if (lim > base) lim = base;
      } else {
#if CLIMB_USE_PITCH
        lim = 20.0f;
#else
        lim = 60.0f;
#endif
        if (lim > base) lim = base;
      }
      if (PIDIMU > lim) PIDIMU = lim;
      if (PIDIMU < -lim) PIDIMU = -lim;

      last_PIDIMU = PIDIMU;
  }
  else {
      imu_fail_cnt++;
      if(imu_fail_cnt > 20) {
          motor_STOP();
          return;
      }
      PIDIMU = last_PIDIMU;
  }

  /* 两轮都转起来之前只预充积分, 纠偏一直加 */
  if (!g_wheels_launched && fabsf(g_wheel_va) > 15.0f && fabsf(g_wheel_vb) > 15.0f) {
    g_wheels_launched = 1;
  }
  if (!g_wheels_launched) {
    if (err_MA < 2000.0f) err_MA = 2000.0f;
    if (err_MB < 2000.0f) err_MB = 2000.0f;
  }

  /* 下去: A=B同底速, pitch差速; 最后5°(-15档)限幅10且只减速不加。 */
  if (M_V_nomal == -15) {
    if (PIDIMU < 0.0f) {
      PID_MB(M_V_nomal - PIDIMU);
      PID_MA(-M_V_nomal);
    } else {
      float ma = -M_V_nomal - PIDIMU;
      if (ma < 0.0f) ma = 0.0f;
      PID_MB(M_V_nomal);
      PID_MA(ma);
    }
  } else if (M_V_nomal < 0) {
    PID_MB(M_V_nomal - PIDIMU);
    PID_MA(-M_V_nomal - PIDIMU);
  } else {
#if CLIMB_USE_PITCH
    /* 当前: A=B=100(末5°降到40), pitch, ±20 */
    PID_MB((float)M_V_nomal - PIDIMU);
    PID_MA(-(float)M_V_nomal - PIDIMU);
#else
    /* 原来: Twist±60, A=80, B=75, 再夹到[20,120] */
    float va = (float)M_V_nomal + PIDIMU;
    float vb = 75.0f - PIDIMU;
    if (va > 120.0f) va = 120.0f;
    if (va < 20.0f) va = 20.0f;
    if (vb > 120.0f) vb = 120.0f;
    if (vb < 20.0f) vb = 20.0f;
    PID_MB(vb);
    PID_MA(-va);
#endif
  }
}

uint8_t GO_FLAG = 0;
/* 视觉纠偏状态: 母机经NRF发 ADJ,<phi>,<psi>,<ax> (执行器见 Adj_Execute) */
static volatile uint8_t g_adj_armed = 0;   /* 本周期收到过ADJ帧=启用视觉纠偏 */
static volatile float g_adj_phi = 0.0f, g_adj_psi = 0.0f, g_adj_ax = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t FENG_PWM_SET = 940-1;
uint16_t FENG_UP = 100;//?10% 100
void FENG_Motor_SET(int8_t key){
  //k3  k4  k6一键停
  if(key == 3){
    FENG_PWM_SET = FENG_PWM_SET + FENG_UP;
    if(FENG_PWM_SET >= 1800){
       FENG_PWM_SET = 1800;
    }
  }
  if(key == 4){
    FENG_PWM_SET = FENG_PWM_SET - FENG_UP;
    if(FENG_PWM_SET <= 940-1){
       FENG_PWM_SET = 939;
    }
  }
  if(key == 6){
    FENG_PWM_SET = 939;
  }
  if(key != 0){
    __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,FENG_PWM_SET);
  }
}

uint16_t DUO_PWM_SET = 500-1;
uint16_t DUO_UP = 142;//?142 14?2.5ms
void DUO_Motor_SET(int8_t key){
  //k1  k2
  if(key == 1){
    DUO_PWM_SET = DUO_PWM_SET + DUO_UP;
    if(DUO_PWM_SET >= 2500-1){
       DUO_PWM_SET = 2500-1;
    }
  }
  if(key == 2){
    DUO_PWM_SET = DUO_PWM_SET - DUO_UP;
    if(DUO_PWM_SET <= 500-1){
       DUO_PWM_SET = 500-1;
    }
  }
  /* 只处理夹持±, key=6/3/4/9/10 不改舵机 */
  if(key == 1 || key == 2){
    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,DUO_PWM_SET);
  }
}
/* USER CODE END 0 */

int8_t Arr_RX[32] = {0};
static uint8_t Coop_PollAbort(void);
static uint8_t Adj_WaitLoop(void);
void Clear_All_PID(void);

uint8_t FENG_Motor_ON(void){
  for(int i = 0;i<6;i++){
    FENG_Motor_SET(3);
    uint32_t t0 = HAL_GetTick();
    while(HAL_GetTick() - t0 < 300u){
      if(Coop_PollAbort()) return 1;
      HAL_Delay(10);
    }
  }
  /* 出发固定油门 1539 */
  FENG_PWM_SET = 1539;
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,FENG_PWM_SET);
  return 0;
}

void FENG_Motor_OFF(void){
  for(int i = 0;i<8;i++){  /* 从1539降回939 */
    FENG_Motor_SET(4);
    HAL_Delay(50);
  }
}

uint32_t DUO_NUM = 1055; /* 与大帅周相同, 作业角度固定, 不读Flash */
void DUO_Motor_ON(uint16_t Degree){//?20000  --   20ms   500  --  0.5ms    1000 -- 1ms   2500 -1  --  2.5ms
  __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,Degree);//??? 
}

void DUO_Motor_OFF(void){
  __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,500-1);//??
}

/* ============ 子母协同协议(子机侧) ============
 * 母机→子机: COOP / GOON / LOCK / STOP
 * 子机→母机: REDY / DONE / ACKD
 * 作业期间周期上报 IMU 帧 'I'+pitch+roll
 * WaitFrame: 0=等到目标帧, 1=中止(STOP/遥控/摇杆), 中止不停风机 */
#define COOP_CMD  "COOP"
#define GOON_CMD  "GOON"
#define LOCK_CMD  "LOCK"
#define STOP_CMD  "STOP"
#define REDY_RSP  "REDY"
#define DONE_RSP  "DONE"
#define ACKD_RSP  "ACKD"
#define ADJ_CMD   "ADJ,"        /* 纠偏帧: ADJ,<phi>,<psi>,<ax> (含ADJ,0,0,0使能) */
#define ADJ_WAIT_MS  180000u    /* 纠偏等待总时限(ms), 超时视为母机失联 */
#define COOP_IMU_PERIOD_MS  200u
#define TSTA_CMD  "TSTA"   /* 母->子: 进入视觉纠偏测试 */
#define TSTB_RSP  "TSTB"   /* 子->母: 测试就绪 */
#define TSTE_CMD  "TSTE"   /* 母->子: 退出测试 */

static uint8_t g_test_mode = 0;
static uint8_t g_coop_prefetched = 0;
static volatile uint8_t g_tsta_pending = 0;   /* 外层收到 TSTA, 待 Test_WaitLoop 消费 */

/* 解析 ADJ, 帧载荷 → 存入 g_adj_* */
static void Adj_ParseStore(const uint8_t *buf){
  char tmp[33];
  char *p;
  memcpy(tmp, buf, 32); tmp[32] = '\0';
  p = tmp + 4;                                   /* 跳过"ADJ," */
  g_adj_phi = (float)atof(p);
  p = strchr(p, ',');
  if(p != NULL){
    g_adj_psi = (float)atof(p + 1);
    p = strchr(p + 1, ',');
    if(p != NULL) g_adj_ax = (float)atof(p + 1);
  }
}

static uint8_t  g_coop_report_en = 0;
static uint32_t g_imu_last_tx_us = 0;

/* 发送任意长度载荷(<=32); 有限重试, 避免对端不在时死循环 */
static uint8_t Coop_SendRaw(const uint8_t *data, uint8_t len, uint8_t retries){
  uint8_t buf[32] = {0};
  uint8_t n = 0;
  if(len > 32) len = 32;
  memcpy(buf, data, len);
  NRF24L01_TX_Mode();
  HAL_Delay(2);
  while(NRF24L01_TxPacket(buf) != TX_OK){
    if(++n >= retries){
      NRF24L01_RX_Mode();
      return 1;
    }
    HAL_Delay(2);
  }
  NRF24L01_RX_Mode();
  return 0;
}

static void Coop_SendFrame(const char *frame){
  uint8_t buf[32] = {0};
  size_t len = strlen(frame);
  if(len > 32) len = 32;
  memcpy(buf, frame, len);
  if(Coop_SendRaw(buf, 32, 20) == 0) printf("%s sent\r\n", frame);
  else                               printf("%s send fail\r\n", frame);
}

/* 轮子运动遥测帧 'J' (11 字节), 母机原样转发 App */
static void Test_Telem(uint8_t stage, int8_t dir, int16_t a_mm, int16_t b_mm,
                       int16_t target_mm, uint8_t pct, uint8_t rc){
  uint8_t f[11];
  f[0] = 'J';
  f[1] = stage;
  f[2] = (uint8_t)dir;
  memcpy(&f[3], &a_mm, 2);
  memcpy(&f[5], &b_mm, 2);
  memcpy(&f[7], &target_mm, 2);
  f[9]  = pct;
  f[10] = rc;
  (void)Coop_SendRaw(f, sizeof(f), 2);
}

static uint8_t Coop_FindFrame(const uint8_t *buf, const char *frame){
  uint8_t i;
  for(i = 0; i + 4 <= 32; i++){
    if(buf[i]   == (uint8_t)frame[0] && buf[i+1] == (uint8_t)frame[1] &&
       buf[i+2] == (uint8_t)frame[2] && buf[i+3] == (uint8_t)frame[3]){
      return 1;
    }
  }
  return 0;
}

static uint8_t Coop_IsProtoFrame(const uint8_t *buf){
  return (uint8_t)(Coop_FindFrame(buf, COOP_CMD) || Coop_FindFrame(buf, GOON_CMD) ||
                   Coop_FindFrame(buf, LOCK_CMD) || Coop_FindFrame(buf, STOP_CMD) ||
                   Coop_FindFrame(buf, REDY_RSP) || Coop_FindFrame(buf, DONE_RSP) ||
                   Coop_FindFrame(buf, ACKD_RSP) || Coop_FindFrame(buf, ADJ_CMD) ||
                   Coop_FindFrame(buf, TSTA_CMD) || Coop_FindFrame(buf, TSTE_CMD));
}

/* 1=中止并写入 Arr_RX; STOP 清摇杆, 遥控帧原样拷贝 */
static uint8_t Coop_TakeAbort(const uint8_t *buf){
  if(Coop_FindFrame(buf, STOP_CMD)){
    memset(Arr_RX, 0, 32);
    printf("STOP received\r\n");
    return 1;
  }
  if(Coop_FindFrame(buf, ADJ_CMD)){
    Adj_ParseStore(buf);        /* 纠偏帧: 记录偏差量+置armed, 不构成中止 */
    return 0;
  }
  if(Coop_IsProtoFrame(buf)){
    return 0;
  }
  if(buf[5] != 0 || buf[1] != 0 || buf[2] != 0){
    memcpy(Arr_RX, buf, 32);
    printf("RC abort\r\n");
    return 1;
  }
  return 0;
}

static uint8_t Coop_PollAbort(void){
  uint8_t buf[32];
  if(NRF24L01_RxPacket(buf) != 0) return 0;
  return Coop_TakeAbort(buf);
}

static uint8_t Coop_WaitFrame(const char *frame){
  uint8_t buf[32];
  NRF24L01_Write_Reg(FLUSH_RX, 0xff);
  while(1){
    if(NRF24L01_RxPacket(buf) == 0){
      if(Coop_FindFrame(buf, frame)){
        printf("%s received\r\n", frame);
        return 0;
      }
      if(Coop_TakeAbort(buf)){
        return 1;
      }
    }
  }
}

static void Coop_ReportIMU(void){
  uint8_t buf[32] = {0};
  if(!g_coop_report_en) return;
  if(micros() - g_imu_last_tx_us < COOP_IMU_PERIOD_MS * 1000u) return;
  g_imu_last_tx_us = micros();
  mpu_dmp_get_data(&pitch, &roll, &yaw);
  buf[0] = 'I';
  memcpy(&buf[1], &pitch, 4);
  memcpy(&buf[5], &roll, 4);
  NRF24L01_TX_Mode();
  HAL_Delay(2);
  (void)NRF24L01_TxPacket(buf);
  NRF24L01_RX_Mode();
}

static uint8_t Coop_DelayReport(uint32_t ms){
  uint32_t t0 = HAL_GetTick();
  while(HAL_GetTick() - t0 < ms){
    Coop_ReportIMU();
    if(Coop_PollAbort()) return 1;
    HAL_Delay(10);
  }
  return 0;
}

/* 舵机作业8s: A给很小向上转速顶住, B短刹 */
static uint8_t Coop_ServoAHold(uint32_t ms){
  uint32_t t0 = HAL_GetTick();
  Clear_All_PID();
  while(HAL_GetTick() - t0 < ms){
    PID_MA(-8.0f);
    MB_STOP();
    Coop_ReportIMU();
    if(Coop_PollAbort()) return 1;
    HAL_Delay(10);
  }
  motor_STOP();
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 4199);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 4199);
  return 0;
}

uint8_t GO_DOWN(float _degree){
  uint32_t loop_time = micros();
  uint32_t t_start = micros();
  while(1){
    if (micros() - loop_time >= 10000) {
      loop_time = micros();
      if(micros() - t_start > 30000000u) break;
      if((roll) >= _degree){
        float d_left = roll - _degree;
        if (d_left < 5.0f) {
          PID_IMU(0, -15);
        } else if (d_left < 10.0f) {
          PID_IMU(0, -40);
        } else {
          PID_IMU(0, -100);
        }
      }
      else{
        break;
      }
      Coop_ReportIMU();
      if(Coop_PollAbort()) return 1;
    }
  }
  return 0;
}

uint8_t GO_UP(float _degree){
  uint32_t loop_time = micros();
  uint32_t t_start = micros();
  while(1){
    if (micros() - loop_time >= 10000) {
      loop_time = micros();
      if(micros() - t_start > 30000000u) return 2; /* 超时: 返回2, 调用方不再上报DONE */
      if((roll) <= _degree ){
        float d_left = _degree - roll;
        if (d_left < 5.0f) {
          PID_IMU(0, 40);   /* 最后5°减速, 与下行对称, 防过冲急刹 */
        } else {
          PID_IMU(0, 100);
        }
      }
      else{
        break;
      }
      Coop_ReportIMU();
      if(Coop_PollAbort()) return 1;

#if 0
      /* 旧逻辑保留: roll 回位后再原地纠扭转角 */
      float tw = IMU_Twist();
      if (tw < 0.0f) tw = -tw;
      if (tw > 2.0f){
        if (t_hold == 0) t_hold = micros();
        if (micros() - t_hold > 4000000u) break;
        PID_IMU(0, 0);
      }
      else{
        break;
      }
#endif
    }
  }
  return 0;
}


void Clear_All_PID(void) {
  err_MA = 0;
  PIDNUM_MA = 0;
  err_MB = 0;
  PIDNUM_MB = 0;
  err_IMU = 0;
  g_wheel_va = 0;
  g_wheel_vb = 0;
  g_wheels_launched = 0;
  MA_COUNTER_old = __HAL_TIM_GET_COUNTER(&htim2);
  MB_COUNTER_old = __HAL_TIM_GET_COUNTER(&htim3);
}

uint8_t Compose_IMU(void){
  float temp_pitch = 0;
  float temp_roll = 0;
  int valid_cnt = 0;
  
  pitch_Compose = 0;
  roll_Compose = 0;

  uint32_t start_time = micros();
  while(valid_cnt < 100 && (micros() - start_time < 1500000)) {
      if(Coop_PollAbort()) return 1;
      if(mpu_dmp_get_data(&pitch, &roll, &yaw) == 0) {
          temp_pitch += pitch;
          temp_roll += roll;
          valid_cnt++;
      }
  }
  
  qref[0]=g_dmp_quat[0]; qref[1]=g_dmp_quat[1]; qref[2]=g_dmp_quat[2]; qref[3]=g_dmp_quat[3];
  if(valid_cnt > 0) {
      pitch_Compose = temp_pitch / valid_cnt;
      roll_Compose = temp_roll / valid_cnt;
  } else {
      pitch_Compose = pitch;
      roll_Compose = roll;
  }
  return 0;
}



uint8_t TX_BUFF[32] = "DONE";
void AUTO(void){
  /* 正常结束: 关风机 ACKD 再等COOP
     中止: 刹轮、保持当前吸力, return 回手动 */
  while (1) {
    motor_STOP();
    DUO_PWM_SET = 500-1;
    DUO_Motor_OFF();
    g_coop_report_en = 0;
    g_adj_armed = 0;   /* 新周期: 重置纠偏武装, 母机发ADJ帧再使能 */
    if(g_coop_prefetched){
      g_coop_prefetched = 0;
      printf("AUTO start (COOP prefetched)\r\n");
    } else {
      printf("AUTO ready, wait COOP...\r\n");
      if(Coop_WaitFrame(COOP_CMD)) return;
    }

    g_coop_report_en = 1;
    g_imu_last_tx_us = 0;
    if(FENG_Motor_ON()) { g_coop_report_en = 0; motor_STOP(); return; }
    /* 1539 稳住 1s 再回 REDY */
    if(Coop_DelayReport(1000)) { g_coop_report_en = 0; motor_STOP(); return; }
    Coop_SendFrame(REDY_RSP);
    if(Coop_WaitFrame(GOON_CMD)) { g_coop_report_en = 0; motor_STOP(); return; }

    if(Compose_IMU()) { g_coop_report_en = 0; motor_STOP(); return; }
    Clear_All_PID();
    if(GO_DOWN(-90)) { g_coop_report_en = 0; motor_STOP(); return; }
    motor_STOP();
    /* 底部静止: A轮CH1/CH2都4199刹车, B轮仍刹车 */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 4199);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 4199);

    if(Coop_DelayReport(200)) { g_coop_report_en = 0; motor_STOP(); return; }
    DUO_Motor_ON(DUO_NUM);
    if(Coop_ServoAHold(8000)) { g_coop_report_en = 0; motor_STOP(); return; }
    DUO_Motor_OFF();
    /* 舵机作业完成后再加吸到1639 */
    FENG_PWM_SET = 1639;
    __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,FENG_PWM_SET);
    if(Coop_DelayReport(500)) { g_coop_report_en = 0; motor_STOP(); return; }

    Clear_All_PID();
    /* GO_UP 返回: 0=到位 1=中止 2=超时。超时/中止都不上报DONE, 保持吸力回手动 */
    if(GO_UP(roll_Compose)) { g_coop_report_en = 0; motor_STOP(); return; }
    motor_STOP();

    /* 视觉纠偏等待: 本周期收过ADJ帧才启用; 先发REDY告知母机"GO_UP结束,可开始视觉纠偏";
 * 母机发GOON(对齐)才发DONE */
    if(g_adj_armed){
      Coop_SendFrame(REDY_RSP);   /* REDY②: GO_UP结束, 通知母机启动视觉纠偏 */
      if(Adj_WaitLoop()){ g_coop_report_en = 0; motor_STOP(); return; }  /* 中止/超时: 不发DONE */
    }

    g_coop_report_en = 0;
    Coop_SendFrame(DONE_RSP);
    if(Coop_WaitFrame(LOCK_CMD)) { motor_STOP(); return; }

    FENG_Motor_OFF();
    Clear_All_PID();
    Coop_SendFrame(ACKD_RSP);
  }
}
/**
  * @brief  The application entry point.
  * @retval int
  */
  

uint8_t DDR_SED[32] = {0};
/* MODE: 9=手操 10=AUTO 11=作业模式(仅风机+舵机, 轮子锁定); key=7 进入, key=9 退出 */
int8_t MODE = 9;//9 ??? 10 ???
//???? ?? ??????? ?? ????
uint32_t Flash_Store[2] = {15,78};
uint32_t Flash_ReadNUM[2] = {0};

/* ================= 视觉纠偏运动规划(2026-10-18) =================
 * 输入: 视觉端测得的三个偏差量  Adj_Execute(phi_deg, psi_deg, ax_mm)
 *       (串口转发未接前, 台架用 作业模式+key8 自检触发一次演示纠偏)
 * 序列: 轴向斜行修正(最扰动,先走) → SPIN 摆正 ψ → MV 走 φ 弧长
 * 每个子动作: 轮速PID慢速 + 编码器里程计计数到量即停 + 超时 + 每10ms轮询中止
 * 返回: 0=完成 1=被中止(STOP/遥控) 2=超时/打滑 */
#define ADJ_PI              3.14159265f
#define ADJ_SPEED_REV       3.0f     /* 纠偏行驶速度(电机轴转/s), 地面测试放慢便于观察 */
#define ADJ_MM_PER_MOTORREV 15.0f    /* ⚠️占位待标定: 里程计每"电机转"对应轮面行走毫米
                                     标定: 跑一段已知指令, 卷尺量实际距离, 按比例修此值 */
#define ADJ_TRACK_MM        100.0f   /* ⚠️占位待标定: 两驱动轮接触点间距(mm) */
#define ADJ_PIPE_R_MM       60.0f    /* ⚠️占位待标定: 管道外半径(mm), φ→弧长换算 */
#define ADJ_DIAG_BETA_DEG   30.0f    /* 轴向斜行定角(deg), 最大自旋=2×β */
#define ADJ_MV_SIGN         (+1)     /* 标定: φ>0 时应往哪边走(+1=下降方向) */
#define ADJ_SPIN_SIGN       (+1)     /* 标定: ψ>0 时应往哪边转(+1=S+方向) */
#define ADJ_AXIAL_SIGN      (+1)     /* 标定: S+歪头后前进, 轴向分量朝哪边(+1=+ax侧) */
#define ADJ_TOL_PHI         0.5f     /* 死区(deg) */
#define ADJ_TOL_PSI         0.5f
#define ADJ_TOL_AX          2.0f     /* 死区(mm), 以内交给夹爪容差 */
#define ADJ_MAX_PHI         10.0f    /* 单次限幅(deg), 超限钳位, 视觉下一轮再补 */
#define ADJ_MAX_PSI         10.0f
#define ADJ_MAX_AX          30.0f    /* 单次限幅(mm) */
#define ADJ_SETTLE_MS       150      /* 段间刹车稳定等待 */
#define ADJ_SLIP_DIFF_MM    8.0f     /* 直行两轮里程差超此值判打滑 */
#define ADJ_SELF_TEST       1        /* 1=作业模式下 key8 触发一次演示纠偏 */
#define ADJ_ST_AXIAL        1
#define ADJ_ST_SPIN         2
#define ADJ_ST_MOVE         3
#define ADJ_ST_SETTLE       4
#define ADJ_TEL_PERIOD_MS   500u     /* 运动遥测周期(ms) */
#define ADJ_SPEED_REV_SLOW  3.0f     /* 测试用慢速(电机转/s), 便于肉眼观察 */
#define ADJ_TEST_PHI        5.0f     /* 自检演示值(配合占位常数观察动作方向) */
#define ADJ_TEST_PSI        (-3.0f)
#define ADJ_TEST_AX         10.0f

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
  o->a_mm += (float)da / 7.0f * ADJ_MM_PER_MOTORREV;  /* 7=每转计数,与MA_V_GET同约定 */
  o->b_mm += (float)db / 7.0f * ADJ_MM_PER_MOTORREV;
}
static uint8_t Adj_Settle(void){
  uint32_t t0 = HAL_GetTick();
  motor_STOP();
  while(HAL_GetTick() - t0 < ADJ_SETTLE_MS){
    if(Coop_PollAbort()) return 1;
    HAL_Delay(10);
  }
  return 0;
}
/* 直行 dist_mm: >0=下降方向(A CH1/B CH4) <0=上升方向(A CH2/B CH3)
 * 返回 0=完成 1=中止 2=超时/打滑 */
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
          Test_Telem(ADJ_ST_MOVE, dir, (int16_t)od.a_mm, (int16_t)od.b_mm,
                     (int16_t)target, pct, 0);
        }
        if(done >= target) break;
      }
      if(fabsf(fabsf(od.a_mm) - fabsf(od.b_mm)) > ADJ_SLIP_DIFF_MM){
        motor_STOP();
        Test_Telem(ADJ_ST_MOVE, dir, (int16_t)od.a_mm, (int16_t)od.b_mm, (int16_t)target, 100, 3);
        printf("ADJ move slip A=%d B=%d\r\n", (int)od.a_mm, (int)od.b_mm); return 2;
      }
      if(HAL_GetTick() - t0 > t_to){
        motor_STOP();
        Test_Telem(ADJ_ST_MOVE, dir, (int16_t)od.a_mm, (int16_t)od.b_mm, (int16_t)target, 100, 3);
        printf("ADJ move timeout\r\n"); return 2;
      }
      if(Coop_PollAbort()){
        motor_STOP();
        Test_Telem(ADJ_ST_MOVE, dir, (int16_t)od.a_mm, (int16_t)od.b_mm, (int16_t)target, 100, 2);
        return 1;
      }
    }
    Test_Telem(ADJ_ST_MOVE, dir, (int16_t)od.a_mm, (int16_t)od.b_mm, (int16_t)target, 100, 1);
  }
  motor_STOP();
  printf("ADJ move %dmm ok (A=%d B=%d)\r\n", (int)target, (int)od.a_mm, (int)od.b_mm);
  return 0;
}
/* 原地旋转 theta_deg: >0=S+方向(A上升/B下降), <0=S-; 返回同上 */
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
      if(theta_deg * (float)ADJ_SPIN_SIGN > 0){ PID_MA(-v); PID_MB(-v); }  /* S+: A上升 B下降 */
      else                                     { PID_MA(+v); PID_MB(+v); } /* S-: A下降 B上升 */
      HAL_Delay(10);
      Adj_OdomUpdate(&od);
      {
        float done = (fabsf(od.a_mm) + fabsf(od.b_mm)) * 0.5f;
        if(HAL_GetTick() - t_tel >= ADJ_TEL_PERIOD_MS){
          uint8_t pct = (arc > 0.01f) ? (uint8_t)(done * 100.0f / arc) : 0;
          if(pct > 100) pct = 100;
          t_tel = HAL_GetTick();
          Test_Telem(ADJ_ST_SPIN, dir, (int16_t)od.a_mm, (int16_t)od.b_mm,
                     (int16_t)arc, pct, 0);
        }
        if(done >= arc) break;
      }
      if(HAL_GetTick() - t0 > t_to){
        motor_STOP();
        Test_Telem(ADJ_ST_SPIN, dir, (int16_t)od.a_mm, (int16_t)od.b_mm, (int16_t)arc, 100, 3);
        printf("ADJ spin timeout\r\n"); return 2;
      }
      if(Coop_PollAbort()){
        motor_STOP();
        Test_Telem(ADJ_ST_SPIN, dir, (int16_t)od.a_mm, (int16_t)od.b_mm, (int16_t)arc, 100, 2);
        return 1;
      }
    }
    Test_Telem(ADJ_ST_SPIN, dir, (int16_t)od.a_mm, (int16_t)od.b_mm, (int16_t)arc, 100, 1);
  }
  motor_STOP();
  printf("ADJ spin %ddeg ok\r\n", (int)theta_deg);
  return 0;
}
/* 轴向斜行修正: 转+β斜下去(进L) → 转−β摆正 → 退回(直行d)
 * 几何: L=|ax|/sinβ, d=|ax|/tanβ → 净位移=纯轴向ax, 朝向回正, 周向归零 */
static uint8_t Adj_Axial(float ax_mm){
  float beta = ADJ_DIAG_BETA_DEG * ADJ_PI / 180.0f;
  float L = fabsf(ax_mm) / sinf(beta);
  float d = fabsf(ax_mm) / tanf(beta);
  float tilt = (ax_mm > 0 ? 1.0f : -1.0f) * (float)ADJ_AXIAL_SIGN;
  uint8_t rc;
  printf("ADJ axial L=%dmm d=%dmm\r\n", (int)L, (int)d);
  rc = Adj_Spin( tilt * ADJ_DIAG_BETA_DEG);  if(rc) return rc;
  if(Adj_Settle()) return 1;
  rc = Adj_Move( +L );                       if(rc) return rc;   /* 斜着下去 */
  if(Adj_Settle()) return 1;
  rc = Adj_Spin( -tilt * ADJ_DIAG_BETA_DEG); if(rc) return rc;   /* 摆正 */
  if(Adj_Settle()) return 1;
  rc = Adj_Move( -d );                       if(rc) return rc;   /* 退回 */
  return 0;
}
/* 规划器入口: 三个偏差量 → 全部纠偏运动; 返回 0完成 1中止 2出错 */
uint8_t Adj_Execute(float phi_deg, float psi_deg, float ax_mm){
  float phi = Adj_Clamp(phi_deg, ADJ_MAX_PHI);
  float psi = Adj_Clamp(psi_deg, ADJ_MAX_PSI);
  float ax  = Adj_Clamp(ax_mm,  ADJ_MAX_AX);
  uint8_t rc = 0;
  Clear_All_PID();
  if(fabsf(phi) < ADJ_TOL_PHI) phi = 0;
  if(fabsf(psi) < ADJ_TOL_PSI) psi = 0;
  if(fabsf(ax)  < ADJ_TOL_AX)  ax  = 0;
  printf("ADJ start phi=%d psi=%d ax=%d\r\n", (int)phi, (int)psi, (int)ax);
  if(ax != 0){ rc = Adj_Axial(ax); if(rc) goto out; }              /* 最扰动先走 */
  if(Adj_Settle()){ rc = 1; goto out; }
  if(psi != 0){ rc = Adj_Spin(-psi * (float)ADJ_SPIN_SIGN); if(rc) goto out; }
  if(Adj_Settle()){ rc = 1; goto out; }
  if(phi != 0){ rc = Adj_Move(phi * ADJ_PI / 180.0f * ADJ_PIPE_R_MM * (float)ADJ_MV_SIGN); if(rc) goto out; }
out:
  motor_STOP();
  printf("ADJ finish rc=%u\r\n", rc);
  return rc;
}
/* 纠偏等待循环(GO_UP到位且已武装时进入):
 * 收 ADJ,φ,ψ,ax → Adj_Execute → 回REDY(可复测)
 * 收 GOON → 对齐确认, 返回0(调用方发DONE)
 * 收 STOP/遥控 → 返回1(中止); 180s无帧 → 返回2
 * 期间 IMU 遥测保持开启 */
static uint8_t Adj_WaitLoop(void){
  uint32_t t0 = HAL_GetTick();
  printf("ADJ wait: hold for vision\r\n");
  while(1){
    uint8_t buf[32];
    Coop_ReportIMU();
    if(NRF24L01_RxPacket(buf) == 0){
      if(Coop_FindFrame(buf, GOON_CMD)){
        printf("ADJ aligned\r\n");
        return 0;                                   /* 对齐: 调用方发DONE */
      }
      if(Coop_FindFrame(buf, ADJ_CMD)){
        uint8_t rc;
        Adj_ParseStore(buf);
        printf("ADJ exec phi=%d psi=%d ax=%d\r\n",
               (int)g_adj_phi, (int)g_adj_psi, (int)g_adj_ax);
        rc = Adj_Execute(g_adj_phi, g_adj_psi, g_adj_ax);
        if(rc == 1) return 1;                       /* 中止 */
        Coop_SendFrame(REDY_RSP);                   /* 步完成, 可复测 */
      }
      else if(Coop_TakeAbort(buf)){
        return 1;                                   /* STOP/遥控接管 */
      }
    }
    if(HAL_GetTick() - t0 > ADJ_WAIT_MS){
      printf("ADJ wait timeout\r\n");
      return 2;
    }
  }
}
/* ============ 视觉纠偏测试模式 (地面调试, 不开风机) ============
 * 母->子: TSTA 进入 / ADJ,<phi>,<psi>,<ax> 下发假偏差 / TSTE 退出 / STOP 中止
 * 子->母: TSTB 就绪 / DONE 本轮执行完毕 / 'J' 轮子运动遥测
 * 返回: 0=未进入(主循环回手动) 1=测试已退出
 * 正常 COOP 帧被预取, 交给 AUTO() 继续 */
static uint8_t Test_WaitLoop(void){
  uint8_t buf[32];
  if(!g_tsta_pending){
  printf("TEST wait: TSTA for test, COOP for AUTO\r\n");
  NRF24L01_Write_Reg(FLUSH_RX, 0xff);
  while(1){
    if(NRF24L01_RxPacket(buf) == 0){
      if(Coop_FindFrame(buf, TSTA_CMD)){
        printf("TSTA received, enter test\r\n");
        break;
      }
      if(Coop_FindFrame(buf, COOP_CMD)){
        printf("COOP received, run AUTO\r\n");
        g_coop_prefetched = 1;
        return 0;
      }
      if(Coop_TakeAbort(buf)){
        printf("TEST abort before start\r\n");
        return 0;
      }
    }
  }
  }

  g_tsta_pending = 0;
  g_test_mode = 1;
  Coop_SendFrame(TSTB_RSP);
  NRF24L01_Write_Reg(FLUSH_RX, 0xff);

  while(1){
    if(NRF24L01_RxPacket(buf) == 0){
      if(Coop_FindFrame(buf, TSTE_CMD)){
        printf("TSTE received, exit test\r\n");
        g_test_mode = 0;
        motor_STOP();
        return 1;
      }
      if(Coop_FindFrame(buf, ADJ_CMD)){
        uint8_t rc;
        Adj_ParseStore(buf);
        printf("TEST ADJ phi=%d psi=%d ax=%d\r\n",
               (int)g_adj_phi, (int)g_adj_psi, (int)g_adj_ax);
        rc = Adj_Execute(g_adj_phi, g_adj_psi, g_adj_ax);
        if(rc == 1){
          printf("TEST exec aborted\r\n");
          g_test_mode = 0;
          return 1;
        }
        Coop_SendFrame(DONE_RSP);
        NRF24L01_Write_Reg(FLUSH_RX, 0xff);
        continue;
      }
      if(Coop_TakeAbort(buf)){
        printf("TEST abort\r\n");
        g_test_mode = 0;
        motor_STOP();
        return 1;
      }
    }
  }
}

static void ApplyRemoteKey(int8_t key){
  if(key == 0) return;
  if(MODE == 11){
    /* 作业模式: 风机(3增/4减/6停); 舵机1=一点一点下压(+142/按) 2=一点一点抬升(-142/按);
       key9退出; key8=纠偏自检(ADJ_SELF_TEST=1时, 用占位常数跑一次演示纠偏) */
    if(key == 9){
      MODE = 9;                 /* key=9 退出作业模式, 回手操 */
      Clear_All_PID();
      printf("WORK MODE off\r\n");
    } else if(key == 8 && ADJ_SELF_TEST){
      printf("ADJ self test: phi=%d psi=%d ax=%d\r\n",
             (int)ADJ_TEST_PHI, (int)ADJ_TEST_PSI, (int)ADJ_TEST_AX);
      Adj_Execute(ADJ_TEST_PHI, ADJ_TEST_PSI, ADJ_TEST_AX);
    } else if(key == 1){
      /* 下压: 一点一点往下(每按+142, 第一版行为), 不直接跳作业角 */
      DUO_Motor_SET(1);
      printf("SERVO down %d\r\n", (int)DUO_PWM_SET);
    } else if(key == 2){
      /* 抬升: 一点一点往上(每按-142), 下限499(舵机规格) */
      DUO_Motor_SET(2);
      printf("SERVO up %d\r\n", (int)DUO_PWM_SET);
    } else if(key == 3 || key == 4 || key == 6){
      FENG_Motor_SET(key);
    }
    return;
  }
  if(key == 9){
    MODE = 9;
  } else if(key == 10){
    MODE = 10;
  } else if(key == 7){
    /* key=7: 进入作业模式(只控风机+舵机), 轮子锁定不动 */
    MODE = 11;
    motor_STOP();
    Clear_All_PID();
    printf("WORK MODE on\r\n");
  } else if(key == 5){
    Flash_Store[0] = DUO_PWM_SET;
    DUO_NUM = DUO_PWM_SET;
    Flash_Write(Flash_Store,1);
    Flash_Read(Flash_ReadNUM,1);
    printf("Flash TEST : %d , %d \r\n",Flash_ReadNUM[0],Flash_ReadNUM[1]);
  }
  FENG_Motor_SET(key);
  DUO_Motor_SET(key);
}

/* ============ 树莓派串口指令链路(USART2: PA2-TX/PA3-RX, 115200, 接树莓派GPIO串口) ============
 * 协议(ASCII行, \n结尾), 每条回复 "OK <cmd>\n" 或 "ERR\n":
 *   W  进入作业模式(MODE=11, 轮子锁定)     X  退出作业模式回手操
 *   F+ 风机+100    F- 风机-100    F0 风机停(939)
 *   S+ 舵机+142    S- 舵机-142
 *   F=nnn  风机PWM直接设定(939~1800)
 *   S=nnnn 舵机PWM直接设定(499~2499)
 * 指令在主循环处理; AUTO运行期间只缓存最新一条, AUTO结束后执行 */
#define PI_LINK_EN          0        /* 0=停用子机Pi串口通信(纠偏走NRF); 1=恢复网页手动控制 */
#define PI_RX_LINE_MAX 24
static uint8_t pi_rx_byte = 0;
static volatile char    pi_rx_line[PI_RX_LINE_MAX];
static volatile uint8_t pi_rx_idx = 0;
static volatile uint8_t pi_rx_ready = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART2){
    uint8_t c = pi_rx_byte;
    if(pi_rx_ready){
      pi_rx_idx = 0;                      /* 上一条还没被主循环取走, 丢弃新行 */
    } else if(c == '\n' || c == '\r'){
      if(pi_rx_idx > 0){
        pi_rx_line[pi_rx_idx] = '\0';
        pi_rx_ready = 1;
      }
      pi_rx_idx = 0;
    } else if(pi_rx_idx < PI_RX_LINE_MAX - 1){
      pi_rx_line[pi_rx_idx++] = (char)c;
    } else {
      pi_rx_idx = 0;                      /* 行超长, 丢弃重新同步 */
    }
    HAL_UART_Receive_IT(&huart2, &pi_rx_byte, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART2){
    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UART_Receive_IT(&huart2, &pi_rx_byte, 1);   /* 出错后重新挂接收 */
  }
}

static void Pi_Reply(const char *s){
  HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), 20);
}

/* 主循环调用: 取出并执行一条树莓派指令 */
static void Pi_CMD_Process(void){
  char cmd[PI_RX_LINE_MAX];
  uint8_t i;
  int v;
  if(!pi_rx_ready) return;
  /* ready=1期间ISR不会写缓冲, 可直接拷贝后再清标志 */
  for(i = 0; i < PI_RX_LINE_MAX; i++) cmd[i] = pi_rx_line[i];
  pi_rx_ready = 0;

  if     (strcmp(cmd, "W")  == 0){ ApplyRemoteKey(7); Pi_Reply("OK W\n");  }
  else if(strcmp(cmd, "X")  == 0){ ApplyRemoteKey(9); Pi_Reply("OK X\n");  }
  else if(strcmp(cmd, "F+") == 0){ ApplyRemoteKey(3); Pi_Reply("OK F+\n"); }
  else if(strcmp(cmd, "F-") == 0){ ApplyRemoteKey(4); Pi_Reply("OK F-\n"); }
  else if(strcmp(cmd, "F0") == 0){ ApplyRemoteKey(6); Pi_Reply("OK F0\n"); }
  else if(strcmp(cmd, "S+") == 0){ ApplyRemoteKey(1); Pi_Reply("OK S+\n"); }
  else if(strcmp(cmd, "S-") == 0){ ApplyRemoteKey(2); Pi_Reply("OK S-\n"); }
  else if(cmd[0]=='F' && cmd[1]=='=' && cmd[2] != 0){
    v = atoi(cmd + 2);
    if(v >= 939 && v <= 1800){
      FENG_PWM_SET = (uint16_t)v;
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, FENG_PWM_SET);
      Pi_Reply("OK F=\n");
    } else { Pi_Reply("ERR\n"); }
  }
  else if(cmd[0]=='S' && cmd[1]=='=' && cmd[2] != 0){
    v = atoi(cmd + 2);
    if(v >= 499 && v <= 2499){
      DUO_PWM_SET = (uint16_t)v;
      DUO_NUM = DUO_PWM_SET;
      DUO_Motor_ON(DUO_PWM_SET);
      Pi_Reply("OK S=\n");
    } else { Pi_Reply("ERR\n"); }
  }
  else { Pi_Reply("ERR\n"); }
}

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  if(PI_LINK_EN) HAL_UART_Receive_IT(&huart2, &pi_rx_byte, 1);   /* 树莓派指令链路: 挂接收中断 */
  MX_USART3_UART_Init();
  
  
  MX_TIM2_Init();// MA AB? A -- TIM2CH1 PA15 ;; B -- TIM2CH2 PB3
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);//7???
  
  
  MX_TIM3_Init();// MB AB? A -- TIM3CH1 PB4 ;; B -- TIM3CH2 PB5
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  
  
  MX_TIM4_Init();// PWMPB6-9 CH1-4 MA(PB6 - IN1 PB7 - IN2)  MB(PB8 - IN3 PB9 - IN4)
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);//
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);//
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);//
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);//
  
  
  MX_TIM5_Init();//50HZ   PA0 PA1 ch1 ch2
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);//
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);//
  
  MX_TIM8_Init();//50HZ    PC6 PC7 ch1 ch2
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);//
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);// 
  printf("\r\n");
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);//
  while(MPU_Init()!=0)
  {
    printf("MPU ??\r\n");
    //break;
    
  }
  while(mpu_dmp_init()!=0)
  {
    printf("MPU_DMP ??\r\n");
    //break;
  }
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);//
  
  
  NRF24L01_Init();
  {
    uint32_t nrf_t0 = HAL_GetTick();          //???: ??1s, ?
    while(NRF24L01_Check() != 0){
      if(HAL_GetTick() - nrf_t0 > 1000u) break;
    }
  }
  NRF24L01_RX_Mode();
  while(HAL_GetTick() < 3000u);               //?3s?
  MODE = 10;
  
  printf("?\r\n");
  //???V_GET
  //????K9 K10
  
  //Flash_Write(Flash_Store,2);
  //Flash_Read(Flash_ReadNUM,2);//????? ??????
  //printf("Flash TEST : %d , %d \r\n",Flash_ReadNUM[0],Flash_ReadNUM[1]);
  uint32_t last_ctrl_time = micros();

  while (1)
  {
    if(PI_LINK_EN) Pi_CMD_Process();   /* 处理树莓派串口指令(PI_LINK_EN=0时停用) */

    /* AUTO/测试模式: 接收由 Test_WaitLoop/AUTO 内部独占, 外层不得抢 RX FIFO */
    if(MODE == 10){
      if(Test_WaitLoop() == 0){
        AUTO();
      }
      MODE = 9;
      g_coop_report_en = 0;
      motor_STOP();
      if(Arr_RX[5] != 0){
        ApplyRemoteKey(Arr_RX[5]);
        Arr_RX[5] = 0;
      }
      continue;
    }

    if(NRF24L01_RxPacket((uint8_t *)Arr_RX)==0){
        /* 手动模式下收到 TSTA: 请求进入测试, 帧内容不在此消费 */
        if(Coop_FindFrame((const uint8_t *)Arr_RX, TSTA_CMD)){
          g_tsta_pending = 1;
          MODE = 10;
          memset(Arr_RX, 0, 32);
          continue;
        }
        if(Arr_RX[5] != 0){
          ApplyRemoteKey(Arr_RX[5]);
          Arr_RX[5] = 0;
        }
    }

    if (micros() - last_ctrl_time >= 10000) {
        last_ctrl_time = micros();

        if(MODE == 9){
            /* 前端: 上/下=前后, 左/右=转向。空中帧相对车体顺时针转了90°, 这里转回来:
               原 [1]=前后 [2]=转向 → 实际前端上写入了[2]、左写入了[1] */
            float fwd  = -(float)Arr_RX[2];
            float yawc =  (float)Arr_RX[1];
            PID_MA(-yawc*2.0f + fwd);
            PID_MB( yawc*2.0f + fwd);
        }
    }

  }

  
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

