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
#include <stdio.h>              /* snprintf: 组无线日志帧 */
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
/* 视觉纠偏: 母机经NRF发 ADJ,<phi>,<psi>,<ax> (执行器见 Adj_Execute);
 * 协同流程 REDY② 后无条件进入纠偏等待, 不再需要 armed 门控 */
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
static uint8_t Test_WaitLoop(void);
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
#define ADJ_CMD   "ADJ,"        /* 纠偏帧: ADJ,<phi>,<psi>,<ax> */
#define ADJ_WAIT_MS  60000u     /* 空闲超时(ms): 整段纠偏≤50s, 60s 足够; 有 ADJ/GOON 刷新 */
#define ADJ_REDY_RESEND_MS  1200u /* 等待指令时补发 REDY; 略疏, 少占 TX 以免漏收 GOON */
#define DONE_RESEND_MS      800u  /* 等 LOCK 时周期补发 DONE */
#define COOP_IMU_PERIOD_MS  200u
static uint8_t g_coop_prefetched = 0;   /* 主循环手动态预取到 COOP, 交给 AUTO() 消费 */
#define TSTA_CMD  "TSTA"   /* mother->child: enter vision-adj test */
#define TSTE_CMD  "TSTE"   /* mother->child: exit test */
static volatile uint8_t g_tsta_pending = 0;   /* outer TSTA/key8, consume in Test_WaitLoop */

/* AE 距离结果(Pi 串口→NRF); 变量提前声明供 AUTO 使用 */
static uint8_t  g_ae_pending = 0;
static uint8_t  g_ae_sent_ok = 0;
static int16_t  g_ae_cm_x10 = 0x7FFF;
static uint8_t  g_ae_near = 0xFF;
static uint8_t  g_ae_pd = 0xFF;
static uint8_t  g_ae_st = 0xFF;
static uint32_t g_ae_last_tx_ms = 0;

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

/* ---- NRF safe TX/RX: 有限重试, 避免 IRQ/假阳性导致死循环 ---- */
volatile uint8_t g_last_tx_status = 0xff;
volatile uint8_t g_last_tx_fifo = 0xff;
volatile uint8_t g_last_tx_result = 0xff;
volatile uint8_t g_last_tx_attempts = 0;

/* 等 TX_DS/MAX_RT 本身, 不能把 RX_DR 拉低的 IRQ 误当成发送结束。 */
static uint8_t safe_TxPacket(uint8_t *txbuf)
{
  uint8_t sta;
  /* CE 先拉低再清标志: 主固件里 IMU/遥测等路径会留下 RX_DR,
     若不先清, IRQ(PA6) 已被拉低, 下面等 IRQ 的循环会立即退出,
     读到的 STATUS 是旧的 RX_DR, 发送根本没启动(实测 STATUS=0x40) */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, 0x70);
  NRF24L01_Write_Reg(FLUSH_TX, 0xFF);
  NRF24L01_Write_Reg(FLUSH_RX, 0xFF);

  NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, 32);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  uint32_t t0 = HAL_GetTick();
  do {
    sta = NRF24L01_Read_Reg(STATUS);
    if (sta == 0xff || (sta & (TX_OK | MAX_TX))) break;
  } while (HAL_GetTick() - t0 < 20u);

  g_last_tx_status = sta;
  g_last_tx_fifo = NRF24L01_Read_Reg(NRF_FIFO_STATUS);
  NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, sta);
  if (sta == 0xff) { NRF24L01_Write_Reg(FLUSH_TX, 0xff); return 0xff; }
  if (sta & MAX_TX) { NRF24L01_Write_Reg(FLUSH_TX, 0xFF); return MAX_TX; }
  if (sta & TX_OK)  { return TX_OK; }
  NRF24L01_Write_Reg(FLUSH_TX, 0xff); /* 超时重试前移除未决 payload */
  return 0xFF;
}

/* 可靠收包: IRQ 引脚判据 + FIFO_STATUS 判空, 避免 SPI 空读假阳性 */
static uint8_t safe_RxPacket(uint8_t *rxbuf)
{
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) != 0) return 1;
  uint8_t sta = NRF24L01_Read_Reg(STATUS);
  NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, sta);
  if ((sta & RX_OK) == 0) return 1;
  uint8_t fifo = NRF24L01_Read_Reg(NRF_FIFO_STATUS);
  if (fifo & 0x01) { NRF24L01_Write_Reg(FLUSH_RX, 0xFF); return 1; }
  NRF24L01_Read_Buf(RD_RX_PLOAD, rxbuf, 32);
  NRF24L01_Write_Reg(FLUSH_RX, 0xFF);
  return 0;
}

/* 发送任意长度载荷(<=32); 有限重试, 避免对端不在时死循环 */
static uint8_t Coop_SendRaw(const uint8_t *data, uint8_t len, uint8_t retries){
  uint8_t buf[32] = {0};
  uint8_t n = 0;
  if(len > 32) len = 32;
  memcpy(buf, data, len);
  NRF24L01_TX_Mode();
  HAL_Delay(2);
  while(safe_TxPacket(buf) != TX_OK){
    if(++n >= retries){
      g_last_tx_attempts = n;
      g_last_tx_result = 1;
      NRF24L01_RX_Mode();
      return 1;
    }
    HAL_Delay(2);
  }
  g_last_tx_attempts = n + 1;
  g_last_tx_result = 0;
  NRF24L01_RX_Mode();
  return 0;
}

static void Coop_SendFrame(const char *frame){
  uint8_t buf[32] = {0};
  size_t len = strlen(frame);
  if(len > 32) len = 32;
  memcpy(buf, frame, len);
  /* 8 次重试: 每次 >=2ms 间隔, 至少覆盖母机 2 个 10ms 采样周期;
     重试过多会让子机长时间停在 TX, 反而丢母机下行帧 */
  if(Coop_SendRaw(buf, 32, 8) == 0) printf("%s sent\r\n", frame);
  else                               printf("%s send fail\r\n", frame);
}

/* 保活补发用: 少重试, 尽快回 RX, 降低漏收 GOON/ADJ 概率 */
static void Coop_SendFrameLight(const char *frame){
  uint8_t buf[32] = {0};
  size_t len = strlen(frame);
  if(len > 32) len = 32;
  memcpy(buf, frame, len);
  (void)Coop_SendRaw(buf, 32, 2);
}

/* 轮子运动遥测帧 'J' (11 字节), 母机原样转发 App */
/* 子机日志帧 'L': [0]='L' [1]=长度 [2..] 文本.
 * 子机 printf 无 _write 重定向(nosys.specs), 日志只能经 NRF 送回母机打印到串口.
 * 只在段边界/遥测周期调用, 不进 10ms 控制环 */
static void Adj_Log(const char *msg){
  uint8_t f[32];
  size_t n = strlen(msg);
  if(n > 30) n = 30;
  f[0] = 'L';
  f[1] = (uint8_t)n;
  memcpy(&f[2], msg, n);
  (void)Coop_SendRaw(f, (uint8_t)(n + 2), 2);
}

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
  if(Coop_IsProtoFrame(buf)){
    return 0;                   /* 协议帧(含ADJ)不构成中止, 各等待循环自己消费 */
  }
  /* Align/Test: fan KEY 3+/4-/6 stop with axes 0 — adjust PWM, do not abort ADJ */
  if(buf[1] == 0 && buf[2] == 0 &&
     (buf[5] == 3 || buf[5] == 4 || buf[5] == 6)){
    FENG_Motor_SET((int8_t)buf[5]);
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
  if(safe_RxPacket(buf) != 0) return 0;
  return Coop_TakeAbort(buf);
}

static uint8_t Coop_WaitFrame(const char *frame){
  uint8_t buf[32];
  NRF24L01_Write_Reg(FLUSH_RX, 0xff);
  while(1){
    if(safe_RxPacket(buf) == 0){
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

/* 等 want 帧, 同时周期补发 resend (解决 DONE/REDY 单次发送被冲掉后双方互锁) */
static uint8_t Coop_WaitFrameResend(const char *want, const char *resend, uint32_t period_ms){
  uint8_t buf[32];
  uint32_t last = HAL_GetTick();
  NRF24L01_Write_Reg(FLUSH_RX, 0xff);
  while(1){
    if(safe_RxPacket(buf) == 0){
      if(Coop_FindFrame(buf, want)){
        printf("%s received\r\n", want);
        return 0;
      }
      if(Coop_TakeAbort(buf)){
        return 1;
      }
    }
    if(HAL_GetTick() - last >= period_ms){
      Coop_SendFrameLight(resend);
      last = HAL_GetTick();
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
  (void)safe_TxPacket(buf);
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

static void Pi_SendCap(void);
static void Ae_Poll(void);

/* 舵机作业: A给很小向上转速顶住, B短刹; 顺带抽 Pi 预测结果(不阻塞) */
static uint8_t Coop_ServoAHold(uint32_t ms){
  uint32_t t0 = HAL_GetTick();
  Clear_All_PID();
  while(HAL_GetTick() - t0 < ms){
    PID_MA(-8.0f);
    MB_STOP();
    Coop_ReportIMU();
    Ae_Poll();
    if(Coop_PollAbort()) return 1;
    HAL_Delay(10);
  }
  motor_STOP();
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 4199);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 4199);
  return 0;
}

/* 探头从 DUO_NUM 线性慢抬到收回位(499), 共 lift_ms;
 * 抬起 go_after_ms 后开始 GO_UP 往回爬 (与剩余抬升重叠)。
 * 返回: 0完成 1中止 2超时 */
static uint8_t Coop_LiftThenGoUp(float target_roll, uint32_t lift_ms, uint32_t go_after_ms){
  uint16_t duo_from = (uint16_t)DUO_NUM;
  uint16_t duo_to = 499;
  uint32_t t0 = HAL_GetTick();
  uint32_t loop_time = micros();
  uint32_t climb_t0 = 0;
  uint8_t climbing = 0;
  uint8_t fan_cut2 = 0;
  uint8_t reached = 0;

  Clear_All_PID();
  printf("AUTO: duo lift %ums, GO_UP after %ums\r\n",
         (unsigned)lift_ms, (unsigned)go_after_ms);

  while(1){
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - t0;

    /* 探头 5s 线性抬起 */
    if(elapsed < lift_ms){
      uint16_t duo = (uint16_t)(duo_from -
        (uint32_t)(duo_from - duo_to) * elapsed / lift_ms);
      DUO_PWM_SET = duo;
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, duo);
    } else if(DUO_PWM_SET != duo_to){
      DUO_PWM_SET = duo_to;
      DUO_Motor_OFF();
    }

    if(!climbing && elapsed >= go_after_ms){
      climbing = 1;
      climb_t0 = micros();
      loop_time = micros();
      Clear_All_PID();
      printf("AUTO: start GO_UP (duo still lifting)\r\n");
    }

    if(!climbing){
      /* 抬起前 1s: 底部刹住 */
      motor_STOP();
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 4199);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 4199);
      Coop_ReportIMU();
      Ae_Poll();
      if(Coop_PollAbort()) return 1;
      HAL_Delay(10);
      continue;
    }

    /* 与 GO_UP 同周期爬坡; 到顶后若探头未抬完则刹住等抬完 */
    if(micros() - loop_time >= 10000u){
      loop_time = micros();
      if(micros() - climb_t0 > 30000000u) return 2;
      Coop_ReportIMU();
      Ae_Poll();
      if(Coop_PollAbort()) return 1;

      if(!reached && (roll <= target_roll)){
        float d_left = target_roll - roll;
        if(!fan_cut2 && d_left < 30.0f){
          fan_cut2 = 1;
          if(FENG_PWM_SET > (uint16_t)(939u + 2u * FENG_UP))
            FENG_PWM_SET = (uint16_t)(FENG_PWM_SET - 2u * FENG_UP);
          else
            FENG_PWM_SET = 939;
          __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, FENG_PWM_SET);
          printf("GO_UP: fan-2 @<%ddeg -> %u\r\n", 30, (unsigned)FENG_PWM_SET);
        }
        if(d_left < 5.0f) PID_IMU(0, 40);
        else               PID_IMU(0, 100);
      } else {
        reached = 1;
        motor_STOP();
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 4199);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 4199);
        if(elapsed >= lift_ms) break;
      }
    }
  }
  DUO_Motor_OFF();
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
  uint8_t fan_cut2 = 0; /* 近顶(剩余<30°)降吸两档, 只做一次, 保持到 AUTO 结束 */
  while(1){
    if (micros() - loop_time >= 10000) {
      loop_time = micros();
      if(micros() - t_start > 30000000u) return 2; /* 超时: 返回2, 调用方不再上报DONE */
      if((roll) <= _degree ){
        float d_left = _degree - roll;
        /* 回来剩余角 <30°: 吸力降两档(每档 FENG_UP), 直到 FENG_Motor_OFF */
        if(!fan_cut2 && d_left < 30.0f){
          fan_cut2 = 1;
          if(FENG_PWM_SET > (uint16_t)(939u + 2u * FENG_UP))
            FENG_PWM_SET = (uint16_t)(FENG_PWM_SET - 2u * FENG_UP);
          else
            FENG_PWM_SET = 939;
          __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, FENG_PWM_SET);
          printf("GO_UP: fan-2 @<%ddeg -> %u\r\n", 30, (unsigned)FENG_PWM_SET);
        }
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
    /* 通知 Pi 常驻服务开始采集; 不等待预测, 抬升不阻塞 */
    g_ae_pending = 0; g_ae_sent_ok = 0; g_ae_last_tx_ms = 0;
    Pi_SendCap();
    /* 探头下压作业 25s */
    if(Coop_ServoAHold(25000)) { g_coop_report_en = 0; motor_STOP(); return; }
    /* 作业后再加吸到1639, 再慢抬探头+往回爬 */
    FENG_PWM_SET = 1639;
    __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,FENG_PWM_SET);
    if(Coop_DelayReport(500)) { g_coop_report_en = 0; motor_STOP(); return; }

    Clear_All_PID();
    /* 探头 5s 慢抬; 抬起满 1s 后开始 GO_UP (与剩余抬升重叠)
     * 0=到位 1=中止 2=超时。超时/中止都不上报DONE, 保持吸力回手动 */
    if(Coop_LiftThenGoUp(roll_Compose, 5000, 1000)) {
      g_coop_report_en = 0; motor_STOP(); return;
    }
    motor_STOP();

    /* 视觉纠偏: GO_UP 结束即通知母机开始纠偏 (REDY②);
     * 母机发 GOON(对齐确认) 后才发 DONE */
    Coop_SendFrame(REDY_RSP);   /* REDY②: GO_UP结束, 通知母机启动视觉纠偏 */
    printf("AUTO: send REDY2, wait vision\r\n");
    if(Adj_WaitLoop()){ g_coop_report_en = 0; motor_STOP(); return; }  /* 中止/超时: 不发DONE */

    g_coop_report_en = 0;
    Coop_SendFrame(DONE_RSP);
    /* DONE 可能被母机侧发送冲掉: 等 LOCK 期间周期补发 */
    if(Coop_WaitFrameResend(LOCK_CMD, DONE_RSP, DONE_RESEND_MS)) { motor_STOP(); return; }

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
#define ADJ_SPEED_REV       80.0f    /* 纠偏速度(电机轴转/s). kp*err=18*80≈1440→占空比~34%; 8.0时仅~3%低于静摩擦,轮不动A=B=0 */
#define ADJ_MM_PER_MOTORREV 0.3555f /* 里程计 mm/电机轴转: π×轮径43 / 减速比380; 卷尺复核可再微调 */
#define ADJ_TRACK_MM        163.0f   /* wheel track mm (measured 163) */
#define ADJ_PIPE_R_MM       250.0f   /* 管道外半径(mm): 用户指定 R=250 (直径约50cm) */
#define ADJ_DIAG_BETA_DEG   20.0f    /* 轴向斜行定角(deg), 最大自旋=2×β */
#define ADJ_MV_SIGN         (+1)     /* 标定: φ>0 时应往哪边走(+1=下降方向) */
#define ADJ_SPIN_SIGN       (+1)     /* 标定: ψ>0 时应往哪边转(+1=S+方向) */
#define ADJ_AXIAL_SIGN      (+1)     /* 标定: S+歪头后前进, 轴向分量朝哪边(+1=+ax侧) */
/* 轴向斜行经验系数: 理论 L=|ax|/sinβ, d=|ax|/tanβ; 按实测日志拟合后按比例修此值 */
#define ADJ_AXIAL_L_K       1.0f     /* 斜行距离 L 的倍率 */
#define ADJ_AXIAL_D_K       1.0f     /* 周向补偿 d 的倍率 */
#define ADJ_TOL_PHI         0.5f     /* 死区(deg) */
#define ADJ_TOL_PSI         0.5f
#define ADJ_TOL_AX          2.0f     /* 死区(mm), 以内交给夹爪容差 */
#define ADJ_MAX_PHI         10.0f    /* 历史限幅常量(现已不钳位, 仅留作参考) */
#define ADJ_MAX_PSI         10.0f    /* 历史限幅常量(现已不钳位) */
#define ADJ_MAX_AX          30.0f    /* 历史限幅常量(现已不钳位) */
#define ADJ_SETTLE_MS       150      /* 段间刹车稳定等待 */
#define ADJ_DEMO_GAIN       1.0f     /* 调试放大 phi/psi; 默认1=真实角度, 调试可改5 */
#define ADJ_SLIP_DIFF_MM    50.0f    /* 历史打滑门限; Adj_Move 打滑检测已按用户要求去掉, 此宏暂保留不用 */
#define ADJ_SELF_TEST       1        /* 1=work-mode key8 enters standalone adj test (TSTA path) */
#define ADJ_ST_AXIAL        1
#define ADJ_ST_SPIN         2
#define ADJ_ST_MOVE         3
#define ADJ_ST_SETTLE       4
#define ADJ_TEL_PERIOD_MS   500u     /* 运动遥测周期(ms) */
/* 周向 φ 开环定时: 代码保留, 默认关闭; 改1后走 Adj_MoveTimed */
#define ADJ_MOVE_TIMED      0
#define ADJ_MOVE_SPEED_REV  15.0f    /* 开环直行速度(电机转/s) */
#define ADJ_MOVE_MIN_MS     600u
#define ADJ_MOVE_MAX_MS     5000u
#define ADJ_TEST_PHI        5.0f     /* 自检演示值(配合占位常数观察动作方向) */
#define ADJ_TEST_PSI        (-3.0f)
#define ADJ_TEST_AX         10.0f

typedef struct { uint16_t a_old, b_old; float a_mm, b_mm; } ADJ_ODOM_T;

/* 最近一次 Adj_Move / Adj_Spin 的实测里程(带符号), 供轴向经验公式拟合 */
static volatile float g_adj_last_mm  = 0.0f;
static volatile float g_adj_last_arc = 0.0f;

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
  {
    char lb[52];
    snprintf(lb, sizeof(lb), "stl i%d/%d", (int)err_MA, (int)err_MB);
    Adj_Log(lb);
  }
  /* 清积分: 上一段(尤其自旋)积累的积分若与下一段指令同向, 起步会猛冲 2 倍稳态速度;
   * 轴向多段斜行逐段累积会更严重, 必须在每段之间清零 */
  Clear_All_PID();
  while(HAL_GetTick() - t0 < ADJ_SETTLE_MS){
    if(Coop_PollAbort()) return 1;
    HAL_Delay(10);
  }
  return 0;
}
/* 直行 dist_mm: >0=下降向 <0=上升向
 * 轮令改回更早规则: A/B 异号(物理同向). 返回 0=完成 1=中止 2=超时/打滑 */
static uint8_t Adj_Move(float dist_mm){
  ADJ_ODOM_T od;
  float target = fabsf(dist_mm);
  float v = ADJ_SPEED_REV;
  /* 超时兜底: 常数未标定前理论时限不可靠, 放宽为 20x + 20s (对齐 wallbot) */
  uint32_t t_to = (uint32_t)(target / (ADJ_SPEED_REV * ADJ_MM_PER_MOTORREV) * 20000.0f) + 20000u;
  uint32_t t0 = HAL_GetTick();
  uint32_t t_tel = 0;
  int8_t dir = (dist_mm > 0) ? 1 : -1;
  if(target < 0.5f) return 0;
  Adj_OdomStart(&od);
  {
    char lb[52];
    snprintf(lb, sizeof(lb), "mvgo t%d n%d to%d",
             (int)target,
             (int)(target / ADJ_MM_PER_MOTORREV * 7.0f),
             (int)(t_to / 1000u));
    Adj_Log(lb);
  }
  while(1){
    /* 改回更早轮向: 直行 A/B 异号 → 物理同向周移 */
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
        {
          char lb[52];
          snprintf(lb, sizeof(lb), "mv A%d B%d p%d w%d/%d",
                   (int)od.a_mm, (int)od.b_mm, (int)pct,
                   (int)g_wheel_va, (int)g_wheel_vb);
          Adj_Log(lb);
        }
        Test_Telem(ADJ_ST_MOVE, dir, (int16_t)od.a_mm, (int16_t)od.b_mm,
                   (int16_t)target, pct, 0);
      }
      if(done >= target) break;
    }
    if(HAL_GetTick() - t0 > t_to){
      motor_STOP();
      Test_Telem(ADJ_ST_MOVE, dir, (int16_t)od.a_mm, (int16_t)od.b_mm, (int16_t)target, 100, 3);
      {
        char lb[52];
        snprintf(lb, sizeof(lb), "mvTO t%d A%d B%d e%d",
                 (int)target, (int)od.a_mm, (int)od.b_mm,
                 (int)((HAL_GetTick()-t0)/1000u));
        Adj_Log(lb);
      }
      return 2;
    }
    if(Coop_PollAbort()){
      motor_STOP();
      Test_Telem(ADJ_ST_MOVE, dir, (int16_t)od.a_mm, (int16_t)od.b_mm, (int16_t)target, 100, 2);
      return 1;
    }
  }
  motor_STOP();
  Test_Telem(ADJ_ST_MOVE, dir, (int16_t)od.a_mm, (int16_t)od.b_mm, (int16_t)target, 100, 1);
  {
    char lb[52];
    snprintf(lb, sizeof(lb), "move ok d%d A%d B%d",
             (int)dist_mm, (int)od.a_mm, (int)od.b_mm);
    Adj_Log(lb);
  }
  /* 记录带指令方向的实际推进量, 供轴向经验公式拟合 */
  g_adj_last_mm = (float)dir * (fabsf(od.a_mm) + fabsf(od.b_mm)) * 0.5f;
  return 0;
}
/* 测试用开环定时直行(默认 ADJ_MOVE_TIMED=0 不启用)。极性与 Adj_Move 一致: 异号=物理同向 */
static uint8_t Adj_MoveTimed(float dist_mm){
  ADJ_ODOM_T od;
  float    dir    = (dist_mm > 0.0f) ? +1.0f : -1.0f;
  float    target = fabsf(dist_mm);
  float    v      = ADJ_MOVE_SPEED_REV;
  uint32_t dur_ms, t0, t_tel = 0;
  if(target < 0.5f) return 0;
  dur_ms = (uint32_t)(target / (v * ADJ_MM_PER_MOTORREV) * 1000.0f);
  if(dur_ms < ADJ_MOVE_MIN_MS) dur_ms = ADJ_MOVE_MIN_MS;
  if(dur_ms > ADJ_MOVE_MAX_MS) dur_ms = ADJ_MOVE_MAX_MS;
  printf("ADJ movetimed dir=%d v=%d dist=%dmm dur=%lums\r\n", (int)dir, (int)v, (int)target, (unsigned long)dur_ms);
  Adj_OdomStart(&od);
  t0 = HAL_GetTick();
  while(HAL_GetTick() - t0 < dur_ms){
    PID_MA( dir * v);
    PID_MB(-dir * v);   /* 异号=物理同向, 与 Adj_Move 一致 */
    HAL_Delay(10);
    Adj_OdomUpdate(&od);
    if(HAL_GetTick() - t_tel >= ADJ_TEL_PERIOD_MS){
      uint32_t el = HAL_GetTick() - t0;
      uint8_t  pct = (uint8_t)(el * 100u / dur_ms);
      if(pct > 100) pct = 100;
      t_tel = HAL_GetTick();
      Test_Telem(ADJ_ST_MOVE, (int8_t)dir, (int16_t)od.a_mm, (int16_t)od.b_mm,
                 (int16_t)target, pct, 0);
    }
    if(Coop_PollAbort()){
      motor_STOP();
      Test_Telem(ADJ_ST_MOVE, (int8_t)dir, (int16_t)od.a_mm, (int16_t)od.b_mm, (int16_t)target, 100, 2);
      return 1;
    }
  }
  motor_STOP();
  Test_Telem(ADJ_ST_MOVE, (int8_t)dir, (int16_t)od.a_mm, (int16_t)od.b_mm, (int16_t)target, 100, 1);
  printf("ADJ movetimed ok %lums A=%d B=%d\r\n", (unsigned long)dur_ms, (int)od.a_mm, (int)od.b_mm);
  return 0;
}
/* 原地旋转 theta_deg: >0=S+ <0=S-; 返回同上
 * 轮令改回更早规则: A/B 同号(物理对转自旋) */
static uint8_t Adj_Spin(float theta_deg){
  ADJ_ODOM_T od;
  float arc = fabsf(theta_deg) * ADJ_PI / 180.0f * (ADJ_TRACK_MM * 0.5f);
  float v = ADJ_SPEED_REV;
  uint32_t t_to = (uint32_t)(arc / (ADJ_SPEED_REV * ADJ_MM_PER_MOTORREV) * 20000.0f) + 20000u;
  uint32_t t0 = HAL_GetTick();
  uint32_t t_tel = 0;
  int8_t dir = (theta_deg * (float)ADJ_SPIN_SIGN > 0) ? 1 : -1;
  if(arc < 0.5f) return 0;
  Adj_OdomStart(&od);
  {
    char lb[52];
    snprintf(lb, sizeof(lb), "spgo a%d n%d to%d",
             (int)arc,
             (int)(arc / ADJ_MM_PER_MOTORREV * 7.0f),
             (int)(t_to / 1000u));
    Adj_Log(lb);
  }
  while(1){
    /* 改回更早轮向: 自旋 A/B 同号 → 物理对转; 方向仍可用 ADJ_SPIN_SIGN */
    if(theta_deg * (float)ADJ_SPIN_SIGN > 0){ PID_MA(-v); PID_MB(-v); }  /* S+: A/B 同号 */
    else                                     { PID_MA(+v); PID_MB(+v); } /* S-: A/B 同号 */
    HAL_Delay(10);
    Adj_OdomUpdate(&od);
    {
      float done = (fabsf(od.a_mm) + fabsf(od.b_mm)) * 0.5f;
      if(HAL_GetTick() - t_tel >= ADJ_TEL_PERIOD_MS){
        uint8_t pct = (arc > 0.01f) ? (uint8_t)(done * 100.0f / arc) : 0;
        if(pct > 100) pct = 100;
        t_tel = HAL_GetTick();
        {
          char lb[52];
          snprintf(lb, sizeof(lb), "sp A%d B%d p%d w%d/%d",
                   (int)od.a_mm, (int)od.b_mm, (int)pct,
                   (int)g_wheel_va, (int)g_wheel_vb);
          Adj_Log(lb);
        }
        Test_Telem(ADJ_ST_SPIN, dir, (int16_t)od.a_mm, (int16_t)od.b_mm,
                   (int16_t)arc, pct, 0);
      }
      if(done >= arc) break;
    }
    if(HAL_GetTick() - t0 > t_to){
      motor_STOP();
      Test_Telem(ADJ_ST_SPIN, dir, (int16_t)od.a_mm, (int16_t)od.b_mm, (int16_t)arc, 100, 3);
      {
        char lb[52];
        snprintf(lb, sizeof(lb), "spTO A%d B%d p%d e%d",
                 (int)od.a_mm, (int)od.b_mm,
                 (int)((arc > 0.01f) ? ((fabsf(od.a_mm)+fabsf(od.b_mm))*50.0f/arc) : 0),
                 (int)((HAL_GetTick()-t0)/1000u));
        Adj_Log(lb);
      }
      return 2;
    }
    if(Coop_PollAbort()){
      motor_STOP();
      Test_Telem(ADJ_ST_SPIN, dir, (int16_t)od.a_mm, (int16_t)od.b_mm, (int16_t)arc, 100, 2);
      return 1;
    }
  }
  motor_STOP();
  Test_Telem(ADJ_ST_SPIN, dir, (int16_t)od.a_mm, (int16_t)od.b_mm, (int16_t)arc, 100, 1);
  {
    char lb[52];
    snprintf(lb, sizeof(lb), "spin ok t%d a%d A%d B%d",
             (int)theta_deg, (int)arc, (int)od.a_mm, (int)od.b_mm);
    Adj_Log(lb);
  }
  /* 记录带指令方向的实际弧长, 供轴向经验公式拟合 */
  g_adj_last_arc = (float)dir * (fabsf(od.a_mm) + fabsf(od.b_mm)) * 0.5f;
  return 0;
}
/* 轴向斜行修正: 转+β斜下去(进L) → 转−β摆正 → 退回(直行d)
 * 几何: L=|ax|/sinβ, d=|ax|/tanβ → 净位移=纯轴向ax, 朝向回正, 周向归零 */
static uint8_t Adj_Axial(float ax_mm){
  float beta = ADJ_DIAG_BETA_DEG * ADJ_PI / 180.0f;
  float L = fabsf(ax_mm) / sinf(beta) * ADJ_AXIAL_L_K;
  float d = fabsf(ax_mm) / tanf(beta) * ADJ_AXIAL_D_K;
  float tilt = (ax_mm > 0 ? 1.0f : -1.0f) * (float)ADJ_AXIAL_SIGN;
  float spin_deg = tilt * ADJ_DIAG_BETA_DEG;
  uint8_t rc;
  float got_spin1, got_L, got_spin2, got_d;
  {
    char lb[52];
    snprintf(lb, sizeof(lb), "axgo ax%d L%d d%d b%d",
             (int)ax_mm, (int)L, (int)d, (int)ADJ_DIAG_BETA_DEG);
    Adj_Log(lb);
  }
  /* 1) 歪头 beta */
  rc = Adj_Spin(spin_deg);  if(rc) return rc;
  got_spin1 = g_adj_last_arc;
  if(Adj_Settle()) return 1;
  /* 2) 斜行 L */
  rc = Adj_Move( +L );      if(rc) return rc;
  got_L = g_adj_last_mm;
  if(Adj_Settle()) return 1;
  /* 3) 摆正 */
  rc = Adj_Spin(-spin_deg); if(rc) return rc;
  got_spin2 = g_adj_last_arc;
  if(Adj_Settle()) return 1;
  /* 4) 周向补偿 d */
  rc = Adj_Move( -d );      if(rc) return rc;
  got_d = g_adj_last_mm;
  {
    char lb[52];
    snprintf(lb, sizeof(lb), "axr s%d/%d L%d/%d",
             (int)spin_deg, (int)got_spin1, (int)L, (int)got_L);
    Adj_Log(lb);
  }
  {
    char lb[52];
    snprintf(lb, sizeof(lb), "axr2 r%d d%d/%d",
             (int)got_spin2, (int)(-d), (int)got_d);
    Adj_Log(lb);
  }
  return 0;
}
/* 融合纠偏规划 (保留 Adj_WaitLoop 视觉复测; 协议不变)
 * 摄像头给出误差 φ(周向deg), ψ(航向deg), ax(轴向mm), 死区清零后:
 *   ψ=0 表示车头已对齐「纯周向直行」姿态.
 *   β = sign(ax)*ADJ_AXIAL_SIGN*ADJ_DIAG_BETA_DEG  目标斜航向(相对周向).
 *
 * 角度加减 (只在规划里用摄像头值做一次, 不再分段各纠一次):
 *   1) 自旋角 θ1 = (β − ψ_cam) * ADJ_SPIN_SIGN
 *      → 从当前航向 ψ 转到对角线 β (ψ=0 且 ax≠0 时退化为旧轴向歪头 β)
 *   2) 斜行 L = |ax|/sin|β| * ADJ_AXIAL_L_K  (ax 只进长度, 不加成角度)
 *      斜行周向分量 s_circ = L * cos|β| (mm, 沿 Move+ 方向)
 *   3) 摆正 θ2 = (0 − β) * ADJ_SPIN_SIGN = −β*ADJ_SPIN_SIGN
 *   4) 周向指令 mm:  dφ_cam − s_circ
 *      即 φ 角先换成弧长再减斜行带上的周向; 等价于
 *      φ_cmd_deg 层面: φ_cam − (s_circ / R / MV_SIGN)*(180/π)
 *   ax≈0: 无斜行, θ=(0−ψ)*SIGN, 再 Move(dφ_cam) — 与旧「只纠航向+周向」一致.
 * Settle 只插在真实运动段之后. */
typedef enum {
  ADJ_FSM_GATE_AX = 0,   /* 兼容旧遥测/日志枚举值, 规划已改为顺序融合 */
  ADJ_FSM_AXIAL,
  ADJ_FSM_SPIN,
  ADJ_FSM_MOVE,
  ADJ_FSM_DONE
} ADJ_FSM_E;

/* 规划器入口: 三个偏差量 → 融合运动; 返回 0完成 1中止 2出错
 * 一次调用内跑完, 母机/App 协议不变; 步完成后仍回 REDY 可复测 */
uint8_t Adj_Execute(float phi_deg, float psi_deg, float ax_mm){
  float phi = phi_deg * ADJ_DEMO_GAIN;
  float psi = psi_deg * ADJ_DEMO_GAIN;
  float ax  = ax_mm;
  uint8_t rc = 0;
  float beta_deg, beta_rad, L, s_circ, dphi_mm, remain_mm, th;
  uint8_t did_move = 0;

  Clear_All_PID();
  if(fabsf(phi) < ADJ_TOL_PHI) phi = 0;
  if(fabsf(psi) < ADJ_TOL_PSI) psi = 0;
  if(fabsf(ax)  < ADJ_TOL_AX)  ax  = 0;
  {
    char lb[52];
    snprintf(lb, sizeof(lb), "start in %d/%d/%d -> %d/%d/%d",
             (int)phi_deg, (int)psi_deg, (int)ax_mm, (int)phi, (int)psi, (int)ax);
    Adj_Log(lb);
  }

  dphi_mm = phi * ADJ_PI / 180.0f * ADJ_PIPE_R_MM * (float)ADJ_MV_SIGN;

  if(ax != 0.0f){
    /* --- 有轴向: 转到 β → 斜行 L → 摆正到 0 → 一次周向(φ−斜行周向) --- */
    beta_deg = (ax > 0.0f ? 1.0f : -1.0f) * (float)ADJ_AXIAL_SIGN * ADJ_DIAG_BETA_DEG;
    beta_rad = fabsf(beta_deg) * ADJ_PI / 180.0f;
    L = fabsf(ax) / sinf(beta_rad) * ADJ_AXIAL_L_K;
    s_circ = L * cosf(beta_rad);          /* Move(+L) 带上的周向 mm */
    remain_mm = dphi_mm - s_circ;        /* 摄像头周向弧长 − 斜行周向 */

    {
      char lb[52];
      snprintf(lb, sizeof(lb), "fuse b%d L%d sc%d r%d",
               (int)beta_deg, (int)L, (int)s_circ, (int)remain_mm);
      Adj_Log(lb);
    }

    /* 1) 航向: β − ψ_cam */
    th = (beta_deg - psi) * (float)ADJ_SPIN_SIGN;
    {
      char lb[52];
      snprintf(lb, sizeof(lb), "sp1 th%d (b-ps)", (int)th);
      Adj_Log(lb);
    }
    rc = Adj_Spin(th);
    if(rc) goto adj_done;
    did_move = 1;
    if(Adj_Settle()){ rc = 1; goto adj_done; }

    /* 2) 斜行 */
    rc = Adj_Move(+L);
    if(rc) goto adj_done;
    if(Adj_Settle()){ rc = 1; goto adj_done; }

    /* 3) 摆正到 ψ=0: 0 − β */
    th = (0.0f - beta_deg) * (float)ADJ_SPIN_SIGN;
    {
      char lb[52];
      snprintf(lb, sizeof(lb), "sp2 th%d (0-b)", (int)th);
      Adj_Log(lb);
    }
    rc = Adj_Spin(th);
    if(rc) goto adj_done;
    if(Adj_Settle()){ rc = 1; goto adj_done; }

    /* 4) 更新后的周向 (不再单独加一次摄像头 φ) */
    if(fabsf(remain_mm) >= 0.5f){
#if ADJ_MOVE_TIMED
      rc = Adj_MoveTimed(remain_mm);
#else
      rc = Adj_Move(remain_mm);
#endif
      if(rc) goto adj_done;
    }
  } else {
    /* --- 无轴向: 只 (0−ψ) 自旋 + 摄像头周向 --- */
    if(psi != 0.0f){
      th = (0.0f - psi) * (float)ADJ_SPIN_SIGN;
      {
        char lb[52];
        snprintf(lb, sizeof(lb), "sp0 th%d (0-ps)", (int)th);
        Adj_Log(lb);
      }
      rc = Adj_Spin(th);
      if(rc) goto adj_done;
      did_move = 1;
      if(Adj_Settle()){ rc = 1; goto adj_done; }
    }
    if(fabsf(dphi_mm) >= 0.5f){
#if ADJ_MOVE_TIMED
      rc = Adj_MoveTimed(dphi_mm);
#else
      rc = Adj_Move(dphi_mm);
#endif
      if(rc) goto adj_done;
    } else if(!did_move){
      /* 三量都在死区: 不空等 Settle */
    }
  }

adj_done:
  motor_STOP();
  {
    char lb[52];
    snprintf(lb, sizeof(lb), "finish rc=%u", (unsigned)rc);
    Adj_Log(lb);
  }
  return rc;
}
/* 纠偏等待循环(GO_UP到位且已武装时进入):
 * 收 ADJ,φ,ψ,ax → Adj_Execute → 回REDY(可复测)
 * 收 GOON → 对齐确认, 返回0(调用方发DONE)
 * 收 STOP/遥控 → 返回1(中止); 空闲 180s 无 ADJ/GOON → 返回2
 * 等待指令期间周期补发 REDY (含 REDY②), 避免母机漏收后双方原地互锁 */
static uint8_t Adj_WaitLoop(void){
  uint32_t t0 = HAL_GetTick();
  uint32_t last_redy = 0;   /* 0=尽快补发一枪 */
  printf("ADJ wait: hold for vision\r\n");
  while(1){
    uint8_t buf[32];
    Coop_ReportIMU();
    Ae_Poll();   /* 预测若晚于抬升, 视觉等待阶段补发 NRF */
    if(safe_RxPacket(buf) == 0){
      if(Coop_FindFrame(buf, GOON_CMD)){
        printf("ADJ aligned, exit wait\r\n");
        return 0;                                   /* 对齐: 调用方发DONE */
      }
      if(Coop_FindFrame(buf, TSTE_CMD)){
        printf("ADJ wait: TSTE, exit\r\n");
        return 1;                                   /* standalone test exit */
      }
      if(Coop_FindFrame(buf, ADJ_CMD)){
        uint8_t rc;
        t0 = HAL_GetTick();                         /* 有下行则刷新空闲超时 */
        Adj_ParseStore(buf);
        printf("ADJ exec phi=%d psi=%d ax=%d\r\n",
               (int)g_adj_phi, (int)g_adj_psi, (int)g_adj_ax);
        rc = Adj_Execute(g_adj_phi, g_adj_psi, g_adj_ax);
        if(rc == 1) return 1;                       /* 中止 */
        Coop_SendFrame(REDY_RSP);                   /* 步完成, 可复测 */
        last_redy = HAL_GetTick();
        t0 = last_redy;
      }
      else if(Coop_TakeAbort(buf)){
        return 1;                                   /* STOP/遥控接管 */
      }
    }
    /* 周期轻量补发 REDY: 覆盖丢包; 少占 TX 以便收到收敛 GOON */
    if(last_redy == 0 || (HAL_GetTick() - last_redy >= ADJ_REDY_RESEND_MS)){
      Coop_SendFrameLight(REDY_RSP);
      last_redy = HAL_GetTick();
    }
    if(HAL_GetTick() - t0 > ADJ_WAIT_MS){
      printf("ADJ wait timeout\r\n");
      return 2;
    }
  }
}

/* standalone vision-adj test (parallel to AUTO):
 * mother->child: TSTA / ADJ,<phi>,<psi>,<ax> / GOON / TSTE / STOP
 * child->mother: REDY (not old TSTB) / DONE
 * reuse Adj_WaitLoop + Adj_Execute; no fake deviation
 * return: 0=not entered (run AUTO) 1=test exited */
static uint8_t Test_WaitLoop(void){
  uint8_t buf[32];
  uint8_t rc;

  /* outer already prefetched COOP for AUTO — do not block again */
  if(g_coop_prefetched){
    return 0;
  }

  if(!g_tsta_pending){
    printf("TEST wait: TSTA for test, COOP for AUTO\r\n");
    NRF24L01_Write_Reg(FLUSH_RX, 0xff);
    while(1){
      if(safe_RxPacket(buf) == 0){
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
          return 1;
        }
      }
    }
  }

  g_tsta_pending = 0;
  g_coop_report_en = 1;
  g_imu_last_tx_us = 0;
  Coop_SendFrame(REDY_RSP);                         /* REDY, not TSTB */
  printf("TEST: REDY sent, Adj_WaitLoop\r\n");

  rc = Adj_WaitLoop();
  g_coop_report_en = 0;
  if(rc != 0){
    printf("TEST adj end rc=%u\r\n", (unsigned)rc);
    motor_STOP();
    return 1;
  }

  Coop_SendFrame(DONE_RSP);
  printf("TEST: DONE, wait TSTE\r\n");
  {
    uint32_t last_done = HAL_GetTick();
    NRF24L01_Write_Reg(FLUSH_RX, 0xff);
    while(1){
      if(safe_RxPacket(buf) == 0){
        if(Coop_FindFrame(buf, TSTE_CMD)){
          printf("TSTE received, exit test\r\n");
          motor_STOP();
          return 1;
        }
        if(Coop_FindFrame(buf, TSTA_CMD)){
          g_coop_report_en = 1;
          g_imu_last_tx_us = 0;
          Coop_SendFrame(REDY_RSP);
          rc = Adj_WaitLoop();
          g_coop_report_en = 0;
          if(rc != 0){ motor_STOP(); return 1; }
          Coop_SendFrame(DONE_RSP);
          last_done = HAL_GetTick();
          NRF24L01_Write_Reg(FLUSH_RX, 0xff);
          continue;
        }
        if(Coop_TakeAbort(buf)){
          printf("TEST abort after DONE\r\n");
          motor_STOP();
          return 1;
        }
      }
      if(HAL_GetTick() - last_done >= DONE_RESEND_MS){
        Coop_SendFrame(DONE_RSP);
        last_done = HAL_GetTick();
      }
    }
  }
}

static void ApplyRemoteKey(int8_t key){
  if(key == 0) return;
  if(MODE == 11){
    /* 作业模式: 风机(3增/4减/6停); 舵机1=一点一点下压(+142/按) 2=一点一点抬升(-142/按);
       key9退出; key8=纠偏自检(ADJ_SELF_TEST=1时进独立TSTA纠偏测试环) */
    if(key == 9){
      MODE = 9;                 /* key=9 退出作业模式, 回手操 */
      Clear_All_PID();
      printf("WORK MODE off\r\n");
    } else if(key == 8 && ADJ_SELF_TEST){
      /* bench: enter standalone adj test (wait mother ADJ/GOON/TSTE) */
      printf("ADJ self test: enter TSTA path\r\n");
      g_tsta_pending = 1;
      MODE = 10;
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

/* ============ 树莓派串口(USART1: PA9-TX/PA10-RX, 115200, 机载直连 Pi GPIO14/15) ============
 * 协议(ASCII行, \n结尾):
 *   子→Pi: CAP          探头下压后通知常驻服务开始采集
 *   Pi→子: R,cm=..,near=..,pd=..,st=..   预测结果(不阻塞抬升)
 *   调试兼用: W/X/F±/S±/F=/S=  (旧手动指令仍可用)
 * NRF 上送母机: 'D' + flags + cm_x10(i16) + near(u8) + st(u8) */
#define PI_LINK_EN          1
#define PI_RX_LINE_MAX      64
static uint8_t pi_rx_byte = 0;
static volatile char    pi_rx_line[PI_RX_LINE_MAX];
static volatile uint8_t pi_rx_idx = 0;
static volatile uint8_t pi_rx_ready = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1){
    uint8_t c = pi_rx_byte;
    if(pi_rx_ready){
      pi_rx_idx = 0;                      /* 上一条还没被取走, 丢弃新行开头并重新攒 */
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
    HAL_UART_Receive_IT(&huart1, &pi_rx_byte, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1){
    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UART_Receive_IT(&huart1, &pi_rx_byte, 1);   /* 出错后重新挂接收 */
  }
}

static void Pi_Reply(const char *s){
  HAL_UART_Transmit(&huart1, (uint8_t*)s, (uint16_t)strlen(s), 20);
}

static void Pi_SendCap(void){
  Pi_Reply("CAP\n");
}

static uint8_t Ae_StCode(const char *st){
  if(st == 0 || st[0] == 0) return 0xFF;
  if(strcmp(st, "ok") == 0) return 0;
  if(strcmp(st, "few") == 0) return 1;
  if(strcmp(st, "bad") == 0) return 2;
  if(strcmp(st, "err") == 0) return 3;
  if(strcmp(st, "busy") == 0) return 4;
  if(strcmp(st, "nopd") == 0) return 5;
  return 0xFE;
}

/* 解析 R,cm=23.5,near=20,pd=1,st=ok */
static void Ae_ParseResultLine(const char *cmd){
  const char *p;
  char key[8];
  char val[16];
  uint8_t ki, vi;
  float cm;
  int near, pd;
  g_ae_cm_x10 = 0x7FFF;
  g_ae_near = 0xFF;
  g_ae_pd = 0xFF;
  g_ae_st = 0xFF;
  p = cmd;
  if(p[0] != 'R' || p[1] != ',') return;
  p += 2;
  while(*p){
    ki = 0; vi = 0;
    while(*p && *p != '=' && ki < sizeof(key)-1) key[ki++] = *p++;
    key[ki] = 0;
    if(*p != '=') break;
    p++;
    while(*p && *p != ',' && vi < sizeof(val)-1) val[vi++] = *p++;
    val[vi] = 0;
    if(*p == ',') p++;
    if(strcmp(key, "cm") == 0 && val[0]){
      cm = (float)atof(val);
      if(cm > -100.0f && cm < 200.0f) g_ae_cm_x10 = (int16_t)(cm * 10.0f + (cm >= 0 ? 0.5f : -0.5f));
    } else if(strcmp(key, "near") == 0 && val[0]){
      near = atoi(val);
      if(near >= 0 && near <= 100) g_ae_near = (uint8_t)near;
    } else if(strcmp(key, "pd") == 0 && val[0]){
      pd = atoi(val);
      if(pd == 0 || pd == 1) g_ae_pd = (uint8_t)pd;
    } else if(strcmp(key, "st") == 0){
      g_ae_st = Ae_StCode(val);
    }
  }
  g_ae_pending = 1;
  g_ae_sent_ok = 0;
}

static void Ae_TrySendNrf(void){
  uint8_t f[32];
  uint8_t flags;
  uint32_t now;
  if(!g_ae_pending || g_ae_sent_ok) return;
  now = HAL_GetTick();
  if(g_ae_last_tx_ms != 0 && (now - g_ae_last_tx_ms) < 200u) return; /* 限速补发 */
  memset(f, 0, sizeof(f));
  flags = 0;
  if(g_ae_cm_x10 != (int16_t)0x7FFF) flags |= 0x01;
  if(g_ae_near != 0xFF) flags |= 0x02;
  if(g_ae_pd != 0xFF){ flags |= 0x04; if(g_ae_pd) flags |= 0x08; }
  f[0] = 'D';
  f[1] = flags;
  memcpy(&f[2], &g_ae_cm_x10, 2);
  f[4] = g_ae_near;
  f[5] = g_ae_st;
  g_ae_last_tx_ms = now;
  if(Coop_SendRaw(f, 8, 3) == 0){
    g_ae_sent_ok = 1;
    g_ae_pending = 0;
  }
}

/* AUTO 各循环空档调用: 抽串口结果 + 尝试 NRF 上送, 绝不长时间阻塞 */
static void Ae_Poll(void){
  char cmd[PI_RX_LINE_MAX];
  uint8_t i;
  if(pi_rx_ready){
    for(i = 0; i < PI_RX_LINE_MAX; i++) cmd[i] = pi_rx_line[i];
    pi_rx_ready = 0;
    if(cmd[0] == 'R' && cmd[1] == ','){
      Ae_ParseResultLine(cmd);
    } else if(strncmp(cmd, "OK", 2) == 0){
      /* CAP/PING 应答, 忽略 */
    }
    /* 其它指令留给主循环 Pi_CMD_Process; AUTO 内不执行手动键 */
  }
  Ae_TrySendNrf();
}

/* 主循环调用: 取出并执行一条树莓派指令(含 R 结果) */
static void Pi_CMD_Process(void){
  char cmd[PI_RX_LINE_MAX];
  uint8_t i;
  int v;
  if(!pi_rx_ready){
    Ae_TrySendNrf();
    return;
  }
  for(i = 0; i < PI_RX_LINE_MAX; i++) cmd[i] = pi_rx_line[i];
  pi_rx_ready = 0;

  if(cmd[0] == 'R' && cmd[1] == ','){
    Ae_ParseResultLine(cmd);
    Ae_TrySendNrf();
    return;
  }
  if(strncmp(cmd, "OK", 2) == 0){ return; }

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
  Ae_TrySendNrf();
}

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  if(PI_LINK_EN) HAL_UART_Receive_IT(&huart1, &pi_rx_byte, 1);   /* 树莓派 USART1 PA9/PA10 */
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

    /* 协同模式: 接收由 AUTO() 内部独占, 外层不得抢 RX FIFO */
    if(MODE == 10){
      /* standalone test first (TSTA/pending); 0 => run AUTO(COOP) */
      if(Test_WaitLoop() == 0){
        AUTO();
      }
      /* 中止/正常退出: 保持吸力, 切手操; key=6 在此停风机 */
      MODE = 9;
      g_coop_report_en = 0;
      motor_STOP();
      if(Arr_RX[5] != 0){
        ApplyRemoteKey(Arr_RX[5]);
        Arr_RX[5] = 0;
      }
      continue;
    }

    if(safe_RxPacket((uint8_t *)Arr_RX)==0){
        /* manual: TSTA=>standalone adj test; COOP=>AUTO */
        if(Coop_FindFrame((const uint8_t *)Arr_RX, TSTA_CMD)){
          g_tsta_pending = 1;
          MODE = 10;
          memset(Arr_RX, 0, 32);
          continue;
        }
        if(Coop_FindFrame((const uint8_t *)Arr_RX, COOP_CMD)){
          g_coop_prefetched = 1;
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