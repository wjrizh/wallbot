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
#include <string.h>          //memcpy(拼IMU帧)
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
void PID_IMU(float u_roll , int M_V_nomal)
{
  float err = 0;
  float kp = 10;
  float ki = 1.5f;
  float PIDIMU = 0;
  (void)u_roll;

  static float last_PIDIMU = 0;
  static uint8_t imu_fail_cnt = 0;
  static uint32_t last_imu_time = 0;

  if(mpu_dmp_get_data(&pitch, &roll, &yaw) == 0) {
      imu_fail_cnt = 0;

      uint32_t current_time = micros();
      float dt = (current_time - last_imu_time) / 1000000.0f;
      if(dt > 0.05f || dt <= 0.0f) dt = 0.01f;
      last_imu_time = current_time;

      err = -IMU_Twist();
      err_IMU = err*ki*dt + err_IMU;

      if(err_IMU > 25){
        err_IMU = 25;
      }
      else if(err_IMU < -25){
        err_IMU = -25;
      }

      PIDIMU = kp*err + err_IMU;

      /* 行进时差速限幅; 下去放到 ±40 便于观察, 上来仍 ±20。接近静止才放宽到 45 */
      float base = (M_V_nomal >= 0 ? (float)M_V_nomal : (float)(-M_V_nomal));
      float lim;
      if (base < 5.0f) {
        lim = 45.0f;
      } else if (M_V_nomal < 0) {
        lim = 40.0f;
        if (lim > base) lim = base;
      } else {
        lim = 40.0f;
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

  /* 两轮都转起来之前: 同样目标、积分预充, 避免一只先走 */
  if (!g_wheels_launched && fabsf(g_wheel_va) > 15.0f && fabsf(g_wheel_vb) > 15.0f) {
    g_wheels_launched = 1;
  }
  if (!g_wheels_launched) {
    if (err_MA < 2000.0f) err_MA = 2000.0f;
    if (err_MB < 2000.0f) err_MB = 2000.0f;
    PIDIMU = 0;
  }

  /* 下去两边一起加减速; 上来仍是同号差速 */
  if (M_V_nomal < 0) {
    PID_MB(M_V_nomal - PIDIMU);
    PID_MA(-M_V_nomal + PIDIMU);
  } else {
    PID_MB(M_V_nomal - PIDIMU);
    PID_MA(-M_V_nomal - PIDIMU);
  }
}

uint8_t GO_FLAG = 0;
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
  //k3  k4
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
  if(key != 0){
    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,DUO_PWM_SET);
  }
}
/* USER CODE END 0 */


void FENG_Motor_ON(void){
  for(int i = 0;i<7;i++){
    FENG_Motor_SET(3);
    HAL_Delay(300);
  }
  /* 出发固定油门 1600 */
  FENG_PWM_SET = 1600;
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,FENG_PWM_SET);
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
 * NRF24L01: 2Mbps, CH40, CRC16, 自动ACK, payload 32字节; 母机PTX, 本机PRX, 同一地址
 * 母机→子机指令帧(4字节ASCII): COOP=母机已下压70+开夹持器130°到位
 *                              GOON=母机已抬起160, 放行子机干活
 *                              LOCK=母机预夹紧40°+夹紧0°完成
 * 子机→母机应答帧(4字节ASCII): REDY=就绪  DONE=工作完成  ACKD=确认
 * 另: 作业期间周期上报IMU帧(9字节)='I'+pitch(float小端)+roll(float小端), 母机等DONE时解析显示
 * (旧协议 FAN_OK/OK_GO/单字节D, 信号8/6 已废弃) */
#define COOP_CMD  "COOP"
#define GOON_CMD  "GOON"
#define LOCK_CMD  "LOCK"
#define REDY_RSP  "REDY"
#define DONE_RSP  "DONE"
#define ACKD_RSP  "ACKD"
#define COOP_IMU_PERIOD_MS  200u   /* IMU帧上报周期 */

static uint8_t  g_coop_report_en = 0;    /* 1=作业期间允许上报IMU帧 */
static uint32_t g_imu_last_tx_us = 0;

/* 切TX发送4字节ASCII帧, 发完切回RX; 失败自动重发直到对方收到(母机在RX态会自动ACK) */
static void Coop_SendFrame(const char *frame){
  uint8_t buf[32] = {0};
  uint8_t i;
  for(i = 0; i < 4; i++) buf[i] = (uint8_t)frame[i];
  NRF24L01_TX_Mode();
  HAL_Delay(2);
  while(NRF24L01_TxPacket(buf) != TX_OK){
    printf("send %s fail, retry\r\n", frame);
    HAL_Delay(2);
  }
  printf("%s sent\r\n", frame);
  NRF24L01_RX_Mode();
}

/* 在32字节payload内滑动查找4字节帧(帧在payload任意位置都能命中) */
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

/* 阻塞等待母机指定帧; 其他包(遥控摇杆/按键等)一律忽略 */
static void Coop_WaitFrame(const char *frame){
  uint8_t buf[32];
  NRF24L01_Write_Reg(FLUSH_RX, 0xff);      /* 丢弃上一阶段残留的包 */
  while(1){
    if(NRF24L01_RxPacket(buf) == 0){
      if(Coop_FindFrame(buf, frame)){
        printf("%s received\r\n", frame);
        return;
      }
    }
  }
}

/* 上报一帧IMU('I'+pitch+roll, 9字节); 显示帧, 丢了等下一周期, 不重试 */
static void Coop_ReportIMU(void){
  uint8_t buf[32] = {0};
  if(!g_coop_report_en) return;
  if(micros() - g_imu_last_tx_us < COOP_IMU_PERIOD_MS * 1000u) return;
  g_imu_last_tx_us = micros();
  mpu_dmp_get_data(&pitch, &roll, &yaw);   /* 顺手刷新显示值, 读不到就用旧值 */
  buf[0] = 'I';
  memcpy(&buf[1], &pitch, 4);
  memcpy(&buf[5], &roll, 4);
  NRF24L01_TX_Mode();
  HAL_Delay(2);
  (void)NRF24L01_TxPacket(buf);
  NRF24L01_RX_Mode();
}

/* 阻塞延时ms, 期间按周期上报IMU帧(替代纯HAL_Delay的等待段) */
static void Coop_DelayReport(uint32_t ms){
  uint32_t t0 = HAL_GetTick();
  while(HAL_GetTick() - t0 < ms){
    Coop_ReportIMU();
    HAL_Delay(50);
  }
}

void GO_DOWN(float _degree){
  uint32_t loop_time = micros();
  uint32_t t_start = micros();
  while(1){
    if (micros() - loop_time >= 10000) {
      loop_time = micros();
      if(micros() - t_start > 30000000u) break;
      if((roll) >= _degree){
        float d_left = roll - _degree;
        if (d_left < 5.0f) {
          PID_IMU(0, -10);
        } else if (d_left < 10.0f) {
          PID_IMU(0, -30);
        } else {
          PID_IMU(0, -80);
        }
      }
      else{
        break;
      }
      Coop_ReportIMU();     /* 作业期间按周期上报IMU帧, 母机WAIT_DONE阶段显示用 */
    }
  }
}

void GO_UP(float _degree){
  uint32_t loop_time = micros();
  uint32_t t_start = micros();
  while(1){
    if (micros() - loop_time >= 10000) {
      loop_time = micros();
      if(micros() - t_start > 30000000u) break;
      if((roll) <= _degree ){
        float d_left = _degree - roll;
        (void)d_left;
        PID_IMU(0, 80);
      }
      else{
        break;
      }
      Coop_ReportIMU();     /* 作业期间按周期上报IMU帧, 母机WAIT_DONE阶段显示用 */

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

void Compose_IMU(){
  float temp_pitch = 0;
  float temp_roll = 0;
  int valid_cnt = 0;
  
  pitch_Compose = 0;
  roll_Compose = 0;

  uint32_t start_time = micros();
  while(valid_cnt < 100 && (micros() - start_time < 1500000)) {
      if(mpu_dmp_get_data(&pitch, &roll, &yaw) == 0) {
          temp_pitch += pitch;
          temp_roll += roll;
          valid_cnt++;
      }
  }
  
  qref[0]=g_dmp_quat[0]; qref[1]=g_dmp_quat[1]; qref[2]=g_dmp_quat[2]; qref[3]=g_dmp_quat[3];  //?, ????
  if(valid_cnt > 0) {
      pitch_Compose = temp_pitch / valid_cnt;
      roll_Compose = temp_roll / valid_cnt;
  } else {
      pitch_Compose = pitch;
      roll_Compose = roll;
  }
}



/* ============ 子母协同自动流程(子机侧) ============
 * 时序: 等 COOP(母机下压70+开夹130°) -> 回 REDY
 *       等 GOON(母机抬起160放行)    -> 作业(吸附下行+舵机作业+回爬, 周期上报IMU帧)
 *       发 DONE(工作完成)           -> 等 LOCK(母机预夹紧40°+夹紧0°已夹住)
 *       回 ACKD                     -> 关风机, 结束回手动
 * 遥控键( Arr_RX[5] ): 1,2=舵机 3,4=风机 5=存Flash 9/10=模式切换, 与协议帧互不冲突
 * ====================================================================== */
int8_t Arr_RX[32] = {0};
void AUTO(void){
  /* ---- 阶段0: 待命(四轮刹车, 铲斗抬起), 等母机COOP ---- */
  motor_STOP();
  DUO_Motor_OFF();                            // 待命期间铲斗保持抬起, 脉宽499
  printf("AUTO ready, wait COOP...\r\n");
  Coop_WaitFrame(COOP_CMD);

  /* ---- 阶段1: 回REDY(子机就绪), 再等母机GOON(已抬起160, 放行) ---- */
  Coop_SendFrame(REDY_RSP);
  Coop_WaitFrame(GOON_CMD);

  /* ---- 阶段2: 子机作业: 吸附下行 + 舵机作业 + 回爬; 期间周期上报IMU帧 ---- */
  g_coop_report_en = 1;
  g_imu_last_tx_us = 0;
  FENG_Motor_ON();
  Coop_DelayReport(4000);                     // 风机起转稳定4s, 期间照常上报IMU

  Compose_IMU();
  Clear_All_PID();
  GO_DOWN(-90);
  motor_STOP();
  /* 底部静止先只让A轮向上顶住, PWM 4199(全占空比); B轮仍刹车 */
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 4199);
  /* 到达目标位置后风机提高到1639并一直保持到结束 */
  FENG_PWM_SET = 1639;
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,FENG_PWM_SET);

  HAL_Delay(200);                             // 到位后先稳定200ms
  DUO_Motor_ON(DUO_NUM);                      // 放下, 固定1055
  Coop_DelayReport(8000);                     // 舵机作业8s
  DUO_Motor_OFF();                            // 抬起
  HAL_Delay(500);

  Clear_All_PID();
  GO_UP(roll_Compose);
  motor_STOP();
  Coop_DelayReport(2000);                     // 回爬结束后停2s

  /* ---- 阶段3: 停IMU上报, 发DONE告诉母机工作完成 ---- */
  g_coop_report_en = 0;
  Coop_SendFrame(DONE_RSP);

  /* ---- 阶段4: 等母机LOCK(已预夹紧40°+夹紧0°夹住本机), 回ACKD ---- */
  Coop_WaitFrame(LOCK_CMD);
  Coop_SendFrame(ACKD_RSP);

  /* ---- 阶段5: 母机已夹紧, 本机风机降速停止, 结束回手动 ---- */
  FENG_Motor_OFF();                           // 风机降回939
  Clear_All_PID();
}
/**
  * @brief  The application entry point.
  * @retval int
  */
  

uint8_t DDR_SED[32] = {0};
int8_t MODE = 9;//9 ??? 10 ???
//???? ?? ??????? ?? ????
uint32_t Flash_Store[2] = {15,78};
uint32_t Flash_ReadNUM[2] = {0};
//K5????? ??
int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
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
    if(NRF24L01_RxPacket((uint8_t *)Arr_RX)==0){
        if(Arr_RX[5] != 0){
          if(Arr_RX[5] == 9){
            MODE = 9;
          }
          else if(Arr_RX[5] == 10){
            MODE = 10;
          }
          else if (Arr_RX[5] == 5){
            Flash_Store[0] = DUO_PWM_SET;
            DUO_NUM = DUO_PWM_SET;
            Flash_Write(Flash_Store,1);
            Flash_Read(Flash_ReadNUM,1);
            printf("Flash TEST : %d , %d \r\n",Flash_ReadNUM[0],Flash_ReadNUM[1]);
          }
          FENG_Motor_SET(Arr_RX[5]);
          DUO_Motor_SET(Arr_RX[5]);
          Arr_RX[5] = 0;
        }
    }

    if (micros() - last_ctrl_time >= 10000) {
        last_ctrl_time = micros();

        if(MODE == 9){
            PID_MA(-Arr_RX[2]*2.0 + Arr_RX[1]*1);
            PID_MB(Arr_RX[2]*2.0 + Arr_RX[1]*1);
        }
    }

    if(MODE == 10){
      AUTO();
      MODE = 9;
      Arr_RX[1] = 0;
      Arr_RX[2] = 0;
      err_MA = 0;
      PIDNUM_MA = 0;
      err_MB = 0;
      PIDNUM_MB = 0;
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