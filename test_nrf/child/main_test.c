/* 子机 NRF24L01 视觉纠偏测试固件
 * 支持: PING->PONG / TSTA->TSTB / ADJ,phi,psi,ax -> 3s 模拟运动(每100ms一帧'J'遥测) -> DONE / TSTE->ACKD
 * 不驱动真电机, 只回虚拟遥测. 串口 USART1 115200
 */
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

void SystemClock_Config(void);
void Error_Handler(void);

/* spi.c 的 NRF24L01_TxPacket 里 while(ReadPin(PA6)!=0) 无超时,
 * IRQ 异常会死循环锁死。这里用带超时的版本替代。 */
static uint8_t safe_TxPacket(uint8_t *txbuf)
{
  uint8_t sta;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  NRF24L01_Write_Buf(0xA0, txbuf, 32);          /* WR_TX_PLOAD */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  uint32_t t0 = HAL_GetTick();
  while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) != 0) {
    if (HAL_GetTick() - t0 > 20) break;          /* 20ms 超时 */
  }

  sta = NRF24L01_Read_Reg(0x07);                 /* STATUS */
  NRF24L01_Write_Reg(0x20 + 0x07, sta);
  if (sta & 0x10) { NRF24L01_Write_Reg(0xE1, 0xFF); return 0x10; }  /* MAX_TX */
  if (sta & 0x20) { return 0x20; }                                 /* TX_OK */
  return 0xFF;
}

/* 可靠收包: IRQ 引脚判据 + FIFO_STATUS 判空, 避免 SPI 空读假阳性 */
static uint8_t safe_RxPacket(uint8_t *rxbuf)
{
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) != 0) return 1;
  uint8_t sta = NRF24L01_Read_Reg(0x07);
  NRF24L01_Write_Reg(0x20 + 0x07, sta);
  if ((sta & 0x40) == 0) return 1;
  uint8_t fifo = NRF24L01_Read_Reg(0x17);
  if (fifo & 0x01) { NRF24L01_Write_Reg(0xE2, 0xFF); return 1; }
  NRF24L01_Read_Buf(0x61, rxbuf, 32);
  NRF24L01_Write_Reg(0xE2, 0xFF);
  return 0;
}

static void send_telem(uint8_t stage, int8_t dir, int16_t a_mm, int16_t b_mm,
                       int16_t target_mm, uint8_t pct, uint8_t rc);

/* ==================== 驱动: 编码器测速 + PID + PWM ====================
 * 逐字移植自主固件 firmware/pabi_F103rct6/Core/Src/main.c
 * A轮 TIM2 (PA15/PB3), B轮 TIM3 (PB4/PB5), PWM TIM4 CH1-4 (PB6-PB9) */

uint16_t MA_COUNTER_new = 0;
uint16_t MA_COUNTER_old = 0;
/* GDB 可直接读取: 每次 PID 测速的原始 10ms 计数差、测速结果和输出。 */
volatile int16_t g_diag_delta_a = 0, g_diag_delta_b = 0;
volatile float g_diag_speed_a = 0, g_diag_speed_b = 0;
float MA_V_GET(void)
{
  int TEMP_COUNTER = 0;
  float V_A = 0;
  const float dt = 0.01f;
  MA_COUNTER_new = __HAL_TIM_GET_COUNTER(&htim2);
  TEMP_COUNTER = MA_COUNTER_new - MA_COUNTER_old;
  g_diag_delta_a = (int16_t)((uint16_t)MA_COUNTER_new - MA_COUNTER_old);
  if(((TEMP_COUNTER) > 10000) || ((TEMP_COUNTER) < -10000)){
    if((TEMP_COUNTER) > 10000) V_A = ((float)(MA_COUNTER_new - (MA_COUNTER_old + 65535)) / 7.0f) / dt;
    else                       V_A = ((float)(MA_COUNTER_new - (MA_COUNTER_old - 65535)) / 7.0f) / dt;
  } else {
    V_A = ((float)(TEMP_COUNTER) / 7.0f) / dt;
  }
  MA_COUNTER_old = MA_COUNTER_new;
  g_diag_speed_a = V_A;
  return V_A;
}

uint16_t MB_COUNTER_new = 0;
uint16_t MB_COUNTER_old = 0;
float MB_V_GET(void)
{
  int TEMP_COUNTER = 0;
  float V_B = 0;
  const float dt = 0.01f;
  MB_COUNTER_new = __HAL_TIM_GET_COUNTER(&htim3);
  TEMP_COUNTER = MB_COUNTER_new - MB_COUNTER_old;
  g_diag_delta_b = (int16_t)((uint16_t)MB_COUNTER_new - MB_COUNTER_old);
  if(((TEMP_COUNTER) > 10000) || ((TEMP_COUNTER) < -10000)){
    if((TEMP_COUNTER) > 10000) V_B = ((float)(MB_COUNTER_new - (MB_COUNTER_old + 65535)) / 7.0f) / dt;
    else                       V_B = ((float)(MB_COUNTER_new - (MB_COUNTER_old - 65535)) / 7.0f) / dt;
  } else {
    V_B = ((float)(TEMP_COUNTER) / 7.0f) / dt;
  }
  MB_COUNTER_old = MB_COUNTER_new;
  g_diag_speed_b = V_B;
  return V_B;
}

void MA_STOP(void){
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,4199);
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,4199);
}
void MB_STOP(void){
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,4199);
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,4199);
}
void motor_STOP(void){ MA_STOP(); MB_STOP(); }

float err_MA = 0;
int PIDNUM_MA = 0;
void PID_MA(float V_Goat){
  float kp = 18, ki = 90;
  float V_now = 0, PIDNUM = 0, err = 0;
  V_Goat = -V_Goat;
  V_now = MA_V_GET();
  if(V_Goat < 0) err = -V_Goat + V_now;
  else           err = V_Goat - V_now;
  float dt = 0.01f;
  float P_term = kp * err;
  float I_inc = err * ki * dt;
  if ((P_term + err_MA > 4199 && I_inc > 0) || (P_term + err_MA < 0 && I_inc < 0)) {
  } else { err_MA += I_inc; }
  if(err_MA > 4000) err_MA = 4000;
  else if(err_MA < -4000) err_MA = -4000;
  PIDNUM = P_term + err_MA;
  PIDNUM_MA = (int)PIDNUM;
  if(PIDNUM_MA > 4199) PIDNUM_MA = 4199;
  else if(PIDNUM_MA < 0) PIDNUM_MA = 0;
  if(V_Goat < 0){
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,PIDNUM_MA);
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
  } else if(V_Goat > 0){
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,PIDNUM_MA);
  } else { MA_STOP(); }
}

float err_MB = 0;
int PIDNUM_MB = 0;
void PID_MB(float V_Goat){
  float kp = 18, ki = 90;
  float V_now = 0, PIDNUM = 0, err = 0;
  V_Goat = -V_Goat;
  V_now = MB_V_GET();
  if(V_Goat < 0) err = -V_Goat + V_now;
  else           err = V_Goat - V_now;
  float dt = 0.01f;
  float P_term = kp * err;
  float I_inc = err * ki * dt;
  if ((P_term + err_MB > 4199 && I_inc > 0) || (P_term + err_MB < 0 && I_inc < 0)) {
  } else { err_MB += I_inc; }
  if(err_MB > 4000) err_MB = 4000;
  else if(err_MB < -4000) err_MB = -4000;
  PIDNUM = P_term + err_MB;
  PIDNUM_MB = (int)PIDNUM;
  if(PIDNUM_MB > 4199) PIDNUM_MB = 4199;
  else if(PIDNUM_MB < 0) PIDNUM_MB = 0;
  if(V_Goat < 0){
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,PIDNUM_MB);
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
  } else if(V_Goat > 0){
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,PIDNUM_MB);
  } else { MB_STOP(); }
}

void Clear_All_PID(void){ err_MA = 0; err_MB = 0; PIDNUM_MA = 0; PIDNUM_MB = 0; }

#include "adj_plan.h"


/* 发一帧: 切TX -> 发 -> 回RX */
static uint8_t send_frame(const uint8_t *buf)
{
  NRF24L01_TX_Mode();
  HAL_Delay(2);
  uint8_t sta = safe_TxPacket((uint8_t*)buf);
  NRF24L01_RX_Mode();
  NRF24L01_Write_Reg(FLUSH_RX, 0xFF);
  return sta;
}

/* MTR,O 与 MTR,P 分开测试, 均限时 300ms, 不能在仅有 ST-Link 供电时运行。
 * D,P 帧: 300ms 原始编码器差、最后一轮 10ms 原始差; D,V 帧: 最后测速
 * (用未标定的 7 count/rev 换算)和实际 PWM。发帧放在停轮之后, 不干扰 PID 周期。 */
static void motor_probe(uint8_t closed_loop)
{
  uint16_t a0 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
  uint16_t b0 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  uint8_t tx[32];
  motor_STOP();
  Clear_All_PID();
  (void)MA_V_GET();
  (void)MB_V_GET();
  if (closed_loop) {
    for (uint8_t i = 0; i < 30; ++i) {
      PID_MA(+50.0f); PID_MB(-50.0f); /* 未标定试验速度, 不是正式值 */
      HAL_Delay(10);
    }
  } else {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 2000);
    for (uint8_t i = 0; i < 30; ++i) {
      HAL_Delay(10);
      (void)MA_V_GET();
      (void)MB_V_GET();
    }
  }
  int16_t a = (int16_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim2) - a0);
  int16_t b = (int16_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim3) - b0);
  int16_t da = g_diag_delta_a, db = g_diag_delta_b;
  int va = (int)g_diag_speed_a, vb = (int)g_diag_speed_b;
  int pa = closed_loop ? PIDNUM_MA : 2000;
  int pb = closed_loop ? PIDNUM_MB : 2000;
  motor_STOP();
  memset(tx, 0, sizeof(tx));
  snprintf((char*)tx, sizeof(tx), "D,%c,%d,%d,%d,%d", closed_loop ? 'P' : 'O', a, b, da, db);
  send_frame(tx);
  memset(tx, 0, sizeof(tx));
  snprintf((char*)tx, sizeof(tx), "D,V,%d,%d,%d,%d", va, vb, pa, pb);
  send_frame(tx);
}

/* 视觉纠偏测试状态(非阻塞) */
#define ST_IDLE   0
#define ST_READY  1
#define ST_RUN    2
static uint8_t  g_st = ST_IDLE;
static uint32_t g_run_start = 0;
static uint32_t g_telem_next = 0;
static uint8_t  g_telem_n = 0;
static int16_t  g_target_mm = 0;
static int8_t   g_dir = 1;
static float    g_phi = 0, g_psi = 0, g_ax = 0;

#define RUN_MS      3000u
#define TELEM_STEP  100u
#define TELEM_TOTAL (RUN_MS / TELEM_STEP)

/* 遥测 'J' 帧 (11 字节), 格式同主工程 */
static void send_telem(uint8_t stage, int8_t dir, int16_t a_mm, int16_t b_mm,
                       int16_t target_mm, uint8_t pct, uint8_t rc)
{
  uint8_t f[32];
  memset(f, 0, 32);
  f[0] = 'J';
  f[1] = stage;
  f[2] = (uint8_t)dir;
  memcpy(&f[3], &a_mm, 2);
  memcpy(&f[5], &b_mm, 2);
  memcpy(&f[7], &target_mm, 2);
  f[9]  = pct;
  f[10] = rc;
  send_frame(f);
}

static void parse_adj(const uint8_t *buf)
{
  char tmp[33];
  char *p;
  memcpy(tmp, buf, 32); tmp[32] = 0;
  p = tmp + 4;
  g_phi = (float)atof(p);
  p = strchr(p, ',');
  if (p) { g_psi = (float)atof(p + 1); p = strchr(p + 1, ','); if (p) g_ax = (float)atof(p + 1); }
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();

  /* 与主固件一致: TIM2/TIM3 编码器, TIM4 四路 PWM 驱动电机 */
  MX_TIM2_Init();
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  MX_TIM3_Init();
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  MX_TIM4_Init();
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  motor_STOP();
  /* 预热测速: MA_V_GET/MB_V_GET 用 old-new 差分, old 初值 0 会算出巨大速度,
     必须先调一次把 old 对齐到当前计数器, 否则 PID 积分被压到 0 电机不动 */
  (void)MA_V_GET();
  (void)MB_V_GET();
  HAL_Delay(20);
  /* gpio.c 把 PA6 配成了 OUTPUT_PP, 但它是 NRF24L01 的 IRQ 输出引脚, 必须为 INPUT */
  {
    GPIO_InitTypeDef gi = {0};
    gi.Pin = GPIO_PIN_6;
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gi);
  }

  printf("\r\n=== NRF24L01 CHILD ALIGN TEST ===\r\n");

  NRF24L01_Init();
  HAL_Delay(50);

  uint8_t chk = 0xFF;
  for (int i = 0; i < 5; i++) { chk = NRF24L01_Check(); if (chk == 0) break; HAL_Delay(100); }
  printf("NRF24L01_Check = %u (%s)\r\n", chk, chk == 0 ? "OK" : "FAIL");

  NRF24L01_RX_Mode();
  printf("RX ready, wait TSTA/ADJ/PING\r\n");

  uint8_t rx[32];
  uint32_t rxCount = 0;

  while (1) {
    if (safe_RxPacket(rx) == 0) {
      if (memcmp(rx, "PING", 4) == 0) {
        rxCount++;
        uint8_t tx[32]; memset(tx, 0, 32);
        char body[24]; int bl = 0;
        for (int i = 0; i < 24 && rx[i] != 0; i++) body[bl++] = (char)rx[i];
        body[bl] = 0;
        snprintf((char*)tx, 32, "PONG:%s C%02XH%uS%02XN%lu", body,
                 NRF24L01_Read_Reg(0x00), NRF24L01_Read_Reg(0x05),
                 NRF24L01_Read_Reg(0x07), (unsigned long)rxCount);
        send_frame(tx);
      } else if (memcmp(rx, "TSTA", 4) == 0) {
        g_st = ST_READY;
        uint8_t tx[32]; memset(tx, 0, 32); memcpy(tx, "TSTB", 4);
        send_frame(tx);
      } else if (memcmp(rx, "TSTE", 4) == 0) {
        g_st = ST_IDLE;
        uint8_t tx[32]; memset(tx, 0, 32); memcpy(tx, "ACKD", 4);
        send_frame(tx);
      } else if (memcmp(rx, "MTR,O", 5) == 0 || memcmp(rx, "MTR,P", 5) == 0) {
        if (g_st == ST_READY) motor_probe(rx[4] == 'P');
      } else if (memcmp(rx, "ADJ,", 4) == 0) {
        /* ADJ 仅跑纠偏规划; 开环/定速闭环必须显式发 MTR,O/P。 */
        uint8_t rc;
        parse_adj(rx);
        printf("ADJ recv phi=%d psi=%d ax=%d\r\n", (int)g_phi, (int)g_psi, (int)g_ax);
        rc = Adj_Execute(g_phi, g_psi, g_ax);
        printf("ADJ done rc=%u\r\n", rc);
        { uint8_t tx[32]; memset(tx, 0, 32); memcpy(tx, "DONE", 4); send_frame(tx); }
        g_st = ST_READY;
      }
    }

  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { while(1); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { while(1); }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
