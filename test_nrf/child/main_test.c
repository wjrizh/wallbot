/* 子机 NRF24L01 视觉纠偏测试固件
 * 支持: PING->PONG / TSTA->TSTB / ADJ,phi,psi,ax -> 3s 模拟运动(每100ms一帧'J'遥测) -> DONE / TSTE->ACKD
 * 不驱动真电机, 只回虚拟遥测. 串口 USART1 115200
 */
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
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
      } else if (memcmp(rx, "ADJ,", 4) == 0) {
        parse_adj(rx);
        float mag = (g_phi < 0 ? -g_phi : g_phi) + (g_psi < 0 ? -g_psi : g_psi);
        g_target_mm = (int16_t)(mag * 10.0f);
        if (g_target_mm == 0) g_target_mm = 100;
        g_dir = (g_phi + g_psi >= 0) ? 1 : -1;
        g_st = ST_RUN;
        g_run_start = HAL_GetTick();
        g_telem_next = TELEM_STEP;
        g_telem_n = 0;
      }
    }

    if (g_st == ST_RUN) {
      uint32_t el = HAL_GetTick() - g_run_start;
      if (g_telem_n < TELEM_TOTAL && el >= g_telem_next) {
        uint8_t pct = (uint8_t)(((uint32_t)(g_telem_n + 1) * 100) / TELEM_TOTAL);
        int16_t a = (int16_t)((int32_t)g_target_mm * (g_telem_n + 1) / TELEM_TOTAL);
        send_telem(1, g_dir, a, a, g_target_mm, pct, 0);
        g_telem_n++;
        g_telem_next += TELEM_STEP;
      }
      if (el >= RUN_MS) {
        uint8_t tx[32]; memset(tx, 0, 32); memcpy(tx, "DONE", 4);
        send_frame(tx);
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
