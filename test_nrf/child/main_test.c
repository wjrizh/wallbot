/* 子机 NRF24L01 回环测试固件
 * 收到 "PING" 帧 -> 回 "PONG:" + 原内容前24字节; 其他帧忽略
 * 串口 USART1 115200, 用于观察
 */
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>

#define CHILD_VERBOSE 0   /* 1=打印每帧 hex(会阻塞 ~15ms, 影响 ping-pong 时序) */

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

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();

  /* gpio.c 把 PA6 配成了 OUTPUT_PP, 但它是 NRF24L01 的 IRQ 输出引脚,
   * 必须为 INPUT, 否则 spi.c 里 while(ReadPin(PA6)!=0) 立即退出, TX 时序错乱 */
  {
    GPIO_InitTypeDef gi = {0};
    gi.Pin = GPIO_PIN_6;
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gi);
  }

  printf("\r\n=== NRF24L01 CHILD PING TEST ===\r\n");

  NRF24L01_Init();
  HAL_Delay(50);

  uint8_t chk = 0xFF;
  for (int i = 0; i < 5; i++) {
    chk = NRF24L01_Check();
    if (chk == 0) break;
    HAL_Delay(100);
  }
  printf("NRF24L01_Check = %u (%s)\r\n", chk, chk == 0 ? "OK" : "FAIL");

  uint8_t cfg = NRF24L01_Read_Reg(0x00);
  uint8_t ch  = NRF24L01_Read_Reg(0x05);
  uint8_t st  = NRF24L01_Read_Reg(0x07);
  printf("CONFIG=%02X RF_CH=%u STATUS=%02X\r\n", cfg, ch, st);

  NRF24L01_RX_Mode();
  printf("RX mode ready, waiting for PING...\r\n");

  uint8_t rx[32];
  uint32_t rxCount = 0;

  while (1) {
    if (safe_RxPacket(rx) == 0) {
      if (memcmp(rx, "PING", 4) != 0) continue;
      rxCount++;
#if CHILD_VERBOSE
      printf("[RX #%lu] ", (unsigned long)rxCount);
      for (int i = 0; i < 32; i++) printf("%02X ", rx[i]);
      printf("\r\n");
#endif

      /* 回包携带子机自身诊断: PONG:<原内容> C=<CONFIG> H=<RF_CH> S=<STATUS> N=<计数> */
      uint8_t tx[32];
      memset(tx, 0, 32);
      {
        char body[24];
        int bl = 0;
        for (int i = 0; i < 24 && rx[i] != 0; i++) body[bl++] = (char)rx[i];
        body[bl] = 0;
        uint8_t c2 = NRF24L01_Read_Reg(0x00);
        uint8_t h2 = NRF24L01_Read_Reg(0x05);
        uint8_t s2 = NRF24L01_Read_Reg(0x07);
        snprintf((char*)tx, 32, "PONG:%s C%02XH%uS%02XN%lu",
                 body, c2, h2, s2, (unsigned long)rxCount);
      }

      NRF24L01_TX_Mode();
      HAL_Delay(2);
      uint8_t sta = safe_TxPacket(tx);
      NRF24L01_RX_Mode();

#if CHILD_VERBOSE
      printf("[TX] status=%02X (%s)\r\n", sta,
             sta == TX_OK ? "TX_OK" : (sta == MAX_TX ? "MAX_TX" : "OTHER"));
#endif
      (void)sta;
      NRF24L01_Write_Reg(FLUSH_RX, 0xff);
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
