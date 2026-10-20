/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_H__
#define __FLASH_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"


#include "stm32f1xx_hal_flash_ex.h" 

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
 
 
#define STM32_FLASH_SIZE 	64 	 	//所选STM32的FLASH容量大小(单位为K) RCT6 为256K 每页1KB
    #if     STM32_FLASH_SIZE <= 256      //设置扇区大小
    #define STM_SECTOR_SIZE     1024    //1K字节
    #else 
    #define STM_SECTOR_SIZE	    2048    //2K字节
#endif
 
#define STM32_FLASH_BASE            0x08000000 		//STM32 FLASH的起始地址
#define FLASH_USER_START_ADDR   ( STM32_FLASH_BASE + STM_SECTOR_SIZE * 62 ) //写Flash的地址，这里从第62页开始
#define FLASH_USER_END_ADDR     ( STM32_FLASH_BASE + STM_SECTOR_SIZE * 64 ) //写Flash的地址，这里以第64页结束
 
 
void Flash_Erase(void); 
void Flash_Write(u32 *pBuffer,u32 NumToWrite);
void Flash_Read(u32 *pBuffer,u32 NumToRead);

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

