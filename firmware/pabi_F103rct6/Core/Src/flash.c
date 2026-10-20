/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "flash.h"

#include <stdio.h>
#include "usart.h"

// 写入数组32位对齐！
static FLASH_EraseInitTypeDef EraseInitStruct;
u32 PAGEError = 0;
 /**********************************************************************************
  * 函数功能: 页擦除
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无 
  */
void Flash_Erase(void)
{  	
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
    EraseInitStruct.NbPages     = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / STM_SECTOR_SIZE;
    
     if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)  
	 {
		 HAL_FLASH_Lock();  
		 printf(" Error...1\r\n");
     Error_Handler();
	 }
}
 /**********************************************************************************
  * 函数功能: 数据写入
  * 输入参数: 写入数据缓存数组指针、写入数据数
  * 返 回 值: 无
  * 说    明：无 
  */    
void Flash_Write(u32 *pBuffer,u32  NumToWrite)
{
 
    u16  i=0;
    u32 Address = FLASH_USER_START_ADDR;
    HAL_FLASH_Unlock();	    //解锁
    Flash_Erase();         //先擦除
                            //再写入
    printf(" 擦除完成，准备写入......\r\n");
     while ( (Address < FLASH_USER_END_ADDR) && (i<NumToWrite)  )    
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, pBuffer[i]) == HAL_OK)
        {
            Address = Address + 4;  //地址后移4个字节
            i++;
        }
        else
		{  
			printf(" Error...2\r\n"); 
            Error_Handler( );            
		}
    }
  
    HAL_FLASH_Lock();   //上锁
 
 
}
 
 /**********************************************************************************
  * 函数功能: 数据读取
  * 输入参数: 读取数据缓存数组指针、读出数据数
  * 返 回 值: 无
  * 说    明：无
  */
void Flash_Read(u32  *pBuffer,u32  NumToRead)
{
    u16  i=0;
    u32 Address = FLASH_USER_START_ADDR;
    
    while ( (Address < FLASH_USER_END_ADDR) && (i<NumToRead)  )
  {
    pBuffer[i++]= *(__IO u32 *)Address;  
    Address = Address + 4;   //地址后移4个字节
  }
  
}




