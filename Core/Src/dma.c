/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.c
  * @brief   本文件提供所有请求的内存到内存DMA传输的配置代码
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @brief  DMA控制器初始化函数
  * @note   使能DMA1和DMA2时钟，并配置所有需要的DMA中断
  *         STM32F4系列有DMA1和DMA2两个DMA控制器，每个有8个数据流
  * @param  None
  * @retval None
  */
void MX_DMA_Init(void)
{

  /**
    * DMA控制器时钟使能
    * STM32F4系列有两个DMA控制器：DMA1和DMA2
    * DMA1有8个数据流，DMA2也有8个数据流
    * 每个数据流可以连接到不同的外设
    */
  __HAL_RCC_DMA2_CLK_ENABLE();  /* 使能DMA2时钟 */
  __HAL_RCC_DMA1_CLK_ENABLE();  /* 使能DMA1时钟 */

  /**
    * DMA中断初始化
    * 为每个需要的DMA数据流配置中断优先级并使能中断
    * 优先级设置为5，子优先级为0
    * 中断用于处理DMA传输完成、半传输完成等事件
    */

  /* DMA1_Stream1中断配置 - 通常用于USART3_RX */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);  /* 设置中断优先级 */
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);           /* 使能中断 */

  /* DMA2_Stream0中断配置 - 通常用于SPI1_RX */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

  /* DMA2_Stream1中断配置 - 通常用于USART6_RX */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

  /* DMA2_Stream2中断配置 - 通常用于USART1_RX */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

  /* DMA2_Stream3中断配置 - 通常用于SPI1_TX */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

  /* DMA2_Stream6中断配置 - 通常用于USART6_TX */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

  /* DMA2_Stream7中断配置 - 通常用于USART1_TX */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */