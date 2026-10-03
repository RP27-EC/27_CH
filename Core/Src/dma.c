/**
  ******************************************************************************
  * @file    dma.c
  * @brief   DMA配置文件 - 提供所有请求的内存到内存DMA传输的配置代码
  * @details 本文件负责初始化DMA1和DMA2控制器的时钟，并配置所有DMA流的中断优先级
  *          和使能中断。DMA（直接内存访问）用于在不占用CPU的情况下进行高速数据传输。
  ******************************************************************************
  * @attention
  * 版权所有 (c) 2025 STMicroelectronics。
  * 保留所有权利。
  *
  * 本软件的许可条款可在软件组件根目录的LICENSE文件中找到。
  * 如果此软件未附带LICENSE文件，则按原样提供。
  ******************************************************************************
  */

/* 包含头文件 */
#include "dma.h"

/**
  * @brief  DMA控制器初始化函数
  * @details 该函数完成以下初始化工作：
  *          1. 使能DMA1和DMA2控制器的时钟
  *          2. 配置所有DMA流的中断优先级（0-15，0最高）
  *          3. 使能所有DMA流的中断
  * @note   中断优先级说明：
  *         - 优先级0：用于需要快速响应的DMA流（如USART3、SPI等）
  *         - 优先级5：用于一般用途的DMA流
  * @retval None
  */
void MX_DMA_Init(void)
{

  /* 使能DMA1和DMA2控制器的时钟，DMA需要时钟才能工作 */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA中断初始化 - 配置中断优先级并使能中断 */

  /* DMA1流0中断配置 - 用于UART5接收 */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);  /* 设置优先级为5（中等优先级） */
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);           /* 使能DMA1流0中断 */

  /* DMA1流1中断配置 - 用于SPI2接收 */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);  /* 设置优先级为5 */
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

  /* DMA1流2中断配置 - 用于SPI2发送 */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);  /* 设置优先级为5 */
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

  /* DMA1流3中断配置 - 用于USART1接收（高优先级） */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);  /* 设置优先级为0（最高优先级） */
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

  /* DMA1流4中断配置 - 用于USART1发送（高优先级） */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);  /* 设置优先级为0（最高优先级） */
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

  /* DMA1流5中断配置 - 用于USART10接收（高优先级） */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);  /* 设置优先级为0（最高优先级） */
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

  /* DMA1流6中断配置 - 用于USART10发送（高优先级） */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);  /* 设置优先级为0（最高优先级） */
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

  /* DMA1流7中断配置 - 用于UART7接收（高优先级） */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);  /* 设置优先级为0（最高优先级） */
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

  /* DMA2流0中断配置 - 用于UART7发送 */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);  /* 设置优先级为0（最高优先级） */
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

  /* DMA2流1中断配置 - 用于UART8接收 */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);  /* 设置优先级为0（最高优先级） */
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

  /* DMA2流2中断配置 - 用于UART8发送 */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);  /* 设置优先级为0（最高优先级） */
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

  /* DMA2流3中断配置 - 用于UART9接收 */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);  /* 设置优先级为0（最高优先级） */
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

  /* DMA2流4中断配置 - 用于UART9发送 */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);  /* 设置优先级为0（最高优先级） */
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

}
