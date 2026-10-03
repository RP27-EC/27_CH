/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   本文件提供USART（通用同步异步收发器）实例的配置代码
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
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* USART句柄实例 */
UART_HandleTypeDef huart1;   /* USART1句柄 */
UART_HandleTypeDef huart3;   /* USART3句柄 */
UART_HandleTypeDef huart6;   /* USART6句柄 */

/* USART的DMA接收和发送句柄，用于DMA传输 */
DMA_HandleTypeDef hdma_usart1_rx;  /* USART1 DMA接收句柄 */
DMA_HandleTypeDef hdma_usart1_tx;  /* USART1 DMA发送句柄 */
DMA_HandleTypeDef hdma_usart3_rx;  /* USART3 DMA接收句柄 */
DMA_HandleTypeDef hdma_usart6_rx;  /* USART6 DMA接收句柄 */
DMA_HandleTypeDef hdma_usart6_tx;  /* USART6 DMA发送句柄 */

/**
  * @brief  USART1初始化函数
  * @note   配置USART1为2Mbps波特率，8位数据，1位停止位，无校验位
  *         用于高速串口通信，可能连接USB转串口模块
  * @param  None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */

  /* USART1实例配置 */
  huart1.Instance = USART1;  /* 选择USART1外设 */

  /**
    * USART1配置参数：
    * BaudRate: 波特率 = 2000000（2Mbps）
    * WordLength: 数据位长度 = 8位
    * StopBits: 停止位 = 1位
    * Parity: 校验位 = 无校验
    * Mode: 模式 = 收发模式（TX和RX都使能）
    * HwFlowCtl: 硬件流控制 = 无
    * OverSampling: 过采样 = 16倍过采样
    */
  huart1.Init.BaudRate = 2000000;               /* 波特率 = 2Mbps */
  huart1.Init.WordLength = UART_WORDLENGTH_8B;  /* 8位数据 */
  huart1.Init.StopBits = UART_STOPBITS_1;       /* 1位停止位 */
  huart1.Init.Parity = UART_PARITY_NONE;        /* 无校验 */
  huart1.Init.Mode = UART_MODE_TX_RX;           /* 收发模式 */
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;  /* 无硬件流控制 */
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;  /* 16倍过采样 */

  /* 初始化USART1，如果失败则调用错误处理函数 */
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief  USART3初始化函数
  * @note   配置USART3为100kbps波特率，8位数据，1位停止位，偶校验位
  *         用于遥控器通信（SBUS协议），SBUS使用偶校验
  * @param  None
  * @retval None
  */
void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */

  /* USART3实例配置 */
  huart3.Instance = USART3;  /* 选择USART3外设 */

  /**
    * USART3配置参数：
    * BaudRate: 波特率 = 100000（100kbps）
    * WordLength: 数据位长度 = 8位
    * StopBits: 停止位 = 1位
    * Parity: 校验位 = 偶校验（SBUS协议要求）
    * Mode: 模式 = 收发模式（TX和RX都使能）
    * HwFlowCtl: 硬件流控制 = 无
    * OverSampling: 过采样 = 16倍过采样
    */
  huart3.Init.BaudRate = 100000;                /* 波特率 = 100kbps */
  huart3.Init.WordLength = UART_WORDLENGTH_8B;  /* 8位数据 */
  huart3.Init.StopBits = UART_STOPBITS_1;       /* 1位停止位 */
  huart3.Init.Parity = UART_PARITY_EVEN;        /* 偶校验（SBUS协议） */
  huart3.Init.Mode = UART_MODE_TX_RX;           /* 收发模式 */
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;  /* 无硬件流控制 */
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;  /* 16倍过采样 */

  /* 初始化USART3，如果失败则调用错误处理函数 */
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief  USART6初始化函数
  * @note   配置USART6为921600波特率，8位数据，1位停止位，无校验位
  *         用于高速串口通信，可能连接其他高速设备
  * @param  None
  * @retval None
  */
void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */

  /* USART6实例配置 */
  huart6.Instance = USART6;  /* 选择USART6外设 */

  /**
    * USART6配置参数：
    * BaudRate: 波特率 = 921600
    * WordLength: 数据位长度 = 8位
    * StopBits: 停止位 = 1位
    * Parity: 校验位 = 无校验
    * Mode: 模式 = 收发模式（TX和RX都使能）
    * HwFlowCtl: 硬件流控制 = 无
    * OverSampling: 过采样 = 16倍过采样
    */
  huart6.Init.BaudRate = 921600;                /* 波特率 = 921600 */
  huart6.Init.WordLength = UART_WORDLENGTH_8B;  /* 8位数据 */
  huart6.Init.StopBits = UART_STOPBITS_1;       /* 1位停止位 */
  huart6.Init.Parity = UART_PARITY_NONE;        /* 无校验 */
  huart6.Init.Mode = UART_MODE_TX_RX;           /* 收发模式 */
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;  /* 无硬件流控制 */
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;  /* 16倍过采样 */

  /* 初始化USART6，如果失败则调用错误处理函数 */
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief  USART底层初始化函数（MSP初始化）
  * @note   由HAL_UART_Init()自动调用，负责USART外设的底层硬件初始化
  *         包括时钟使能、GPIO配置、DMA配置、中断配置等
  * @param  uartHandle: USART句柄指针，包含USART配置信息
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};  /* GPIO初始化结构体 */

  /* 检查是哪个USART实例需要初始化 */
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */

    /* USART1时钟使能 */
    __HAL_RCC_USART1_CLK_ENABLE();

    /* 使能GPIOB和GPIOA时钟，因为USART1引脚在这些端口上 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /**USART1 GPIO配置
    PB7     ------> USART1_RX（接收引脚）
    PA9     ------> USART1_TX（发送引脚）
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7;              /* 选择PB7引脚（RX） */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;        /* 复用推挽输出模式 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;            /* 无上下拉 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  /* 非常高的速度（100MHz） */
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;   /* 复用功能选择USART1 */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);        /* 初始化GPIO */

    GPIO_InitStruct.Pin = GPIO_PIN_9;              /* 选择PA9引脚（TX） */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;        /* 复用推挽输出模式 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;            /* 无上下拉 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  /* 非常高的速度（100MHz） */
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;   /* 复用功能选择USART1 */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);        /* 初始化GPIO */

    /**
      * USART1 DMA初始化
      * 配置DMA用于USART1的数据传输，提高传输效率
      */

    /* USART1 RX DMA初始化（接收） */
    hdma_usart1_rx.Instance = DMA2_Stream2;  /* 使用DMA2的Stream2 */
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;  /* DMA通道4 */
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;  /* 外设到内存（接收） */
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;     /* 外设地址不递增 */
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;         /* 内存地址递增 */
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;  /* 外设数据对齐：字节 */
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;    /* 内存数据对齐：字节 */
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;              /* 循环模式（连续接收） */
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;     /* 高优先级 */
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;  /* 禁用FIFO模式 */

    /* 初始化DMA接收，如果失败则调用错误处理函数 */
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    /* 将DMA接收句柄链接到USART句柄 */
    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1 TX DMA初始化（发送） */
    hdma_usart1_tx.Instance = DMA2_Stream7;  /* 使用DMA2的Stream7 */
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;  /* DMA通道4 */
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;  /* 内存到外设（发送） */
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;      /* 外设地址不递增 */
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;          /* 内存地址递增 */
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;  /* 外设数据对齐：字节 */
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;    /* 内存数据对齐：字节 */
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;                 /* 普通模式（非循环） */
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_HIGH;      /* 高优先级 */
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;   /* 禁用FIFO模式 */

    /* 初始化DMA发送，如果失败则调用错误处理函数 */
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    /* 将DMA发送句柄链接到USART句柄 */
    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1中断初始化 */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);  /* 设置中断优先级 */
    HAL_NVIC_EnableIRQ(USART1_IRQn);           /* 使能中断 */

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */

    /* USART3时钟使能 */
    __HAL_RCC_USART3_CLK_ENABLE();

    /* 使能GPIOC时钟，因为USART3引脚在GPIOC上 */
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /**USART3 GPIO配置
    PC11     ------> USART3_RX（接收引脚）
    PC10     ------> USART3_TX（发送引脚）
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_10;  /* 选择PC11和PC10引脚 */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;          /* 复用推挽输出模式 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;              /* 无上下拉 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  /* 非常高的速度（100MHz） */
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;     /* 复用功能选择USART3 */
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);          /* 初始化GPIO */

    /**
      * USART3 DMA初始化（仅接收）
      * SBUS协议通常只需要接收数据
      */

    /* USART3 RX DMA初始化（接收） */
    hdma_usart3_rx.Instance = DMA1_Stream1;  /* 使用DMA1的Stream1 */
    hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;  /* DMA通道4 */
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;  /* 外设到内存（接收） */
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;     /* 外设地址不递增 */
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;         /* 内存地址递增 */
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;  /* 外设数据对齐：字节 */
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;    /* 内存数据对齐：字节 */
    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;              /* 循环模式（连续接收） */
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_HIGH;     /* 高优先级 */
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;  /* 禁用FIFO模式 */

    /* 初始化DMA接收，如果失败则调用错误处理函数 */
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    /* 将DMA接收句柄链接到USART句柄 */
    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

    /* USART3中断初始化 */
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);  /* 设置中断优先级 */
    HAL_NVIC_EnableIRQ(USART3_IRQn);           /* 使能中断 */

  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */

    /* USART6时钟使能 */
    __HAL_RCC_USART6_CLK_ENABLE();

    /* 使能GPIOG时钟，因为USART6引脚在GPIOG上 */
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /**USART6 GPIO配置
    PG14     ------> USART6_TX（发送引脚）
    PG9     ------> USART6_RX（接收引脚）
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_9;  /* 选择PG14和PG9引脚 */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;         /* 复用推挽输出模式 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;             /* 无上下拉 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  /* 非常高的速度（100MHz） */
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;    /* 复用功能选择USART6 */
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);         /* 初始化GPIO */

    /**
      * USART6 DMA初始化
      * 配置DMA用于USART6的数据传输，包括接收和发送
      */

    /* USART6 RX DMA初始化（接收） */
    hdma_usart6_rx.Instance = DMA2_Stream1;  /* 使用DMA2的Stream1 */
    hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;  /* DMA通道5 */
    hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;  /* 外设到内存（接收） */
    hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;     /* 外设地址不递增 */
    hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;         /* 内存地址递增 */
    hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;  /* 外设数据对齐：字节 */
    hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;    /* 内存数据对齐：字节 */
    hdma_usart6_rx.Init.Mode = DMA_CIRCULAR;              /* 循环模式（连续接收） */
    hdma_usart6_rx.Init.Priority = DMA_PRIORITY_HIGH;     /* 高优先级 */
    hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;  /* 禁用FIFO模式 */

    /* 初始化DMA接收，如果失败则调用错误处理函数 */
    if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK)
    {
      Error_Handler();
    }

    /* 将DMA接收句柄链接到USART句柄 */
    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart6_rx);

    /* USART6 TX DMA初始化（发送） */
    hdma_usart6_tx.Instance = DMA2_Stream6;  /* 使用DMA2的Stream6 */
    hdma_usart6_tx.Init.Channel = DMA_CHANNEL_5;  /* DMA通道5 */
    hdma_usart6_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;  /* 内存到外设（发送） */
    hdma_usart6_tx.Init.PeriphInc = DMA_PINC_DISABLE;      /* 外设地址不递增 */
    hdma_usart6_tx.Init.MemInc = DMA_MINC_ENABLE;          /* 内存地址递增 */
    hdma_usart6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;  /* 外设数据对齐：字节 */
    hdma_usart6_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;    /* 内存数据对齐：字节 */
    hdma_usart6_tx.Init.Mode = DMA_NORMAL;                 /* 普通模式（非循环） */
    hdma_usart6_tx.Init.Priority = DMA_PRIORITY_LOW;       /* 低优先级 */
    hdma_usart6_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;   /* 禁用FIFO模式 */

    /* 初始化DMA发送，如果失败则调用错误处理函数 */
    if (HAL_DMA_Init(&hdma_usart6_tx) != HAL_OK)
    {
      Error_Handler();
    }

    /* 将DMA发送句柄链接到USART句柄 */
    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart6_tx);

    /* USART6中断初始化 */
    HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);  /* 设置中断优先级 */
    HAL_NVIC_EnableIRQ(USART6_IRQn);           /* 使能中断 */

  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

/**
  * @brief  USART底层反初始化函数（MSP反初始化）
  * @note   由HAL_UART_DeInit()自动调用，负责USART外设的底层硬件反初始化
  *         包括时钟禁用、GPIO恢复、DMA反初始化、中断禁用等
  * @param  uartHandle: USART句柄指针，包含USART配置信息
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  /* 检查是哪个USART实例需要反初始化 */
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */

    /* 禁用USART1外设时钟 */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO配置恢复
    PB7     ------> USART1_RX
    PA9     ------> USART1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);  /* 恢复PB7引脚 */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9);  /* 恢复PA9引脚 */

    /* USART1 DMA反初始化 */
    HAL_DMA_DeInit(uartHandle->hdmarx);  /* 反初始化DMA接收 */
    HAL_DMA_DeInit(uartHandle->hdmatx);  /* 反初始化DMA发送 */

    /* USART1中断禁用 */
    HAL_NVIC_DisableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */

    /* 禁用USART3外设时钟 */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO配置恢复
    PC11     ------> USART3_RX
    PC10     ------> USART3_TX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_11|GPIO_PIN_10);  /* 恢复PC11和PC10引脚 */

    /* USART3 DMA反初始化 */
    HAL_DMA_DeInit(uartHandle->hdmarx);  /* 反初始化DMA接收 */

    /* USART3中断禁用 */
    HAL_NVIC_DisableIRQ(USART3_IRQn);

  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */

    /* 禁用USART6外设时钟 */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO配置恢复
    PG14     ------> USART6_TX
    PG9     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_14|GPIO_PIN_9);  /* 恢复PG14和PG9引脚 */

    /* USART6 DMA反初始化 */
    HAL_DMA_DeInit(uartHandle->hdmarx);  /* 反初始化DMA接收 */
    HAL_DMA_DeInit(uartHandle->hdmatx);  /* 反初始化DMA发送 */

    /* USART6中断禁用 */
    HAL_NVIC_DisableIRQ(USART6_IRQn);

  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */