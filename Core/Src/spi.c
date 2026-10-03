/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   本文件提供SPI（串行外设接口）实例的配置代码
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
#include "spi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* SPI1句柄实例，用于控制SPI1外设 */
SPI_HandleTypeDef hspi1;

/* SPI1的DMA接收和发送句柄，用于DMA传输 */
DMA_HandleTypeDef hdma_spi1_rx;  /* SPI1 DMA接收句柄 */
DMA_HandleTypeDef hdma_spi1_tx;  /* SPI1 DMA发送句柄 */

/**
  * @brief  SPI1初始化函数
  * @note   配置SPI1为主机模式，双线全双工，8位数据大小
  *         时钟极性高电平有效，时钟相位第二个边沿采样
  *         软件片选（NSS），波特率预分频器16，MSB先发
  * @param  None
  * @retval None
  */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */

  /* SPI1实例配置 */
  hspi1.Instance = SPI1;  /* 选择SPI1外设 */

  /**
    * SPI配置参数：
    * Mode: 主机模式，SPI1作为主设备
    * Direction: 双线全双工模式，可以同时发送和接收
    * DataSize: 8位数据大小
    * CLKPolarity: 时钟极性高电平（CPOL=1）
    * CLKPhase: 时钟相位第二个边沿采样（CPHA=1）
    * 模式为SPI Mode 3（CPOL=1, CPHA=1）
    */
  hspi1.Init.Mode = SPI_MODE_MASTER;          /* 主机模式 */
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;  /* 双线全双工 */
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;    /* 8位数据 */
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH; /* 时钟极性高电平 */
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;      /* 时钟相位第二个边沿 */

  /**
    * SPI功能配置：
    * NSS: 软件片选管理，使用GPIO手动控制片选引脚
    * BaudRatePrescaler: 波特率预分频器 = 16
    * FirstBit: MSB先发（最高位优先）
    * TIMode: 禁用TI帧格式
    * CRCCalculation: 禁用CRC计算
    */
  hspi1.Init.NSS = SPI_NSS_SOFT;              /* 软件片选 */
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;  /* 预分频器 = 16 */
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;     /* MSB先发 */
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;     /* 禁用TI模式 */
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;  /* 禁用CRC */
  hspi1.Init.CRCPolynomial = 10;              /* CRC多项式（未使用） */

  /* 初始化SPI1，如果失败则调用错误处理函数 */
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief  SPI底层初始化函数（MSP初始化）
  * @note   由HAL_SPI_Init()自动调用，负责SPI外设的底层硬件初始化
  *         包括时钟使能、GPIO配置、DMA配置等
  * @param  spiHandle: SPI句柄指针，包含SPI配置信息
  * @retval None
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};  /* GPIO初始化结构体 */

  /* 检查是哪个SPI实例需要初始化 */
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */

    /* SPI1时钟使能 */
    __HAL_RCC_SPI1_CLK_ENABLE();

    /* 使能GPIOB和GPIOA时钟，因为SPI1引脚在这些端口上 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /**SPI1 GPIO配置
    PB4     ------> SPI1_MISO（主入从出）
    PB3     ------> SPI1_SCK（时钟）
    PA7     ------> SPI1_MOSI（主出从入）
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_3;  /* 选择PB4和PB3引脚 */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;        /* 复用推挽输出模式 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;            /* 无上下拉 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  /* 非常高的速度（100MHz） */
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;     /* 复用功能选择SPI1 */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);        /* 初始化GPIO */

    GPIO_InitStruct.Pin = GPIO_PIN_7;              /* 选择PA7引脚 */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;        /* 复用推挽输出模式 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;            /* 无上下拉 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  /* 非常高的速度（100MHz） */
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;     /* 复用功能选择SPI1 */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);        /* 初始化GPIO */

    /**
      * SPI1 DMA初始化
      * 配置DMA用于SPI1的数据传输，提高传输效率
      */

    /* SPI1 RX DMA初始化（接收） */
    hdma_spi1_rx.Instance = DMA2_Stream0;  /* 使用DMA2的Stream0 */
    hdma_spi1_rx.Init.Channel = DMA_CHANNEL_3;  /* DMA通道3 */
    hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;  /* 外设到内存（接收） */
    hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;     /* 外设地址不递增 */
    hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;         /* 内存地址递增 */
    hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;  /* 外设数据对齐：字节 */
    hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;    /* 内存数据对齐：字节 */
    hdma_spi1_rx.Init.Mode = DMA_NORMAL;                 /* 普通模式（非循环） */
    hdma_spi1_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH; /* 非常高的优先级 */
    hdma_spi1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;   /* 禁用FIFO模式 */

    /* 初始化DMA接收，如果失败则调用错误处理函数 */
    if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    /* 将DMA接收句柄链接到SPI句柄 */
    __HAL_LINKDMA(spiHandle,hdmarx,hdma_spi1_rx);

    /* SPI1 TX DMA初始化（发送） */
    hdma_spi1_tx.Instance = DMA2_Stream3;  /* 使用DMA2的Stream3 */
    hdma_spi1_tx.Init.Channel = DMA_CHANNEL_3;  /* DMA通道3 */
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;  /* 内存到外设（发送） */
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;      /* 外设地址不递增 */
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;          /* 内存地址递增 */
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;  /* 外设数据对齐：字节 */
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;    /* 内存数据对齐：字节 */
    hdma_spi1_tx.Init.Mode = DMA_NORMAL;                 /* 普通模式（非循环） */
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_HIGH;      /* 高优先级 */
    hdma_spi1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;   /* 禁用FIFO模式 */

    /* 初始化DMA发送，如果失败则调用错误处理函数 */
    if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    /* 将DMA发送句柄链接到SPI句柄 */
    __HAL_LINKDMA(spiHandle,hdmatx,hdma_spi1_tx);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
}

/**
  * @brief  SPI底层反初始化函数（MSP反初始化）
  * @note   由HAL_SPI_DeInit()自动调用，负责SPI外设的底层硬件反初始化
  *         包括时钟禁用、GPIO恢复、DMA反初始化等
  * @param  spiHandle: SPI句柄指针，包含SPI配置信息
  * @retval None
  */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  /* 检查是哪个SPI实例需要反初始化 */
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */

    /* 禁用SPI1外设时钟 */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO配置恢复
    PB4     ------> SPI1_MISO
    PB3     ------> SPI1_SCK
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4|GPIO_PIN_3);  /* 恢复PB4和PB3引脚 */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);              /* 恢复PA7引脚 */

    /* SPI1 DMA反初始化 */
    HAL_DMA_DeInit(spiHandle->hdmarx);  /* 反初始化DMA接收 */
    HAL_DMA_DeInit(spiHandle->hdmatx);  /* 反初始化DMA发送 */

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */