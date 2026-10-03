/**
  ******************************************************************************
  * @file    spi.c
  * @brief   SPI配置文件 - 提供SPI外设实例的配置代码
  * @details 本文件配置两个SPI外设：
  *          - SPI2: 用于BMI088惯性测量单元（IMU）通信，全双工模式，支持DMA传输
  *          - SPI6: 用于RGB LED灯带控制，仅发送模式（单向）
  ******************************************************************************
  * @attention
  * 版权所有 (c) 2025 STMicroelectronics。保留所有权利。
  ******************************************************************************
  */

/* 包含头文件 */
#include "spi.h"

/**
  * SPI句柄和DMA句柄变量
  * 这些全局变量用于存储SPI外设的配置状态和DMA传输状态
  */
SPI_HandleTypeDef hspi2;           /* SPI2实例句柄 - 用于BMI088 IMU通信 */
SPI_HandleTypeDef hspi6;           /* SPI6实例句柄 - 用于RGB LED控制 */
DMA_HandleTypeDef hdma_spi2_rx;    /* SPI2接收DMA句柄 */
DMA_HandleTypeDef hdma_spi2_tx;    /* SPI2发送DMA句柄 */

/**
  * @brief  SPI2初始化函数
  * @details 配置SPI2外设用于BMI088 IMU通信：
  *          - 主机模式
  *          - 全双工（2线）模式
  *          - 8位数据宽度
  *          - 时钟极性：高电平空闲
  *          - 时钟相位：第二个边沿采样
  *          - 软件控制片选（NSS）
  *          - 波特率预分频器：32
  *          - MSB先发送
  * @note   SPI模式配置说明：
  *         - CLK Polarity High: 时钟空闲时为高电平
  *         - CLK Phase 2Edge: 数据在第二个时钟边沿采样
  *         - 这对应SPI模式3 (CPOL=1, CPHA=1)
  * @retval None
  */
void MX_SPI2_Init(void)
{

  /* SPI2外设实例配置 */
  hspi2.Instance = SPI2;                               /* 使用SPI2外设 */
  hspi2.Init.Mode = SPI_MODE_MASTER;                   /* 主机模式 */
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;         /* 2线全双工模式（MOSI+MISO） */
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;             /* 8位数据宽度 */
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;          /* 时钟空闲时为高电平 */
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;               /* 数据在第二个时钟边沿采样 */
  hspi2.Init.NSS = SPI_NSS_SOFT;                       /* 软件控制片选信号 */
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;  /* 波特率预分频器：32 */
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;              /* MSB（最高位）先发送 */
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;              /* 禁用TI模式 */
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;  /* 禁用CRC计算 */
  hspi2.Init.CRCPolynomial = 0x0;                      /* CRC多项式（未使用） */
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;         /* 禁用NSS脉冲模式 */
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;       /* NSS低电平有效 */

  /* SPI2扩展配置（STM32H7特有） */
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;  /* FIFO阈值：1个数据 */
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;  /* 主机SS空闲时间 */
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;             /* 禁用IO交换 */

  /* 初始化SPI2，失败时调用错误处理函数 */
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief  SPI6初始化函数
  * @details 配置SPI6外设用于RGB LED灯带控制：
  *          - 主机模式
  *          - 仅发送（单向）模式
  *          - 8位数据宽度
  *          - 时钟极性：低电平空闲
  *          - 时钟相位：第二个边沿采样
  *          - 软件控制片选
  *          - 波特率预分频器：4（高速）
  * @note   SPI模式配置说明：
  *         - CLK Polarity Low: 时钟空闲时为低电平
  *         - CLK Phase 2Edge: 数据在第二个时钟边沿采样
  *         - 这对应SPI模式1 (CPOL=0, CPHA=1)
  *         - RGB LED（如WS2812）通常需要特定的时序，使用SPI模拟
  * @retval None
  */
void MX_SPI6_Init(void)
{

  /* SPI6外设实例配置 */
  hspi6.Instance = SPI6;                               /* 使用SPI6外设 */
  hspi6.Init.Mode = SPI_MODE_MASTER;                   /* 主机模式 */
  hspi6.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;  /* 仅发送模式（单向） */
  hspi6.Init.DataSize = SPI_DATASIZE_8BIT;             /* 8位数据宽度 */
  hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;           /* 时钟空闲时为低电平 */
  hspi6.Init.CLKPhase = SPI_PHASE_2EDGE;               /* 数据在第二个时钟边沿采样 */
  hspi6.Init.NSS = SPI_NSS_SOFT;                       /* 软件控制片选 */
  hspi6.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;  /* 波特率预分频器：4（高速） */
  hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;              /* MSB先发送 */
  hspi6.Init.TIMode = SPI_TIMODE_DISABLE;              /* 禁用TI模式 */
  hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;  /* 禁用CRC计算 */
  hspi6.Init.CRCPolynomial = 0x0;
  hspi6.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;          /* 使能NSS脉冲模式 */
  hspi6.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;       /* NSS低电平有效 */

  /* SPI6扩展配置 */
  hspi6.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi6.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi6.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi6.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi6.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi6.Init.IOSwap = SPI_IO_SWAP_DISABLE;

  /* 初始化SPI6，失败时调用错误处理函数 */
  if (HAL_SPI_Init(&hspi6) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief  SPI底层硬件初始化函数（MSP初始化）
  * @param  spiHandle: SPI句柄指针，指向要初始化的SPI实例
  * @details 该函数由HAL库在HAL_SPI_Init()中自动调用，完成以下工作：
  *          1. 配置外设时钟源
  *          2. 使能SPI时钟
  *          3. 配置GPIO引脚为复用功能
  *          4. 配置DMA通道（如果启用）
  *          5. 配置NVIC中断
  * @note   SPI2引脚分配：
  *         - PC1 -> SPI2_MOSI (主出从入)
  *         - PC2_C -> SPI2_MISO (主入从出)
  *         - PB13 -> SPI2_SCK (时钟)
  * @note   SPI6引脚分配：
  *         - PA5 -> SPI6_SCK
  *         - PA7 -> SPI6_MOSI
  * @retval None
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /* SPI2硬件初始化 */
  if(spiHandle->Instance==SPI2)
  {

    /**
      * 配置SPI2时钟源
      * SPI2使用PLL时钟源，确保稳定的时钟频率
      */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* 使能SPI2时钟 */
    __HAL_RCC_SPI2_CLK_ENABLE();

    /* 使能GPIO端口时钟 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /**
      * SPI2 GPIO配置
      * PC1 -> SPI2_MOSI (主出从入引脚)
      * PC2_C -> SPI2_MISO (主入从出引脚)
      */
    GPIO_InitStruct.Pin = BMI_MOSI_Pin|BMI_MISO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;             /* 复用推挽输出模式 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;                 /* 无上拉/下拉 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;        /* 低速 */
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;          /* 复用功能5：SPI2 */
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /**
      * SPI2时钟引脚配置
      * PB13 -> SPI2_SCK (时钟引脚)
      */
    GPIO_InitStruct.Pin = BMI_SCK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;          /* 复用功能5：SPI2 */
    HAL_GPIO_Init(BMI_SCK_GPIO_Port, &GPIO_InitStruct);

    /**
      * SPI2 DMA配置 - 用于高速数据传输
      * DMA可以减轻CPU负担，在SPI通信时自动传输数据
      */

    /* SPI2接收DMA配置 - DMA1流1 */
    hdma_spi2_rx.Instance = DMA1_Stream1;              /* 使用DMA1流1 */
    hdma_spi2_rx.Init.Request = DMA_REQUEST_SPI2_RX;   /* DMA请求：SPI2接收 */
    hdma_spi2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;  /* 传输方向：外设到内存 */
    hdma_spi2_rx.Init.PeriphInc = DMA_PINC_DISABLE;    /* 外设地址不递增（SPI数据寄存器） */
    hdma_spi2_rx.Init.MemInc = DMA_MINC_ENABLE;        /* 内存地址递增 */
    hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;  /* 外设数据对齐：字节 */
    hdma_spi2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;    /* 内存数据对齐：字节 */
    hdma_spi2_rx.Init.Mode = DMA_NORMAL;               /* 普通模式（非循环） */
    hdma_spi2_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;  /* 非常高优先级 */
    hdma_spi2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE; /* 禁用FIFO模式 */
    if (HAL_DMA_Init(&hdma_spi2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    /* 将DMA与SPI句柄关联 */
    __HAL_LINKDMA(spiHandle,hdmarx,hdma_spi2_rx);

    /* SPI2发送DMA配置 - DMA1流2 */
    hdma_spi2_tx.Instance = DMA1_Stream2;              /* 使用DMA1流2 */
    hdma_spi2_tx.Init.Request = DMA_REQUEST_SPI2_TX;   /* DMA请求：SPI2发送 */
    hdma_spi2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;  /* 传输方向：内存到外设 */
    hdma_spi2_tx.Init.PeriphInc = DMA_PINC_DISABLE;    /* 外设地址不递增 */
    hdma_spi2_tx.Init.MemInc = DMA_MINC_ENABLE;        /* 内存地址递增 */
    hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi2_tx.Init.Mode = DMA_NORMAL;
    hdma_spi2_tx.Init.Priority = DMA_PRIORITY_MEDIUM;  /* 中等优先级 */
    hdma_spi2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi2_tx) != HAL_OK)
    {
      Error_Handler();
    }

    /* 将DMA与SPI句柄关联 */
    __HAL_LINKDMA(spiHandle,hdmatx,hdma_spi2_tx);

    /* SPI2中断配置 - 用于DMA传输完成中断 */
    HAL_NVIC_SetPriority(SPI2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI2_IRQn);
  }
  /* SPI6硬件初始化 */
  else if(spiHandle->Instance==SPI6)
  {

    /**
      * 配置SPI6时钟源
      * SPI6使用D3PCLK1时钟源
      */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI6;
    PeriphClkInitStruct.Spi6ClockSelection = RCC_SPI6CLKSOURCE_D3PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* 使能SPI6时钟 */
    __HAL_RCC_SPI6_CLK_ENABLE();

    /* 使能GPIOA端口时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /**
      * SPI6 GPIO配置
      * PA5 -> SPI6_SCK (时钟引脚)
      * PA7 -> SPI6_MOSI (主出从入引脚)
      */
    GPIO_InitStruct.Pin = RGB_SCK_Pin|RGB_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_SPI6;          /* 复用功能8：SPI6 */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI6中断配置 */
    HAL_NVIC_SetPriority(SPI6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI6_IRQn);
  }
}

/**
  * @brief  SPI底层硬件反初始化函数（MSP反初始化）
  * @param  spiHandle: SPI句柄指针，指向要反初始化的SPI实例
  * @details 该函数由HAL库在HAL_SPI_DeInit()中自动调用，完成以下工作：
  *          1. 禁用SPI时钟
  *          2. 恢复GPIO引脚为默认状态
  *          3. 反初始化DMA通道
  *          4. 禁用NVIC中断
  * @retval None
  */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  /* SPI2反初始化 */
  if(spiHandle->Instance==SPI2)
  {
    /* 禁用SPI2时钟 */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /* 恢复SPI2 GPIO引脚为默认状态 */
    HAL_GPIO_DeInit(GPIOC, BMI_MOSI_Pin|BMI_MISO_Pin);
    HAL_GPIO_DeInit(BMI_SCK_GPIO_Port, BMI_SCK_Pin);

    /* 反初始化SPI2 DMA通道 */
    HAL_DMA_DeInit(spiHandle->hdmarx);                 /* 反初始化接收DMA */
    HAL_DMA_DeInit(spiHandle->hdmatx);                 /* 反初始化发送DMA */

    /* 禁用SPI2中断 */
    HAL_NVIC_DisableIRQ(SPI2_IRQn);
  }
  /* SPI6反初始化 */
  else if(spiHandle->Instance==SPI6)
  {
    /* 禁用SPI6时钟 */
    __HAL_RCC_SPI6_CLK_DISABLE();

    /* 恢复SPI6 GPIO引脚为默认状态 */
    HAL_GPIO_DeInit(GPIOA, RGB_SCK_Pin|RGB_MOSI_Pin);

    /* 禁用SPI6中断 */
    HAL_NVIC_DisableIRQ(SPI6_IRQn);
  }
}
