/**
  ******************************************************************************
  * @file    usart.c
  * @brief   USART配置文件 - 提供所有UART/USART实例的配置代码
  * @details 本文件配置7个串口外设：
  *          - UART5: 遥控器接收（DBUS协议），100kbps，9位数据，偶校验
  *          - UART7: 调试串口/通信，115200bps
  *          - UART8: 调试串口/通信，115200bps
  *          - UART9: 调试串口/通信，115200bps
  *          - USART1: 调试串口/通信，115200bps
  *          - USART2: RS485通信，115200bps，支持方向控制（DE引脚）
  *          - USART10: 调试串口/通信，115200bps
  ******************************************************************************
  * @attention
  * 版权所有 (c) 2025 STMicroelectronics。保留所有权利。
  ******************************************************************************
  */

/* 包含头文件 */
#include "usart.h"

/**
  * UART句柄变量 - 用于存储每个UART实例的配置状态
  */
UART_HandleTypeDef huart5;      /* UART5实例句柄 - 遥控器接收 */
UART_HandleTypeDef huart7;      /* UART7实例句柄 */
UART_HandleTypeDef huart8;      /* UART8实例句柄 */
UART_HandleTypeDef huart9;      /* UART9实例句柄 */
UART_HandleTypeDef huart1;      /* USART1实例句柄 */
UART_HandleTypeDef huart2;      /* USART2实例句柄 - RS485 */
UART_HandleTypeDef huart10;     /* USART10实例句柄 */

/**
  * DMA句柄变量 - 用于UART的DMA传输
  */
DMA_HandleTypeDef hdma_uart5_rx;    /* UART5接收DMA */
DMA_HandleTypeDef hdma_uart7_rx;    /* UART7接收DMA */
DMA_HandleTypeDef hdma_uart7_tx;    /* UART7发送DMA */
DMA_HandleTypeDef hdma_uart8_rx;    /* UART8接收DMA */
DMA_HandleTypeDef hdma_uart8_tx;    /* UART8发送DMA */
DMA_HandleTypeDef hdma_uart9_rx;    /* UART9接收DMA */
DMA_HandleTypeDef hdma_uart9_tx;    /* UART9发送DMA */
DMA_HandleTypeDef hdma_usart1_rx;   /* USART1接收DMA */
DMA_HandleTypeDef hdma_usart1_tx;   /* USART1发送DMA */
DMA_HandleTypeDef hdma_usart10_rx;  /* USART10接收DMA */
DMA_HandleTypeDef hdma_usart10_tx;  /* USART10发送DMA */

/**
  * @brief  UART5初始化函数
  * @details 配置UART5用于遥控器（DBUS协议）接收：
  *          - 波特率：100kbps（DBUS标准波特率）
  *          - 数据位：9位（包含奇偶校验位）
  *          - 停止位：2位
  *          - 校验位：偶校验
  *          - 模式：仅接收（RX）
  *          - 硬件流控：无
  *          - 过采样：16倍过采样
  * @note   DBUS协议特点：
  *         - 波特率100kbps
  *         - 8位数据 + 奇偶校验 + 停止位
  *         - 帧格式：9位数据（8位数据 + 1位校验）
  * @retval None
  */
void MX_UART5_Init(void)
{

  /* UART5实例配置 */
  huart5.Instance = UART5;                            /* 使用UART5外设 */
  huart5.Init.BaudRate = 100000;                      /* 波特率：100kbps */
  huart5.Init.WordLength = UART_WORDLENGTH_9B;        /* 数据位：9位 */
  huart5.Init.StopBits = UART_STOPBITS_2;             /* 停止位：2位 */
  huart5.Init.Parity = UART_PARITY_EVEN;              /* 校验位：偶校验 */
  huart5.Init.Mode = UART_MODE_RX;                    /* 模式：仅接收 */
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;        /* 无硬件流控 */
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;    /* 16倍过采样 */
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;  /* 禁用单比特采样 */
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;   /* 时钟预分频：不分频 */
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;  /* 无高级功能 */

  /* 初始化UART5 */
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }

  /* 配置UART5 FIFO阈值 */
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }

  /* 禁用UART5 FIFO模式 */
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief  UART7初始化函数
  * @details 配置UART7用于调试或通信：
  *          - 波特率：115200bps
  *          - 数据位：8位
  *          - 停止位：1位
  *          - 无校验
  *          - 模式：收发（TX+RX）
  *          - 无硬件流控
  * @retval None
  */
void MX_UART7_Init(void)
{

  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;                      /* 波特率：115200bps */
  huart7.Init.WordLength = UART_WORDLENGTH_8B;        /* 数据位：8位 */
  huart7.Init.StopBits = UART_STOPBITS_1;             /* 停止位：1位 */
  huart7.Init.Parity = UART_PARITY_NONE;              /* 无校验 */
  huart7.Init.Mode = UART_MODE_TX_RX;                 /* 收发模式 */
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief  UART8初始化函数
  * @details 配置UART8用于调试或通信
  * @retval None
  */
void MX_UART8_Init(void)
{

  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart8.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart8, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart8, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart8) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief  UART9初始化函数
  * @details 配置UART9用于调试或通信
  * @retval None
  */
void MX_UART9_Init(void)
{

  huart9.Instance = UART9;
  huart9.Init.BaudRate = 115200;
  huart9.Init.WordLength = UART_WORDLENGTH_8B;
  huart9.Init.StopBits = UART_STOPBITS_1;
  huart9.Init.Parity = UART_PARITY_NONE;
  huart9.Init.Mode = UART_MODE_TX_RX;
  huart9.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart9.Init.OverSampling = UART_OVERSAMPLING_16;
  huart9.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart9.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart9.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart9) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart9, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart9, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart9) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief  USART1初始化函数
  * @details 配置USART1用于调试或通信
  * @retval None
  */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief  USART2初始化函数
  * @details 配置USART2用于RS485通信：
  *          - 波特率：115200bps
  *          - 数据位：8位
  *          - 停止位：1位
  *          - 无校验
  *          - 模式：收发（TX+RX）
  *          - RS485方向控制：使能（DE引脚高电平有效）
  * @note   RS485通信特点：
  *         - 半双工通信，需要方向控制
  *         - DE引脚控制收发方向
  *         - 高电平：发送模式
  *         - 低电平：接收模式
  * @retval None
  */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  /* 使用RS485扩展初始化 - 使能方向控制（DE引脚） */
  if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief  USART10初始化函数
  * @details 配置USART10用于调试或通信
  * @retval None
  */
void MX_USART10_UART_Init(void)
{

  huart10.Instance = USART10;
  huart10.Init.BaudRate = 115200;
  huart10.Init.WordLength = UART_WORDLENGTH_8B;
  huart10.Init.StopBits = UART_STOPBITS_1;
  huart10.Init.Parity = UART_PARITY_NONE;
  huart10.Init.Mode = UART_MODE_TX_RX;
  huart10.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart10.Init.OverSampling = UART_OVERSAMPLING_16;
  huart10.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart10.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart10.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart10, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart10, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart10) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief  UART/USART底层硬件初始化函数（MSP初始化）
  * @param  uartHandle: UART句柄指针，指向要初始化的UART实例
  * @details 该函数由HAL库在HAL_UART_Init()中自动调用，完成以下工作：
  *          1. 配置外设时钟源
  *          2. 使能UART时钟
  *          3. 配置GPIO引脚为复用功能
  *          4. 配置DMA通道（如果启用）
  *          5. 配置NVIC中断
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /* UART5硬件初始化 - 遥控器接收 */
  if(uartHandle->Instance==UART5)
  {
    /* 配置UART5时钟源 */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART5;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* 使能UART5时钟 */
    __HAL_RCC_UART5_CLK_ENABLE();

    /* 使能GPIO端口时钟 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /**
      * UART5 GPIO配置
      * PC12 -> UART5_TX
      * PD2 -> UART5_RX (DBUS接收)
      */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DBUS_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(DBUS_RX_GPIO_Port, &GPIO_InitStruct);

    /**
      * UART5接收DMA配置 - DMA1流0
      * 使用循环模式，持续接收遥控器数据
      */
    hdma_uart5_rx.Instance = DMA1_Stream0;
    hdma_uart5_rx.Init.Request = DMA_REQUEST_UART5_RX;
    hdma_uart5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;  /* 外设到内存 */
    hdma_uart5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart5_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart5_rx.Init.Mode = DMA_CIRCULAR;            /* 循环模式 - 持续接收 */
    hdma_uart5_rx.Init.Priority = DMA_PRIORITY_LOW;    /* 低优先级 */
    hdma_uart5_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart5_rx) != HAL_OK)
    {
      Error_Handler();
    }

    /* 将DMA与UART句柄关联 */
    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart5_rx);

    /* UART5中断配置 */
    HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
  }
  /* UART7硬件初始化 */
  else if(uartHandle->Instance==UART7)
  {
    /* 配置UART7时钟源 */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART7;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_UART7_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /**
      * UART7 GPIO配置
      * PE7 -> UART7_RX
      * PE8 -> UART7_TX
      */
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_UART7;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /**
      * UART7接收DMA配置 - DMA1流7
      * 循环模式，高优先级
      */
    hdma_uart7_rx.Instance = DMA1_Stream7;
    hdma_uart7_rx.Init.Request = DMA_REQUEST_UART7_RX;
    hdma_uart7_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart7_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart7_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart7_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart7_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart7_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart7_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_uart7_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart7_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart7_rx);

    /**
      * UART7发送DMA配置 - DMA2流0
      * 普通模式，高优先级
      */
    hdma_uart7_tx.Instance = DMA2_Stream0;
    hdma_uart7_tx.Init.Request = DMA_REQUEST_UART7_TX;
    hdma_uart7_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;  /* 内存到外设 */
    hdma_uart7_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart7_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart7_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart7_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart7_tx.Init.Mode = DMA_NORMAL;               /* 普通模式 */
    hdma_uart7_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_uart7_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart7_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_uart7_tx);

    HAL_NVIC_SetPriority(UART7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART7_IRQn);
  }
  /* UART8硬件初始化 */
  else if(uartHandle->Instance==UART8)
  {
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART8;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_UART8_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /**
      * UART8 GPIO配置
      * PE0 -> UART8_RX
      * PE1 -> UART8_TX
      */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART8;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* UART8 DMA配置 */
    hdma_uart8_rx.Instance = DMA2_Stream1;
    hdma_uart8_rx.Init.Request = DMA_REQUEST_UART8_RX;
    hdma_uart8_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart8_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart8_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart8_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart8_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart8_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart8_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_uart8_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart8_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart8_rx);

    hdma_uart8_tx.Instance = DMA2_Stream2;
    hdma_uart8_tx.Init.Request = DMA_REQUEST_UART8_TX;
    hdma_uart8_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_uart8_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart8_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart8_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart8_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart8_tx.Init.Mode = DMA_NORMAL;
    hdma_uart8_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_uart8_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart8_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_uart8_tx);

    HAL_NVIC_SetPriority(UART8_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART8_IRQn);
  }
  /* UART9硬件初始化 */
  else if(uartHandle->Instance==UART9)
  {
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART9;
    PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16910CLKSOURCE_D2PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_UART9_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /**
      * UART9 GPIO配置
      * PD14 -> UART9_RX
      * PD15 -> UART9_TX
      */
    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_UART9;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* UART9 DMA配置 */
    hdma_uart9_rx.Instance = DMA2_Stream3;
    hdma_uart9_rx.Init.Request = DMA_REQUEST_UART9_RX;
    hdma_uart9_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart9_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart9_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart9_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart9_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart9_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart9_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_uart9_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart9_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart9_rx);

    hdma_uart9_tx.Instance = DMA2_Stream4;
    hdma_uart9_tx.Init.Request = DMA_REQUEST_UART9_TX;
    hdma_uart9_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_uart9_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart9_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart9_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart9_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart9_tx.Init.Mode = DMA_NORMAL;
    hdma_uart9_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_uart9_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart9_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_uart9_tx);

    HAL_NVIC_SetPriority(UART9_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART9_IRQn);
  }
  /* USART1硬件初始化 */
  else if(uartHandle->Instance==USART1)
  {
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16910CLKSOURCE_D2PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /**
      * USART1 GPIO配置
      * PA9 -> USART1_TX
      * PA10 -> USART1_RX
      */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA配置 */
    hdma_usart1_rx.Instance = DMA1_Stream3;
    hdma_usart1_rx.Init.Request = DMA_REQUEST_USART1_RX;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    hdma_usart1_tx.Instance = DMA1_Stream4;
    hdma_usart1_tx.Init.Request = DMA_REQUEST_USART1_TX;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  }
  /* USART2硬件初始化 - RS485通信 */
  else if(uartHandle->Instance==USART2)
  {
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /**
      * USART2 GPIO配置
      * PD4 -> USART2_DE (RS485方向控制)
      * PD5 -> USART2_TX
      * PD6 -> USART2_RX
      */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  }
  /* USART10硬件初始化 */
  else if(uartHandle->Instance==USART10)
  {
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART10;
    PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16910CLKSOURCE_D2PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_USART10_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /**
      * USART10 GPIO配置
      * PE2 -> USART10_RX
      * PE3 -> USART10_TX
      */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART10;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_USART10;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* USART10 DMA配置 */
    hdma_usart10_rx.Instance = DMA1_Stream5;
    hdma_usart10_rx.Init.Request = DMA_REQUEST_USART10_RX;
    hdma_usart10_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart10_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart10_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart10_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart10_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart10_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart10_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_usart10_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart10_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart10_rx);

    hdma_usart10_tx.Instance = DMA1_Stream6;
    hdma_usart10_tx.Init.Request = DMA_REQUEST_USART10_TX;
    hdma_usart10_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart10_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart10_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart10_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart10_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart10_tx.Init.Mode = DMA_NORMAL;
    hdma_usart10_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_usart10_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart10_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart10_tx);

    HAL_NVIC_SetPriority(USART10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART10_IRQn);
  }
}

/**
  * @brief  UART/USART底层硬件反初始化函数（MSP反初始化）
  * @param  uartHandle: UART句柄指针
  * @details 该函数由HAL库在HAL_UART_DeInit()中自动调用，完成以下工作：
  *          1. 禁用UART时钟
  *          2. 恢复GPIO引脚为默认状态
  *          3. 反初始化DMA通道
  *          4. 禁用NVIC中断
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  /* UART5反初始化 */
  if(uartHandle->Instance==UART5)
  {
    __HAL_RCC_UART5_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);
    HAL_GPIO_DeInit(DBUS_RX_GPIO_Port, DBUS_RX_Pin);

    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_NVIC_DisableIRQ(UART5_IRQn);
  }
  /* UART7反初始化 */
  else if(uartHandle->Instance==UART7)
  {
    __HAL_RCC_UART7_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_7|GPIO_PIN_8);
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);
    HAL_NVIC_DisableIRQ(UART7_IRQn);
  }
  /* UART8反初始化 */
  else if(uartHandle->Instance==UART8)
  {
    __HAL_RCC_UART8_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_0|GPIO_PIN_1);
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);
    HAL_NVIC_DisableIRQ(UART8_IRQn);
  }
  /* UART9反初始化 */
  else if(uartHandle->Instance==UART9)
  {
    __HAL_RCC_UART9_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_14|GPIO_PIN_15);
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);
    HAL_NVIC_DisableIRQ(UART9_IRQn);
  }
  /* USART1反初始化 */
  else if(uartHandle->Instance==USART1)
  {
    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  }
  /* USART2反初始化 */
  else if(uartHandle->Instance==USART2)
  {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6);
  }
  /* USART10反初始化 */
  else if(uartHandle->Instance==USART10)
  {
    __HAL_RCC_USART10_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_2|GPIO_PIN_3);
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);
    HAL_NVIC_DisableIRQ(USART10_IRQn);
  }
}
