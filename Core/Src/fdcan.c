/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   FDCAN配置文件 - 提供FDCAN（灵活数据速率CAN）实例的配置代码
  * @details 本文件配置三个FDCAN外设（FDCAN1、FDCAN2、FDCAN3）：
  *          - FDCAN1: 用于高速CAN通信，连接电机控制器
  *          - FDCAN2: 用于第二路CAN通信，连接其他外设
  *          - FDCAN3: 用于第三路CAN通信，连接传感器等
  *          所有FDCAN都配置为经典CAN模式（非FD模式），支持自动重传。
  ******************************************************************************
  * @attention
  * 版权所有 (c) 2025 STMicroelectronics。保留所有权利。
  ******************************************************************************
  */

/* 包含头文件 */
#include "fdcan.h"

/**
  * FDCAN句柄变量 - 用于存储每个FDCAN实例的配置状态
  * 这些是全局变量，供其他模块访问FDCAN外设
  */
FDCAN_HandleTypeDef hfdcan1;  /* FDCAN1实例句柄 */
FDCAN_HandleTypeDef hfdcan2;  /* FDCAN2实例句柄 */
FDCAN_HandleTypeDef hfdcan3;  /* FDCAN3实例句柄 */

/**
  * @brief  FDCAN1初始化函数
  * @details 配置FDCAN1外设的参数：
  *          - 帧格式：经典CAN（非FD模式）
  *          - 模式：正常模式
  *          - 自动重传：使能
  *          - 波特率：由预分频器和时间段计算
  *          - 消息RAM偏移：0（使用起始位置）
  *          - 标准过滤器数量：5个
  *          - 扩展过滤器数量：5个
  *          - 接收FIFO0：32个元素，每元素8字节
  *          - 发送FIFO/队列：32个元素，每元素8字节
  * @note   波特率计算公式：
  *         fCAN = PCLK / (Prescaler * (SyncSeg + TimeSeg1 + TimeSeg2))
  *         当前配置：假设PCLK=80MHz，Prescaler=5，TimeSeg1=14，TimeSeg2=5
  *         波特率 = 80MHz / (5 * (1 + 14 + 5)) = 800kbps
  * @retval None
  */
void MX_FDCAN1_Init(void)
{

  /* FDCAN1外设实例配置 */
  hfdcan1.Instance = FDCAN1;                          /* 使用FDCAN1外设 */
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;     /* 经典CAN帧格式，非FD模式 */
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;              /* 正常收发模式 */
  hfdcan1.Init.AutoRetransmission = ENABLE;           /* 使能自动重传，发送失败时自动重试 */
  hfdcan1.Init.TransmitPause = DISABLE;               /* 禁用发送暂停，连续发送帧之间不暂停 */
  hfdcan1.Init.ProtocolException = DISABLE;           /* 禁用协议异常处理 */

  /* 位时序配置 - 用于设置CAN总线波特率 */
  hfdcan1.Init.NominalPrescaler = 5;                  /* 标称位时序预分频器 */
  hfdcan1.Init.NominalSyncJumpWidth = 1;              /* 同步跳转宽度 */
  hfdcan1.Init.NominalTimeSeg1 = 14;                  /* 时间段1（包含传播段和相位缓冲段1） */
  hfdcan1.Init.NominalTimeSeg2 = 5;                   /* 时间段2（相位缓冲段2） */

  /* 数据段位时序配置（FD模式下使用，经典模式下与标称配置相同） */
  hfdcan1.Init.DataPrescaler = 5;                     /* 数据段预分频器 */
  hfdcan1.Init.DataSyncJumpWidth = 1;                 /* 数据段同步跳转宽度 */
  hfdcan1.Init.DataTimeSeg1 = 14;                     /* 数据段时间段1 */
  hfdcan1.Init.DataTimeSeg2 = 5;                      /* 数据段时间段2 */

  /* 消息RAM配置 */
  hfdcan1.Init.MessageRAMOffset = 0;                  /* 消息RAM偏移量（字节），从起始位置开始 */

  /* 过滤器配置 */
  hfdcan1.Init.StdFiltersNbr = 5;                     /* 标准帧过滤器数量（11位ID） */
  hfdcan1.Init.ExtFiltersNbr = 5;                     /* 扩展帧过滤器数量（29位ID） */

  /* 接收缓冲区配置 */
  hfdcan1.Init.RxFifo0ElmtsNbr = 32;                 /* 接收FIFO0元素数量 */
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8; /* FIFO0每元素8字节数据 */
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;                  /* 接收FIFO1元素数量（未使用） */
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_24;/* FIFO1每元素24字节（未使用） */
  hfdcan1.Init.RxBuffersNbr = 0;                      /* 接收缓冲区数量（未使用专用缓冲区） */
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;    /* 接收缓冲区每元素8字节 */

  /* 发送缓冲区配置 */
  hfdcan1.Init.TxEventsNbr = 0;                       /* 发送事件FIFO元素数量 */
  hfdcan1.Init.TxBuffersNbr = 0;                      /* 发送专用缓冲区数量（未使用） */
  hfdcan1.Init.TxFifoQueueElmtsNbr = 32;             /* 发送FIFO/队列元素数量 */
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION; /* 发送FIFO模式（先进先出） */
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;      /* 发送每元素8字节 */

  /* 初始化FDCAN1，失败时调用错误处理函数 */
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief  FDCAN2初始化函数
  * @details 配置FDCAN2外设，参数与FDCAN1类似，但消息RAM偏移不同
  * @note   FDCAN2的消息RAM偏移为1280字节，因为FDCAN1使用了前面的RAM空间
  * @retval None
  */
void MX_FDCAN2_Init(void)
{

  /* FDCAN2外设实例配置 */
  hfdcan2.Instance = FDCAN2;                          /* 使用FDCAN2外设 */
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;     /* 经典CAN帧格式 */
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;              /* 正常模式 */
  hfdcan2.Init.AutoRetransmission = ENABLE;           /* 使能自动重传 */
  hfdcan2.Init.TransmitPause = DISABLE;               /* 禁用发送暂停 */
  hfdcan2.Init.ProtocolException = DISABLE;           /* 禁用协议异常 */

  /* 位时序配置 */
  hfdcan2.Init.NominalPrescaler = 5;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 14;
  hfdcan2.Init.NominalTimeSeg2 = 5;
  hfdcan2.Init.DataPrescaler = 5;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 14;
  hfdcan2.Init.DataTimeSeg2 = 5;

  /* 消息RAM偏移量 - FDCAN2使用FDCAN1之后的RAM空间 */
  hfdcan2.Init.MessageRAMOffset = 1280;               /* 偏移1280字节 */

  /* 过滤器和缓冲区配置 */
  hfdcan2.Init.StdFiltersNbr = 5;
  hfdcan2.Init.ExtFiltersNbr = 5;
  hfdcan2.Init.RxFifo0ElmtsNbr = 32;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 8;              /* 发送FIFO/队列：8个元素 */
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;

  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief  FDCAN3初始化函数
  * @details 配置FDCAN3外设，使用接收FIFO1而不是FIFO0
  * @note   FDCAN3使用接收FIFO1，与其他两个FDCAN不同
  *         消息RAM偏移为1709字节
  * @retval None
  */
void MX_FDCAN3_Init(void)
{

  /* FDCAN3外设实例配置 */
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = ENABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = DISABLE;

  /* 位时序配置 */
  hfdcan3.Init.NominalPrescaler = 5;
  hfdcan3.Init.NominalSyncJumpWidth = 1;
  hfdcan3.Init.NominalTimeSeg1 = 14;
  hfdcan3.Init.NominalTimeSeg2 = 5;
  hfdcan3.Init.DataPrescaler = 5;
  hfdcan3.Init.DataSyncJumpWidth = 1;
  hfdcan3.Init.DataTimeSeg1 = 14;
  hfdcan3.Init.DataTimeSeg2 = 5;

  /* 消息RAM偏移量 - FDCAN3使用FDCAN2之后的RAM空间 */
  hfdcan3.Init.MessageRAMOffset = 1709;               /* 偏移1709字节 */

  /* 过滤器配置 */
  hfdcan3.Init.StdFiltersNbr = 5;
  hfdcan3.Init.ExtFiltersNbr = 5;

  /* 接收缓冲区配置 - FDCAN3使用FIFO1而不是FIFO0 */
  hfdcan3.Init.RxFifo0ElmtsNbr = 0;                  /* FIFO0未使用 */
  hfdcan3.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.RxFifo1ElmtsNbr = 32;                 /* 使用FIFO1，32个元素 */
  hfdcan3.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.RxBuffersNbr = 0;
  hfdcan3.Init.RxBufferSize = FDCAN_DATA_BYTES_8;

  /* 发送缓冲区配置 */
  hfdcan3.Init.TxEventsNbr = 0;
  hfdcan3.Init.TxBuffersNbr = 0;
  hfdcan3.Init.TxFifoQueueElmtsNbr = 20;             /* 发送FIFO/队列：20个元素 */
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan3.Init.TxElmtSize = FDCAN_DATA_BYTES_8;

  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief  FDCAN时钟使能计数器
  * @details 由于FDCAN1/2/3共享同一个时钟源，使用引用计数来管理时钟使能/禁用
  *          当第一个FDCAN初始化时使能时钟，最后一个FDCAN反初始化时禁用时钟
  */
static uint32_t HAL_RCC_FDCAN_CLK_ENABLED=0;

/**
  * @brief  FDCAN底层硬件初始化函数（MSP初始化）
  * @param  fdcanHandle: FDCAN句柄指针，指向要初始化的FDCAN实例
  * @details 该函数由HAL库在HAL_FDCAN_Init()中自动调用，完成以下工作：
  *          1. 使能FDCAN时钟（使用引用计数）
  *          2. 配置GPIO引脚为复用功能
  *          3. 配置NVIC中断
  * @note   FDCAN引脚分配：
  *         - FDCAN1: PD0(RX), PD1(TX) - 复用功能AF9
  *         - FDCAN2: PB5(RX), PB6(TX) - 复用功能AF9
  *         - FDCAN3: PD12(RX), PD13(TX) - 复用功能AF5
  * @retval None
  */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* FDCAN1硬件初始化 */
  if(fdcanHandle->Instance==FDCAN1)
  {
    /* 使能FDCAN时钟（共享时钟源，使用引用计数） */
    HAL_RCC_FDCAN_CLK_ENABLED++;                      /* 引用计数加1 */
    if(HAL_RCC_FDCAN_CLK_ENABLED==1){                 /* 第一个FDCAN初始化时使能时钟 */
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    /* 使能GPIOD端口时钟 */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**
      * FDCAN1 GPIO配置
      * PD0 -> FDCAN1_RX (接收引脚)
      * PD1 -> FDCAN1_TX (发送引脚)
      */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;           /* 复用推挽输出模式 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;               /* 无上拉/下拉 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;      /* 低速（CAN总线速度有限） */
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;      /* 复用功能9：FDCAN1 */
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* FDCAN1中断配置 */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);     /* 中断优先级5 */
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);              /* 使能FDCAN1中断0 */
  }
  /* FDCAN2硬件初始化 */
  else if(fdcanHandle->Instance==FDCAN2)
  {
    /* 使能FDCAN时钟 */
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if(HAL_RCC_FDCAN_CLK_ENABLED==1){
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    /* 使能GPIOB端口时钟 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**
      * FDCAN2 GPIO配置
      * PB5 -> FDCAN2_RX
      * PB6 -> FDCAN2_TX
      */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;      /* 复用功能9：FDCAN2 */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* FDCAN2中断配置 */
    HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
  }
  /* FDCAN3硬件初始化 */
  else if(fdcanHandle->Instance==FDCAN3)
  {
    /* 使能FDCAN时钟 */
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if(HAL_RCC_FDCAN_CLK_ENABLED==1){
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    /* 使能GPIOD端口时钟 */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**
      * FDCAN3 GPIO配置
      * PD12 -> FDCAN3_RX
      * PD13 -> FDCAN3_TX
      */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_FDCAN3;      /* 复用功能5：FDCAN3（注意与FDCAN1/2不同） */
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* FDCAN3中断配置 */
    HAL_NVIC_SetPriority(FDCAN3_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN3_IT0_IRQn);
  }
}

/**
  * @brief  FDCAN底层硬件反初始化函数（MSP反初始化）
  * @param  fdcanHandle: FDCAN句柄指针，指向要反初始化的FDCAN实例
  * @details 该函数由HAL库在HAL_FDCAN_DeInit()中自动调用，完成以下工作：
  *          1. 禁用FDCAN中断
  *          2. 恢复GPIO引脚为默认状态
  *          3. 禁用FDCAN时钟（使用引用计数）
  * @retval None
  */
void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  /* FDCAN1反初始化 */
  if(fdcanHandle->Instance==FDCAN1)
  {
    /* 禁用FDCAN时钟（引用计数减1） */
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if(HAL_RCC_FDCAN_CLK_ENABLED==0){                 /* 最后一个FDCAN反初始化时禁用时钟 */
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }

    /* 恢复FDCAN1 GPIO引脚为默认状态 */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* 禁用FDCAN1中断 */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  }
  /* FDCAN2反初始化 */
  else if(fdcanHandle->Instance==FDCAN2)
  {
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if(HAL_RCC_FDCAN_CLK_ENABLED==0){
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5|GPIO_PIN_6);
    HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
  }
  /* FDCAN3反初始化 */
  else if(fdcanHandle->Instance==FDCAN3)
  {
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if(HAL_RCC_FDCAN_CLK_ENABLED==0){
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_12|GPIO_PIN_13);
    HAL_NVIC_DisableIRQ(FDCAN3_IT0_IRQn);
  }
}
