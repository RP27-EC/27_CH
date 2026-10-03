/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   本文件提供CAN（控制器局域网）实例的配置代码
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
#include "can.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* CAN1和CAN2的句柄实例，用于控制对应的CAN外设 */
CAN_HandleTypeDef hcan1;  /* CAN1句柄 */
CAN_HandleTypeDef hcan2;  /* CAN2句柄 */

/**
  * @brief  CAN1初始化函数
  * @note   配置CAN1为正常模式，波特率预分频器为3
  *         时间段1为10TQ，时间段2为3TQ，同步跳转宽度为1TQ
  *         自动总线关闭和自动唤醒功能使能
  * @param  None
  * @retval None
  */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */

  /* CAN1实例配置 */
  hcan1.Instance = CAN1;  /* 选择CAN1外设 */

  /**
    * CAN时序配置：
    * 波特率 = APB1时钟 / (预分频器 * (时间段1 + 时间段2 + 1))
    * 假设APB1时钟为42MHz，波特率 = 42MHz / (3 * (10 + 3 + 1)) = 1Mbps
    */
  hcan1.Init.Prescaler = 3;                  /* 预分频器 = 3 */
  hcan1.Init.Mode = CAN_MODE_NORMAL;         /* 正常模式 */
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;   /* 同步跳转宽度 = 1个时间量子(TQ) */
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;       /* 时间段1 = 10TQ */
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;        /* 时间段2 = 3TQ */

  /**
    * CAN功能配置：
    * TimeTriggeredMode: 禁用时间触发通信模式
    * AutoBusOff: 使能自动总线关闭恢复（当总线错误达到阈值时自动恢复）
    * AutoWakeUp: 使能自动唤醒（当CAN总线有活动时自动唤醒）
    * AutoRetransmission: 禁用自动重传（错误帧不重传）
    * ReceiveFifoLocked: 禁用接收FIFO锁定（当FIFO满时新消息覆盖旧消息）
    * TransmitFifoPriority: 使能发送FIFO优先级（按发送请求顺序发送）
    */
  hcan1.Init.TimeTriggeredMode = DISABLE;    /* 禁用时间触发模式 */
  hcan1.Init.AutoBusOff = ENABLE;            /* 使能自动总线关闭 */
  hcan1.Init.AutoWakeUp = ENABLE;            /* 使能自动唤醒 */
  hcan1.Init.AutoRetransmission = DISABLE;   /* 禁用自动重传 */
  hcan1.Init.ReceiveFifoLocked = DISABLE;    /* 禁用接收FIFO锁定 */
  hcan1.Init.TransmitFifoPriority = ENABLE;  /* 使能发送FIFO优先级 */

  /* 初始化CAN1，如果失败则调用错误处理函数 */
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief  CAN2初始化函数
  * @note   配置CAN2为正常模式，参数与CAN1相同
  *         CAN2与CAN1共享同一个时钟源，所以预分频器设置相同
  * @param  None
  * @retval None
  */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */

  /* CAN2实例配置 */
  hcan2.Instance = CAN2;  /* 选择CAN2外设 */

  /**
    * CAN2时序配置与CAN1相同：
    * 波特率 = APB1时钟 / (预分频器 * (时间段1 + 时间段2 + 1))
    */
  hcan2.Init.Prescaler = 3;                  /* 预分频器 = 3 */
  hcan2.Init.Mode = CAN_MODE_NORMAL;         /* 正常模式 */
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;   /* 同步跳转宽度 = 1TQ */
  hcan2.Init.TimeSeg1 = CAN_BS1_10TQ;       /* 时间段1 = 10TQ */
  hcan2.Init.TimeSeg2 = CAN_BS2_3TQ;        /* 时间段2 = 3TQ */

  /* CAN2功能配置与CAN1相同 */
  hcan2.Init.TimeTriggeredMode = DISABLE;    /* 禁用时间触发模式 */
  hcan2.Init.AutoBusOff = ENABLE;            /* 使能自动总线关闭 */
  hcan2.Init.AutoWakeUp = ENABLE;            /* 使能自动唤醒 */
  hcan2.Init.AutoRetransmission = DISABLE;   /* 禁用自动重传 */
  hcan2.Init.ReceiveFifoLocked = DISABLE;    /* 禁用接收FIFO锁定 */
  hcan2.Init.TransmitFifoPriority = ENABLE;  /* 使能发送FIFO优先级 */

  /* 初始化CAN2，如果失败则调用错误处理函数 */
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * CAN1时钟使能计数器
  * 用于管理CAN1和CAN2共享的时钟，确保只在第一次使用时使能时钟
  */
static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

/**
  * @brief  CAN底层初始化函数（MSP初始化）
  * @note   由HAL_CAN_Init()自动调用，负责CAN外设的底层硬件初始化
  *         包括时钟使能、GPIO配置、中断配置等
  * @param  canHandle: CAN句柄指针，包含CAN配置信息
  * @retval None
  */
void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};  /* GPIO初始化结构体 */

  /* 检查是哪个CAN实例需要初始化 */
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */

    /**
    * CAN1时钟使能管理：
    * 使用计数器确保CAN1时钟只使能一次
    * 因为CAN1和CAN2共享时钟，需要特殊处理
    */
    HAL_RCC_CAN1_CLK_ENABLED++;  /* 递增计数器 */
    if(HAL_RCC_CAN1_CLK_ENABLED==1){  /* 如果是第一次使能 */
      __HAL_RCC_CAN1_CLK_ENABLE();    /* 使能CAN1时钟 */
    }

    /* 使能GPIOD时钟，因为CAN1的RX和TX引脚在GPIOD上 */
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /**CAN1 GPIO配置
    PD0     ------> CAN1_RX（CAN1接收引脚）
    PD1     ------> CAN1_TX（CAN1发送引脚）
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;  /* 选择PD0和PD1引脚 */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;        /* 复用推挽输出模式 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;            /* 无上下拉 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  /* 非常高的速度（100MHz） */
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;     /* 复用功能选择CAN1 */
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);        /* 初始化GPIO */

    /* CAN1中断初始化 */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);  /* 设置中断优先级：主优先级5，子优先级0 */
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);           /* 使能CAN1接收FIFO0中断 */

  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */

    /* CAN2时钟使能 */
    __HAL_RCC_CAN2_CLK_ENABLE();

    /**
    * CAN1时钟使能管理（CAN2也需要CAN1时钟）：
    * 递增计数器并检查是否需要使能CAN1时钟
    */
    HAL_RCC_CAN1_CLK_ENABLED++;  /* 递增计数器 */
    if(HAL_RCC_CAN1_CLK_ENABLED==1){  /* 如果是第一次使能 */
      __HAL_RCC_CAN1_CLK_ENABLE();    /* 使能CAN1时钟 */
    }

    /* 使能GPIOB时钟，因为CAN2的RX和TX引脚在GPIOB上 */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /**CAN2 GPIO配置
    PB5     ------> CAN2_RX（CAN2接收引脚）
    PB6     ------> CAN2_TX（CAN2发送引脚）
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;  /* 选择PB5和PB6引脚 */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;        /* 复用推挽输出模式 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;            /* 无上下拉 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  /* 非常高的速度（100MHz） */
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;     /* 复用功能选择CAN2 */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);        /* 初始化GPIO */

    /* CAN2中断初始化 */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 5, 0);  /* 设置中断优先级：主优先级5，子优先级0 */
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);           /* 使能CAN2接收FIFO0中断 */

  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

/**
  * @brief  CAN底层反初始化函数（MSP反初始化）
  * @note   由HAL_CAN_DeInit()自动调用，负责CAN外设的底层硬件反初始化
  *         包括时钟禁用、GPIO恢复、中断禁用等
  * @param  canHandle: CAN句柄指针，包含CAN配置信息
  * @retval None
  */
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  /* 检查是哪个CAN实例需要反初始化 */
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */

    /**
    * CAN1时钟禁用管理：
    * 递减计数器，只有当计数器为0时才真正禁用时钟
    */
    HAL_RCC_CAN1_CLK_ENABLED--;  /* 递减计数器 */
    if(HAL_RCC_CAN1_CLK_ENABLED==0){  /* 如果计数器为0 */
      __HAL_RCC_CAN1_CLK_DISABLE();   /* 禁用CAN1时钟 */
    }

    /**CAN1 GPIO配置恢复
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);  /* 恢复PD0和PD1引脚为默认状态 */

    /* CAN1中断禁用 */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);  /* 禁用CAN1接收FIFO0中断 */

  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */

    /* 禁用CAN2外设时钟 */
    __HAL_RCC_CAN2_CLK_DISABLE();

    /**
    * CAN1时钟禁用管理（CAN2也需要CAN1时钟）：
    * 递减计数器并检查是否需要禁用CAN1时钟
    */
    HAL_RCC_CAN1_CLK_ENABLED--;  /* 递减计数器 */
    if(HAL_RCC_CAN1_CLK_ENABLED==0){  /* 如果计数器为0 */
      __HAL_RCC_CAN1_CLK_DISABLE();   /* 禁用CAN1时钟 */
    }

    /**CAN2 GPIO配置恢复
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5|GPIO_PIN_6);  /* 恢复PB5和PB6引脚为默认状态 */

    /* CAN2中断禁用 */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);  /* 禁用CAN2接收FIFO0中断 */

  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */