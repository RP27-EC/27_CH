/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   本文件提供定时器（TIM）实例的配置代码
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
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* 定时器句柄实例 */
TIM_HandleTypeDef htim1;   /* 定时器1句柄（高级定时器） */
TIM_HandleTypeDef htim4;   /* 定时器4句柄（通用定时器） */
TIM_HandleTypeDef htim10;  /* 定时器10句柄（通用定时器） */

/**
  * @brief  TIM1初始化函数
  * @note   配置TIM1为PWM输出模式，用于控制电机或其他设备
  *         预分频器167，周期19999，PWM频率 = 168MHz / (167+1) / (19999+1) = 50Hz
  *         适用于舵机控制（周期20ms）
  * @param  None
  * @retval None
  */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};           /* 主模式配置结构体 */
  TIM_OC_InitTypeDef sConfigOC = {0};                    /* 输出比较配置结构体 */
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};  /* 刹车和死区配置结构体 */

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */

  /* TIM1实例配置 */
  htim1.Instance = TIM1;  /* 选择TIM1外设（高级定时器） */

  /**
    * TIM1基本配置：
    * Prescaler: 预分频器 = 167，定时器时钟 = 168MHz / (167+1) = 1MHz
    * CounterMode: 向上计数模式
    * Period: 周期/自动重装载值 = 19999，定时器溢出频率 = 1MHz / (19999+1) = 50Hz
    * ClockDivision: 时钟分频 = 1（不分频）
    * RepetitionCounter: 重复计数器 = 0（高级定时器特有）
    * AutoReloadPreload: 自动重装载预加载禁用
    */
  htim1.Init.Prescaler = 167;                      /* 预分频器 = 167 */
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;     /* 向上计数模式 */
  htim1.Init.Period = 19999;                       /* 周期 = 19999 */
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;  /* 时钟分频 = 1 */
  htim1.Init.RepetitionCounter = 0;                /* 重复计数器 = 0 */
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  /* 禁用自动重装载预加载 */

  /* 初始化TIM1为PWM模式，如果失败则调用错误处理函数 */
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  /**
    * 主模式配置：
    * MasterOutputTrigger: 主输出触发 = 复位（不使用触发输出）
    * MasterSlaveMode: 主从模式 = 禁用
    */
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;     /* 主输出触发 = 复位 */
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;  /* 禁用主从模式 */

  /* 配置主从同步，如果失败则调用错误处理函数 */
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /**
    * PWM通道1配置：
    * OCMode: 输出比较模式 = PWM1（当计数器 < 比较值时输出有效电平）
    * Pulse: 脉冲值/比较值 = 0（初始占空比为0%）
    * OCPolarity: 输出极性 = 高电平有效
    * OCNPolarity: 互补输出极性 = 高电平有效
    * OCFastMode: 输出快速模式 = 禁用
    * OCIdleState: 空闲状态输出 = 复位
    * OCNIdleState: 互补输出空闲状态 = 复位
    */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;              /* PWM模式1 */
  sConfigOC.Pulse = 0;                             /* 初始脉冲值 = 0 */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;      /* 输出极性高电平 */
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;    /* 互补输出极性高电平 */
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;       /* 禁用快速模式 */
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;   /* 空闲状态复位 */
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; /* 互补输出空闲状态复位 */

  /* 配置TIM1通道1为PWM输出，如果失败则调用错误处理函数 */
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /**
    * 刹车和死区配置：
    * OffStateRunMode: 运行模式下关闭状态 = 禁用
    * OffStateIDLEMode: 空闲模式下关闭状态 = 禁用
    * LockLevel: 锁定级别 = 无
    * DeadTime: 死区时间 = 0（无死区）
    * BreakState: 刹车输入 = 禁用
    * BreakPolarity: 刹车极性 = 高电平
    * AutomaticOutput: 自动输出 = 禁用
    */
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;      /* 禁用运行模式关闭状态 */
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;     /* 禁用空闲模式关闭状态 */
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;           /* 无锁定 */
  sBreakDeadTimeConfig.DeadTime = 0;                            /* 死区时间 = 0 */
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;          /* 禁用刹车输入 */
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;  /* 刹车极性高电平 */
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;  /* 禁用自动输出 */

  /* 配置刹车和死区，如果失败则调用错误处理函数 */
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

  /* TIM1后初始化（配置GPIO） */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief  TIM4初始化函数
  * @note   配置TIM4为PWM输出模式，用于控制电机或其他设备
  *         预分频器83，周期499，PWM频率 = 84MHz / (83+1) / (499+1) = 2kHz
  *         适用于电机控制或其他需要较高频率PWM的应用
  * @param  None
  * @retval None
  */
void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};  /* 时钟源配置结构体 */
  TIM_MasterConfigTypeDef sMasterConfig = {0};       /* 主模式配置结构体 */
  TIM_OC_InitTypeDef sConfigOC = {0};                /* 输出比较配置结构体 */

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */

  /* TIM4实例配置 */
  htim4.Instance = TIM4;  /* 选择TIM4外设（通用定时器） */

  /**
    * TIM4基本配置：
    * Prescaler: 预分频器 = 83，定时器时钟 = 84MHz / (83+1) = 1MHz
    * CounterMode: 向上计数模式
    * Period: 周期/自动重装载值 = 499，定时器溢出频率 = 1MHz / (499+1) = 2kHz
    * ClockDivision: 时钟分频 = 1（不分频）
    * AutoReloadPreload: 自动重装载预加载使能
    */
  htim4.Init.Prescaler = 83;                      /* 预分频器 = 83 */
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;    /* 向上计数模式 */
  htim4.Init.Period = 499;                        /* 周期 = 499 */
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;  /* 时钟分频 = 1 */
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;  /* 使能自动重装载预加载 */

  /* 初始化TIM4基础功能，如果失败则调用错误处理函数 */
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  /**
    * 时钟源配置：
    * ClockSource: 时钟源 = 内部时钟
    */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;  /* 内部时钟 */

  /* 配置时钟源，如果失败则调用错误处理函数 */
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* 初始化TIM4为PWM模式，如果失败则调用错误处理函数 */
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  /**
    * 主模式配置（与TIM1相同）：
    * MasterOutputTrigger: 主输出触发 = 复位
    * MasterSlaveMode: 主从模式 = 禁用
    */
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  /* 配置主从同步，如果失败则调用错误处理函数 */
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /**
    * PWM通道3配置：
    * OCMode: 输出比较模式 = PWM1
    * Pulse: 脉冲值/比较值 = 0（初始占空比为0%）
    * OCPolarity: 输出极性 = 高电平有效
    * OCFastMode: 输出快速模式 = 禁用
    */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;              /* PWM模式1 */
  sConfigOC.Pulse = 0;                             /* 初始脉冲值 = 0 */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;      /* 输出极性高电平 */
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;       /* 禁用快速模式 */

  /* 配置TIM4通道3为PWM输出，如果失败则调用错误处理函数 */
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

  /* TIM4后初始化（配置GPIO） */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief  TIM10初始化函数
  * @note   配置TIM10为PWM输出模式
  *         预分频器0，周期4999，PWM频率 = 168MHz / (0+1) / (4999+1) = 33.6kHz
  *         适用于LED调光或其他需要较高频率PWM的应用
  * @param  None
  * @retval None
  */
void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};  /* 输出比较配置结构体 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */

  /* TIM10实例配置 */
  htim10.Instance = TIM10;  /* 选择TIM10外设（通用定时器） */

  /**
    * TIM10基本配置：
    * Prescaler: 预分频器 = 0，定时器时钟 = 168MHz / (0+1) = 168MHz
    * CounterMode: 向上计数模式
    * Period: 周期/自动重装载值 = 4999，定时器溢出频率 = 168MHz / (4999+1) = 33.6kHz
    * ClockDivision: 时钟分频 = 1（不分频）
    * AutoReloadPreload: 自动重装载预加载禁用
    */
  htim10.Init.Prescaler = 0;                      /* 预分频器 = 0 */
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;   /* 向上计数模式 */
  htim10.Init.Period = 4999;                      /* 周期 = 4999 */
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;  /* 时钟分频 = 1 */
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  /* 禁用自动重装载预加载 */

  /* 初始化TIM10基础功能，如果失败则调用错误处理函数 */
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }

  /* 初始化TIM10为PWM模式，如果失败则调用错误处理函数 */
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }

  /**
    * PWM通道1配置：
    * OCMode: 输出比较模式 = PWM1
    * Pulse: 脉冲值/比较值 = 0（初始占空比为0%）
    * OCPolarity: 输出极性 = 高电平有效
    * OCFastMode: 输出快速模式 = 禁用
    */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;              /* PWM模式1 */
  sConfigOC.Pulse = 0;                             /* 初始脉冲值 = 0 */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;      /* 输出极性高电平 */
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;       /* 禁用快速模式 */

  /* 配置TIM10通道1为PWM输出，如果失败则调用错误处理函数 */
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

  /* TIM10后初始化（配置GPIO） */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief  定时器PWM MSP初始化函数
  * @note   由HAL_TIM_PWM_Init()自动调用，负责定时器PWM模式的底层硬件初始化
  *         包括时钟使能等
  * @param  tim_pwmHandle: 定时器PWM句柄指针
  * @retval None
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  /* 检查是哪个定时器实例需要初始化 */
  if(tim_pwmHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */

    /* TIM1时钟使能 */
    __HAL_RCC_TIM1_CLK_ENABLE();

  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
}

/**
  * @brief  定时器基础MSP初始化函数
  * @note   由HAL_TIM_Base_Init()自动调用，负责定时器基础模式的底层硬件初始化
  *         包括时钟使能、中断配置等
  * @param  tim_baseHandle: 定时器基础句柄指针
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  /* 检查是哪个定时器实例需要初始化 */
  if(tim_baseHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspInit 0 */

  /* USER CODE END TIM4_MspInit 0 */

    /* TIM4时钟使能 */
    __HAL_RCC_TIM4_CLK_ENABLE();

    /* TIM4中断初始化 */
    HAL_NVIC_SetPriority(TIM4_IRQn, 5, 0);  /* 设置中断优先级 */
    HAL_NVIC_EnableIRQ(TIM4_IRQn);           /* 使能中断 */

  /* USER CODE BEGIN TIM4_MspInit 1 */

  /* USER CODE END TIM4_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM10)
  {
  /* USER CODE BEGIN TIM10_MspInit 0 */

  /* USER CODE END TIM10_MspInit 0 */

    /* TIM10时钟使能 */
    __HAL_RCC_TIM10_CLK_ENABLE();

  /* USER CODE BEGIN TIM10_MspInit 1 */

  /* USER CODE END TIM10_MspInit 1 */
  }
}

/**
  * @brief  定时器后初始化函数（GPIO配置）
  * @note   在定时器初始化完成后调用，配置定时器输出引脚的GPIO复用功能
  * @param  timHandle: 定时器句柄指针
  * @retval None
  */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};  /* GPIO初始化结构体 */

  /* 检查是哪个定时器实例需要后初始化 */
  if(timHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */

    /* 使能GPIOE时钟，因为TIM1通道1输出引脚在PE9 */
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /**TIM1 GPIO配置
    PE9     ------> TIM1_CH1（定时器1通道1输出）
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;              /* 选择PE9引脚 */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;        /* 复用推挽输出模式 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;            /* 无上下拉 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;   /* 低速（2MHz） */
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;     /* 复用功能选择TIM1 */
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);        /* 初始化GPIO */

  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }
  else if(timHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspPostInit 0 */

  /* USER CODE END TIM4_MspPostInit 0 */

    /* 使能GPIOB时钟，因为TIM4通道3输出引脚在PB8 */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /**TIM4 GPIO配置
    PB8     ------> TIM4_CH3（定时器4通道3输出）
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;              /* 选择PB8引脚 */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;        /* 复用推挽输出模式 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;            /* 无上下拉 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;   /* 低速（2MHz） */
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;     /* 复用功能选择TIM4 */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);        /* 初始化GPIO */

  /* USER CODE BEGIN TIM4_MspPostInit 1 */

  /* USER CODE END TIM4_MspPostInit 1 */
  }
  else if(timHandle->Instance==TIM10)
  {
  /* USER CODE BEGIN TIM10_MspPostInit 0 */

  /* USER CODE END TIM10_MspPostInit 0 */

    /* 使能GPIOF时钟，因为TIM10通道1输出引脚在PF6 */
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /**TIM10 GPIO配置
    PF6     ------> TIM10_CH1（定时器10通道1输出）
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;              /* 选择PF6引脚 */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;        /* 复用推挽输出模式 */
    GPIO_InitStruct.Pull = GPIO_PULLUP;            /* 使能上拉电阻 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  /* 高速（50MHz） */
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM10;    /* 复用功能选择TIM10 */
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);        /* 初始化GPIO */

  /* USER CODE BEGIN TIM10_MspPostInit 1 */

  /* USER CODE END TIM10_MspPostInit 1 */
  }

}

/**
  * @brief  定时器PWM MSP反初始化函数
  * @note   由HAL_TIM_PWM_DeInit()自动调用，负责定时器PWM模式的底层硬件反初始化
  *         包括时钟禁用等
  * @param  tim_pwmHandle: 定时器PWM句柄指针
  * @retval None
  */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  /* 检查是哪个定时器实例需要反初始化 */
  if(tim_pwmHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */

    /* 禁用TIM1外设时钟 */
    __HAL_RCC_TIM1_CLK_DISABLE();

  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
}

/**
  * @brief  定时器基础MSP反初始化函数
  * @note   由HAL_TIM_Base_DeInit()自动调用，负责定时器基础模式的底层硬件反初始化
  *         包括时钟禁用、中断禁用等
  * @param  tim_baseHandle: 定时器基础句柄指针
  * @retval None
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  /* 检查是哪个定时器实例需要反初始化 */
  if(tim_baseHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspDeInit 0 */

  /* USER CODE END TIM4_MspDeInit 0 */

    /* 禁用TIM4外设时钟 */
    __HAL_RCC_TIM4_CLK_DISABLE();

    /* TIM4中断禁用 */
    HAL_NVIC_DisableIRQ(TIM4_IRQn);

  /* USER CODE BEGIN TIM4_MspDeInit 1 */

  /* USER CODE END TIM4_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM10)
  {
  /* USER CODE BEGIN TIM10_MspDeInit 0 */

  /* USER CODE END TIM10_MspDeInit 0 */

    /* 禁用TIM10外设时钟 */
    __HAL_RCC_TIM10_CLK_DISABLE();

  /* USER CODE BEGIN TIM10_MspDeInit 1 */

  /* USER CODE END TIM10_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */