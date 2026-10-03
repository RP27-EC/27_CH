/**
  ******************************************************************************
  * @file    tim.c
  * @brief   定时器配置文件 - 提供TIM外设实例的配置代码
  * @details 本文件配置TIM12定时器，用于蜂鸣器PWM输出控制：
  *          - TIM12通道2：蜂鸣器驱动（PB15引脚）
  *          - PWM频率：25kHz（由预分频器和周期计算）
  *          - 初始占空比：50%
  ******************************************************************************
  * @attention
  * 版权所有 (c) 2025 STMicroelectronics。保留所有权利。
  ******************************************************************************
  */

/* 包含头文件 */
#include "tim.h"

/* TIM12句柄变量 - 用于存储定时器配置状态 */
TIM_HandleTypeDef htim12;

/**
  * @brief  TIM12初始化函数
  * @details 配置TIM12定时器用于蜂鸣器PWM输出：
  *          - 预分频器：24-1=23
  *          - 计数模式：向上计数
  *          - 周期：2000-1=1999
  *          - 时钟分频：不分频
  *          - 自动重装载预装载：禁用
  *          - PWM模式：PWM1模式
  *          - 初始脉冲值：1000（50%占空比）
  *          - 输出极性：高电平有效
  * @note   PWM频率计算：
  *         假设定时器时钟为24MHz
  *         PWM频率 = 定时器时钟 / ((预分频器+1) * (周期+1))
  *         = 24MHz / (24 * 2000) = 500Hz
  *         如果需要更高频率（如25kHz用于电机控制），可以调整参数
  * @retval None
  */
void MX_TIM12_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* TIM12实例配置 */
  htim12.Instance = TIM12;                            /* 使用TIM12定时器 */
  htim12.Init.Prescaler = 24-1;                       /* 预分频器：23（实际分频24） */
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;       /* 向上计数模式 */
  htim12.Init.Period = 2000-1;                        /* 周期：1999（自动重装载值） */
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; /* 时钟分频：不分频 */
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  /* 禁用自动重装载预装载 */

  /* 初始化TIM12为PWM模式 */
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }

  /* 主输出配置 - 用于高级定时器（TIM1/TIM8），TIM12不需要 */
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;  /* 触发输出：复位事件 */
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;  /* 禁用主从模式 */
  if (HAL_TIMEx_MasterConfigSynchronization(&htim12, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* PWM通道2配置 - 用于蜂鸣器输出 */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;                 /* PWM1模式：CNT < CCR时输出有效电平 */
  sConfigOC.Pulse = 1000;                             /* 初始脉冲值：1000（50%占空比） */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;          /* 输出极性：高电平有效 */
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;           /* 禁用快速模式 */

  /* 配置TIM12通道2为PWM输出 */
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  /* 调用MSP后初始化函数，配置GPIO引脚 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief  TIM底层硬件初始化函数（MSP初始化）
  * @param  tim_pwmHandle: TIM句柄指针，指向要初始化的定时器实例
  * @details 该函数由HAL库在HAL_TIM_PWM_Init()中自动调用，完成以下工作：
  *          1. 使能定时器时钟
  *          2. 配置GPIO引脚为复用功能
  * @retval None
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM12)
  {
    /* 使能TIM12时钟 */
    __HAL_RCC_TIM12_CLK_ENABLE();
  }
}

/**
  * @brief  TIM MSP后初始化函数
  * @param  timHandle: TIM句柄指针
  * @details 该函数在定时器初始化完成后调用，用于配置定时器输出引脚的GPIO
  * @note   TIM12通道2引脚分配：
  *         - PB15 -> TIM12_CH2 (蜂鸣器输出)
  * @retval None
  */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if(timHandle->Instance==TIM12)
  {
    /* 使能GPIOB端口时钟 */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /**
      * TIM12 GPIO配置
      * PB15 -> TIM12_CH2 (蜂鸣器PWM输出)
      */
    GPIO_InitStruct.Pin = BUZZER_Pin;                  /* 蜂鸣器引脚 */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;             /* 复用推挽输出模式 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;                 /* 无上拉/下拉 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;        /* 低速（PWM频率不高） */
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM12;         /* 复用功能2：TIM12 */
    HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  }

}

/**
  * @brief  TIM底层硬件反初始化函数（MSP反初始化）
  * @param  tim_pwmHandle: TIM句柄指针
  * @details 该函数由HAL库在HAL_TIM_PWM_DeInit()中自动调用，完成以下工作：
  *          1. 禁用定时器时钟
  *          2. 恢复GPIO引脚为默认状态
  * @retval None
  */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM12)
  {
    /* 禁用TIM12时钟 */
    __HAL_RCC_TIM12_CLK_DISABLE();
  }
}
