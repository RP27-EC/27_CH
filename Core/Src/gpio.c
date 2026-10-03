/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   本文件提供所有使用GPIO引脚的配置代码
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @brief  GPIO初始化函数
  * @note   配置所有使用的GPIO引脚，包括LED、按键、SPI片选、中断引脚等
  *         使能所有需要的GPIO端口时钟
  * @param  None
  * @retval None
  */
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};  /* GPIO初始化结构体 */

  /**
    * GPIO端口时钟使能
    * STM32F4系列有多个GPIO端口（A-H），每个端口有16个引脚
    * 使用GPIO前必须先使能对应端口的时钟
    */
  __HAL_RCC_GPIOB_CLK_ENABLE();  /* 使能GPIOB时钟 */
  __HAL_RCC_GPIOG_CLK_ENABLE();  /* 使能GPIOG时钟 */
  __HAL_RCC_GPIOA_CLK_ENABLE();  /* 使能GPIOA时钟 */
  __HAL_RCC_GPIOD_CLK_ENABLE();  /* 使能GPIOD时钟 */
  __HAL_RCC_GPIOC_CLK_ENABLE();  /* 使能GPIOC时钟 */
  __HAL_RCC_GPIOH_CLK_ENABLE();  /* 使能GPIOH时钟 */
  __HAL_RCC_GPIOF_CLK_ENABLE();  /* 使能GPIOF时钟 */
  __HAL_RCC_GPIOE_CLK_ENABLE();  /* 使能GPIOE时钟 */

  /**
    * 配置GPIO输出初始电平
    * 在配置GPIO为输出模式之前，先设置输出电平
    * 避免配置瞬间产生毛刺
    */

  /* 设置LED引脚初始电平为低电平（LED熄灭） */
  HAL_GPIO_WritePin(GPIOH, LED3_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /* 设置SPI片选引脚初始电平为高电平（未选中设备） */
  HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);

  /* 设置SPI片选引脚初始电平为高电平（未选中设备） */
  HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);

  /**
    * 配置LED引脚
    * LED3_Pin, LED2_Pin, LED1_Pin配置为推挽输出模式
    * 用于控制三个LED的亮灭
    */
  GPIO_InitStruct.Pin = LED3_Pin|LED2_Pin|LED1_Pin;  /* 选择LED引脚 */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;        /* 推挽输出模式 */
  GPIO_InitStruct.Pull = GPIO_NOPULL;                /* 无上下拉 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;       /* 低速（2MHz） */
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);            /* 初始化GPIO */

  /**
    * 配置按键引脚
    * KEY_Pin配置为下降沿触发中断模式
    * 上拉电阻使能，按下按键时引脚变为低电平
    */
  GPIO_InitStruct.Pin = KEY_Pin;                      /* 选择按键引脚 */
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;        /* 下降沿触发中断 */
  GPIO_InitStruct.Pull = GPIO_PULLUP;                 /* 使能上拉电阻 */
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);     /* 初始化GPIO */

  /**
    * 配置加速度传感器SPI片选引脚
    * CS1_ACCEL_Pin配置为推挽输出模式
    * 用于选择加速度传感器SPI设备
    */
  GPIO_InitStruct.Pin = CS1_ACCEL_Pin;                /* 选择加速度传感器片选引脚 */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;         /* 推挽输出模式 */
  GPIO_InitStruct.Pull = GPIO_PULLUP;                 /* 使能上拉电阻 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;       /* 高速（50MHz） */
  HAL_GPIO_Init(CS1_ACCEL_GPIO_Port, &GPIO_InitStruct);  /* 初始化GPIO */

  /**
    * 配置传感器中断引脚
    * INT1_ACCEL_Pin和INT1_GRYO_Pin配置为下降沿触发中断模式
    * 用于接收加速度传感器和陀螺仪的中断信号
    */
  GPIO_InitStruct.Pin = INT1_ACCEL_Pin|INT1_GRYO_Pin;  /* 选择传感器中断引脚 */
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;          /* 下降沿触发中断 */
  GPIO_InitStruct.Pull = GPIO_PULLUP;                   /* 使能上拉电阻 */
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);               /* 初始化GPIO */

  /**
    * 配置陀螺仪SPI片选引脚
    * CS1_GYRO_Pin配置为推挽输出模式
    * 用于选择陀螺仪SPI设备
    */
  GPIO_InitStruct.Pin = CS1_GYRO_Pin;                  /* 选择陀螺仪片选引脚 */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;          /* 推挽输出模式 */
  GPIO_InitStruct.Pull = GPIO_PULLUP;                  /* 使能上拉电阻 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;        /* 高速（50MHz） */
  HAL_GPIO_Init(CS1_GYRO_GPIO_Port, &GPIO_InitStruct);  /* 初始化GPIO */

  /**
    * EXTI中断初始化
    * 配置外部中断线的优先级并使能中断
    * 优先级设置为5，子优先级为0
    */

  /* EXTI0中断初始化 - 用于EXTI0引脚（可能是按键或其他外部中断） */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);  /* 设置中断优先级 */
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);           /* 使能中断 */

  /* EXTI4中断初始化 - 用于EXTI4引脚（可能是传感器中断） */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  /* EXTI9_5中断初始化 - 用于EXTI5-9引脚（可能是传感器中断） */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */