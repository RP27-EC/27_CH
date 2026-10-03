/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   GPIO配置文件 - 提供所有使用的GPIO引脚的配置代码
  * @details 本文件负责初始化所有GPIO引脚，包括：
  *          - 电源控制引脚（PWR_OUT1/2_EN, PWR_5V_EN）
  *          - SPI片选引脚（CS1_ACCEL, CS1_GYRO）
  *          - PWM输出引脚（云台俯仰、BMI温度控制）
  *          - 水泵控制引脚（PUMP_EN1/2/3）
  *          - 传感器中断引脚（BMI_ACCEL_INT, BMI_GYRO_INT）
  *          - 串口通信引脚（USART3）
  *          - 按键输入引脚（KEY_INT）
  ******************************************************************************
  * @attention
  * 版权所有 (c) 2025 STMicroelectronics。保留所有权利。
  ******************************************************************************
  */

/* 包含头文件 */
#include "gpio.h"

/**
  * @brief  GPIO初始化函数
  * @details 该函数完成以下工作：
  *          1. 使能所有GPIO端口的时钟
  *          2. 配置GPIO输出初始电平
  *          3. 配置各引脚的工作模式（输入/输出/复用/模拟）
  *          4. 配置GPIO中断
  * @note   引脚复用功能说明：
  *         - PA0 -> TIM2_CH1_ETR (云台俯仰PWM)
  *         - PB1 -> TIM3_CH4 (BMI温度控制PWM)
  *         - PB14 -> USART3_DE (RS485方向控制)
  *         - PD8 -> USART3_TX
  *         - PD9 -> USART3_RX
  * @retval None
  */
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* 使能所有GPIO端口的时钟 - GPIO工作需要时钟 */
  __HAL_RCC_GPIOE_CLK_ENABLE();  /* 使能GPIOE端口时钟 */
  __HAL_RCC_GPIOC_CLK_ENABLE();  /* 使能GPIOC端口时钟 */
  __HAL_RCC_GPIOH_CLK_ENABLE();  /* 使能GPIOH端口时钟 */
  __HAL_RCC_GPIOA_CLK_ENABLE();  /* 使能GPIOA端口时钟 */
  __HAL_RCC_GPIOB_CLK_ENABLE();  /* 使能GPIOB端口时钟 */
  __HAL_RCC_GPIOD_CLK_ENABLE();  /* 使能GPIOD端口时钟 */

  /**
    * 配置GPIO输出初始电平
    * 这些引脚在系统启动时需要设置为特定电平，以确保外设正确初始化
    */

  /* 电源输出使能引脚 - 初始为低电平（关闭） */
  HAL_GPIO_WritePin(GPIOC, PWR_OUT2_EN_Pin|PWR_OUT1_EN_Pin, GPIO_PIN_RESET);

  /* 5V电源和SPI片选引脚 - 初始为高电平 */
  /* PWR_5V_EN: 5V电源使能（高电平有效） */
  /* CS1_ACCEL/CS1_GYRO: SPI片选（低电平有效，所以初始为高） */
  HAL_GPIO_WritePin(GPIOC, PWR_5V_EN_Pin|CS1_ACCEL_Pin|CS1_GYRO_Pin, GPIO_PIN_SET);

  /* 水泵1控制引脚 - 初始为低电平（关闭） */
  HAL_GPIO_WritePin(PUMP_EN1_GPIO_Port, PUMP_EN1_Pin, GPIO_PIN_RESET);

  /* 水泵2和3控制引脚 - 初始为低电平（关闭） */
  HAL_GPIO_WritePin(GPIOE, PUMP_EN2_Pin|PUMP_EN3_Pin, GPIO_PIN_RESET);

  /**
    * 配置GPIO引脚工作模式
    */

  /* 配置电源输出使能引脚和5V电源使能引脚 */
  /* 这些是普通GPIO输出引脚，用于控制电源开关 */
  GPIO_InitStruct.Pin = PWR_OUT2_EN_Pin|PWR_OUT1_EN_Pin|PWR_5V_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;         /* 推挽输出模式 */
  GPIO_InitStruct.Pull = GPIO_NOPULL;                 /* 无上拉/下拉 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;        /* 低速（开关控制不需要高速） */
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* 配置SPI片选引脚（加速度计和陀螺仪） */
  /* SPI片选需要高速切换，所以配置为高速模式 */
  GPIO_InitStruct.Pin = CS1_ACCEL_Pin|CS1_GYRO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;         /* 推挽输出模式 */
  GPIO_InitStruct.Pull = GPIO_NOPULL;                 /* 无上拉/下拉 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  /* 非常高速（SPI通信需要） */
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**
    * 配置复用功能引脚（PWM输出）
    */

  /* 配置云台俯仰PWM引脚 - TIM2通道1 */
  GPIO_InitStruct.Pin = GIMB_PITCH_PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;             /* 复用推挽输出模式 */
  GPIO_InitStruct.Pull = GPIO_NOPULL;                 /* 无上拉/下拉 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;        /* 低速（PWM频率不高） */
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;          /* 复用功能1：TIM2 */
  HAL_GPIO_Init(GIMB_PITCH_PWM_GPIO_Port, &GPIO_InitStruct);

  /* 配置水泵1控制引脚 - 带下拉电阻，确保初始状态为关闭 */
  GPIO_InitStruct.Pin = PUMP_EN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;         /* 推挽输出模式 */
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;               /* 下拉电阻（默认低电平） */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PUMP_EN1_GPIO_Port, &GPIO_InitStruct);

  /* 配置BMI温度控制PWM引脚 - TIM3通道4 */
  GPIO_InitStruct.Pin = BMI_TEMP_PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;             /* 复用推挽输出模式 */
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;          /* 复用功能2：TIM3 */
  HAL_GPIO_Init(BMI_TEMP_PWM_GPIO_Port, &GPIO_InitStruct);

  /* 配置水泵2和3控制引脚 - 带下拉电阻 */
  GPIO_InitStruct.Pin = PUMP_EN2_Pin|PUMP_EN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;               /* 下拉电阻（默认低电平） */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /**
    * 配置中断输入引脚
    */

  /* 配置BMI传感器中断引脚 - 上升沿触发 */
  /* 加速度计和陀螺仪数据就绪时产生上升沿中断 */
  GPIO_InitStruct.Pin = BMI_ACCEL_INT_Pin|BMI_GYRO_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;         /* 上升沿触发中断 */
  GPIO_InitStruct.Pull = GPIO_NOPULL;                 /* 无上拉/下拉（外部已有） */
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /**
    * 配置串口通信引脚（USART3）
    */

  /* 配置USART3方向控制引脚（RS485 DE） - PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;             /* 复用推挽输出 */
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;        /* 复用功能7：USART3 */
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* 配置USART3 TX和RX引脚 - PD8(TX), PD9(RX) */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;        /* 复用功能7：USART3 */
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* 配置按键输入引脚 - 无上拉/下拉，外部电路已配置 */
  GPIO_InitStruct.Pin = KEY_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;             /* 输入模式 */
  GPIO_InitStruct.Pull = GPIO_NOPULL;                 /* 无上拉/下拉 */
  HAL_GPIO_Init(KEY_INT_GPIO_Port, &GPIO_InitStruct);

  /* 配置模拟开关 - PC3引脚连接到模拟输入 */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_CLOSE);

  /**
    * 配置GPIO中断
    * EXTI15_10_IRQn用于处理引脚10-15的外部中断
    * 包括BMI传感器中断引脚
    */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);        /* 中断优先级5 */
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);                 /* 使能中断 */

}
