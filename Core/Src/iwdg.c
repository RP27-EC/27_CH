/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    iwdg.c
  * @brief   本文件提供IWDG（独立看门狗）实例的配置代码
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
#include "iwdg.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* IWDG句柄实例，用于控制独立看门狗外设 */
IWDG_HandleTypeDef hiwdg;

/**
  * @brief  IWDG初始化函数
  * @note   配置独立看门狗，预分频器为4，重装载值为4095
  *         IWDG使用独立的低速时钟（LSI，约32kHz）
  *         超时时间 = (预分频器 * 重装载值) / LSI频率
  *         超时时间 = (4 * 4095) / 32000 ≈ 0.512秒
  * @param  None
  * @retval None
  */
void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */

  /* IWDG实例配置 */
  hiwdg.Instance = IWDG;  /* 选择独立看门狗外设 */

  /**
    * IWDG配置参数：
    * Prescaler: 预分频器设置为4
    * Reload: 重装载值设置为4095（最大值）
    * 超时时间计算：(4 * 4095) / 32000 ≈ 0.512秒
    * 如果在0.512秒内没有喂狗，系统将复位
    */
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;  /* 预分频器 = 4 */
  hiwdg.Init.Reload = 4095;                 /* 重装载值 = 4095 */

  /* 初始化IWDG，如果失败则调用错误处理函数 */
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */