/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   本文件提供ADC（模数转换器）实例的配置代码
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
#include "adc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* ADC1和ADC3的句柄实例，用于控制对应的ADC外设 */
ADC_HandleTypeDef hadc1;  /* ADC1句柄 */
ADC_HandleTypeDef hadc3;  /* ADC3句柄 */

/**
  * @brief  ADC1初始化函数
  * @note   配置ADC1为12位分辨率，软件触发，单次转换模式
  *         使用内部参考电压通道，采样时间为3个时钟周期
  * @param  None
  * @retval None
  */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};  /* ADC通道配置结构体 */

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** 配置ADC全局特性：时钟、分辨率、数据对齐和转换次数
    * ClockPrescaler: ADC时钟分频为PCLK/4
    * Resolution: 12位分辨率，精度最高
    * ScanConvMode: 禁用扫描模式，仅进行单通道转换
    * ContinuousConvMode: 禁用连续转换，每次转换需手动触发
    * DiscontinuousConvMode: 禁用不连续转换模式
    * ExternalTrigConvEdge: 无外部触发边沿，使用软件触发
    * ExternalTrigConv: 软件触发转换
    * DataAlign: 数据右对齐
    * NbrOfConversion: 只进行1次转换
    * DMAContinuousRequests: 禁用DMA连续请求
    * EOCSelection: 单次转换完成标志
    */
  hadc1.Instance = ADC1;  /* 选择ADC1外设 */
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;  /* ADC时钟 = PCLK/4 */
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;             /* 12位分辨率 */
  hadc1.Init.ScanConvMode = DISABLE;                      /* 禁用扫描模式 */
  hadc1.Init.ContinuousConvMode = DISABLE;                /* 禁用连续转换 */
  hadc1.Init.DiscontinuousConvMode = DISABLE;             /* 禁用不连续转换 */
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;  /* 无外部触发 */
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;       /* 软件触发 */
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;             /* 右对齐 */
  hadc1.Init.NbrOfConversion = 1;                         /* 1次转换 */
  hadc1.Init.DMAContinuousRequests = DISABLE;             /* 禁用DMA连续请求 */
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;         /* 单次转换完成标志 */

  /* 初始化ADC1，如果失败则调用错误处理函数 */
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** 配置ADC1的转换通道
    * Channel: 选择内部参考电压通道（VREFINT）
    * Rank: 转换序列中的排名（1表示第一个转换）
    * SamplingTime: 采样时间为3个时钟周期（较短，适合快速转换）
    */
  sConfig.Channel = ADC_CHANNEL_VREFINT;      /* 选择内部参考电压通道 */
  sConfig.Rank = 1;                           /* 转换序列第1位 */
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;  /* 采样时间3个周期 */

  /* 配置ADC1通道，如果失败则调用错误处理函数 */
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief  ADC3初始化函数
  * @note   配置ADC3为12位分辨率，软件触发，单次转换模式
  *         使用外部通道8（PF10引脚），采样时间为3个时钟周期
  * @param  None
  * @retval None
  */
void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};  /* ADC通道配置结构体 */

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** 配置ADC3全局特性
    * 与ADC1配置相同，使用PCLK/4时钟，12位分辨率，软件触发
    */
  hadc3.Instance = ADC3;  /* 选择ADC3外设 */
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;  /* ADC时钟 = PCLK/4 */
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;             /* 12位分辨率 */
  hadc3.Init.ScanConvMode = DISABLE;                      /* 禁用扫描模式 */
  hadc3.Init.ContinuousConvMode = DISABLE;                /* 禁用连续转换 */
  hadc3.Init.DiscontinuousConvMode = DISABLE;             /* 禁用不连续转换 */
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;  /* 无外部触发 */
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;       /* 软件触发 */
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;             /* 右对齐 */
  hadc3.Init.NbrOfConversion = 1;                         /* 1次转换 */
  hadc3.Init.DMAContinuousRequests = DISABLE;             /* 禁用DMA连续请求 */
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;         /* 单次转换完成标志 */

  /* 初始化ADC3，如果失败则调用错误处理函数 */
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** 配置ADC3的转换通道
    * Channel: 选择通道8（对应PF10引脚）
    * Rank: 转换序列中的排名（1表示第一个转换）
    * SamplingTime: 采样时间为3个时钟周期
    */
  sConfig.Channel = ADC_CHANNEL_8;           /* 选择通道8 */
  sConfig.Rank = 1;                           /* 转换序列第1位 */
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;  /* 采样时间3个周期 */

  /* 配置ADC3通道，如果失败则调用错误处理函数 */
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief  ADC底层初始化函数（MSP初始化）
  * @note   由HAL_ADC_Init()自动调用，负责ADC外设的底层硬件初始化
  *         包括时钟使能、GPIO配置等
  * @param  adcHandle: ADC句柄指针，包含ADC配置信息
  * @retval None
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};  /* GPIO初始化结构体 */

  /* 检查是哪个ADC实例需要初始化 */
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1时钟使能 - ADC1不需要额外的GPIO配置，因为使用内部参考电压 */
    __HAL_RCC_ADC1_CLK_ENABLE();
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspInit 0 */

  /* USER CODE END ADC3_MspInit 0 */
    /* ADC3时钟使能 */
    __HAL_RCC_ADC3_CLK_ENABLE();

    /* 使能GPIOF时钟，因为ADC3通道8使用PF10引脚 */
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /**ADC3 GPIO配置
    PF10引脚配置为模拟输入模式，用于ADC3通道8
    GPIO_NOPULL: 无上下拉，模拟输入模式通常不需要上下拉
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;          /* 选择PF10引脚 */
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;    /* 模拟模式 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;         /* 无上下拉 */
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);     /* 初始化GPIO */

  /* USER CODE BEGIN ADC3_MspInit 1 */

  /* USER CODE END ADC3_MspInit 1 */
  }
}

/**
  * @brief  ADC底层反初始化函数（MSP反初始化）
  * @note   由HAL_ADC_DeInit()自动调用，负责ADC外设的底层硬件反初始化
  *         包括时钟禁用、GPIO恢复等
  * @param  adcHandle: ADC句柄指针，包含ADC配置信息
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  /* 检查是哪个ADC实例需要反初始化 */
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* 禁用ADC1外设时钟 */
    __HAL_RCC_ADC1_CLK_DISABLE();
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspDeInit 0 */

  /* USER CODE END ADC3_MspDeInit 0 */
    /* 禁用ADC3外设时钟 */
    __HAL_RCC_ADC3_CLK_DISABLE();

    /**ADC3 GPIO配置恢复
    PF10引脚恢复为默认状态（浮空输入）
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_10);

  /* USER CODE BEGIN ADC3_MspDeInit 1 */

  /* USER CODE END ADC3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */