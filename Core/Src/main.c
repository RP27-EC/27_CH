/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 主程序主体
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
#include "main.h"          /* 主程序头文件 */
#include "cmsis_os.h"      /* CMSIS-RTOS头文件 */
#include "adc.h"           /* ADC头文件 */
#include "can.h"           /* CAN头文件 */
#include "dma.h"           /* DMA头文件 */
#include "spi.h"           /* SPI头文件 */
#include "tim.h"           /* 定时器头文件 */
#include "usart.h"         /* USART头文件 */
#include "usb_device.h"    /* USB设备头文件 */
#include "gpio.h"          /* GPIO头文件 */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "iwdg.h"          /* 独立看门狗头文件 */
#include "driver.h"        /* 驱动层头文件 */
#include "device.h"        /* 设备层头文件 */
#include "rc_protocol.h"   /* 遥控协议头文件 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);    /* 系统时钟配置函数原型 */
void MX_FREERTOS_Init(void);      /* FreeRTOS初始化函数原型 */
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/


  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */


  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();        /* GPIO初始化 */
  MX_DMA_Init();         /* DMA初始化 */
  MX_SPI1_Init();        /* SPI1初始化 */
  MX_TIM10_Init();       /* TIM10初始化 */
  MX_CAN1_Init();        /* CAN1初始化 */
  MX_CAN2_Init();        /* CAN2初始化 */
  MX_USART3_UART_Init(); /* USART3初始化 */
  MX_USART1_UART_Init(); /* USART1初始化 */
  MX_USART6_UART_Init(); /* USART6初始化 */
  MX_TIM4_Init();       
  MX_ADC1_Init();       
  MX_ADC3_Init();       
  MX_TIM1_Init();      

  /* USER CODE BEGIN 2 */


  // MX_IWDG_Init();       /* 独立看门狗初始化*/
  MX_USB_DEVICE_Init();  /* USB设备初始化 */
  DEVICE_Init();         /* 设备层初始化 */
  DRIVER_Init();         /* 驱动层初始化 */
  CAN_Filter_Init();     /* CAN过滤器初始化 */


  /* USER CODE END 2 */

  MX_FREERTOS_Init();


  osKernelStart();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};  /* 振荡器配置结构体 */
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};  /* 时钟配置结构体 */

  /** 配置主内部稳压器输出电压
    * 使能PWR时钟
    * 配置电压调节器输出为Scale1模式（高性能）
    */
  __HAL_RCC_PWR_CLK_ENABLE();  /* 使能PWR时钟 */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  /* 配置电压调节器 */

  /** 根据RCC_OscInitTypeDef结构体中的参数初始化RCC振荡器
    * OscillatorType: 振荡器类型，使用HSE（外部高速）和LSI（内部低速）
    * HSEState: 使能HSE振荡器
    * LSIState: 使能LSI振荡器（用于看门狗）
    * PLL: 使能PLL，配置PLL参数
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;        /* 使能HSE */
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;        /* 使能LSI */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;    /* 使能PLL */
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;  /* PLL时钟源 = HSE */

  /**
    * PLL配置参数：
    * PLLM = 6: PLL输入分频器，HSE(8MHz) / 6 = 1.33MHz（VCO输入）
    * PLLN = 168: PLL输出倍频器，1.33MHz * 168 = 224MHz（VCO输出）
    * PLLP = 2: PLL系统时钟分频器，224MHz / 2 = 112MHz（系统时钟）
    * PLLQ = 7: PLL USB时钟分频器，224MHz / 7 = 32MHz（USB时钟）
    */
  RCC_OscInitStruct.PLL.PLLM = 6;                 /* PLLM = 6 */
  RCC_OscInitStruct.PLL.PLLN = 168;               /* PLLN = 168 */
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;     /* PLLP = 2 */
  RCC_OscInitStruct.PLL.PLLQ = 7;                 /* PLLQ = 7 */

  /* 配置振荡器，如果失败则调用错误处理函数 */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** 初始化CPU、AHB和APB总线时钟
    * ClockType: 时钟类型，包括HCLK、SYSCLK、PCLK1、PCLK2
    * SYSCLKSource: 系统时钟源 = PLL时钟
    * AHBCLKDivider: AHB总线时钟分频器 = 1（168MHz）
    * APB1CLKDivider: APB1总线时钟分频器 = 4（42MHz）
    * APB2CLKDivider: APB2总线时钟分频器 = 2（84MHz）
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  /* 系统时钟源 = PLL */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;         /* AHB分频 = 1 */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;          /* APB1分频 = 4 */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;          /* APB2分频 = 2 */

  /* 配置时钟，Flash等待周期设置为5个等待周期 */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  非阻塞模式下的定时器周期回调函数
  * @note   当TIM2中断发生时调用此函数，在HAL_TIM_IRQHandler()内部
  *         直接调用HAL_IncTick()递增全局变量"uwTick"作为应用时基
  * @param  htim: 定时器句柄指针
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  static uint16_t i = 0;  /* 静态变量，用于TIM4的计数 */
  if (htim->Instance == TIM4)  /* 如果是TIM4中断 */
  {

  }
  /* USER CODE END Callback 0 */

  /* 如果是TIM2中断，递增系统时基 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();  /* 递增HAL库的时基计数器 */
  }

  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  错误处理函数
  * @note   当发生错误时调用此函数
  *         关闭所有中断，进入死循环
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();  /* 关闭所有中断 */
  while (1)         /* 死循环 */
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  报告发生assert_param错误的源文件名和行号
  * @note   当使用USE_FULL_ASSERT宏定义时，检查函数参数的有效性
  * @param  file: 指向源文件名的指针
  * @param  line: assert_param错误发生的行号
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
