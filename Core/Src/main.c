/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 主程序入口文件
  * @details        本文件是系统的主程序入口，负责：
  *                 1. 系统时钟配置
  *                 2. 外设初始化
  *                 3. FreeRTOS操作系统初始化
  *                 4. 启动任务调度器
  *                 5. 错误处理
  ******************************************************************************
  * @attention
  * 版权所有 (c) 2025 STMicroelectronics。保留所有权利。
  ******************************************************************************
  */

/* 包含头文件 */
#include "main.h"          /* 主程序头文件 */
#include "cmsis_os.h"      /* CMSIS-OS2接口头文件 */
#include "dma.h"           /* DMA配置头文件 */
#include "fdcan.h"         /* FDCAN配置头文件 */
#include "spi.h"           /* SPI配置头文件 */
#include "tim.h"           /* 定时器配置头文件 */
#include "usart.h"         /* 串口配置头文件 */
#include "gpio.h"          /* GPIO配置头文件 */

/* 用户自定义头文件 */
#include "device.h"        /* 设备层头文件（传感器、电机等） */
#include "driver.h"        /* 驱动层头文件（底层驱动） */

/* 函数原型声明 */
void SystemClock_Config(void);      /* 系统时钟配置函数 */
void PeriphCommonClock_Config(void); /* 外设公共时钟配置函数 */
void MX_FREERTOS_Init(void);        /* FreeRTOS初始化函数 */

/**
  * @brief  主函数 - 程序入口点
  * @details 该函数完成以下工作：
  *          1. 使能CPU缓存（I-Cache）
  *          2. 初始化HAL库（复位外设、初始化Flash、Systick）
  *          3. 配置系统时钟
  *          4. 配置外设公共时钟
  *          5. 初始化所有GPIO引脚
  *          6. 初始化DMA控制器
  *          7. 初始化所有外设（SPI、FDCAN、TIM、USART等）
  *          8. 初始化设备层和驱动层
  *          9. 初始化FreeRTOS并创建任务
  *          10. 启动任务调度器
  * @note   启动调度器后，程序永远不会返回到main函数
  * @retval int 返回值（正常情况下不会执行到这里）
  */
int main(void)
{

  /* 使能CPU指令缓存 - 提高程序执行速度 */
  SCB_EnableICache();

  /* MCU配置 - 复位所有外设、初始化Flash接口、配置Systick */
  HAL_Init();

  /* 配置系统时钟 - 使用HSE和PLL配置系统时钟 */
  SystemClock_Config();

  /* 配置外设公共时钟 - 为FDCAN等外设配置独立的时钟源 */
  PeriphCommonClock_Config();

  /**
    * 初始化所有配置的外设
    * 按照依赖关系顺序初始化：先GPIO，再DMA，然后是各通信接口
    */
  MX_GPIO_Init();           /* 初始化GPIO引脚 */
  MX_DMA_Init();            /* 初始化DMA控制器 */
  MX_SPI2_Init();           /* 初始化SPI2（BMI088 IMU） */
  MX_FDCAN1_Init();         /* 初始化FDCAN1 */
  MX_SPI6_Init();           /* 初始化SPI6（RGB LED） */
  MX_TIM12_Init();          /* 初始化TIM12（蜂鸣器） */
  MX_USART2_UART_Init();   /* 初始化USART2（RS485） */
  MX_FDCAN2_Init();         /* 初始化FDCAN2 */
  MX_FDCAN3_Init();         /* 初始化FDCAN3 */
  MX_USART10_UART_Init();  /* 初始化USART10 */
  MX_UART5_Init();          /* 初始化UART5（遥控器） */
  MX_UART7_Init();          /* 初始化UART7 */
  MX_USART1_UART_Init();   /* 初始化USART1 */
  MX_UART8_Init();          /* 初始化UART8 */
  MX_UART9_Init();          /* 初始化UART9 */

  /**
    * 初始化设备层和驱动层
    * DEVICE_Init(): 初始化所有设备（传感器、执行器等）
    * DRIVER_Init(): 初始化底层驱动（DMA、中断等）
    * @note   这些函数在CubeMX生成的外设初始化之后调用
    *         因为设备初始化依赖于外设（如SPI、UART等）
    */
  DEVICE_Init();
  DRIVER_Init();

  /**
    * FreeRTOS操作系统初始化
    * osKernelInitialize(): 初始化RTOS内核
    * MX_FREERTOS_Init(): 创建所有任务、信号量、队列等
    * osKernelStart(): 启动任务调度器
    * @note   osKernelStart()之后，控制权交给RTOS调度器
    *         main函数中的while(1)循环永远不会执行
    */
  osKernelInitialize();     /* 初始化RTOS内核 */
  MX_FREERTOS_Init();       /* 创建任务和同步对象 */
  osKernelStart();          /* 启动任务调度器 */

  /* 以下代码在正常情况下不会执行（调度器接管后不会返回） */
  while (1)
  {
    /* 空循环 - 仅供RTOS调度器使用 */
  }
}

/**
  * @brief  系统时钟配置函数
  * @details 该函数配置STM32H7的系统时钟：
  *          - 使用HSE（外部高速晶振）作为时钟源
  *          - 通过PLL1倍频到目标频率
  *          - 配置AHB、APB1、APB2、APB3、APB4总线时钟
  * @note   时钟配置说明：
  *         - HSE频率：通常为8MHz或25MHz（取决于晶振）
  *         - PLL1配置：PLLM=2, PLLN=40, PLLP=1
  *         - 系统时钟 = HSE / PLLM * PLLN / PLLP
  *         - 假设HSE=24MHz：系统时钟 = 24/2*40/1 = 480MHz
  *         - AHB时钟 = 系统时钟 / 2 = 240MHz
  *         - APB1/APB2/APB3/APB4 = AHB / 2 = 120MHz
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* 配置电源管理 - 使用LDO稳压器供电 */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /* 配置主内部稳压器输出电压 - 使用最高性能配置 */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  /* 等待电压稳定 */
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /**
    * 配置HSE振荡器和PLL1
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;  /* 使用HSE振荡器 */
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                    /* 使能HSE */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                /* 使能PLL */
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;        /* PLL时钟源：HSE */
  RCC_OscInitStruct.PLL.PLLM = 2;                             /* PLL预分频器：2 */
  RCC_OscInitStruct.PLL.PLLN = 40;                            /* PLL倍频系数：40 */
  RCC_OscInitStruct.PLL.PLLP = 1;                             /* PLL输出分频器：1 */
  RCC_OscInitStruct.PLL.PLLQ = 2;                             /* PLLQ输出分频器：2 */
  RCC_OscInitStruct.PLL.PLLR = 2;                             /* PLLR输出分频器：2 */
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;          /* PLL输入电压范围 */
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;          /* PLL VCO模式：宽范围 */
  RCC_OscInitStruct.PLL.PLLFRACN = 0;                         /* PLL分数分频：无 */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /**
    * 配置CPU、AHB和APB总线时钟
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  /* 系统时钟源：PLL输出 */
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;         /* 系统时钟分频：1 */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;           /* AHB时钟分频：2 */
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;          /* APB3时钟分频：2 */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;          /* APB1时钟分频：2 */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;          /* APB2时钟分频：2 */
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;          /* APB4时钟分频：2 */

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  外设公共时钟配置函数
  * @details 该函数为特定外设配置独立的时钟源：
  *          - FDCAN时钟：使用PLL2
  * @note   FDCAN需要精确的时钟来保证波特率准确
  *         使用PLL2可以独立配置，不影响系统时钟
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**
    * 配置PLL2 - 为FDCAN提供时钟
    * PLL2参数：
    * - PLL2M = 24: PLL2预分频器
    * - PLL2N = 200: PLL2倍频系数
    * - PLL2P = 4: PLL2输出分频器
    * - FDCAN时钟 = HSE / PLL2M * PLL2N / PLL2P
    * - 假设HSE=24MHz：FDCAN时钟 = 24/24*200/4 = 200MHz
    */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.PLL2.PLL2M = 24;
  PeriphClkInitStruct.PLL2.PLL2N = 200;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;  /* FDCAN时钟源：PLL2 */
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  定时器周期溢出回调函数（非阻塞模式）
  * @param  htim: 定时器句柄指针
  * @details 当TIM2中断发生时，HAL库会调用此函数
  *          该函数直接调用HAL_IncTick()来递增全局变量"uwTick"
  *          "uwTick"用作应用的时间基准（毫秒级）
  * @note   这是SysTick的替代实现
  *         当SysTick用于FreeRTOS时，TIM2作为HAL库的时间基准
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* 检查是否是TIM2中断 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();  /* 递增HAL库的毫秒计数器 */
  }
}

/**
  * @brief  错误处理函数
  * @details 当发生错误时调用此函数：
  *          1. 关闭所有中断
  *          2. 进入无限循环，系统停止运行
  * @note   在实际应用中，可以在此处添加错误记录、LED闪烁、蜂鸣器报警等
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();    /* 关闭所有中断 */
  while (1)
  {
    /* 系统停止运行，等待复位 */
  }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  断言失败报告函数
  * @param  file: 源文件名指针
  * @param  line: 断言失败的行号
  * @details 当assert_param()宏检测到参数错误时调用此函数
  *          可以在此处添加错误报告代码（如串口输出）
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* 用户可以在此添加错误报告代码 */
  /* 例如：printf("Wrong parameters value: file %s on line %d\r\n", file, line); */
}
#endif /* USE_FULL_ASSERT */
