/**
 ******************************************************************************
 * @file    WL_debug_uart.c
 * @brief   调试串口驱动文件 - 配合VOFA+上位机使用的调试输出工具
 ******************************************************************************
 * @attention
 *
 * 该文件提供了一个简易的串口调试打印功能，配套VOFA+上位机软件使用。
 * 也可以配合其他串口调试工具使用。
 *
 * 使用方法：
 *   第一步：在WL_debug_uart.h中按需求宏定义（选择板型和串口号）
 *           例如：#define OUR_BOARD 或 #define C_BOARD
 *           例如：#define USE_USART6 或 #define USE_USART1 等
 *   第二步：在main.c中调用 WL_UART_Init() 完成初始化
 *   第三步：在某个任务中调用 WL_UART_printf(char *format, ...) 进行打印
 *           任务执行频率决定数据发送频率
 *
 * 支持的板型：
 *   - OUR_BOARD（队内主控板）：支持USART6、USART3
 *   - C_BOARD（C板）：支持USART1、UART7
 *
 * 注意事项：
 *   - 不需要自己去配置串口，加上这两个文件就可以使用
 *   - 如果已经通过CubeMX配置了串口，则需要注释掉对应的配置，否则会重复定义
 *   - 默认波特率为115200
 *   - 使用VOFA+时需要选择FireWater数据协议（在软件内点?查看说明）
 *
 * @Author: Ye Jinyi
 * @First  Edit Date: 2024.11.28
 * @Latest Edit Date: 2024.11.29
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
/* 包含调试串口头文件、标准输入输出、可变参数和字符串操作头文件 */
#include "WL_debug_uart.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/*============================================================================*/
/*                     使用队内主控板（OUR_BOARD）的配置                        */
/*============================================================================*/
#ifdef OUR_BOARD

  /*======================== 使用串口六（USART6）========================*/
  #ifdef USE_USART6

  /**
   * @brief USART6串口句柄实例
   */
  UART_HandleTypeDef huart6;

  /**
   * @brief  USART6初始化函数
   * @param  无
   * @retval 无
   * @note   配置USART6用于调试输出（仅发送模式）
   *         GPIO引脚：PC6(TX), PC7(RX)
   *         波特率：115200, 数据位：8, 停止位：1, 无校验
   *         仅使能发送模式（UART_MODE_TX）
   */
  void WL_UART_Init(void)
  {
      /* 使能USART6和GPIOC的时钟 */
      __HAL_RCC_USART6_CLK_ENABLE();
      __HAL_RCC_GPIOC_CLK_ENABLE();

      /**USART6 GPIO配置
      PC6     ------> USART6_TX（发送引脚）
      PC7     ------> USART6_RX（接收引脚）
      */
      GPIO_InitTypeDef GPIO_InitStruct = {0};
      GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;       /* 配置PC6和PC7引脚 */
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;               /* 复用推挽输出模式 */
      GPIO_InitStruct.Pull = GPIO_NOPULL;                   /* 无上下拉 */
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;    /* 高速模式 */
      GPIO_InitStruct.Alternate = GPIO_AF8_USART6;          /* 复用功能：USART6 */

      HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);               /* 应用GPIO配置 */

      /* 配置USART6参数 */
      huart6.Instance = USART6;                             /* 选择USART6实例 */
      huart6.Init.BaudRate = 115200;                        /* 波特率：115200 */
      huart6.Init.WordLength = UART_WORDLENGTH_8B;          /* 数据位：8位 */
      huart6.Init.StopBits = UART_STOPBITS_1;               /* 停止位：1位 */
      huart6.Init.Parity = UART_PARITY_NONE;                /* 无校验位 */
      huart6.Init.Mode = UART_MODE_TX;                      /* 仅发送模式 */
      huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;         /* 无硬件流控 */
      huart6.Init.OverSampling = UART_OVERSAMPLING_16;      /* 16倍过采样 */

      HAL_UART_Init(&huart6);                               /* 初始化USART6 */
  }

  /**
   * @brief  调试串口打印函数（格式化输出）
   * @param  format: 格式化字符串（与printf格式相同）
   * @param  ...: 可变参数列表
   * @retval 无
   * @note   将格式化后的字符串通过USART6发送到上位机
   *         缓冲区大小为100字节，请注意不要超过
   *         使用HAL_UART_Transmit阻塞发送，超时1000ms
   */
  void WL_UART_printf(char *format, ...)
  {
      char String[100];              /* 格式化字符串缓冲区（100字节） */
      va_list arg;                   /* 声明可变参数列表 */
      va_start(arg, format);         /* 初始化可变参数列表，指向format后的参数 */
      vsprintf(String, format, arg); /* 将格式化字符串和参数写入缓冲区 */
      va_end(arg);                   /* 结束可变参数处理 */

      /* 通过USART6阻塞发送格式化字符串 */
      HAL_UART_Transmit(&huart6, (uint8_t *)String, strlen(String), 1000);
  }
  #endif
  /* 使用串口六部分结束 */

  /*======================== 使用串口三（USART3）========================*/
  #ifdef USE_USART3

  /**
   * @brief USART3串口句柄实例
   */
  UART_HandleTypeDef huart3;

  /**
   * @brief  USART3初始化函数
   * @param  无
   * @retval 无
   * @note   配置USART3用于调试输出（仅发送模式）
   *         GPIO引脚：PB10(TX), PB11(RX)
   *         波特率：115200, 数据位：8, 停止位：1, 无校验
   */
  void WL_UART_Init(void)
  {
      /* 使能USART3和GPIOB的时钟 */
      __HAL_RCC_USART3_CLK_ENABLE();
      __HAL_RCC_GPIOB_CLK_ENABLE();

      /**USART3 GPIO配置
      PB10    ------> USART3_TX（发送引脚）
      PB11    ------> USART3_RX（接收引脚）
      */
      GPIO_InitTypeDef GPIO_InitStruct = {0};
      GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;      /* 配置PB10和PB11引脚 */
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;                /* 复用推挽输出模式 */
      GPIO_InitStruct.Pull = GPIO_NOPULL;                    /* 无上下拉 */
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;     /* 高速模式 */
      GPIO_InitStruct.Alternate = GPIO_AF7_USART3;           /* 复用功能：USART3 */

      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);                /* 应用GPIO配置 */

      /* 配置USART3参数 */
      huart3.Instance = USART3;                              /* 选择USART3实例 */
      huart3.Init.BaudRate = 115200;                         /* 波特率：115200 */
      huart3.Init.WordLength = UART_WORDLENGTH_8B;           /* 数据位：8位 */
      huart3.Init.StopBits = UART_STOPBITS_1;                /* 停止位：1位 */
      huart3.Init.Parity = UART_PARITY_NONE;                 /* 无校验位 */
      huart3.Init.Mode = UART_MODE_TX;                       /* 仅发送模式 */
      huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;          /* 无硬件流控 */
      huart3.Init.OverSampling = UART_OVERSAMPLING_16;       /* 16倍过采样 */

      HAL_UART_Init(&huart3);                                /* 初始化USART3 */
  }

  /**
   * @brief  调试串口打印函数（格式化输出）
   * @param  format: 格式化字符串
   * @param  ...: 可变参数列表
   * @retval 无
   * @note   通过USART3发送格式化字符串到上位机
   */
  void WL_UART_printf(char *format, ...)
  {
      char String[100];              /* 格式化字符串缓冲区 */
      va_list arg;                   /* 可变参数列表 */
      va_start(arg, format);         /* 初始化可变参数列表 */
      vsprintf(String, format, arg); /* 格式化字符串 */
      va_end(arg);                   /* 结束可变参数处理 */

      /* 通过USART3阻塞发送 */
      HAL_UART_Transmit(&huart3, (uint8_t *)String, strlen(String), 1000);
  }
  #endif
  /* 使用串口三部分结束 */

#endif
/* 使用队内主控板结束 */


/*============================================================================*/
/*                        使用C板（C_BOARD）的配置                             */
/*============================================================================*/
#ifdef C_BOARD

  /*======================== 使用串口一（USART1）========================*/
  #ifdef USE_USART1

  /**
   * @brief USART1串口句柄实例
   */
  UART_HandleTypeDef huart1;

  /**
   * @brief  USART1初始化函数
   * @param  无
   * @retval 无
   * @note   配置USART1用于调试输出（仅发送模式）
   *         GPIO引脚：PA9(TX), PB7(RX)
   *         波特率：115200, 数据位：8, 停止位：1, 无校验
   *         注意：TX和RX引脚分布在不同的GPIO端口（PA和PB）
   */
  void WL_UART_Init(void)
  {
      /* 使能USART1、GPIOB和GPIOA的时钟 */
      __HAL_RCC_USART1_CLK_ENABLE();
      __HAL_RCC_GPIOB_CLK_ENABLE();
      __HAL_RCC_GPIOA_CLK_ENABLE();

      /**USART1 GPIO配置
      PB7     ------> USART1_RX（接收引脚）
      PA9     ------> USART1_TX（发送引脚）
      */

      /* 配置PB7为USART1_RX（复用推挽） */
      GPIO_InitTypeDef GPIO_InitStruct = {0};
      GPIO_InitStruct.Pin = GPIO_PIN_7;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

      /* 配置PA9为USART1_TX（复用推挽） */
      GPIO_InitStruct.Pin = GPIO_PIN_9;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

      /* 配置USART1参数 */
      huart1.Instance = USART1;
      huart1.Init.BaudRate = 115200;
      huart1.Init.WordLength = UART_WORDLENGTH_8B;
      huart1.Init.StopBits = UART_STOPBITS_1;
      huart1.Init.Parity = UART_PARITY_NONE;
      huart1.Init.Mode = UART_MODE_TX;
      huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
      huart1.Init.OverSampling = UART_OVERSAMPLING_16;

      HAL_UART_Init(&huart1);
  }

  /**
   * @brief  调试串口打印函数（格式化输出）
   * @param  format: 格式化字符串
   * @param  ...: 可变参数列表
   * @retval 无
   * @note   通过USART1发送格式化字符串到上位机
   */
  void WL_UART_printf(char *format, ...)
  {
      char String[100];
      va_list arg;
      va_start(arg, format);
      vsprintf(String, format, arg);
      va_end(arg);

      HAL_UART_Transmit(&huart1, (uint8_t *)String, strlen(String), 1000);
  }

  #endif
  /* 使用串口一部分结束 */

  /*======================== 使用串口七（UART7）========================*/
  #ifdef USE_USART7


  /**
   * @brief UART7串口句柄实例
   */
  UART_HandleTypeDef huart7;

  /**
   * @brief  UART7初始化函数
   * @param  无
   * @retval 无
   * @note   配置UART7用于调试输出（仅发送模式）
   *         GPIO引脚：PE7(TX), PE8(RX) - 同一GPIO端口
   *         波特率：115200, 数据位：8, 停止位：1, 无校验
   */
  void WL_UART_Init(void)
  {
      /* 使能UART7和GPIOE的时钟 */
      __HAL_RCC_UART7_CLK_ENABLE();
      __HAL_RCC_GPIOE_CLK_ENABLE();

      /**UART7 GPIO配置
      PE7     ------> UART7_TX（发送引脚）
      PE8     ------> UART7_RX（接收引脚）
      */
      GPIO_InitTypeDef GPIO_InitStruct = {0};
      GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8;        /* 配置PE7和PE8引脚 */
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;                /* 复用推挽输出模式 */
      GPIO_InitStruct.Pull = GPIO_NOPULL;                    /* 无上下拉 */
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;     /* 高速模式 */
      GPIO_InitStruct.Alternate = GPIO_AF7_UART7;            /* 复用功能：UART7 */
      HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);                /* 应用GPIO配置 */

      /* 配置UART7参数 */
      huart7.Instance = UART7;
      huart7.Init.BaudRate = 115200;
      huart7.Init.WordLength = UART_WORDLENGTH_8B;
      huart7.Init.StopBits = UART_STOPBITS_1;
      huart7.Init.Parity = UART_PARITY_NONE;
      huart7.Init.Mode = UART_MODE_TX;
      huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
      huart7.Init.OverSampling = UART_OVERSAMPLING_16;

      HAL_UART_Init(&huart7);
  }

  /**
   * @brief  调试串口打印函数（格式化输出）
   * @param  format: 格式化字符串
   * @param  ...: 可变参数列表
   * @retval 无
   * @note   通过UART7发送格式化字符串到上位机
   */
  void WL_UART_printf(char *format, ...)
  {
      char String[100];
      va_list arg;
      va_start(arg, format);
      vsprintf(String, format, arg);
      va_end(arg);

      HAL_UART_Transmit(&huart7, (uint8_t *)String, strlen(String), 1000);
  }

  #endif
  /* 使用串口七部分结束 */

#endif
/* 使用C板结束 */
