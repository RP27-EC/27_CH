/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------- 
 * @Files:WL_debug_uart.c/h
 * 
 * @Author: Ye Jinyi
 * 
 * @First  Edit Date: 2024.11.28
 * @Latest Edit Date: 2024.11.29
 * 
 * @illustrate: 配套vofa+使用，也可以自己用别的
 *              使用方法，第一步：在WL_debug_uart.h按需求宏定义
 *                        第二步：在main.c调用WL_UART_Init(void)初始化
 *                        第三步：在某个任务调用WL_UART_printf(char *format, ...)，任务执行频率决定发送频率
 * 
 * @attention: 不用自己去配串口，加上这两个文件就可以用。如果配了就要把配的注释掉，否则会重复定义。
 *             波特率为115200，也可以自己改。
 *             使用vofa+就要用软件内的FireWater数据协议，在软件内点？就可以看。
 * 
 *-------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/* Including Files  ------------------------------------------------------------------------------------------------------------------------------------------------- */
#include "WL_debug_uart.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* Separated Code Blocks ------------------------------------------------------------------------------------------------------------------------------------------------- */
/* 使用队内主控开始 */
#ifdef OUR_BOARD
  
  /* 使用串口六部分开始 */
  #ifdef USE_USART6
  
  UART_HandleTypeDef huart6;
  
  void WL_UART_Init(void)
  {
      __HAL_RCC_USART6_CLK_ENABLE();
      __HAL_RCC_GPIOC_CLK_ENABLE();
      /**USART6 GPIO Configuration
      PC6     ------> USART6_TX
      PC7     ------> USART6_RX
      */
      GPIO_InitTypeDef GPIO_InitStruct = {0};
      GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
      
      HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    
    huart6.Instance = USART6;
    huart6.Init.BaudRate = 115200;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    
    HAL_UART_Init(&huart6);
    
  }
  
  void WL_UART_printf(char *format, ...)
  {
      char String[100];
      va_list arg;
      va_start(arg, format);
      vsprintf(String, format, arg);
      va_end(arg);
      
      HAL_UART_Transmit(&huart6, (uint8_t *)String, strlen(String), 1000);
  }
  #endif
  /* 使用串口六部分结束 */
  
  /* 使用串口三部分开始 */
  #ifdef USE_USART3
  
  UART_HandleTypeDef huart3;
  
  void WL_UART_Init(void)
  {
      __HAL_RCC_USART3_CLK_ENABLE();
      __HAL_RCC_GPIOB_CLK_ENABLE();
      /**USART3 GPIO Configuration
      PB10     ------> USART3_TX
      PB11     ------> USART3_RX
      */
      GPIO_InitTypeDef GPIO_InitStruct = {0};
      GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
      
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
      
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    
    HAL_UART_Init(&huart3);
    
  }
  
  void WL_UART_printf(char *format, ...)
  {
      char String[100];
      va_list arg;
      va_start(arg, format);
      vsprintf(String, format, arg);
      va_end(arg);
      
      HAL_UART_Transmit(&huart3, (uint8_t *)String, strlen(String), 1000);
  }
  #endif
  /* 使用串口三部分结束 */
  
#endif
/* 使用队内主控结束 */


/* 使用C板开始 */
#ifdef C_BOARD
  
  /* 使用串口一部分开始 */
  #ifdef USE_USART1
  
  UART_HandleTypeDef huart1;
  
  void WL_UART_Init(void)
  {
      __HAL_RCC_USART1_CLK_ENABLE();
      __HAL_RCC_GPIOB_CLK_ENABLE();
      __HAL_RCC_GPIOA_CLK_ENABLE();
      
      /**USART1 GPIO Configuration
      PB7     ------> USART1_RX
      PA9     ------> USART1_TX
      */
      GPIO_InitTypeDef GPIO_InitStruct = {0};
      GPIO_InitStruct.Pin = GPIO_PIN_7;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
      
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
      
      GPIO_InitStruct.Pin = GPIO_PIN_9;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
      
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
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
  
  /* 使用串口六部分开始 */
  #ifdef USE_USART7
	
  
  UART_HandleTypeDef huart7;
  
  void WL_UART_Init(void)
  {
      __HAL_RCC_UART7_CLK_ENABLE();
      __HAL_RCC_GPIOE_CLK_ENABLE();
      
      /**USART6 GPIO Configuration
      PG14     ------> USART6_TX
      PG9     ------> USART6_RX
      */
      GPIO_InitTypeDef GPIO_InitStruct = {0};
      GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF7_UART7;
      HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
      
      
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
  /* 使用串口六部分结束 */
  
#endif
/* 使用C板结束 */
