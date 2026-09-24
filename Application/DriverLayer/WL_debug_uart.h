#ifndef WL_DEBUG_UART
#define WL_DEBUG_UART

/* Including Files -------------------------------------------------------------------------------------------------------------------------------------------------- */

#include "stm32h7xx_hal.h"

/* Macro ------------------------------------------------------------------------------------------------------------------------------------------------- */
/*选择定义：队内绿色小主控定义 -- OUR_BOARD ，并根据需要定义 USE_USART3 或 USE_USART6 */
/*选择定义：C板定义 ------------- C_BOARD   ，并根据需要定义 USE_USART1 或 USE_USART6 */
#define C_BOARD
#define USE_USART7

/* Define Exported Variables Type ----------------------------------------------------------------------------------------------------------------------------------- */


/* Define Privated Variables Type ----------------------------------------------------------------------------------------------------------------------------------- */


/* Exported Variables Declarations ---------------------------------------------------------------------------------------------------------------------------------- */
/* 使用队内主控开始 */
#ifdef OUR_BOARD
  
  #ifdef USE_USART6
      extern UART_HandleTypeDef huart6;
  #endif
  
  #ifdef USE_USART3
      extern UART_HandleTypeDef huart3;
  #endif
  
#endif
/* 使用队内主控结束 */


/* 使用C板开始 */
#ifdef C_BOARD
  
  #ifdef USE_USART1
      extern UART_HandleTypeDef huart1;
  #endif
  
  #ifdef USE_USART6
      extern UART_HandleTypeDef huart6;
  #endif
  
#endif
/* 使用C板结束 */

/* Exported Functions Declarations ---------------------------------------------------------------------------------------------------------------------------------- */
void WL_UART_Init(void);
void WL_UART_printf(char *format, ...);

#endif
