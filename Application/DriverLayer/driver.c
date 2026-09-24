/**
 ******************************************************************************
 * @file        driver.c
 * @author      RobotPilots@2020
 * @brief       Drivers' Manager.
 ******************************************************************************
 * @attention   
 * 
 * Copyright 2020 RobotPilots
 *  
 * @Version     V1.0
 * @date        9-September-2020
 ****************************************************************************
 */
 
/* Includes ------------------------------------------------------------------*/
#include "driver.h"
#include "WL_debug_uart.h"
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void DRIVER_Init(void)
{
	USART10_Init();
	USART5_Init();
	USART1_Init();
	USART7_Init();
	USART8_Init();
	USART9_Init();
	CAN1_Filter_Init();
	CAN2_Filter_Init();
	CAN3_Filter_Init();
}
