/**
 ******************************************************************************
 * @file        drv_tick.c
 * @author      RobotPilots@2020
 * @brief       HAL时基驱动 - 提供微秒级和毫秒级延时功能
 *              本文件基于TIM2定时器实现了高精度的时间获取和延时函数。
 *              当使用FreeRTOS时，系统存在两套时基：
 *              1. SysTick - 用于RTOS任务调度（Cortex-M4内核定时器）
 *              2. HalTick - 用于HAL库延时（TIM2定时器，本文件实现）
 *              这样做的原因是FreeRTOS会占用SysTick，HAL库需要独立的时基。
 ******************************************************************************
 * @attention
 *
 * Copyright 2020 RobotPilots
 *
 * @note
 * 使用cubemx生成FREERTOS后会建议将SYS的时基切换成除SysTick之外的定时器
 * 从而系统会存在两套时基，①用于RTOS的SysTick ②用于HAL的HalTick
 * SysTick 使用cortex-m4内核的SysTick (SysTick->VAL会在启动任务调度器之后才更新)
 * HalTick 在本工程里面使用TIM2 (TIM2->CNT可提供微妙级延时)
 * # delay_us 和 delay_ms 不会引起任务调度(阻塞型)
 *
 * @Version     V1.0
 * @date        15-September-2020
 ****************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_tick.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/**
  * HAL时基全局变量，存储当前微秒级时间戳
  * 由 micros() 函数更新
  */
uint32_t haltick = 0;

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  获取当前微秒级时间戳
  * @param  无
  * @retval 当前时间（微秒），范围约0~4294967295(约71分钟溢出)
  * @note   计算公式: 时间 = HAL_GetTick() * 1000 + TIM2->CNT
  *         - HAL_GetTick(): 毫秒级计数（由TIM2中断累加）
  *         - TIM2->CNT: 当前毫秒内的微秒计数（TIM2计数器当前值）
  *         两部分组合得到微秒级精度的时间戳
  *         Freq:1MHz => TIM2每个计数=1微秒
  *         Period:1ms => TIM2每1毫秒产生一次中断
  *         使用register变量提高执行效率
  */
uint32_t micros(void)
{
	register uint32_t ms, us;  /* 使用register关键字提示编译器将变量放入寄存器 */

	ms = HAL_GetTick();  /* 获取毫秒级计数值（TIM2中断累加） */
	/* 选用定时器2作为HAL时基的TimeBase */
	/* Freq:1MHz => 1Tick = 1us */
	/* Period:1ms */
	us = TIM2->CNT;  /* 读取TIM2计数器当前值，获取毫秒内的微秒偏移 */

	/* 组合计算微秒级时间戳 */
    haltick = ms * 1000 + us;

	return haltick;
}

/**
  * @brief  微秒级阻塞延时
  * @param  us: 要延时的微秒数
  * @retval 无
  * @note   阻塞型延时，会占用CPU时间，不会引起任务调度
  *         原理：记录起始时间，循环等待直到经过指定微秒数
  *         精度受限于循环体执行时间和中断影响，适合短延时(<10ms)
  */
void delay_us(uint32_t us)
{
	uint32_t now = micros();  /* 记录延时开始时的时间戳 */

	/* 忙等待：不断查询当前时间直到经过指定微秒数 */
	while((micros() - now) < us);
}

/**
  * @brief  毫秒级阻塞延时
  * @param  ms: 要延时的毫秒数
  * @retval 无
  * @note   阻塞型延时，会占用CPU时间，不会引起任务调度
  *         实现方式：将毫秒延时分解为多个1000微秒的延时
  *         适合不需要RTOS任务调度的场景
  */
void delay_ms(uint32_t ms)
{
	while(ms--)
		delay_us(1000);  /* 每次延时1000微秒=1毫秒 */
}


