/**
 ******************************************************************************
 * @file        drv_tick.c
 * @author      RobotPilots@2020
 * @brief       系统滴答定时器驱动 - 提供精确的微秒和毫秒级延时功能
 ******************************************************************************
 * @attention
 *
 * 使用Cubemx生成FreeRTOS后，系统会存在两套时基：
 *   1. SysTick - 用于RTOS的任务调度（Cortex-M4内核定时器）
 *   2. HalTick - 用于HAL库的延时和超时判断（本工程使用TIM2）
 *
 * SysTick: 使用Cortex-M4内核的SysTick定时器
 *          SysTick->VAL会在启动任务调度器之后才更新
 *
 * HalTick: 使用TIM2定时器作为HAL时基
 *          TIM2->CNT可提供微秒级精度的时间读取
 *          频率配置为1MHz，即1个计数=1微秒
 *          周期为1ms
 *
 * 注意：delay_us 和 delay_ms 是阻塞型延时，不会引起任务调度
 *
 * Copyright 2020 RobotPilots
 *
 * @Version     V1.0
 * @date        15-September-2020
 ****************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
/* 包含滴答定时器驱动头文件 */
#include "drv_tick.h"

/* Private macro -------------------------------------------------------------*/
/* 私有宏定义（当前未使用） */

/* Private function prototypes -----------------------------------------------*/
/* 私有函数前置声明（当前未使用） */

/* Private typedef -----------------------------------------------------------*/
/* 私有类型定义（当前未使用） */

/* Private variables ---------------------------------------------------------*/
/* 私有变量（当前未使用） */

/* Exported variables --------------------------------------------------------*/

/**
 * @brief 全局毫秒计数变量
 *        用于存储当前时间（微秒精度），由micros()函数更新
 */
uint32_t haltick = 0;

/* Private functions ---------------------------------------------------------*/
/* 私有函数（当前未使用） */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  获取当前时间（微秒精度）
 * @param  无
 * @retval 当前时间，单位为微秒（us）
 * @note   该函数通过组合HAL_GetTick()的毫秒值和TIM2计数器的微秒值
 *         实现微秒级精度的时间戳
 *         计算公式：总微秒数 = 毫秒数 * 1000 + TIM2当前计数值
 *         TIM2配置为1MHz频率，每个计数代表1微秒
 */
uint32_t micros(void)
{
  register uint32_t ms, us;  /* 使用register关键字建议编译器将变量放入寄存器 */

  /* 获取HAL库的毫秒计数值（基于SysTick） */
  ms = HAL_GetTick();

  /* 读取TIM2计数器的当前值（微秒级精度） */
  /* TIM2配置为1MHz => 1个Tick = 1us */
  /* 定时器周期为1ms */
  us = TIM2->CNT;

  /* 组合毫秒和微秒，计算总微秒数 */
  haltick = ms * 1000 + us;

  return haltick;
}

/**
 * @brief  微秒级延时函数（阻塞型）
 * @param  us: 要延时的微秒数
 * @retval 无
 * @note   阻塞型延时，通过不断查询当前时间来实现
 *         不会引起任务调度，在中断中也可以使用
 *         精度取决于TIM2的1MHz时钟配置
 */
void delay_us(uint32_t us)
{
  /* 记录延时开始时的时间戳 */
  uint32_t now = micros();

  /* 忙等待，直到经过指定的微秒数 */
  while((micros() - now) < us);
}

/**
 * @brief  毫秒级延时函数（阻塞型）
 * @param  ms: 要延时的毫秒数
 * @retval 无
 * @note   通过循环调用delay_us(1000)实现毫秒级延时
 *         阻塞型延时，不会引起任务调度
 *         注意：在RTOS任务中使用时会阻塞当前任务
 */
void delay_ms(uint32_t ms)
{
  /* 每次延时1000微秒（1毫秒），循环ms次 */
  while(ms--)
    delay_us(1000);
}

/* Define Exported Functions ------------------------------------------------*/

/**
 * @brief  DWT（Data Watchpoint and Trace）模块初始化
 * @param  无
 * @retval 无
 * @note   DWT是ARM Cortex-M内核提供的硬件调试和跟踪组件
 *         启用DWT的周期计数器(CYCCNT)可以提供最高精度的时钟周期级计时
 *         适用于需要极高精度的性能测量场景
 *         操作步骤：
 *         1. 检查并启用跟踪功能（DEMCR的TRCENA位）
 *         2. 清零周期计数器
 *         3. 使能周期计数器
 */
void DWT_Init(void)
{
  /* 检查是否已经启用了跟踪功能，如果没有则启用 */
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
  {
    /* 启用DWT和ITM等跟踪功能 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  }

  /* 清零周期计数器，从0开始计数 */
  DWT->CYCCNT = 0;

  /* 使能DWT周期计数器，开始计数 */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief  获取DWT周期计数器的当前值
 * @param  无
 * @retval 当前的时钟周期数
 * @note   返回值为自DWT_Init()调用以来经过的CPU时钟周期数
 *         可用于精确测量代码执行时间
 *         计算时间公式：时间(秒) = 周期数 / CPU主频(Hz)
 *         例如：CPU主频168MHz时，168000个周期=1ms
 */
uint32_t DWT_GetCycleCount(void)
{
  return DWT->CYCCNT;
}
