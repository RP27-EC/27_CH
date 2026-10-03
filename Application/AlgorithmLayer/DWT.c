/**
  ******************************************************************************
  * @file    : DWT.c
  * @author  : Ye Jinyi
  * @version : V1.0
  * @date    : 2014.12.2
  * @brief   : DWT（Data Watchpoint and Trace）数据观察点与跟踪模块驱动
  *
  * @details : DWT是ARM Cortex-M系列内核中的一个硬件调试功能模块，其中的周期计数器
  *            （CYCCNT）可以用于精确测量代码执行时间。通过读取DWT->CYCCNT寄存器的值，
  *            可以获得从DWT开启以来的时钟周期数。
  *
  *            使用方法：
  *            1. 调用 DWT_Init() 初始化DWT模块
  *            2. 在需要计时的代码段前后分别读取 CYCCNT 值
  *            3. 时钟周期数之差除以系统时钟频率即为运行时间（秒）
  *               例如：执行时间 = (CYCCNT_end - CYCCNT_start) / SystemCoreClock
  *
  * @attention: DWT->CYCCNT 是32位计数器，会溢出回零。对于高频率时钟（如400MHz），
  *             约10.7秒溢出一次。在短时间测量中溢出概率较小，若担心溢出可多次测量取平均。
  *
  ******************************************************************************
  */

/* 包含头文件 ------------------------------------------------------------------*/
#include "DWT.h"  /* DWT模块头文件，包含寄存器定义和接口声明 */

/**
  * @brief  初始化并开启DWT周期计数器
  * @details 操作步骤：
  *          1. 检查并使能DEMCR（调试异常和监视控制寄存器）的TRCENA位
  *          2. 将周期计数器CYCCNT清零
  *          3. 使能CYCCNT计数器开始计数
  *
  * @note   该函数通常在系统启动时调用一次即可
  * @retval None
  */
void DWT_Init(void)
{
    /* 检查跟踪功能是否已启用（TRCENA位） */
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
    {
        /* 使能DEMCR寄存器的TRCENA位，开启跟踪功能 */
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }

    /* 清零周期计数器，从0开始计数 */
    DWT->CYCCNT = 0;

    /* 使能CYCCNT计数器，开始记录时钟周期 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
  * @brief  获取当前DWT周期计数器的值
  * @details 返回自DWT_Init()调用以来经过的时钟周期数
  *
  * @note   计算实际时间公式：
  *         时间(秒) = 周期数 / 系统时钟频率(Hz)
  *         例如：系统时钟400MHz，CYCCNT差值为4000000，则耗时0.01秒
  *
  * @retval uint32_t: 当前周期计数器的值（32位，会溢出）
  */
uint32_t DWT_GetCycleCount(void)
{
    return DWT->CYCCNT;
}
