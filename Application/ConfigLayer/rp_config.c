/**
  ******************************************************************************
  * @file        rp_config.c
  * @author      RobotPilots
  * @Version     v1.0
  * @brief       RobotPilots机器人底层硬件配置 —— ARM汇编内联指令
  * @details     本文件提供了一组直接操作ARM Cortex-M内核寄存器的汇编指令封装函数。
  *              这些函数通常用于以下场景：
  *              - 低功耗模式：WFI（Wait For Interrupt）等待中断唤醒
  *              - 中断管理：全局中断的开启和关闭
  *              - 栈操作：设置主栈指针MSP的值（用于 bootloader 跳转等）
  *
  *              注意：Keil MDK使用 __asm 关键字进行C语言内联汇编，
  *              与GCC语法不同。THUMB指令集下不支持内联汇编，
  *              因此采用独立的汇编函数方式实现。
  *
  * @update
  *              v1.0(9-September-2020)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  执行WFI（Wait For Interrupt）汇编指令
  * @details 让CPU进入低功耗等待状态，直到收到中断信号才恢复执行。
  *          常用于空闲任务或低功耗模式，减少CPU功耗。
  *
  *          注意：THUMB指令集不支持C语言内联汇编，因此使用独立的汇编函数。
  *
  * @retval None
  */
__asm void WFI_SET(void)
{
    WFI;          /* ARM WFI指令：等待中断，CPU进入低功耗暂停状态 */
}

/**
  * @brief  关闭全局中断（CPSID I指令）
  * @details 通过CPSID I指令关闭全局可屏蔽中断（IRQ）。
  *          注意：此操作不会屏蔽Fault和NMI中断（不可屏蔽中断）。
  *          常用于临界区保护，防止多任务或中断嵌套导致的数据竞争。
  *
  * @retval None
  */
__asm void INTX_DISABLE(void)
{
    CPSID   I     /* 关闭全局可屏蔽中断 */
    BX      LR    /* 返回调用者（LR寄存器保存了返回地址） */
}

/**
  * @brief  开启全局中断（CPSIE I指令）
  * @details 通过CPSIE I指令重新开启全局可屏蔽中断。
  *          应与INTX_DISABLE配对使用，确保中断恢复。
  *
  * @retval None
  */
__asm void INTX_ENABLE(void)
{
    CPSIE   I     /* 开启全局可屏蔽中断 */
    BX      LR    /* 返回调用者 */
}

/**
  * @brief  设置主栈指针（MSP）的值
  * @details 通过MSR指令直接设置ARM主栈指针寄存器。
  *          常用于：
  *          - Bootloader跳转到应用程序前设置栈指针
  *          - 异常处理后恢复栈指针
  *          - 手动管理栈空间
  *
  * @param  addr : 要设置的栈顶地址（必须4字节对齐）
  * @retval None
  */
__asm void MSR_MSP(uint32_t addr)
{
    MSR MSP, r0   /* 将r0寄存器的值（即参数addr）写入MSP寄存器 */
    BX r14        /* 返回调用者（BX LR的另一种写法） */
}
