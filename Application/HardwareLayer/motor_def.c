/**
  ******************************************************************************
  * @file    motor_def.c
  * @brief   电机通用定义与PID结构体初始化
  *          本文件提供了电机PID控制器结构体的通用初始化函数。
  *          motor_pid_t 结构体定义了PID控制所需的基本参数：
  *          - kp: 比例增益
  *          - ki: 积分增益
  *          - kd: 微分增益
  *          - target: 目标值
  *          - measure: 测量值(反馈值)
  *          - err: 当前误差
  *          - last_err: 上一次误差
  *          - integral: 积分累加值
  *          - out: PID输出
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motor_def.h"

/**
  * @brief  电机PID结构体初始化 - 使用外部传入的PID参数初始化目标PID结构体
  * @param  motor_pid:       目标PID结构体指针，需要被初始化的PID实例
  * @param  extern_motor_pid: 源PID结构体，包含初始化参数值
  * @retval 无
  * @note   通过结构体直接赋值的方式完成初始化
  *         如果传入空指针则直接返回，避免空指针解引用
  *         使用场景：在电机初始化阶段，为各电机PID控制器设置默认参数
  */
void motor_pid_init(motor_pid_t *motor_pid, motor_pid_t extern_motor_pid)
{
	if (motor_pid == NULL) return;  /* 空指针检查，防止程序崩溃 */
	*motor_pid = extern_motor_pid;  /* 整体结构体赋值，复制所有PID参数 */
}

 
