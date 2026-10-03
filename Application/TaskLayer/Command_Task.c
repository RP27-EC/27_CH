/**
  ******************************************************************************
  * @file    Command_Task.c
  * @brief   指令更新任务
  *          更新整车标志位和模式
  * @details 本文件实现FreeRTOS指令更新任务，负责处理遥控器输入的模式切换指令，
  *          更新整车的运行模式和标志位。
  *
  *          当前该任务的主体逻辑已被注释掉（keyboard_update），
  *          保留任务框架以便后续扩展使用。
  *
  *          任务调度信息：
  *          - 任务周期：由osDelay控制，当前为1ms
  *          - 优先级：中等（具体优先级由FreeRTOS配置决定）
  ******************************************************************************
  */
#include "Command_Task.h"
#include "rc_sensor.h"
#include "rc_protocol.h"

/**
  * @brief  指令更新任务主函数
  * @details FreeRTOS任务函数，在无限循环中执行以下操作：
  *          1. 键盘状态更新（当前已注释，由主控制任务统一处理）
  *          2. 周期性等待，保持任务调度节拍
  *
  * @param  argument : 任务参数指针（未使用）
  * @retval None
  *
  * @note   该任务当前为轻量级占位任务，主要的指令处理逻辑
  *         已集成到control_task.c的主控制循环中
  */
void StartCommandTask(void const * argument)
{
    for(;;)
    {
        /* 键盘状态更新（当前已注释，改由control_task统一处理） */
//      keyboard_update(rc_sensor.info);

        /* 等待1ms，让出CPU给其他任务 */
        osDelay(1);
    }
}
