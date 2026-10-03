/**
 * @file    monitor_task.c
 * @brief   监控任务
 *          1. 各模块心跳失联检测
 *          2. 监控遥控器状态，软件复位
 */

/* Includes ------------------------------------------------------------------*/
#include "monitor_task.h"
#include "DM_Motor.h"
#include "motor.h"       /* 六摩擦轮电机组心跳 */


void StartMonitorTask(void const *argument)
{

    for (;;)
    {
        if (Pitch_Motor.single_heart_beat != NULL) {
            Pitch_Motor.single_heart_beat(&Pitch_Motor);
        }

        /* 六摩擦轮 RM3508 心跳失联检测（前后两个发送组） */
        rm_motor_list_heart_beat();

        osDelay(1);  /* 延时1ms，让出CPU资源给其他任务 */
    }
}
