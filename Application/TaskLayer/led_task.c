/**
 * @file    led_task.c
 * @brief   LED控制任务
 *
 * 本文件实现了FreeRTOS LED控制任务，用于控制板载LED指示灯的状态显示。
 * 可根据设备工作状态（如正常运行、故障报警、通信状态等）控制LED闪烁频率和模式。
 * 当前为空框架，可根据实际需求扩展LED控制逻辑。
 */

/* Includes ------------------------------------------------------------------*/
#include "led_task.h"   /* LED控制任务头文件 */

/**
 * @brief  LED控制任务入口函数
 * @param  argument: 任务参数指针（未使用）
 * @note   FreeRTOS任务函数，无限循环执行，周期为1ms。
 *         可在此处添加LED状态控制、呼吸灯效果、故障指示等逻辑。
 */
void StartLedTask(void const * argument)
{

    for(;;)  /* 无限循环，任务持续运行 */
    {
        /* 预留接口：可在此处添加LED状态控制逻辑 */

        osDelay(1);  /* 延时1ms，让出CPU资源给其他任务 */
    }
}
