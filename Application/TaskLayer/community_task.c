/**
 * @file    community_task.c
 * @brief   社区通信任务
 *
 * 本文件实现了FreeRTOS社区通信任务，用于模块间或与上位机之间的
 * 通信数据处理。当前为空框架，可根据实际需求扩展通信逻辑。
 */

/* Includes ------------------------------------------------------------------*/
#include "community_task.h" /* 社区通信任务头文件 */

/**
 * @brief  社区通信任务入口函数
 * @param  argument: 任务参数指针（未使用）
 * @note   FreeRTOS任务函数，无限循环执行，周期为1ms。
 *         可在此处添加串口通信、网络通信等数据收发逻辑。
 */
void StartCommunityTask(void const *argument)
{

    for (;;)  /* 无限循环，任务持续运行 */
    {
        /* 预留接口：可在此处添加通信数据处理逻辑 */

        osDelay(1);  /* 延时1ms，让出CPU资源给其他任务 */
    }
}
