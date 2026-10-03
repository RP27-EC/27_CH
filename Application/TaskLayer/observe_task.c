/**
  ******************************************************************************
  * @file    observe_task.c
  * @brief   状态量观测任务
  *          1. 更新底盘各种状态变量
  * @details 本文件实现FreeRTOS状态量观测任务，负责周期性地采集和更新
  *          底盘各传感器的状态数据，包括：
  *          - IMU惯性测量单元数据更新（加速度、角速度、姿态角）
  *          - 底盘电机状态更新（编码器值、转速、温度等，通过CAN中断自动更新）
  *          - 云台电机状态更新
  *
  *          该任务在RTOS调度下周期运行，确保各传感器数据保持最新，
  *          供控制任务（control_task）使用。
  *
  *          任务调度信息：
  *          - 任务周期：1ms
  *          - 优先级：较高（数据采集需要及时）
  ******************************************************************************
  */
#include "observe_task.h"
#include "imu_sensor.h"
#include "chassis_motor.h"
#include "gimbal_motor.h"

/**
  * @brief  状态量观测任务主函数
  * @details FreeRTOS任务函数，在无限循环中执行以下操作：
  *          1. 调用IMU传感器的update函数，读取并处理IMU原始数据，
  *             更新加速度、角速度、欧拉角等姿态信息
  *          2. 周期性等待1ms，保持任务调度节拍
  *
  *          注：底盘电机和云台电机的状态数据通过CAN接收中断自动更新，
  *          无需在此任务中显式调用。
  *
  * @param  argument : 任务参数指针（未使用）
  * @retval None
  */
void StartUpdataTask(void const * argument)
{
    for(;;)
    {
        /* 更新IMU传感器数据（读取BMI088等IMU芯片的最新姿态数据） */
        imu_sensor.update(&imu_sensor);

        /* 等待1ms，让出CPU给其他任务 */
        osDelay(1);
    }
}
