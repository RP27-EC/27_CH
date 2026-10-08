/**
  ******************************************************************************
  * @file    monitor_task.c
  * @brief   监控任务
  *          1. 各模块心跳失联检测
  *          2. 监控遥控器状态，软件复位
  * @details 本文件实现FreeRTOS监控任务，负责对系统中各关键模块进行
  *          心跳检测和离线监控，确保系统安全运行。
  *
  *          监控的模块包括：
  *          - 遥控器（RC传感器）：检测DBUS通信是否正常
  *          - IMU惯性测量单元：检测姿态传感器是否在线
  *          - KT9025 Yaw电机：检测云台Yaw轴电机通信是否正常
  *
  *          当某个模块的心跳计数器超过阈值时，该模块会被标记为离线，
  *          上层控制逻辑会据此执行安全保护动作（如停止电机、进入休眠等）。
  *
  *          任务调度信息：
  *          - 任务周期：1ms
  *          - 优先级：最高（监控任务需要优先执行）
  ******************************************************************************
  */
#include "monitor_task.h"
#include "device.h"
#include "gimbal_motor.h"
#include "kt_yaw_motor.h"
#include "cap.h"

/**
  * @brief  监控任务主函数
  * @details FreeRTOS任务函数，在无限循环中周期执行各模块的心跳检测：
  *
  *          1. 遥控器心跳检测：检查DBUS串口是否持续收到数据，
  *             若超过阈值未收到则标记遥控器离线
  *          2. IMU传感器心跳检测：检查IMU SPI/I2C通信是否正常，
  *             若通信失败次数过多则标记IMU离线
  *          3. KT电机心跳检测：检查KT9025 Yaw电机CAN通信是否正常
  *
  * @param  argument : 任务参数指针（未使用）
  * @retval None
  */
void StartMonitorTask(void const * argument)
{
    for(;;)
    {
        /* 遥控器心跳检测：递增离线计数器，收到数据时在中断中清零 */
        rc_sensor.heart_beat(&rc_sensor);

        /* IMU传感器心跳检测：检查IMU通信状态 */
        imu_sensor.heart_beat(&imu_sensor.work_state);

        /* KT9025 Yaw电机心跳检测：检查CAN通信是否正常 */
        KT_Yaw_Heartbeat();

        /* 裁判系统已停用（不做功率限制、不读裁判数据），心跳刷新注释掉 */
        // My_Judge_Realtime_Task(&My_Judge);

        /* 超级电容心跳检测：300ms 收不到 0x211 判离线。
           超电当前为只读调试：heartbeat 只维护 cap.status，控制路径不消费 */
        cap.heartbeat(&cap);

        /* 等待1ms，让出CPU给其他任务 */
        osDelay(1);
    }
}
