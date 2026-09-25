/**
  ******************************************************************************
  * @file           : control_task.c
  * @brief          : FreeRTOS控制任务
  * @details        : 底盘和云台的主控制任务，运行周期2ms。
  *                   负责读取遥控器输入、检查系统状态、调用底盘和云台控制函数。
  ******************************************************************************
  */

#include "control_task.h"

#include "chassis.h"       /* 底盘控制模块 */
#include "gimbal_motor.h"  /* 云台电机模块 */
#include "rc_sensor.h"     /* 遥控器传感器模块 */
#include "gimbal.h"
#include "car_control.h"
#include "communicate.h"   /* 板间通信模块 */

/**
  * @brief  控制任务主函数
  * @details FreeRTOS任务函数，运行周期2ms，负责：
  *          1. 初始化底盘、云台、状态机与通信
  *          2. 循环读取遥控器输入
  *          3. 检查遥控器与板间通信在线状态（失联保护）
  *          4. 调用云台和底盘控制函数更新电机
  *
  * @param  argument: 任务参数（未使用）
  * @retval None
  */
void StartCtrlTask(void *argument)
{
    uint32_t next_wake;
    (void)argument;  // 不让编译器报 warning

    Chassis_Init();  /* 初始化底盘，确保上电后静止 */
    Gimbal_Init(&hero_gimbal);
    Car_Control_Init(&hero_car);
    Communicate_Init();

    osDelay(100);

    next_wake = osKernelGetTickCount();  /* 获取当前时间戳 */

    for (;;) {
        /* 0. 板间通信看门狗超时检测 */
        Communicate_Check_Timeout();

        /* 1. 状态机更新（解析摇杆与拨轮切模） */
        Car_Control_Update(&hero_car);

        /* 2. 云台控制更新（执行 Yaw 轴串级 PID 计算并驱动 KT 电机） */
        Gimbal_Update(&hero_gimbal, hero_car.mode);

        /* 3. 失联保护检测：整车休眠或遥控器离线时触发保护 */
        const bool chassis_failsafe = (hero_car.mode == CAR_MODE_SLEEP ||
                                       rc_sensor.work_state != DEV_ONLINE);

        /*==========================================================================
         * 底盘控制更新
         * 读取遥控器三轴通道值与云台相对夹角，调用底盘控制函数
         *==========================================================================*/
        Chassis_Update(rc_sensor.info->ch3,                  /* 前后平移 (ch3) */
                       rc_sensor.info->ch2,                  /* 左右平移 (ch2) */
                       rc_sensor.info->ch0,                  /* 机械自旋 (ch0) */
                       hero_gimbal.yaw.feedback_mec_angle,   /* 云台相对车头夹角 Δθ */
                       hero_car.mode,                        /* 整车当前模式 */
                       chassis_failsafe);                    /* 失联保护标志 */

        /* 任务调度：等待2ms后再次执行 */
        next_wake += 2U;              /* 设置唤醒时间为最开始进入这个函数2ms后 */
        osDelayUntil(next_wake);      /* 精确延时到下次唤醒时间 OS 让任务进入 Blocked 阻塞态，让出 CPU */
    }
}
