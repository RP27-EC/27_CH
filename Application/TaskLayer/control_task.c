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
#include "carctrl.h"

/**
  * @brief  控制任务主函数
  * @details FreeRTOS任务函数，运行周期2ms，负责：
  *          1. 初始化底盘
  *          2. 循环读取遥控器输入
  *          3. 检查遥控器在线状态（失联保护）
  *          4. 调用底盘控制函数更新电机转速
  *          5. 控制云台偏航电机（当前为停止状态）
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

    osDelay(100);

    next_wake = osKernelGetTickCount();  /* 获取当前时间戳 */
    /* 主控制循环 */
    for (;;) {
        /*==========================================================================
         * 失联保护检测
         * 当遥控器离线或存在错误时，触发失联保护，立即停止底盘
         *==========================================================================*/
        Car_Control_Update(&hero_car);
        Gimbal_Update(&hero_gimbal, hero_car.mode);
        const bool chassis_failsafe = (hero_car.mode != CAR_MODE_MECH);

        /*==========================================================================
         * 云台偏航电机控制（当前为停止状态）
         * 将偏航电机设置为睡眠模式，力矩清零
         *==========================================================================*/
        /*==========================================================================
         * 底盘控制更新
         * 读取遥控器三轴通道值，调用底盘控制函数
         * 通道映射：
         * - ch3: 前后方向（遥控器右摇杆上下）
         * - ch2: 左右方向（遥控器右摇杆左右）
         * - ch0: 旋转方向（遥控器左摇杆左右）
         *==========================================================================*/
        Chassis_Update(rc_sensor.info->ch3,   /* 前后方向通道值 */
                       rc_sensor.info->ch2,   /* 左右方向通道值 */
                       rc_sensor.info->ch0,   /* 旋转方向通道值 */
                       chassis_failsafe);


        /* 任务调度：等待2ms后再次执行 */
        next_wake += 2U;              /* 设置唤醒时间为最开始进入这个函数2ms后 */
        osDelayUntil(next_wake);      /* 精确延时到下次唤醒时间  OS 让任务进入 Blocked 阻塞态，让出 CPU*/
    }
}
