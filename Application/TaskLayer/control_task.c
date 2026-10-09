/**
  ******************************************************************************
  * @file           : control_task.c
  * @brief          : FreeRTOS控制任务 —— 整车主控制循环
  * @details        : 本文件实现底盘和云台的主控制任务，是整个软件系统的核心调度任务。
  *                   运行周期2ms（500Hz），负责以下工作：
  *
  *                   1. 系统初始化：底盘、云台、状态机、板间通信初始化
  *                   2. 板间通信超时检测（看门狗）
  *                   3. 整车状态机更新（模式切换与动作指令解析）
  *                   4. 云台控制更新（Yaw轴串级PID闭环）
  *                   5. 失联保护检测
  *                   6. 底盘控制更新（麦轮运动学解算与电机驱动）
  *
  *                   控制流程（每2ms执行一次）：
  *                   Communicate_Check_Timeout -> Car_Control_Update -> Gimbal_Update -> Chassis_Update
  ******************************************************************************
  */

#include "control_task.h"

#include "chassis.h"       /* 底盘控制模块 */
#include "gimbal_motor.h"  /* 云台电机模块 */
#include "rc_sensor.h"     /* 遥控器传感器模块 */
#include "rc_protocol.h"   /* 键盘按键状态机 */
#include "gimbal.h"
#include "car_control.h"
#include "communicate.h"   /* 板间通信模块 */
#include "shoot.h"         /* 拨弹控制模块 */
#include "rp_config.h"


void StartCtrlTask(void *argument)
{
    uint32_t next_wake;
    (void)argument;  /* 消除未使用参数警告 */

    /* ====== 系统初始化阶段 ====== */
    Chassis_Init();              /* 初始化底盘，确保上电后所有轮子静止 */
    Gimbal_Init(&hero_gimbal);   /* 初始化云台，设置PID参数、进入休眠模式 */
    Car_Control_Init(&hero_car); /* 初始化整车状态机，默认SLEEP模式 */
    Communicate_Init();          /* 初始化板间通信，标记上层C板为离线 */
    Shoot_Init();                /* 初始化拨弹电机与拨弹状态机 */

    osDelay(100);  /* 等待100ms，确保各硬件设备就绪 */

    next_wake = osKernelGetTickCount();  /* 获取RTOS当前时间戳作为基准 */


    for (;;) {

        /*板间通信看门狗*/
        Communicate_Check_Timeout();

        /* 键盘/鼠标按键状态机：DBUS 中断只解析出电平，边沿/长按在这里流转 */
        keyboard_update(rc_sensor.info);

        /*整车状态机更新 */
        Car_Control_Update(&hero_car);

        /*云台控制更新*/
        Gimbal_Update(&hero_gimbal, hero_car.mode);

        /* 当整车处于休眠模式或遥控器离线时，触发底盘失联保护 */
        const bool chassis_failsafe = (hero_car.mode == CAR_MODE_SLEEP ||
                                       rc_sensor.work_state != DEV_ONLINE);

        /* 拨弹：消费整车标志，驱动拨盘 RM3508，并写 is_fric_on */
        Shoot_Update(&hero_car, chassis_failsafe);

        /* 通过 0xD1 发给上层板（pitch 目标 + 域标志 + 摩擦轮标志）
           GYRO/SPIN：发 IMU 域目标，pitch_use_imu=1，上层用自带 IMU 闭环
           MECH：发编码器机械目标，pitch_use_imu=0，上层编码器闭环 */
        const bool pitch_imu_domain =
            (hero_gimbal.mode == GIMBAL_MODE_GYRO); /* SPIN 在云台侧也映射为 GYRO */
        board_tx_info.pitch_encoder_tar =
            pitch_imu_domain ? hero_gimbal.pitch.target_imu_angle
                             : hero_gimbal.pitch.target_mec_angle;
        board_tx_info.pitch_use_imu = pitch_imu_domain ? 1U : 0U;
        board_tx_info.pitch_enable =
            (hero_gimbal.mode != GIMBAL_MODE_SLEEP &&
             hero_gimbal.mode != GIMBAL_MODE_INIT &&
             !hero_gimbal.pitch.target_rebase_pending &&
             (!pitch_imu_domain ||
              (board_rx_info.is_pitch_online && board_rx_info.is_pitch_mech_online)) &&
             rc_sensor.work_state == DEV_ONLINE &&
             rc_sensor.errno == NONE_ERR) ? 1U : 0U;
        Board_Tx_D1();

        /* 底盘三轴通道：按输入源取摇杆 ch3/ch2/ch0 或键鼠 WASD 折算值，
           统一为 SBUS 计数域，底盘模块不需要知道操作手用的是哪套 */
        int16_t chassis_fwd, chassis_lat, chassis_rot;
        Car_Chassis_Channel_Get(&hero_car, &chassis_fwd, &chassis_lat, &chassis_rot);

        Chassis_Update(chassis_fwd,                          /* 前后平移通道 */
                       chassis_lat,                          /* 左右平移通道 */
                       chassis_rot,                          /* 自旋转通道 */
                       hero_gimbal.yaw.feedback_mec_angle,   /* 云台相对车头夹角 Δθ（弧度） */
                       hero_gimbal.yaw.align_dir,            /* 双向就近对齐：FRONT/BACK */
                       hero_car.mode,                        /* 整车当前模式 */
                       chassis_failsafe);                    /* 失联保护标志 */

        /*--- 任务调度：精确等待2ms后再次执行 ---*/
        next_wake += 2U;              /* 设置目标唤醒时间 = 初始时间 + 2ms */
        osDelayUntil(next_wake);      /* 精确延时到目标时间，让出CPU给其他低优先级任务 */
    }
}
