/**
  ******************************************************************************
  * @file           : device.c
  * @brief          : 设备管理层 - 统一初始化所有外设和传感器
  * @details        : 作为整个设备层的入口，负责按顺序初始化所有硬件设备：
  *                   1. 遥控器（UART通信）
  *                   2. IMU惯性测量单元（SPI通信）
  *                   3. 底盘电机组（CAN通信）
  *                   4. Yaw轴KT电机（CAN通信）
  *                   5. 超级电容（只读调试，心跳/状态）
  *                   （裁判系统已停用：不做功率限制、不读裁判数据）
  ******************************************************************************
  */

#include "device.h"
#include "kt_yaw_motor.h"
#include "shoot_motor.h"

/**
  * @brief  设备列表全局对象
  * @details 持有各设备的指针引用，方便统一管理和访问
  */
dev_list_t dev_list = {
    .rc_sen = &rc_sensor,  /* 遥控器传感器指针 */
};


void DEVICE_Init(void)
{
    /* 初始化遥控器（UART通信） */
    dev_list.rc_sen->init(dev_list.rc_sen);

    /* 初始化IMU惯性测量单元（BMI088 + 姿态解算） */
    imu_sensor.init(&imu_sensor);

    /* 初始化底盘电机组（四个M3508电机，CAN通信） */
    Chassis_Motor_Group.group_init(&Chassis_Motor_Group);

    /* 初始化拨弹 RM3508（CAN2 0x205 / 控制 0x1FF） */
    Shoot_Motor_Init();

    /* 初始化云台Yaw轴电机（铂昌KT电机，CAN通信） */
    KT_Yaw_Init();

    /* 裁判系统已停用（不做功率限制、不读裁判数据），初始化注释掉 */
    // My_Judge_Init();

    cap.init(&cap);
}
