/**
  ******************************************************************************
  * @file           : gimbal_motor.c
  * @brief          : 云台电机驱动模块（达妙电机DM系列）
  * @details        : 定义云台Yaw轴电机的配置参数和对象。
  *                   使用达妙(DM)系列电机通过FDCAN总线通信，
  *                   配置了电机的ID、CAN总线、接收/发送/状态信息等。
  ******************************************************************************
  */

#include "gimbal_motor.h"
#include "DM_Motor.h"    /* 达妙电机驱动头文件 */

/**
  * ==================== Yaw轴电机配置 ====================
  * 达妙电机通过CAN总线通信，每个电机有独立的标准ID
  */

/**
  * @brief  Yaw轴电机出厂信息
  * @details 配置电机的CAN通信参数：
  *          - stdId: CAN标准ID为0x002，用于标识该电机
  *          - hcan: 使用FDCAN1总线进行通信
  */
Motor_DM_Born_Info_t Yaw_Born_Info =
{
	.stdId = 0x002,          /* Yaw电机CAN标准ID */
	.hcan = &hfdcan1,        /* 使用FDCAN1总线 */
};

/* Yaw电机接收数据缓冲区，用于存储电机反馈的角度、速度、力矩等信息 */
Motor_DM_Rx_Info_t Yaw_Rx_Info_t;

/* Yaw电机发送数据缓冲区，用于存储控制指令 */
Motor_DM_Tx_Info_t Yaw_Tx_Info_t;

/* Yaw电机状态信息，记录电机在线/离线、错误等状态 */
Motor_DM_State_t Yaw_State_t;

/**
  * @brief  Yaw轴电机完整对象
  * @details 将所有子模块组合成完整的电机控制对象：
  *          - born_info: 出厂信息（CAN ID、总线等）
  *          - rx_info: 接收数据缓冲区
  *          - tx_info: 发送数据缓冲区
  *          - state: 电机状态
  *          - single_init: 单电机初始化函数指针
  */
Motor_DM_t Yaw_Motor =
{
	.born_info = &Yaw_Born_Info,    /* 绑定出厂信息 */
	.rx_info = &Yaw_Rx_Info_t,      /* 绑定接收缓冲区 */
	.tx_info = &Yaw_Tx_Info_t,      /* 绑定发送缓冲区 */
	.state = &Yaw_State_t,          /* 绑定状态信息 */
	.single_init = &DM_Single_Motor_Init,  /* 绑定初始化函数 */
};