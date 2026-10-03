/**
  ******************************************************************************
  * @file           : kt_yaw_motor.c
  * @brief          : 铂昌(KT)系列Yaw轴电机驱动模块
  * @details        : 封装铂昌KT9025电机的初始化、心跳检测和力矩控制接口。
  *                   使用CAN总线通信，支持闭环电流（力矩）控制模式。
  ******************************************************************************
  */

#include "kt_yaw_motor.h"

/**
  * @brief  KT电机对象数组（当前仅使用一个电机作为Yaw轴）
  * @details 配置参数：
  *          - tx_id: 发送CAN ID
  *          - rx_id: 接收CAN ID（电机反馈数据）
  *          - drive_type: 驱动类型，使用M_CAN1（CAN1总线）
  *          - motor_type: 电机型号KT9025
  *          - init: 初始化函数指针
  */
KT_motor_t kt_motor[1] = {
    {
        .KT_motor_info = {
            .id = {
                .tx_id = KT_YAW_TX_ID,       /* Yaw电机发送ID */
                .rx_id = KT_YAW_RX_ID,       /* Yaw电机接收ID */
                .drive_type = M_CAN1,         /* 使用CAN1总线 */
                .motor_type = KT9025,         /* 铂昌KT9025电机 */
            },
        },
        .init = KT_motor_class_init,  /* 绑定电机类初始化函数 */
    },
};

/**
  * @brief  Yaw轴KT电机初始化
  * @retval None
  * @details 调用电机对象的初始化函数，配置CAN通信参数
  */
void KT_Yaw_Init(void)
{
    kt_motor[0].init(&kt_motor[0]);
}

/**
  * @brief  Yaw轴KT电机心跳检测
  * @retval None
  * @details 周期性调用，检测电机通信是否正常
  */
void KT_Yaw_Heartbeat(void)
{
    kt_motor[0].heartbeat(&kt_motor[0]);
}

/**
  * @brief  发送闭环电流（力矩）控制命令到Yaw轴电机
  * @param  iq: 目标力矩电流值（有符号16位整数）
  * @retval None
  * @details 控制流程：
  *          1. 将目标电流值写入电机控制缓冲区
  *          2. 发送闭环电流控制命令到CAN总线
  * @note   力矩电流的正负决定电机转动方向
  */
void KT_Yaw_SendIq(int16_t iq)
{
    kt_motor[0].W_iqControl(&kt_motor[0], iq);                    /* 写入闭环电流控制值 */
    kt_motor[0].tx_W_cmd(&kt_motor[0], TORQUE_CLOSE_LOOP_ID);     /* 发送闭环电流控制命令 */
}
