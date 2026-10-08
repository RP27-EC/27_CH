/**
  ******************************************************************************
  * @file    can_protocol.c
  * @brief   CAN通信中断分发实现
  * @details 本文件实现了三路CAN总线（CAN1、CAN2、CAN3）接收中断的分发逻辑。
  *          当CAN总线接收到数据帧时，根据帧ID分发到对应的设备驱动函数进行处理。
  *
  *          CAN1总线：底盘4个电机 + KT9025 Yaw电机
  *          CAN2总线：上层C板（板间通信，IMU数据）
  *          CAN3总线：超级电容模块（暂未实现）
  ******************************************************************************
  */

#include "can_protocol.h"
#include "chassis_motor.h"
#include "gimbal_motor.h"
#include "shoot_motor.h"   /* 拨弹 RM3508 */
#include "cap_protocol.h"
#include "cap.h"
#include "kt_yaw_motor.h"
#include "communicate.h"   /* 引入板间通信解包模块 */

/**
  * @brief  CAN1接收中断分发函数
  * @details 根据CAN帧ID将接收到的数据分发到对应的设备驱动。
  *
  *          CAN1总线上挂载的设备：
  *          - KT9025 Yaw电机（ID由KT_YAW_RX_ID定义）
  *          - 底盘4个轮子电机（ID: 0x201~0x204）
  *
  * @param  rxId  : 接收到的CAN帧ID
  * @param  rxBuf : CAN接收缓冲区，包含8字节数据
  * @retval None
  */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
    switch (rxId) {
        case KT_YAW_RX_ID:
            /* KT9025 Yaw轴电机反馈数据接收 */
            kt_motor[0].get_info(&kt_motor[0], rxBuf);
            break;

        case CHASSIS_CAN_ID_LF: // 0x201 —— 左前轮电机反馈
            Chassis_Motor[0].rx(&Chassis_Motor[0], rxBuf);
            break;

        case CHASSIS_CAN_ID_RF: // 0x202 —— 右前轮电机反馈
            Chassis_Motor[1].rx(&Chassis_Motor[1], rxBuf);
            break;

        case CHASSIS_CAN_ID_LB: // 0x203 —— 左后轮电机反馈
            Chassis_Motor[2].rx(&Chassis_Motor[2], rxBuf);
            break;

        case CHASSIS_CAN_ID_RB: // 0x204 —— 右后轮电机反馈
            Chassis_Motor[3].rx(&Chassis_Motor[3], rxBuf);
            break;

        default:
            break;
    }
}

/**
  * @brief  CAN2接收中断分发函数
  * @details 处理CAN2总线上的数据帧：
  *          - 0x021：上层C板 yaw IMU（板间通信）
  *          - 0x022：上层C板 pitch IMU（板间通信）
  *          - 0x205：拨弹 RM3508 反馈
  *
  * @param  rxId  : 接收到的CAN帧ID
  * @param  rxBuf : CAN接收缓冲区，包含8字节数据
  * @retval None
  */
void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
    switch (rxId) {
        case SLAVE_RX_ID: // 0x021 —— 上层C板发来的yaw IMU数据
            Board_Rx_C1(rxBuf);
            break;

        case SLAVE_RX_PITCH_IMU_ID: // 0x022 —— 上层C板发来的pitch IMU数据
            Board_Rx_Pitch(rxBuf);
            break;

        case SLAVE_RX_STATE_ID: // 0x023 —— 上层C板状态帧（摩擦轮就绪等）
            Board_Rx_State(rxBuf);
            break;

        case SHOOT_MOTOR_CAN_ID: // 0x205 —— 拨弹 RM3508 反馈
            Shoot_Motor.rx(&Shoot_Motor, rxBuf);
            Shoot_Motor.state->offline_cnt = 0;// 收到反馈就清零离线计数器，方便心跳判断
            Shoot_Motor.state->status = DEV_ONLINE;
            break;

        default:
            break;
    }
}

/**
  * @brief  CAN3接收中断分发函数
  * @details CAN3挂载超级电容模块，按帧ID分发：
  *          - 0x211：超电板状态帧（功率 / 电容电压 / 电容电流 / 放电能力）
  *
  * @param  rxId  : 接收到的CAN帧ID
  * @param  rxBuf : CAN接收缓冲区，包含8字节数据
  * @retval None
  */
void CAN3_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
    switch (rxId) {
        case CAP_RX_ID: // 0x211 —— 超电板状态帧
            cap.rx(&cap, rxBuf);
            break;

        default:
            break;
    }
}
