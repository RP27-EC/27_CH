/**
 ******************************************************************************
 * @file    can_protocol.c
 * @brief   CAN通信中断分发实现
 ******************************************************************************
 */

#include "can_protocol.h"
#include "chassis_motor.h"
#include "gimbal_motor.h"
#include "cap_protocol.h"
#include "judge.h"
#include "cap.h"
#include "kt_yaw_motor.h"
#include "communicate.h"   /* 引入板间通信解包模块 */

void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
    switch (rxId) {
        case KT_YAW_RX_ID:
            kt_motor[0].get_info(&kt_motor[0], rxBuf);
            break;
        case CHASSIS_CAN_ID_LF: // 0x201
            Chassis_Motor[0].rx(&Chassis_Motor[0], rxBuf);
            break;
        case CHASSIS_CAN_ID_RF: // 0x202
            Chassis_Motor[1].rx(&Chassis_Motor[1], rxBuf);
            break;
        case CHASSIS_CAN_ID_LB: // 0x203
            Chassis_Motor[2].rx(&Chassis_Motor[2], rxBuf);
            break;
        case CHASSIS_CAN_ID_RB: // 0x204
            Chassis_Motor[3].rx(&Chassis_Motor[3], rxBuf);
            break;
        default:
            break;
    }
}

void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
    switch (rxId) {
        /* 接收上层 C 板发来的 IMU 姿态与角速度数据 */
        case SLAVE_RX_ID: // 0x021
            Board_Rx_C1(rxBuf);
            break;

        default:
            break;
    }
}

void CAN3_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
    (void)rxId;
    (void)rxBuf;
}
