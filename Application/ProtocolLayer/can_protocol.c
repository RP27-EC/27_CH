/**
 * @file    can_protocol.c
 * @brief   CAN总线通信协议解析模块
 *
 * 本文件实现了CAN1和CAN2两条总线的数据接收分发处理。
 * 根据接收到的CAN消息ID，将数据分发给对应的模块进行处理，
 * 如底盘电机反馈、云台电机反馈、摩擦轮反馈等。
 */

/* Includes ------------------------------------------------------------------*/
#include "board_comm.h"
#include "can_protocol.h"   /* CAN协议头文件 */
#include "motor.h"          /* 六摩擦轮电机实例 */


void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
    switch (rxId)
    {
        /* 六摩擦轮 RM3508 反馈（CAN1），转速/电流/温度由驱动层解析 */
        case ID_FRIC_F_U:   /* 0x201 前层上摩擦轮 */
            F_U_Fric.rx(&F_U_Fric, rxBuf);
            break;

        case ID_FRIC_F_R:   /* 0x202 前层右摩擦轮 */
            F_R_Fric.rx(&F_R_Fric, rxBuf);
            break;

        case ID_FRIC_F_L:   /* 0x203 前层左摩擦轮 */
            F_L_Fric.rx(&F_L_Fric, rxBuf);
            break;

        case ID_FRIC_B_U:   /* 0x205 后层上摩擦轮 */
            B_U_Fric.rx(&B_U_Fric, rxBuf);
            break;

        case ID_FRIC_B_R:   /* 0x206 后层右摩擦轮 */
            B_R_Fric.rx(&B_R_Fric, rxBuf);
            break;

        case ID_FRIC_B_L:   /* 0x207 后层左摩擦轮 */
            B_L_Fric.rx(&B_L_Fric, rxBuf);
            break;

        default:
            break;  /* 未识别的CAN ID，直接忽略 */
    }
}


void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{

    switch (canId)
    {

        case SLAVE_RX_PITCH_TAR_CAN_ID: // 0xD1，板间，接收下层板的遥控Pitch目标角度
            Board_Comm_Rx_D1(rxBuf);
            break;
        case 0x011U:
            if (Pitch_Motor.rx != NULL) { //达秒 DM 电机 MIT 协议反馈，接收电机信息
                Pitch_Motor.rx(&Pitch_Motor, rxBuf);
            }
            break;
        default:
            break;  /* 未识别的CAN ID，直接忽略 */
    }
}
