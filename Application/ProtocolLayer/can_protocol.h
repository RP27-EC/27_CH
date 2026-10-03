/**
 ******************************************************************************
 * @file    can_protocol.h
 * @brief   CAN通信协议层
 ******************************************************************************
 * @attention
 *
 * Copyright 2024 RobotPilots
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include "driver.h"
#include "device.h"

/* Exported macro ------------------------------------------------------------*/
/* 下主控CAN ID */
#define SLAVE_TX_ID 
#define SLAVE_RX_ID 

/*
//CAN1 —— 六摩擦轮 RM3508（反馈 ID = 0x200 + 电机ID，发送分两组打包）
#define ID_FRIC_F_U 	0x201 //0x200 组槽位 0
#define ID_FRIC_F_R 	0x202 //0x200 组槽位 1
#define ID_FRIC_F_L 	0x203 //0x200 组槽位 2
#define ID_FRIC_B_U 	0x205 //0x1FF 组槽位 0
#define ID_FRIC_B_R 	0x206 //0x1FF 组槽位 1
#define ID_FRIC_B_L 	0x207 //0x1FF 组槽位 2
（以上六个 ID 宏实际定义在 motor.h，与电机实例放在一起）

//CAN2 —— 板间通信 + Pitch 达妙电机
#define SLAVE_RX_PITCH_TAR_CAN_ID  0x0D1  // 收下层板 Pitch 目标 + is_fric_on
#define MASTER_RX_YAW_IMU_CAN_ID   0x021  // 发下层板 yaw IMU
#define MASTER_RX_STATE_CAN_ID     0x023  // 发下层板摩擦轮状态
#define ID_PITCH_DM                0x011  // Pitch 达妙 MIT 协议反馈
*/


/* Exported functions --------------------------------------------------------*/
void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
void CAN_SendAll(void);
void CAN_Send(void);
void CAN_SendAllZero(void);


#endif
