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

/*CAN1*/
#define CHASSIS_CAN_ID_LF			 RM3508_CAN_ID_201
#define CHASSIS_CAN_ID_RF			 RM3508_CAN_ID_202
#define CHASSIS_CAN_ID_LB			 RM3508_CAN_ID_203
#define CHASSIS_CAN_ID_RB			 RM3508_CAN_ID_204

/*CAN2*/
#define GIMBAL_CAN_ID_PITCH			 GM6020_CAN_ID_205
#define GIMBAL_CAN_ID_YAW			 GM6020_CAN_ID_206
#define FRIC_CAN_ID_LEFT		     RM3508_CAN_ID_201
#define FRIC_CAN_ID_RIGHT			 RM3508_CAN_ID_202
#define LAUNCH_CAN_ID_DIAL	         RM2006_CAN_ID_203

/* Exported functions --------------------------------------------------------*/
void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
void cap_data_send(uint8_t can_num);

#endif
