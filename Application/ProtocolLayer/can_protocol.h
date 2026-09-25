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
/* 板间通信 CAN ID（下层板与上层 C 板） */
#define SLAVE_RX_ID                  (0x021)    /* 接收上层 C 板发来的数据 (yaw_imu, yaw_v) */
#define SLAVE_TX_ID                  (0x022)    /* 发送给上层 C 板的数据 (整车状态, pitch目标等) */

/* CAN1 电机 ID (底盘 M3508 与 KT9025) */
#define CHASSIS_CAN_ID_LF            (0x201)
#define CHASSIS_CAN_ID_RF            (0x202)
#define CHASSIS_CAN_ID_LB            (0x203)
#define CHASSIS_CAN_ID_RB            (0x204)

/* Exported functions --------------------------------------------------------*/
void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
void CAN3_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
void cap_data_send(uint8_t can_num);

#endif /* __CAN_PROTOCOL_H */
