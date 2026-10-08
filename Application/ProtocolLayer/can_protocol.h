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
#define SLAVE_RX_ID                  (0x021)    /* 接收上层 C 板数据 (yaw_imu, yaw_v) */
#define SLAVE_RX_PITCH_IMU_ID        (0x022)    /* 接收上层 C 板 pitch IMU (pitch, pitch_v) */
#define SLAVE_RX_STATE_ID            (0x023)    /* 接收上层 C 板状态帧 (摩擦轮就绪等) */
#define BOARD_TX_PITCH_TAR_ID        (0x0D1)    /* 发送 pitch 目标给上层 C 板 */

/* CAN1 电机 ID (底盘 M3508 与 KT9025) */
#define CHASSIS_CAN_ID_LF            (0x201)
#define CHASSIS_CAN_ID_RF            (0x202)
#define CHASSIS_CAN_ID_LB            (0x203)
#define CHASSIS_CAN_ID_RB            (0x204)

/* CAN2 拨弹 RM3508 反馈 ID（电调 ID=5） */
#define SHOOT_MOTOR_CAN_ID           (0x205)

/* Exported functions --------------------------------------------------------*/
void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
void CAN3_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
void cap_data_send(uint8_t can_num);

#endif /* __CAN_PROTOCOL_H */
