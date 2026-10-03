#ifndef BOARD_COMM_H
#define BOARD_COMM_H

#include "drv_can.h"
#include <stdbool.h>

#define MASTER_RX_YAW_IMU_CAN_ID   (0x021U)  /* 发送给下层板的 IMU ID */
#define SLAVE_RX_PITCH_TAR_CAN_ID  (0x0D1U)  /* 接收下层板的 Pitch 目标 ID */

/* 板间通信接收数据结构体 */
typedef struct {
    float pitch_encoder_tar; /* 下层板发来的编码器机械目标角，单位 rad */
    uint8_t pitch_enable;  /* 下层板是否允许上层闭环 */
    uint16_t offline_cnt;  /* 离线计数器 */
    bool is_online;        /* 下层板在线状态 */
} Board_Rx_Info_t;

extern Board_Rx_Info_t board_rx_info;

/* 发送函数 */
HAL_StatusTypeDef Board_Comm_Send_IMU(float yaw_angle_rad, float yaw_rate_rad_s);

/* 接收解析与看门狗函数 */
void Board_Comm_Rx_D1(uint8_t *rx_buf);
void Board_Comm_Check_Timeout(void);

#endif
