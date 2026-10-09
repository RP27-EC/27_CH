#ifndef BOARD_COMM_H
#define BOARD_COMM_H

#include "drv_can.h"
#include <stdbool.h>

#define MASTER_RX_YAW_IMU_CAN_ID   (0x021U)  /* 发送给下层板的 IMU ID */
#define MASTER_RX_STATE_CAN_ID     (0x023U)  /* 发送给下层板的摩擦轮状态帧 ID */
#define MASTER_RX_PITCH_IMU_CAN_ID (0x022U)
#define SLAVE_RX_PITCH_TAR_CAN_ID  (0x0D1U)  /* 接收下层板的 Pitch 目标 ID */

/* 0x023：byte[0] 状态，byte[1..4] 机械角，byte[5] 有效标记 */
#define FRIC_STATE_SPEED           (1U << 0)  /* 六摩擦轮实测均已达速 */
#define FRIC_STATE_WORK            (1U << 1)  /* 六摩擦轮电机全部在线 */
#define PITCH_MECH_VALID_MARKER    (0xA5U)    /* 0x023 byte[5]：byte[1..4] 为有效机械角 */

/* 板间通信接收数据结构体 */
typedef struct {
    float pitch_encoder_tar; /* 下层板发来的编码器机械目标角，单位 rad */
    uint8_t pitch_enable;  /* 下层板是否允许上层闭环 */
    uint8_t is_fric_on;
    uint8_t pitch_use_imu;    /* 0xD1 byte[6]：下层板请求使用 IMU Pitch 闭环 */
    uint16_t offline_cnt;  /* 离线计数器 */
    bool is_online;        /* 下层板在线状态 */
} Board_Rx_Info_t;

/* 板间通信发送数据结构体 */
typedef struct {
    uint8_t fric_state;    /* 0x023 byte[0]：摩擦轮状态 flags，见 FRIC_STATE_* */
} Board_Tx_Info_t;

extern Board_Rx_Info_t board_rx_info;
extern Board_Tx_Info_t board_tx_info;

/* 发送函数 */
HAL_StatusTypeDef Board_Comm_Send_IMU(float yaw_angle_rad, float yaw_rate_rad_s);
HAL_StatusTypeDef Board_Comm_Send_Pitch_IMU(float pitch_angle_rad, float pitch_rate_rad_s);
HAL_StatusTypeDef Board_Comm_Send_State(float pitch_mech_rad, bool pitch_valid);

/* 接收解析与看门狗函数 */
void Board_Comm_Rx_D1(uint8_t *rx_buf);
void Board_Comm_Check_Timeout(void);

#endif
