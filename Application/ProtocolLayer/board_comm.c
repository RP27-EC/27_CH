#include "board_comm.h"
#include <string.h>

extern CAN_HandleTypeDef hcan2;

Board_Rx_Info_t board_rx_info = {
    .pitch_encoder_tar = 0.0f,
    .pitch_enable = 0U,
    .pitch_use_imu = 0U,
    .is_fric_on = 0U,
    .offline_cnt = 0,
    .is_online = false,
};

Board_Tx_Info_t board_tx_info = {
    .fric_state = 0U,
};


//取上层的imu已经得到的yaw角度和角速度，装在局部变量缓冲区，发送给下层板
HAL_StatusTypeDef Board_Comm_Send_IMU(float yaw_angle_rad, float yaw_rate_rad_s)
{
    uint8_t tx_buf[8];
    memcpy(&tx_buf[0], &yaw_angle_rad, sizeof(float));
    memcpy(&tx_buf[4], &yaw_rate_rad_s, sizeof(float));
    return CAN_SendData(&hcan2, MASTER_RX_YAW_IMU_CAN_ID, tx_buf);
}

/**
 * @brief  发送 0x022 pitch IMU 给下层板
 * @note   与 yaw 的 0x021 同构：byte[0..3] 角度，byte[4..7] 角速度。
 *         下层板用它做 GYRO 域 pitch 目标软限位与模式切入同步；
 *         上层闭环本身仍用本板 imu_sensor，不依赖 0x022 回环。
 */
HAL_StatusTypeDef Board_Comm_Send_Pitch_IMU(float pitch_angle_rad, float pitch_rate_rad_s)
{
    uint8_t tx_buf[8];
    memcpy(&tx_buf[0], &pitch_angle_rad, sizeof(float));
    memcpy(&tx_buf[4], &pitch_rate_rad_s, sizeof(float));
    return CAN_SendData(&hcan2, MASTER_RX_PITCH_IMU_CAN_ID, tx_buf);
}

/**
 * @brief  发送 0x023 摩擦轮状态帧给下层板
 * @note   byte[0] 为 flags：bit0 已达速，bit1 全部在线。
 *         下层板用它做摩擦轮就绪安全锁，未就绪不许拨盘动作。
 *         每个控制周期无条件发送，避免下层板因超时退回延时门禁。
 */
HAL_StatusTypeDef Board_Comm_Send_State(void)
{
    uint8_t tx_buf[8] = {0};
    tx_buf[0] = board_tx_info.fric_state;
    /* tx_buf[1..7] 预留给后续扩展 */
    return CAN_SendData(&hcan2, MASTER_RX_STATE_CAN_ID, tx_buf);
}

//解析下层板发送的 0xD1 报文，得到遥控数据的pitch目标与摩擦轮开关
void Board_Comm_Rx_D1(uint8_t *rx_buf)
{
    if (rx_buf == NULL) {
        return;
    }

    /* 前 4 字节：pitch 目标。域由 byte[6] 声明，不在这里解释。 */
    memcpy(&board_rx_info.pitch_encoder_tar, &rx_buf[0], sizeof(float));
    board_rx_info.pitch_enable = rx_buf[4] ? 1U : 0U;
    board_rx_info.is_fric_on = rx_buf[5]; /* 0xD1 byte[5]：摩擦轮开关 */
    board_rx_info.pitch_use_imu = rx_buf[6] ? 1U : 0U; /* 1=GYRO/SPIN：IMU 域闭环 */

    /* 收到有效数据，清零离线计数器，标记在线 */
    board_rx_info.offline_cnt = 0;
    board_rx_info.is_online = true;
}

/**
 * @brief  通信超时看门狗（2ms 周期调用一次）
 * @note   超过 100ms（50 个周期）收不到 0xD1，判定下层板离线，保护电机
 */
void Board_Comm_Check_Timeout(void)
{
    if (board_rx_info.offline_cnt < 50) {
        board_rx_info.offline_cnt++;
    } else {
        board_rx_info.is_online = false;
    }
}
