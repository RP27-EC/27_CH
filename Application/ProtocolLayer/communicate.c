#include "communicate.h"
#include "gimbal.h"
#include "drv_can.h"
#include <string.h>


Board_Rx_Info_t board_rx_info;
Board_Tx_Info_t board_tx_info;

void Communicate_Init(void) {
    /* 清零整个接收信息结构体 */
    memset(&board_rx_info, 0, sizeof(Board_Rx_Info_t));
    memset(&board_tx_info, 0, sizeof(Board_Tx_Info_t));

    /* 初始状态标记上层C板为离线 */
    board_rx_info.is_online = false;
    board_rx_info.is_state_online = false;
}


void Board_Rx_C1(const uint8_t *rxBuf) {
    /* 空指针保护 */
    if (rxBuf == NULL) return;

    /* 提取yaw轴IMU角度*/
    memcpy(&board_rx_info.yaw_imu, &rxBuf[0], sizeof(float));

    /* 提取yaw轴IMU角速度 */
    memcpy(&board_rx_info.yaw_v,   &rxBuf[4], sizeof(float));

    /* 重置离线计数器，标记上层C板在线 */
    board_rx_info.offline_cnt = 0;
    board_rx_info.is_online = true;

    /* 将IMU数据同步到云台对象，供Gimbal_Yaw_Pid_Cal()串级PID使用 */
    hero_gimbal.yaw.imu_angle = board_rx_info.yaw_imu;  
    hero_gimbal.yaw.imu_speed = board_rx_info.yaw_v;    
}



/**
  * @brief  0x023 状态帧接收（上层 C 板 -> 下层板）
  * @details byte[0] 为 flags：
  *          bit0 is_fric_speed：六摩擦轮实测均已达速
  *          bit1 is_fric_work ：六摩擦轮电机全部在线
  *          供拨弹模块的摩擦轮就绪安全锁使用。
  * @param  rxBuf : CAN接收缓冲区
  */
void Board_Rx_State(const uint8_t *rxBuf)
{
    if (rxBuf == NULL) return;

    board_rx_info.fric_state    = rxBuf[0];
    board_rx_info.is_fric_speed = (rxBuf[0] & FRIC_STATE_SPEED) != 0U;
    board_rx_info.is_fric_work  = (rxBuf[0] & FRIC_STATE_WORK)  != 0U;

    /* 重置离线计数器，标记状态帧在线 */
    board_rx_info.state_offline_cnt = 0;
    board_rx_info.is_state_online = true;
}


/**
  * @brief  CAN2 发送 0xD1 报文到上层板
  */
void Board_Tx_D1(void)
{
    uint8_t tx_data[8] = {0};

    /* 将 float pitch 目标角直接拷贝到前 4 字节 (tx_data[0..3]) */
    memcpy(&tx_data[0], &board_tx_info.pitch_encoder_tar, sizeof(float)); /* 0xD1：发送编码器机械目标角(rad) */
    tx_data[4] = board_tx_info.pitch_enable;
    tx_data[5] = board_tx_info.is_fric_on; /* 摩擦轮开启标志，供上层板摩擦轮 */

    /* tx_data[6..7] 预留给后续扩展 */

    CAN_SendData(&hfdcan2, 0xD1, tx_data);
}

/* 通信超时检测*/
void Communicate_Check_Timeout(void) {
    if (board_rx_info.offline_cnt < BOARD_C_OFFLINE_TIMEOUT_CNT) {
        board_rx_info.offline_cnt++;  /* 未超时，递增计数器 */
    } else {
        board_rx_info.is_online = false;  /* 超时，标记上层C板离线 */
    }

    /* 0x023 状态帧超时：上层板未升级时始终为离线，拨弹退化为延时门禁 */
    if (board_rx_info.state_offline_cnt < BOARD_C_OFFLINE_TIMEOUT_CNT) {
        board_rx_info.state_offline_cnt++;
    } else {
        board_rx_info.is_state_online = false;
    }
}
