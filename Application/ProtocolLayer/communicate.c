/**
  ******************************************************************************
  * @file           : communicate.c
  * @brief          : 板间通信协议实现与解包
  ******************************************************************************
  */

#include "communicate.h"
#include "gimbal.h"
#include <string.h>

Board_Rx_Info_t board_rx_info;

void Communicate_Init(void) {
    memset(&board_rx_info, 0, sizeof(Board_Rx_Info_t));
    board_rx_info.is_online = false;
}

/**
 * @brief  解包上层 C 板发来的 0x21 帧
 * @param  rxBuf: CAN 接收到的 8 字节数据缓存
 */
void Board_Rx_C1(const uint8_t *rxBuf) {
    if (rxBuf == NULL) return;

    /* 提取 8 字节数据：Byte 0~3 为 yaw_imu，Byte 4~7 为 yaw_v */
    memcpy(&board_rx_info.yaw_imu, &rxBuf[0], sizeof(float));
    memcpy(&board_rx_info.yaw_v,   &rxBuf[4], sizeof(float));

    /* 刷新看门狗并标记在线 */
    board_rx_info.offline_cnt = 0;
    board_rx_info.is_online = true;

    /* 同步给云台对象（供串级 PID 闭环使用） */
    hero_gimbal.yaw.imu_angle = board_rx_info.yaw_imu;
    hero_gimbal.yaw.imu_speed = board_rx_info.yaw_v;
}

/**
 * @brief  通信超时检测（放在 2ms 控制循环中）
 */
void Communicate_Check_Timeout(void) {
    if (board_rx_info.offline_cnt < BOARD_C_OFFLINE_TIMEOUT_CNT) {
        board_rx_info.offline_cnt++;
    } else {
        board_rx_info.is_online = false; // 超时判定上层 C 板离线
    }
}
