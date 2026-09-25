#ifndef COMMUNICATE_H
#define COMMUNICATE_H

#include <stdint.h>
#include <stdbool.h>

#define BOARD_C_OFFLINE_TIMEOUT_CNT (50)  // 50 * 2ms = 100ms 判定离线

typedef struct {
    float yaw_imu;          // 空间绝对 Yaw 角度 (rad)
    float yaw_v;            // 空间绝对 Yaw 陀螺仪角速度 (rad/s)
    uint16_t offline_cnt;   // 掉线计时器
    bool is_online;         // 上层板在线标志
} Board_Rx_Info_t;

extern Board_Rx_Info_t board_rx_info;

void Communicate_Init(void);
void Board_Rx_C1(const uint8_t *rxBuf);
void Communicate_Check_Timeout(void);

#endif /* COMMUNICATE_H */