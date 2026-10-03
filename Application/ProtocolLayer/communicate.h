#ifndef COMMUNICATE_H
#define COMMUNICATE_H

#include <stdint.h>
#include <stdbool.h>

#define BOARD_C_OFFLINE_TIMEOUT_CNT (50)  /* 50 * 2ms = 100ms 判定离线 */

/* 0x023 状态帧 flags 位定义（byte[0]），CAN ID 见 can_protocol.h */
#define FRIC_STATE_SPEED             (1U << 0)  /* 摩擦轮已达速 */
#define FRIC_STATE_WORK              (1U << 1)  /* 摩擦轮电机全部在线 */

typedef struct {
    float yaw_imu;          /* 空间绝对 Yaw 角度 (rad) */
    float yaw_v;            /* 空间绝对 Yaw 陀螺仪角速度 (rad/s) */
    uint16_t offline_cnt;   /* 0x021 掉线计时器 */
    bool is_online;         /* 0x021 上层板在线标志 */

    /* ---- 0x023 状态帧（摩擦轮就绪安全锁用） ---- */
    uint8_t  fric_state;        /* 原始 flags 字节 */
    bool     is_fric_speed;     /* 六摩擦轮实测均已达速 */
    bool     is_fric_work;      /* 六摩擦轮电机全部在线 */
    bool     is_state_online;   /* 0x023 在线（上层板是否回传状态帧） */
    uint16_t state_offline_cnt; /* 0x023 掉线计时器 */
} Board_Rx_Info_t;

typedef struct {
    float pitch_encoder_tar; /* 编码器机械目标角(rad)，与上位机字段保持一致 */
    uint8_t pitch_enable;
    uint8_t is_fric_on;      /* 下层->上层：摩擦轮开启（拨弹总开关且未卡弹停机） */
} Board_Tx_Info_t;

extern Board_Rx_Info_t board_rx_info;
extern Board_Tx_Info_t board_tx_info;

void Communicate_Init(void);
void Board_Rx_C1(const uint8_t *rxBuf);
void Board_Rx_State(const uint8_t *rxBuf);
void Board_Tx_D1(void);
void Communicate_Check_Timeout(void);

#endif /* COMMUNICATE_H */
