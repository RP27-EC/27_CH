#ifndef COMMUNICATE_H
#define COMMUNICATE_H

#include <stdint.h>
#include <stdbool.h>

#define BOARD_C_OFFLINE_TIMEOUT_CNT (50)  /* 50 * 2ms = 100ms 判定离线 */

/* 0x023：byte[0] 状态，byte[1..4] 机械角，byte[5] 有效标记 */
#define FRIC_STATE_SPEED             (1U << 0)  /* 摩擦轮已达速 */
#define FRIC_STATE_WORK              (1U << 1)  /* 摩擦轮电机全部在线 */
#define PITCH_MECH_VALID_MARKER      (0xA5U)    /* 0x023 byte[5]：机械角有效 */

typedef struct {
    float yaw_imu;          /* 空间绝对 Yaw 角度 (rad) —— 0x021 */
    float yaw_v;            /* 空间绝对 Yaw 陀螺仪角速度 (rad/s) —— 0x021 */
    float pitch_imu;        /* 云台 Pitch 角 (rad) —— 0x022，上层 C 板 IMU */
    float pitch_v;          /* 云台 Pitch 角速度 (rad/s) —— 0x022 */
    uint16_t offline_cnt;   /* 0x021 掉线计时器 */
    uint16_t pitch_offline_cnt; /* 0x022 掉线计时器 */
    bool is_online;         /* 0x021 上层板在线标志 */
    bool is_pitch_online;   /* 0x022 pitch IMU 在线标志 */

    /* ---- 0x023 状态帧（摩擦轮就绪安全锁用） ---- */
    uint8_t  fric_state;        /* 原始 flags 字节 */
    bool     is_fric_speed;     /* 六摩擦轮实测均已达速 */
    bool     is_fric_work;      /* 六摩擦轮电机全部在线 */
    bool     is_state_online;   /* 0x023 在线（上层板是否回传状态帧） */
    float    pitch_mech_angle;  /* 0x023 byte[1..4]：上层编码器机械角 (rad) */
    bool     is_pitch_mech_online; /* 收到有效机械角，且 0x023 未超时 */
    uint16_t state_offline_cnt; /* 0x023 掉线计时器 */
    /* Watch 诊断，不改变 CAN 帧布局。reason: 0=有效,1=标记错误,2=角度错误,3=超时/未收到 */
    uint8_t pitch_mech_invalid_reason;
    uint8_t state_last_marker;
    uint32_t state_rx_count;
    uint32_t state_invalid_count;
    uint32_t state_timeout_count; /* 仅在线转超时时累加，避免每周期重复计数 */
} Board_Rx_Info_t;

typedef struct {
    float pitch_encoder_tar; /* MECH：编码器机械目标角；GYRO：IMU 域目标角 (rad) */
    uint8_t pitch_enable;
    uint8_t pitch_use_imu;   /* 1=GYRO/SPIN：上层按 IMU 域闭环；0=MECH：编码器闭环 */
    uint8_t is_fric_on;      /* 下层->上层：摩擦轮开启（拨弹总开关且未卡弹停机） */
} Board_Tx_Info_t;

extern Board_Rx_Info_t board_rx_info;
extern Board_Tx_Info_t board_tx_info;

void Communicate_Init(void);
void Board_Rx_C1(const uint8_t *rxBuf);
void Board_Rx_Pitch(const uint8_t *rxBuf);
void Board_Rx_State(const uint8_t *rxBuf);
void Board_Tx_D1(void);
void Communicate_Check_Timeout(void);

#endif /* COMMUNICATE_H */
