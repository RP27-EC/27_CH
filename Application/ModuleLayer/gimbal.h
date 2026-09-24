/**
  ******************************************************************************
  * @file           : gimbal.h
  * @brief          : 云台控制模块（KT9025 机械模式适配版）
  ******************************************************************************
  */

#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"
#include "carctrl.h"

/* ========================================================================= */
/*                          KT9025 硬件与几何参数                            */
/* ========================================================================= */
#define KT_ENCODER_FULL_RANGE       (65536.0f)   // 16位编码器满量程 (2^16)
#define KT_ENCODER_HALF_RANGE       (32768.0f)   // 半圈值

/* 战队实际机械零点偏置 (对应原老代码中的 25823.f + 32768.f) */
#define YAW_KT_ENCODER_MIDDLE       (25823.0f + 32768.0f)

/* Pitch 轴软限位 (rad) */
#define PITCH_MIN_MEC_ANGLE         (-0.21f)     // 机械下限位 (约 -12°)
#define PITCH_MAX_MEC_ANGLE         (0.81f)      // 机械上限位 (约 +46°)

/* 摇杆控制灵敏度 */
#define RC_YAW_MECH_SENSITIVITY     (0.0004f)    // 机械模式 Yaw 摇杆增量步长 (rad)
#define RC_PITCH_MECH_SENSITIVITY   (0.0002f)    // 机械模式 Pitch 摇杆增量步长 (rad)

/* 云台运行模式 */
typedef enum {
    GIMBAL_MODE_SLEEP = 0,      // 休眠/关控
    GIMBAL_MODE_INIT,           // 上电自动归中复位
    GIMBAL_MODE_MECH,           // 机械模式
} GimbalMode_e;

/* 云台复位状态 */
typedef enum {
    GIMBAL_RESET_PENDING = 0,
    GIMBAL_RESET_DONE,
} GimbalReset_e;

/* Yaw 轴执行结构体 (KT9025 物理控制) */
typedef struct {
    float target_angle;         // 目标机械角度 (rad, 相对车头)
    float feedback_angle;       // 当前机械角度 (rad, 相对车头)
    float feedback_speed;       // 当前电机转速 (rad/s)

    pid_ctrl_t angle_pid;
    pid_ctrl_t speed_pid;
    int16_t give_current;       // 最终输出控制电流
} Gimbal_Yaw_t;

/* Pitch 轴目标管理 (发往上层 C 板) */
typedef struct {
    float target_mec_angle;     // 目标机械角度 (rad)
    float min_limit;            // 软限位下限
    float max_limit;            // 软限位上限
} Gimbal_Pitch_Target_t;

/* 云台整机对象 */
typedef struct {
    GimbalMode_e mode;
    GimbalReset_e reset_state;
    uint16_t init_timer;

    Gimbal_Yaw_t yaw;
    Gimbal_Pitch_Target_t pitch;
} Gimbal_t;

extern Gimbal_t hero_gimbal;

/* 外部调用接口 */
void Gimbal_Init(Gimbal_t *gimbal);
void Gimbal_Update(Gimbal_t *gimbal, CarMode_e car_mode);

#endif /* GIMBAL_H */
