/**
  ******************************************************************************
  * @file           : gimbal.h
  * @brief          : 上层板云台 Pitch 控制头文件
  ******************************************************************************
  */
#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"

/* GYRO 域 Pitch 目标静态限位 (rad)：IMU 世界域，标定后定死不再调。
 * 下板 gimbal.h 同名同值，改一处必须改两处。 */
#define PITCH_MIN_IMU_ANGLE  (0.04f)
#define PITCH_MAX_IMU_ANGLE  (0.70f)

/* Pitch 执行与控制数据结构体 */
typedef struct {
    float mec_min_rad;
    float mec_max_rad;


    /* 机械角闭环：目标、反馈和速度统一使用 rad / rad/s。*/
    float target_pitch;      /* 下层 0xD1 发送的机械目标角 (rad) */
    float feedback_pitch;    /* 达妙编码器机械角反馈 (rad) */
    float feedback_speed;    /* 编码器速度 (rad/s)，保留用于观测 */
    float imu_pitch;
    float imu_speed;         /* 两种模式的速度内环反馈 (rad/s) */
    
    bool use_imu;            /* 角度外环与 PID 参数组：0=机械，1=IMU */


    /* 重力/拉簧前馈（MECH/GYRO 两模式同一套 cos，Keil Watch 可在线调整） */
    float gravity_max_torque; /* 前馈幅值 (N·m)，正负号决定补偿方向 */
    float gravity_phase_deg;  /* 前馈相位 (deg)，修正机构零位与质心偏差 */

    float max_torque_limit;   /* 最终输出力矩限幅 (N·m) */
    float max_speed_limit;    /* 外环目标角速度限幅 (rad/s) */


    pid_ctrl_t mech_angle_pid;
    pid_ctrl_t mech_speed_pid;
    pid_ctrl_t imu_angle_pid;
    pid_ctrl_t imu_speed_pid;

    float give_torque;       /* 最终下发给达妙 J4310 电机的力矩 (N·m) */
} Gimbal_Pitch_t;

/* 云台整机管理对象 */
typedef struct gimbal_struct {
    Gimbal_Pitch_t pitch;

    void (*init)(void);      /* 初始化函数指针 */
    void (*work)(void);      /* 2ms 周期工作函数指针 (对应 gimbal.work()) */
} Gimbal_t;

extern Gimbal_t gimbal;

/* 外部调用函数声明 */
void Gimbal_Init(void);
void Gimbal_Work(void);
void Gimbal_External_Date_Update(void); /* 刷新反馈和控制域，不驱动电机 */
void Gimbal_Safe_Stop(void);

#endif /* GIMBAL_H */
