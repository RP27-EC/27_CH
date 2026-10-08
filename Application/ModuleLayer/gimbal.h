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

/* IMU pitch 与编码器 mec「抬头为正」的一致性宏。
   上机若发现 GYRO 域 pitch 反向（推摇杆头往反方向走），
   只改这里的符号，不要去动下层板目标生成和矩阵。
   2026-10-08 上机锁定（Watch 看已乘宏的 imu_*，快速抬/放）：
   - 曾试 SIGN=+1 / RATE=-1 → 向下 pitch 增、rate 正（与 mec 反）。
   - **本版 SIGN=-1 / RATE=-1 上机确认正确**：抬头为正，与 mec 一致。
   - 期望：抬 → imu_pitch 增、rate>0；放 → 二者皆减。
   - SIGN 与 RATE 成对标定，禁止只改一个。
   - 静置 pitch 可为负（零点/安装/Mahony 残差），PID 吃相对误差，不改宏。
   - 力矩 MECH：give_torque<0=抬头；target_speed=0 时
     give_torque=+kp*rate → 抬头 rate>0 输出低头力矩，阻尼为负反馈。 */
#define PITCH_IMU_SIGN        (-1.0f)  /* 角度：抬头为正（与 mec 一致） */
#define PITCH_IMU_RATE_SIGN   (-1.0f)   /* 角速度：与 imu_pitch 同号，抬头为正 */

/* Pitch 执行与控制数据结构体 */
typedef struct {
    float mec_min_rad;
    float mec_max_rad;

    /* 机械角闭环：目标、反馈和速度统一使用 rad / rad/s。 */
    float target_pitch;      /* 当前域目标：MECH=编码器机械角；GYRO=IMU 域角 (rad)，来自 0xD1 */
    float feedback_pitch;    /* 达妙编码器机械角反馈 (rad)；MECH 闭环与 MECH 重力前馈用；GYRO 软限位折算也用 */
    float feedback_speed;    /* 达妙编码器机械角速度 (rad/s)，MECH 内环反馈 */

    /* IMU 闭环反馈（GYRO/SPIN，pitch_use_imu=1 时启用） */
    uint8_t use_imu;         /* 0xD1 pitch_use_imu */
    float imu_pitch;         /* 本板 IMU pitch (rad)，与 0x022 同符号 */
    float imu_pitch_rate;    /* 本板 IMU pitch 角速度 (rad/s) */

    float gravity_max_torque; /* 重力前馈幅值 (N·m)，正负号决定补偿方向；MECH/GYRO 共用（同一套机构） */
    float gravity_phase_deg;  /* MECH 重力相位 (度)：吃编码器时修正机构零位与质心偏差 */
    float gyro_gravity_phase_deg; /* GYRO 重力相位 (度)：吃 IMU 世界俯仰时的相位，零位参考与编码器不同，须独立整定 */
    float max_torque_limit;   /* 最终输出力矩限幅 (N·m)，MECH/GYRO 共用硬顶 */
    float max_speed_limit;    /* MECH 外环目标角速度限幅 (rad/s) */
    float gyro_max_speed_limit; /* GYRO 外环目标角速度限幅 (rad/s)，与 MECH 独立 */

    /* MECH（编码器闭环）PID：已验证路径，勿与 GYRO 混用增益 */
    pid_ctrl_t mec_angle_pid;   /* MECH 外环角度 PID */
    pid_ctrl_t mec_speed_pid;   /* MECH 内环速度 PID */

    /* GYRO（IMU 闭环）PID：独立整定。IMU 噪声/滞后与编码器不同，
       不能沿用 mec_* 增益；初值仅保证能闭环，上机必须单独调。 */
    pid_ctrl_t gyro_angle_pid;  /* GYRO 外环角度 PID */
    pid_ctrl_t gyro_speed_pid;  /* GYRO 内环速度 PID */

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
void Gimbal_Safe_Stop(void);

#endif /* GIMBAL_H */
