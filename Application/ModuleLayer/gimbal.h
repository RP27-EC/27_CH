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

/* Pitch 执行与控制数据结构体 */
typedef struct {
    float mec_min_rad;
    float mec_max_rad;
    /* 机械角闭环：目标、反馈和速度统一使用 rad / rad/s。*/
    float target_pitch;      /* 下层 0xD1 发送的机械目标角 (rad) */
    float feedback_pitch;    /* 达妙编码器机械角反馈 (rad) */
    float feedback_speed;
    float imu_pitch;
    float imu_speed;
    bool use_imu;            /* 闭环反馈源选择：0=编码器，1=IMU */


    /* 分段重力前馈补偿参数（Keil Watch 可在线调整，全部为角度制） */
    float mech_gravity_max_torque; /* 机械模式满补偿力矩 (N·m) */
    float mech_gravity_zero_deg;   /* 机械模式衰减起点 (deg) */
    float mech_gravity_edge_deg;   /* 机械模式衰减宽度 (deg) */
    float imu_gravity_max_torque;  /* IMU 模式满补偿力矩 (N·m) */
    float imu_gravity_zero_deg;    /* IMU 模式衰减起点 (deg) */
    float imu_gravity_edge_deg;    /* IMU 模式衰减宽度 (deg) */

    /* IMU 模式目标角软限位（rad，IMU 世界系） */
    float imu_min_rad;   /* IMU 模式目标角下限 (rad) */
    float imu_max_rad;   /* IMU 模式目标角上限 (rad) */
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
void Gimbal_Safe_Stop(void);

#endif /* GIMBAL_H */
