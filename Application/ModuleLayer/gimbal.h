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
    /* 机械角闭环：目标、反馈和速度统一使用 rad / rad/s。 */
    float target_pitch;      /* 下层 0xD1 发送的机械目标角 (rad) */
    float feedback_pitch;    /* 达妙编码器机械角反馈 (rad) */
    float feedback_speed;    /* 达妙编码器机械角速度 (rad/s) */


    float gravity_max_torque; /* 重力前馈幅值 (N·m)，正负号决定补偿方向 */
    float gravity_phase_deg;  /* 重力前馈相位 (度)，修正机构零位与质心偏差 */
    float max_torque_limit;   /* 最终输出力矩限幅 (N·m) */
    float max_speed_limit;    /* 外环目标角速度限幅 (rad/s) */


    pid_ctrl_t angle_pid;    /* 外环角度 PID (纯 P) */
    pid_ctrl_t speed_pid;    /* 内环速度 PID (PI) */

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
