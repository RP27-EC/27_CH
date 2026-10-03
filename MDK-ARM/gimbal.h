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

typedef struct {
    /* rad 与 rad/s */
    float target_pitch;      /* 期望目标角度 (来自下层板 0xD1) */
    float feedback_pitch;    /* 当前反馈角度 (来自上层板 IMU) */
    float feedback_speed;    /* 当前角速度 (来自上层板 IMU 陀螺仪) */

    /* 串级双环 PID */
    pid_ctrl_t angle_pid;    /* 外环角度 PID (纯 P) */
    pid_ctrl_t speed_pid;    /* 内环速度 PID (PI) */

    float give_torque;       /* 最终下发给达妙 J4310 电机的力矩 (N·m) */
} Gimbal_Pitch_t;


typedef struct gimbal_struct {
    Gimbal_Pitch_t pitch;

    void (*init)(void);      /* 初始化函数指针 */
    void (*work)(void);      /* 2ms 周期工作函数指针 (对应 gimbal.work()) */
} Gimbal_t;

extern Gimbal_t gimbal;

/* 外部调用函数声明 */
void Gimbal_Init(void);
void Gimbal_Work(void);

#endif /* GIMBAL_H */
