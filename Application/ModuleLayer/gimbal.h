/**
  ******************************************************************************
  * @file           : gimbal.h
  * @brief          : 云台控制模块头文件（支持机械模式与陀螺仪自稳模式）
  * @details        : 声明云台数据结构、PID模式枚举、常量宏定义与控制接口
  ******************************************************************************
  */

#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"
#include "car_control.h"

#define DEG_TO_RAD                  (0.01745329251994329577f)

/* ========================================================================= */
/*                          KT9025 硬件与几何参数                            */
/* ========================================================================= */
#define KT_ENCODER_FULL_RANGE       (65536.0f)   /* 16位编码器满量程 (2^16) */
#define KT_ENCODER_HALF_RANGE       (32768.0f)   /* 半圈值 */

/* 战队机械零点偏置 (对应移相后的基准值) */
#define YAW_KT_ENCODER_MIDDLE       (25823.0f + 32768.0f)

/* Pitch 轴软限位 (rad) */
#define PITCH_MIN_DEG               (-12.0f)
#define PITCH_MAX_DEG               (20.0535f) /* 编码器坐标向上最大 0.35rad */
#define PITCH_MIN_MEC_ANGLE         (PITCH_MIN_DEG * DEG_TO_RAD)
#define PITCH_MAX_MEC_ANGLE         (0.35f) /* 机械目标硬软件上限，避免超过 +0.35rad */

/* 摇杆控制灵敏度 (2ms 周期下的积分步长) */
#define RC_YAW_MECH_SENSITIVITY     (0.0004f)    /* 机械模式 Yaw 灵敏度 */
#define RC_YAW_GYRO_DEADZONE        (20.0f)      /* GYRO 模式摇杆死区，过滤中位偏差，此处的数值是摇杆通道值 */

#define RC_PITCH_DEG_PER_TICK       (0.0002865f)
#define RC_PITCH_MECH_SENSITIVITY   (RC_PITCH_DEG_PER_TICK * DEG_TO_RAD)
#define RC_PITCH_MECH_DEADZONE      (30.0f)  /* 右摇杆上下(ch1)中位死区，单位SBUS计数 */

/* 云台运行模式 */
typedef enum {
    GIMBAL_MODE_SLEEP = 0,     
    GIMBAL_MODE_INIT,           /* 上电自动寻中归位 */
    GIMBAL_MODE_MECH,         
    GIMBAL_MODE_GYRO,     
} GimbalMode_e;

/* 云台复位状态 */
typedef enum {
    GIMBAL_RESET_PENDING = 0,   /* 复位中 */
    GIMBAL_RESET_DONE,          /* 复位完成 */
} GimbalReset_e;

/* PID 控制器类型枚举 */
typedef enum {
    GIMBAL_PID_MEC = 0,         /* 机械角度闭环 (使用编码器反馈) */
    GIMBAL_PID_GYRO,            /* 陀螺仪空间闭环 (使用上层板 IMU 反馈) */
} GimbalPidMode_e;


/* Yaw 轴执行结构体 */
typedef struct {
    /* 机械模式数据 (底盘相对坐标系) */
    float target_mec_angle;     /* 目标机械角度 (rad) */
    float feedback_mec_angle;   /* 当前机械角度 (rad, 给底盘跟随使用) */
    float feedback_mec_speed;   /* 当前电机转速 (rad/s) */

    /* 陀螺仪模式数据 (上层 C 板大地惯性坐标系) */
    float target_imu_angle;     /* 目标空间绝对角度 (rad) */
    float imu_angle;            /* 上层 C 板发来的绝对角度 yaw_imu (rad) */
    float imu_speed;            /* 上层 C 板发来的绝对角速度 yaw_v (rad/s) */
    float gyro_sensitivity;      /* GYRO 模式每周期、每单位摇杆输入的目标角增量 (rad) */

    /* 控制模式与双环 PID */
    GimbalPidMode_e pid_mode;   /* 当前激活的 PID 模式 */
  
    pid_ctrl_t mec_angle_pid;//机械模式pid
    pid_ctrl_t mec_speed_pid;

    pid_ctrl_t gyro_angle_pid;//陀螺仪模式pid
    pid_ctrl_t gyro_speed_pid;
    
    int16_t give_current;       /* 最终下发力矩电流 */
} Gimbal_Yaw_t;


/* Pitch 轴目标管理 */
typedef struct {
    float target_mec_angle;     /* 目标机械角度 (rad) */
    float min_limit;            /* 机械下限位 */
    float max_limit;            /* 机械上限位 */
} Gimbal_Pitch_Target_t;


/* 云台整机管理对象 */
typedef struct {
    GimbalMode_e mode;
    GimbalMode_e last_mode;
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
