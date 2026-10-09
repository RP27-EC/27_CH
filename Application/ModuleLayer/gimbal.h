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

    /* IMU 域限位边界参考：IMU 角 - 机械角的锁存值。实时坐标差里混着
       两个传感器的更新时差，云台一抖边界就跟着抖，必须锁存。 */
    float limit_offset;      /* 锁存的 (imu_pitch - feedback_pitch) (rad) */
    bool limit_offset_valid; /* 切模式/安全停机后重新锁存 */
    float held_target;       /* 贴界锁存的目标值：外打期间不变 (rad) */
    bool target_held;        /* IMU 域目标处于边界保持状态 */
    bool held_at_max;        /* true=锁在上限 / false=锁在下限 */


    /* 分段重力前馈补偿参数（Keil Watch 可在线调整，全部为角度制） */
    float mech_gravity_max_torque; /* 机械模式满补偿力矩 (N·m) */
    float mech_gravity_zero_deg;   /* 机械模式衰减起点 (deg) */
    float mech_gravity_edge_deg;   /* 机械模式衰减宽度 (deg) */


    float imu_gravity_max_torque;  /* IMU 模式补偿幅值 (N·m) - cos 公式用 */
    // float imu_gravity_zero_deg;    /* IMU 模式衰减起点 (deg) - 已弃用：IMU 改回余弦模型 */
    // float imu_gravity_edge_deg;    /* IMU 模式衰减宽度 (deg) - 已弃用：IMU 改回余弦模型 */
    float imu_gravity_phase_deg;   /* IMU 模式补偿相位 (deg)：cos 公式相位偏移 */

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
