/**
  ******************************************************************************
  * @file           : gimbal.c
  * @brief          : 上层板云台 Pitch 控制实现（达妙 J4310 双环 PID 闭环）
  ******************************************************************************
  */
#include "gimbal.h"
#include "board_comm.h"
#include "imu_sensor.h"       /* 上层板 IMU 传感器驱动 */
#include "dm_motor.h"         /* 达妙 J4310 电机驱动头文件 */
#include <math.h>

#define DEG_TO_RAD (0.01745329251994329577f)

/* 云台全局管理实体 */
Gimbal_t gimbal = {
    .init = Gimbal_Init,
    .work = Gimbal_Work,
};

/* 内部私有函数声明 */
static void Gimbal_External_Date_Update(void);
static void Gimbal_Gyro_Update(void);
static void Gimbal_Pitch_Pid_Cal(void);

/**
  * @brief  云台 Pitch 轴初始化
  */
void Gimbal_Init(void)
{
    Pitch_Motor.single_init(&Pitch_Motor);
    /* ====== 外环角度 PID（纯 P 控制，建立跟踪刚度） ====== */
    gimbal.pitch.angle_pid.kp = 15.0f;
    gimbal.pitch.angle_pid.ki = 0.0f;
    gimbal.pitch.angle_pid.kd = 0.0f;
    gimbal.pitch.angle_pid.out_max = 8.0f;          /* 限制最大角速度 8 rad/s */
    gimbal.pitch.angle_pid.integral_max = 0.0f;

    /* ====== 内环速度 PID（PI 控制，提供高阻尼与抗扰） ====== */
    /* 达妙 J4310 额定力矩约 3.5 ~ 7 N·m，输出限幅建议先设 3.5 N·m */
    gimbal.pitch.speed_pid.kp = 0.8f;
    gimbal.pitch.speed_pid.ki = 0.02f;
    gimbal.pitch.speed_pid.kd = 0.0f;
    gimbal.pitch.speed_pid.out_max = 3.5f;          /* 限制最大力矩 3.5 N·m */
    gimbal.pitch.speed_pid.integral_max = 1.0f;

    /* 数据清零 */
    gimbal.pitch.target_pitch = 0.0f;
    gimbal.pitch.feedback_pitch = 0.0f;
    gimbal.pitch.feedback_speed = 0.0f;
    gimbal.pitch.give_torque = 0.0f;

    pid_clear(&gimbal.pitch.angle_pid);
    pid_clear(&gimbal.pitch.speed_pid);
}

/**
  * @brief  步骤 1：读 IMU 姿态（外部数据更新）
  */
static void Gimbal_External_Date_Update(void)
{
    /* 
     * 将 IMU 的 pitch 角度 (度) 转为 (弧度)
     * 极性特别注意：向上抬头时，角度数值必须是增大！
     * 若抬头发现在减小，直接在前面加负号：-(imu_sensor.info->base_info.pitch * DEG_TO_RAD)
     */
    gimbal.pitch.feedback_pitch = imu_sensor.info->base_info.pitch * DEG_TO_RAD;
    gimbal.pitch.feedback_speed = imu_sensor.info->base_info.rate_pitch * DEG_TO_RAD;
}

/**
  * @brief  步骤 2：读板间目标角度（来自下板 0xD1）
  */
static void Gimbal_Gyro_Update(void)
{
    /* 从板间通信接收缓存中获取下层板计算出的 pitch 目标角 */
    gimbal.pitch.target_pitch = board_rx_info.pitch_encoder_tar;
}

/**
  * @brief  步骤 3：Pitch 轴双环 PID 闭环计算与力矩输出
  */
static void Gimbal_Pitch_Pid_Cal(void)
{
    /* ====== 通信断联安全保护 ====== */
    /* 若超过 100ms 没收到下层板 0xD1 报文，电机必须卸力清空 PID，防止暴走 */
    if (!board_rx_info.is_online) {
        pid_clear(&gimbal.pitch.angle_pid);
        pid_clear(&gimbal.pitch.speed_pid);
        gimbal.pitch.give_torque = 0.0f;
        Pitch_Motor.tx_info->torque = 0.0f;
        Pitch_Motor.single_set_torque(&Pitch_Motor);
        return;
    }

    /* ====== 1. 外环角度 PID 计算 ====== */
    gimbal.pitch.angle_pid.err = gimbal.pitch.target_pitch - gimbal.pitch.feedback_pitch;
    single_pid_ctrl(&gimbal.pitch.angle_pid);

    /* 外环输出即为期望角速度 (rad/s) */
    float target_speed = gimbal.pitch.angle_pid.out;

    /* ====== 2. 内环速度 PID 计算 ====== */
    gimbal.pitch.speed_pid.target = target_speed;
    gimbal.pitch.speed_pid.measure = gimbal.pitch.feedback_speed;
    pid_err_cal(&gimbal.pitch.speed_pid);
    single_pid_ctrl(&gimbal.pitch.speed_pid);

    /* ====== 3. 达妙 DM-J4310 电机力矩下发 ====== */
    gimbal.pitch.give_torque = gimbal.pitch.speed_pid.out;

    /* 下发力矩到 Pitch 电机 */
    Pitch_Motor.tx_info->torque = gimbal.pitch.give_torque;
    Pitch_Motor.single_set_torque(&Pitch_Motor);
}

/**
  * @brief  云台 Pitch 控制顶层调度函数（对应 gimbal.work()）
  */
void Gimbal_Work(void)
{
    Gimbal_External_Date_Update();  /* 1. 读 IMU */
    Gimbal_Gyro_Update();           /* 2. 读下层板 0xD1 目标 */
    Gimbal_Pitch_Pid_Cal();         /* 3. 双环 PID 闭环并输出电机 */
}
