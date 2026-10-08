/**
  ******************************************************************************
  * @file           : gimbal.c
  * @brief          : 上层板云台 Pitch 控制实现（达妙 J4310 双环 PID 闭环）
  ******************************************************************************
  */
#include "gimbal.h"
#include "board_comm.h"
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

/* 本车标定：云台处于机械中位时的电机编码器角 (rad，约 42.2°) */
#define PITCH_MOTOR_ENCODER_MIDDLE (0.736945152f)
#define PITCH_MOTOR_HALF_RANGE     (3.14159265358979323846f)
static void Gimbal_Mech_Target_Update(void);
static void Gimbal_Pitch_Pid_Cal(void);

void Gimbal_Safe_Stop(void)
{
    pid_clear(&gimbal.pitch.angle_pid);
    pid_clear(&gimbal.pitch.speed_pid);
    gimbal.pitch.give_torque = 0.0f;
    if (Pitch_Motor.single_sleep != NULL) {
        Pitch_Motor.single_sleep(&Pitch_Motor);
    }
}

/*
 * Pitch 重力前馈补偿：
 * 幅值和相位通过 gimbal.pitch 成员在线调整；
 * 若补偿方向相反，将 gravity_max_torque 改为负值。
 */
static float Pitch_Gravity_Compensation(float pitch_rad)
{
    return gimbal.pitch.gravity_max_torque *
           cosf(pitch_rad + gimbal.pitch.gravity_phase_deg * DEG_TO_RAD);
}


void Gimbal_Init(void)
{
    Pitch_Motor.single_init(&Pitch_Motor);
    gimbal.pitch.gravity_max_torque = 1.5f;//重力前馈此处
    gimbal.pitch.gravity_phase_deg = 0.0f;
    gimbal.pitch.max_torque_limit = 3.5f;//力矩限制从3.5下降至1.8防止暴走砸头
    gimbal.pitch.max_speed_limit = 6.0f;//最大角速度从8.0下降至6.0rad/s
  
    /* ====== 外环角度 PID（纯 P 控制，建立跟踪刚度） ====== */
    gimbal.pitch.angle_pid.kp = 12.0f;
    gimbal.pitch.angle_pid.ki = 0.0f;
    gimbal.pitch.angle_pid.kd = 0.0f;
    gimbal.pitch.angle_pid.out_max = gimbal.pitch.max_speed_limit;
    gimbal.pitch.angle_pid.integral_max = 0.0f;

    /* ====== 内环速度 PID（PI 控制，提供高阻尼与抗扰） ====== */
    /* 达妙 J4310 额定力矩约 3.5 ~ 7 N·m，输出限幅建议先设 3.5 N·m */
    gimbal.pitch.speed_pid.kp = 0.8f;  // 保持已验证的运动方向参数
    gimbal.pitch.speed_pid.ki = 0.1f;  //保持小积分消摩擦力
    gimbal.pitch.speed_pid.kd = 0.0f;
    gimbal.pitch.speed_pid.out_max = gimbal.pitch.max_torque_limit;
    gimbal.pitch.speed_pid.integral_max = 1.0f;

    /* 数据清零 */
    gimbal.pitch.target_pitch = 0.0f;
    gimbal.pitch.feedback_pitch = 0.0f;
    gimbal.pitch.feedback_speed = 0.0f;
    /* Feedback convention: up is positive, down is negative. */
    gimbal.pitch.mec_min_rad = -12.0f * DEG_TO_RAD;
    gimbal.pitch.mec_max_rad = 0.35f;
    gimbal.pitch.give_torque = 0.0f;

    pid_clear(&gimbal.pitch.angle_pid);
    pid_clear(&gimbal.pitch.speed_pid);
}


static void Gimbal_External_Date_Update(void)
{
    /*
     * Pitch 主反馈改用达妙 J4310 内部编码器。
     * motor_angle 已由驱动解码为 -PI~PI(rad)；参考工程使用
     * “编码器中值 + 方向取反”得到机械坐标。
     * 这里不再把 IMU Pitch 用作闭环反馈，避免 IMU 角度范围异常影响控制。
     */
    float mec_angle = -(Pitch_Motor.rx_info->motor_angle - PITCH_MOTOR_ENCODER_MIDDLE);
    while (mec_angle > PITCH_MOTOR_HALF_RANGE) mec_angle -= 2.0f * PITCH_MOTOR_HALF_RANGE;
    while (mec_angle < -PITCH_MOTOR_HALF_RANGE) mec_angle += 2.0f * PITCH_MOTOR_HALF_RANGE;

    gimbal.pitch.feedback_pitch = mec_angle;
    gimbal.pitch.feedback_speed = -Pitch_Motor.rx_info->speed;

}

static void Gimbal_Mech_Target_Update(void)
{
    /* 从 0xD1 接收缓存中获取下层板计算出的 Pitch 机械目标角(rad) */
    /* 0xD1 接收编码器机械目标角(rad)。 */

    /* 下位机与编码器反馈统一为“向上为正”，不再反向解释目标角。 */
    gimbal.pitch.target_pitch = board_rx_info.pitch_encoder_tar;
    if (gimbal.pitch.target_pitch > gimbal.pitch.mec_max_rad) {
        gimbal.pitch.target_pitch = gimbal.pitch.mec_max_rad;
    } else if (gimbal.pitch.target_pitch < gimbal.pitch.mec_min_rad) {
        gimbal.pitch.target_pitch = gimbal.pitch.mec_min_rad;
    }
}


static void Gimbal_Pitch_Pid_Cal(void)
{
    /* ====== 通信断联安全保护 ====== */
    /* 若超过 100ms 没收到下层板 0xD1 报文，电机必须卸力清空 PID，防止暴走 */
    if (!board_rx_info.is_online || !board_rx_info.pitch_enable ||
        Pitch_Motor.single_set_torque == NULL ||
        Pitch_Motor.tx_info == NULL) {
        Gimbal_Safe_Stop();
        return;
    }

    /* 每个控制周期同步限幅，使 Keil Watch 中的修改立即生效。 */
    gimbal.pitch.angle_pid.out_max = gimbal.pitch.max_speed_limit;
    gimbal.pitch.speed_pid.out_max = gimbal.pitch.max_torque_limit;

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
    /* PID 动态力矩叠加重力前馈，并限制最终输出范围 */
    /* 编码器反馈坐标已取反，电机力矩方向也需取反；重力前馈保持原有方向。 */
    /* 保持已验证的电机方向：反馈坐标取反后，PID 力矩需要取反；重力前馈方向不变。 */
    gimbal.pitch.give_torque = -gimbal.pitch.speed_pid.out +
                               Pitch_Gravity_Compensation(gimbal.pitch.feedback_pitch);
    

    if (gimbal.pitch.give_torque > gimbal.pitch.max_torque_limit) {
        gimbal.pitch.give_torque = gimbal.pitch.max_torque_limit;
    } else if (gimbal.pitch.give_torque < -gimbal.pitch.max_torque_limit) {
        gimbal.pitch.give_torque = -gimbal.pitch.max_torque_limit;
    }

//    /* Hard mechanical boundary: only allow torque back toward the valid range. */
//    if (gimbal.pitch.feedback_pitch >= gimbal.pitch.mec_max_rad &&
//        gimbal.pitch.give_torque > 0.0f) {
//        gimbal.pitch.give_torque = 0.0f;
//    } else if (gimbal.pitch.feedback_pitch <= gimbal.pitch.mec_min_rad &&
//               gimbal.pitch.give_torque < 0.0f) {
//        gimbal.pitch.give_torque = 0.0f;
//    }

    /* 下发力矩到 Pitch 电机 */
    Pitch_Motor.tx_info->torque = gimbal.pitch.give_torque;
    Pitch_Motor.single_set_torque(&Pitch_Motor);
}

/*---------运用上面函数，云台 Pitch 控制顶层调度函数（对应 gimbal.work()）*/
void Gimbal_Work(void)
{
    Gimbal_External_Date_Update();  /* 1. 读达妙编码器机械角/速度 */
    Gimbal_Mech_Target_Update();    /* 2. 读下层板 0xD1 机械目标 */
    Gimbal_Pitch_Pid_Cal();         /* 3. 双环 PID 闭环并输出电机 */
}
