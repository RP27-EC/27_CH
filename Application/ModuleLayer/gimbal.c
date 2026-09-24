/**
  ******************************************************************************
  * @file           : gimbal.c
  * @brief          : 云台机械模式实现（KT9025 电机驱动）
  ******************************************************************************
  */

#include "gimbal.h"
#include "rc_sensor.h"
#include "kt_yaw_motor.h"
#include <math.h>

#define PI 3.14159265358979323846f

Gimbal_t hero_gimbal;

/* 外部 KT 电机对象（引用战队 drv 层的 kt_motor） */
/* ========================================================================= */
/*                           数学工具函数（防卡死设计）                       */
/* ========================================================================= */

/**
 * @brief 将弧度角度规范化在 [-PI, PI] 范围内
 */
static float Angle_Normalize_PI(float rad) {
    while (rad > PI)  rad -= 2.0f * PI;
    while (rad < -PI) rad += 2.0f * PI;
    return rad;
}

/**
 * @brief 计算最短路径角度误差 (target - feedback)
 */
static float Calc_Shortest_Angle_Err(float target, float feedback) {
    return Angle_Normalize_PI(target - feedback);
}

/* ========================================================================= */
/*                          1. 云台初始化与复位流程                          */
/* ========================================================================= */
void Gimbal_Init(Gimbal_t *gimbal) {
    gimbal->mode = GIMBAL_MODE_SLEEP;
    gimbal->reset_state = GIMBAL_RESET_PENDING;
    gimbal->init_timer = 0;

    /* Pitch 软限位 */
    gimbal->pitch.min_limit = PITCH_MIN_MEC_ANGLE;
    gimbal->pitch.max_limit = PITCH_MAX_MEC_ANGLE;
    gimbal->pitch.target_mec_angle = 0.0f;

    /* Yaw 串级 PID 参数（KT9025 直驱电机经验初值） */
    // 外环：角度误差(rad) -> 期望角速度(rad/s)，限速 15 rad/s
    gimbal->yaw.angle_pid.kp = 15.0f;
    gimbal->yaw.angle_pid.ki = 0.0f;
    gimbal->yaw.angle_pid.kd = 0.0f;
    gimbal->yaw.angle_pid.out_max = 15.0f;
    gimbal->yaw.angle_pid.integral_max = 0.0f;
    gimbal->yaw.angle_pid.a = 1.0f;
    // 内环：角速度误差(rad/s) -> 控制电流(DAC)，KT9025 最大输出通常为 2048 或 4096
    gimbal->yaw.speed_pid.kp = 180.0f;
    gimbal->yaw.speed_pid.ki = 1.5f;
    gimbal->yaw.speed_pid.kd = 0.0f;
    gimbal->yaw.speed_pid.out_max = 1500.0f;
    gimbal->yaw.speed_pid.integral_max = 400.0f;
    gimbal->yaw.speed_pid.a = 1.0f;
    pid_clear(&gimbal->yaw.angle_pid);
    pid_clear(&gimbal->yaw.speed_pid);
}

/**
 * @brief 上电自动寻找机械零点归中
 */
static void Gimbal_Init_Step(Gimbal_t *gimbal) {
    gimbal->init_timer++;

    // 目标角度直接定在车头正前方 0 弧度
    gimbal->yaw.target_angle = 0.0f;
    gimbal->pitch.target_mec_angle = 0.0f;

    // 当角度接近车头且速度极低，或超过 1.5s (750 * 2ms)，判定复位成功
    float angle_err = fabsf(gimbal->yaw.feedback_angle);
    if ((angle_err < 0.05f && fabsf(gimbal->yaw.feedback_speed) < 0.2f) || (gimbal->init_timer >= 750)) {
        gimbal->reset_state = GIMBAL_RESET_DONE;
        gimbal->init_timer = 0;
    }
}

/* ========================================================================= */
/*                          2. 机械模式目标解算                              */
/* ========================================================================= */
static void Gimbal_Mech_Control(Gimbal_t *gimbal) {
    /* 1. 读取遥控器通道（ch0: Yaw左右，ch1: Pitch上下） */
    float rc_pitch_input =  (float)rc_sensor.info->ch1;  // 右手系：上推为正

    /* 2. 目标角度积分累加（摇杆控制角速度） */
    gimbal->yaw.target_angle = 0.0f;
    gimbal->pitch.target_mec_angle += rc_pitch_input * RC_PITCH_MECH_SENSITIVITY;

    /* 3. 角度安全范围约束 */
    gimbal->yaw.target_angle = Angle_Normalize_PI(gimbal->yaw.target_angle);
    
    // Pitch 轴严格限幅在物理机械限位以内（防止打碎发射箱）
    if (gimbal->pitch.target_mec_angle > gimbal->pitch.max_limit) {
        gimbal->pitch.target_mec_angle = gimbal->pitch.max_limit;
    }
    if (gimbal->pitch.target_mec_angle < gimbal->pitch.min_limit) {
        gimbal->pitch.target_mec_angle = gimbal->pitch.min_limit;
    }
}

/* ========================================================================= */
/*                     3. KT9025 串级 PID 计算与输出驱动                      */
/* ========================================================================= */
static void Gimbal_Yaw_Execute(Gimbal_t *gimbal) {
    if (gimbal->mode == GIMBAL_MODE_SLEEP) {
        pid_clear(&gimbal->yaw.angle_pid);
        pid_clear(&gimbal->yaw.speed_pid);
        gimbal->yaw.give_current = 0;
        kt_motor[0].base_info.motor_out = 0;
        KT_Yaw_SendIq(0);
        return;
    }

    /* 1. 外环角度环：最短路径角误差 -> 期望角速度 (rad/s) */
    float angle_err = Calc_Shortest_Angle_Err(gimbal->yaw.target_angle, gimbal->yaw.feedback_angle);
    gimbal->yaw.angle_pid.err = angle_err;
    single_pid_ctrl(&gimbal->yaw.angle_pid);
    float target_speed = gimbal->yaw.angle_pid.out;

    /* 2. 内环速度环：期望角速度 vs 反馈转速 -> 输出电流 */
    gimbal->yaw.speed_pid.target = target_speed;
    gimbal->yaw.speed_pid.measure = gimbal->yaw.feedback_speed;
    pid_err_cal(&gimbal->yaw.speed_pid);
    single_pid_ctrl(&gimbal->yaw.speed_pid);
    float out_current = gimbal->yaw.speed_pid.out;

    gimbal->yaw.give_current = (int16_t)out_current;

    /* 3. 写入 KT 电机驱动缓冲 */
    kt_motor[0].base_info.motor_out = gimbal->yaw.give_current;
    KT_Yaw_SendIq(gimbal->yaw.give_current);
}

/* ========================================================================= */
/*                          4. 云台主工作循环                                */
/* ========================================================================= */
void Gimbal_Update(Gimbal_t *gimbal, CarMode_e car_mode) {
    /* 1. 模式映射 */
    if (car_mode == CAR_MODE_SLEEP ||
        rc_sensor.work_state != DEV_ONLINE ||
        rc_sensor.errno != NONE_ERR ||
        kt_motor[0].KT_motor_info.state_info.work_state != M_ONLINE) {
        gimbal->mode = GIMBAL_MODE_SLEEP;
        gimbal->reset_state = GIMBAL_RESET_PENDING;
    } else if (car_mode == CAR_MODE_INIT || gimbal->reset_state == GIMBAL_RESET_PENDING) {
        gimbal->mode = GIMBAL_MODE_INIT;
    } else {
        gimbal->mode = GIMBAL_MODE_MECH;
    }

    /* 2. 读取 KT9025 16位编码器反馈，转换为车身基准弧度 [-PI, PI] */
    float raw_encoder = (float)kt_motor[0].KT_motor_info.rx_info.encoder;
    float encoder_diff = raw_encoder - YAW_KT_ENCODER_MIDDLE;

    // 16位编码器过零回绕约束到 [-32768, 32768]
    while (encoder_diff > KT_ENCODER_HALF_RANGE)  encoder_diff -= KT_ENCODER_FULL_RANGE;
    while (encoder_diff < -KT_ENCODER_HALF_RANGE) encoder_diff += KT_ENCODER_FULL_RANGE;

    // 转换为标准弧度 (rad)
    gimbal->yaw.feedback_angle = encoder_diff * (PI / KT_ENCODER_HALF_RANGE);

    // KT9025 转速反馈（dps 转成 rad/s）
    gimbal->yaw.feedback_speed = (float)kt_motor[0].KT_motor_info.rx_info.speed * (PI / 180.0f);

    /* 3. 根据模式计算目标 */
    switch (gimbal->mode) {
        case GIMBAL_MODE_INIT:
            Gimbal_Init_Step(gimbal);
            break;

        case GIMBAL_MODE_MECH:
            Gimbal_Mech_Control(gimbal);
            break;

        case GIMBAL_MODE_SLEEP:
        default:
            break;
    }

    /* 4. 执行 KT9025 Yaw 轴闭环输出 */
    Gimbal_Yaw_Execute(gimbal);
}
