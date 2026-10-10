/**
  ******************************************************************************
  * @file           : gimbal.c
  * @brief          : 上层板云台 Pitch 控制实现（达妙 J4310 双环 PID 闭环）
  ******************************************************************************
  */
#include "gimbal.h"
#include "board_comm.h"
#include "imu_sensor.h"
#include "dm_motor.h"         /* 达妙 J4310 电机驱动头文件 */
#include <math.h>

#define DEG_TO_RAD (0.01745329251994329577f)

/* 云台全局管理实体 */
Gimbal_t gimbal = {
    .init = Gimbal_Init,
    .work = Gimbal_Work,
};

/* 内部私有函数声明 */
void Gimbal_External_Date_Update(void);

/* 本车标定：云台处于机械中位时的电机编码器中值(rad，约 42.2°) */
#define PITCH_MOTOR_ENCODER_MIDDLE (0.736945152f)
#define PITCH_MOTOR_HALF_RANGE     (3.14159265358979323846f)
static void Gimbal_Mech_Target_Update(void);
static void Gimbal_Pitch_Pid_Cal(void);

static void Gimbal_Pitch_Clear_Pids(void)
{
    pid_clear(&gimbal.pitch.mech_angle_pid);
    pid_clear(&gimbal.pitch.mech_speed_pid);
    pid_clear(&gimbal.pitch.imu_angle_pid);
    pid_clear(&gimbal.pitch.imu_speed_pid);
}

void Gimbal_Safe_Stop(void)
{
    Gimbal_Pitch_Clear_Pids();
    gimbal.pitch.give_torque = 0.0f;
    if (Pitch_Motor.single_sleep != NULL) {
        Pitch_Motor.single_sleep(&Pitch_Motor);
    }
}

/*
 * Pitch 重力/拉簧前馈补偿（一条 cos，MECH/GYRO 两模式同一套）：
 *   输入永远是编码器机械角，不随 use_imu 切换。
 *   拉簧受力只认机械相对角；吃编码器还切断
 *   "枪管微震 -> IMU 跳 -> 前馈跳 -> 力矩跳" 的自激回路。
 * 幅值/相位 Keil Watch 可在线调整；若补偿方向相反，
 * 将 gravity_max_torque 改为负值。
 */
static float Pitch_Gravity_Compensation(float pitch_rad)
{
    return gimbal.pitch.gravity_max_torque *
           cosf(pitch_rad + gimbal.pitch.gravity_phase_deg * DEG_TO_RAD);
}


void Gimbal_Init(void)
{
    Pitch_Motor.single_init(&Pitch_Motor);
    /* 重力/拉簧前馈初始参数 = 原版跑通值；上车静悬标定后定死。 */
    gimbal.pitch.gravity_max_torque = 2.0f; /* cos 幅值 (N·m) */
    gimbal.pitch.gravity_phase_deg = 0.0f;   /* cos 相位 (deg) */

    gimbal.pitch.max_torque_limit = 5.5f;//力矩限制从5.5下降
    gimbal.pitch.max_speed_limit = 6.0f;//最大角速度从6.0下降到3.0rad/s，现在已改回6.0f
  
    /* ====== 外环====== */
    gimbal.pitch.mech_angle_pid.kp = 22.0f;
    gimbal.pitch.mech_angle_pid.ki = 0.0f;
    gimbal.pitch.mech_angle_pid.kd = 0.0f;
    gimbal.pitch.mech_angle_pid.out_max = gimbal.pitch.max_speed_limit;
    gimbal.pitch.mech_angle_pid.integral_max = 0.0f;

    /* ====== 内环 ====== */
    /* 达妙 J4310 额定力矩为 3.5 ~ 7 N·m，输出限幅建议先设为 3.5 N·m */
    gimbal.pitch.mech_speed_pid.kp = 2.3f;  // 保持已验证的运动方向参数
    gimbal.pitch.mech_speed_pid.ki = 0.0f;  //保持小积分消摩擦。
    gimbal.pitch.mech_speed_pid.kd = 0.0f;
    gimbal.pitch.mech_speed_pid.out_max = gimbal.pitch.max_torque_limit;
    gimbal.pitch.mech_speed_pid.integral_max = 1.0f;
    // gimbal.pitch.imu_angle_pid = gimbal.pitch.mech_angle_pid;  /* 已弃用：IMU/机械必须独立调参 */
    // gimbal.pitch.imu_speed_pid = gimbal.pitch.mech_speed_pid;  /* 已弃用：IMU/机械必须独立调参 */

    /* ================ IMU 模式独立 PID=================== */
    /* 外环角度 PID：跟踪刚度，IMU 噪声大时 kp 应比机械模式略小 */
    gimbal.pitch.imu_angle_pid.kp = 21.0f;                    /* 起步同机械，IMU 调稳后再降 */
    gimbal.pitch.imu_angle_pid.ki = 0.0f;
    gimbal.pitch.imu_angle_pid.kd = 0.0f;
    gimbal.pitch.imu_angle_pid.out_max = gimbal.pitch.max_speed_limit;
    gimbal.pitch.imu_angle_pid.integral_max = 0.0f;

    /* 内环 */
    gimbal.pitch.imu_speed_pid.kp = 2.3f;                     /* 起步同机械，按 IMU 实测调 */
    gimbal.pitch.imu_speed_pid.ki = 0.015f;                   /* 保持小积分消摩擦力（同原版） */
    gimbal.pitch.imu_speed_pid.kd = 0.0f;
    gimbal.pitch.imu_speed_pid.out_max = gimbal.pitch.max_torque_limit;
    gimbal.pitch.imu_speed_pid.integral_max = 1.0f;

    /* 数据清零 */
    gimbal.pitch.target_pitch = 0.0f;
    gimbal.pitch.feedback_pitch = 0.0f;
    gimbal.pitch.feedback_speed = 0.0f;
    gimbal.pitch.imu_pitch = 0.0;
    gimbal.pitch.imu_speed = 0.0f;
    gimbal.pitch.use_imu = false;
    /* Feedback convention: up is positive, down is negative. */
    gimbal.pitch.mec_min_rad = -18.0f * DEG_TO_RAD;
    gimbal.pitch.mec_max_rad = 0.55f;   /* 机械上软限位：0.55 rad ≈ +31.5° */
    gimbal.pitch.give_torque = 0.0f;

    Gimbal_Pitch_Clear_Pids();
}


void Gimbal_External_Date_Update(void)
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
    /* 当前安装下实测原始 Pitch 抬头增大，与机械角和角速度同向。 */
    gimbal.pitch.imu_pitch = imu_sensor.info->base_info.pitch * DEG_TO_RAD;
    /* IMU 角速度约定向上为正，保持原符号。*/
    gimbal.pitch.imu_speed = imu_sensor.info->base_info.rate_pitch * DEG_TO_RAD;
    if (gimbal.pitch.use_imu != (board_rx_info.pitch_use_imu != 0U)) {
        Gimbal_Pitch_Clear_Pids();
    }
    gimbal.pitch.use_imu = (board_rx_info.pitch_use_imu != 0U);

}

static void Gimbal_Mech_Target_Update(void)
{
    /* 从 0xD1 接收缓存中获取下层板计算出的 Pitch 目标角(rad)。
     * 下位机与编码器反馈统一为“向上为正”，不再反向解释目标角。
     * 0xD1 的目标角域随 pitch_use_imu 声明：MECH=编码器机械域，
     * GYRO/SPIN=IMU 世界域（见下板 communicate.h）。 */
    gimbal.pitch.target_pitch = board_rx_info.pitch_encoder_tar;

    /* 最终限幅（设计文档的 final authority）：两域都是常数边界，
     * 与下板 gimbal.h 同名同值（PITCH_MIN/MAX_IMU_ANGLE）。
     * 打满时每拍截到同一个数，目标是水平直线。 */
    const float min_limit = gimbal.pitch.use_imu ? PITCH_MIN_IMU_ANGLE : gimbal.pitch.mec_min_rad;
    const float max_limit = gimbal.pitch.use_imu ? PITCH_MAX_IMU_ANGLE : gimbal.pitch.mec_max_rad;
    if (gimbal.pitch.target_pitch > max_limit) {
        gimbal.pitch.target_pitch = max_limit;
    } else if (gimbal.pitch.target_pitch < min_limit) {
        gimbal.pitch.target_pitch = min_limit;
    }
}


static void Gimbal_Pitch_Pid_Cal(void)
{
    /* ====== 通信断联安全保护 ====== */
    /* 若超过 100ms 没收到下层板 0xD1 报文，电机必须卸力清空 PID，防止暴走。*/
    if (!board_rx_info.is_online || !board_rx_info.pitch_enable ||
        Pitch_Motor.single_set_torque == NULL ||
        Pitch_Motor.tx_info == NULL) {
        Gimbal_Safe_Stop();
        return;
    }

    /* 每个控制周期同步限幅，使 Keil Watch 中的修改立即生效。*/
    pid_ctrl_t *angle_pid = gimbal.pitch.use_imu ? &gimbal.pitch.imu_angle_pid : &gimbal.pitch.mech_angle_pid;
    pid_ctrl_t *speed_pid = gimbal.pitch.use_imu ? &gimbal.pitch.imu_speed_pid : &gimbal.pitch.mech_speed_pid;
    angle_pid->out_max = gimbal.pitch.max_speed_limit;
    speed_pid->out_max = gimbal.pitch.max_torque_limit;

    /* ====== 1. 外环角度 PID 计算 ====== */
    /* 外环按模式选角度反馈；两种模式的速度内环均使用 IMU 角速度。
     * PID 参数仍按模式独立选择。 */
    const float feedback_pitch = gimbal.pitch.use_imu ? gimbal.pitch.imu_pitch : gimbal.pitch.feedback_pitch;
    const float feedback_speed = gimbal.pitch.imu_speed;
    angle_pid->target = gimbal.pitch.target_pitch;
    angle_pid->measure = feedback_pitch;
    pid_err_cal(angle_pid);
    single_pid_ctrl(angle_pid);

    /* 外环输出即为期望角速度 (rad/s) */
    float target_speed = angle_pid->out;

    /* ====== 2. 内环速度 PID 计算 ====== */
    speed_pid->target = target_speed;
    speed_pid->measure = feedback_speed;
    pid_err_cal(speed_pid);
    single_pid_ctrl(speed_pid);

    /* ====== 3. 达妙 DM-J4310 电机力矩下发 ====== */
    /* PID 动态力矩叠加重力前馈，并限制最终输出范围*/
    /* 编码器反馈坐标已取反，电机力矩方向也需取反；重力前馈保持原有方向。*/
    /* 保持已验证的电机方向：反馈坐标取反后，PID 力矩需要取反；重力前馈方向不变。*/
    /* 前馈永远吃编码器机械角 gimbal.pitch.feedback_pitch（不随 use_imu 切换）。*/
    gimbal.pitch.give_torque = -speed_pid->out +
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

/* Top-level Pitch control scheduler, called from gimbal.work(). */
void Gimbal_Work(void)
{
    Gimbal_External_Date_Update();
    Gimbal_Mech_Target_Update();
    Gimbal_Pitch_Pid_Cal();
}
