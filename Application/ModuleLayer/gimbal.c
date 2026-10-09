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
#include "rp_math.h"          /* LinearMapClamp 分段线性映射 */
#include <math.h>

#define DEG_TO_RAD (0.01745329251994329577f)

/* 与下板同款的坐标差锁存阈值 / 限位内缩余量：相对上次锁存的坐标差
 * 累计变化超过阈值才跟随机身姿态刷新，两端各内缩同等余量。 */
#define PITCH_LIMIT_FRAME_HYST_RAD  (0.5f * DEG_TO_RAD)
#define PITCH_LIMIT_MARGIN_RAD      (PITCH_LIMIT_FRAME_HYST_RAD)

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
    /* 断联/失能期间姿态可能已经变了，恢复后按当前反馈重建限位参考。 */
    gimbal.pitch.limit_offset_valid = false;
    gimbal.pitch.target_held = false;
    if (Pitch_Motor.single_sleep != NULL) {
        Pitch_Motor.single_sleep(&Pitch_Motor);
    }
}

/*
 * Pitch 重力前馈补偿（分段线性，单边界，按闭环模式选参数组）：
 *   低角度区（<= zero_deg）            输出满补偿 max_torque
 *   归零沿（zero_deg ~ zero_deg+edge） 满补偿 -> 0 线性衰减
 *   高角度区（>= zero_deg+edge）       不补偿（输出 0）
 * 机械模式用 mech_gravity_*，IMU 模式用 imu_gravity_*，互不影响。
 * 全部参数为角度制，Keil Watch 可在线调整；edge 设为 0 即硬切换。
 */
static float Pitch_Gravity_Compensation(float pitch_rad)
{
    if (gimbal.pitch.use_imu)
    {
        /* IMU 模式：余弦模型（最初版本，物理公式） */
        return gimbal.pitch.imu_gravity_max_torque *
               cosf(pitch_rad + gimbal.pitch.imu_gravity_phase_deg * DEG_TO_RAD);
    }

    /* 机械模式：分段线性（满补 -> 线性衰减 -> 归零） */
    const float pitch_deg = pitch_rad / DEG_TO_RAD;

    const float max_torque = gimbal.pitch.mech_gravity_max_torque;
    const float zero       = gimbal.pitch.mech_gravity_zero_deg;
    const float edge       = gimbal.pitch.mech_gravity_edge_deg;

    /* 低角度区：满补偿（含整个低头行程，无下边界截止） */
    if (pitch_deg <= zero)
        return max_torque;

    /* 归零沿：满补 -> 0 线性衰减 */
    if (edge > 0.0f && pitch_deg < zero + edge)
        return LinearMapClamp(zero, zero + edge, pitch_deg, max_torque, 0.0f);

    /* 高角度区：不补偿 */
    return 0.0f;
}


void Gimbal_Init(void)
{
    Pitch_Motor.single_init(&Pitch_Motor);
    /* 机械模式分段重力前馈初始参数（上车实测后在 Keil Watch 中细调） */
    gimbal.pitch.mech_gravity_max_torque = 3.0f;  /* feedback_pitch < 0.134 rad 满补 3 N·m */
    gimbal.pitch.mech_gravity_zero_deg = 7.7f;    /* 衰减起点 = 0.134 rad ≈ +7.7° */
    gimbal.pitch.mech_gravity_edge_deg = 14.6f;    /* 衰减宽度 = 0.389-0.134 rad ≈ 14.6°，到 +22.3° 归零 */

    /* IMU 模式余弦重力前馈初始参数（imu_pitch 世界系仰角） */
    gimbal.pitch.imu_gravity_max_torque = 3.0f;  /* cos 公式幅值，0 rad 附近给 3 N·m */
    // gimbal.pitch.imu_gravity_zero_deg = 28.6f;   /* 已弃用：IMU 改回余弦模型 */
    // gimbal.pitch.imu_gravity_edge_deg = 8.0f;    /* 已弃用：IMU 改回余弦模型 */
    gimbal.pitch.imu_gravity_phase_deg = 0.0f;    /* cos 相位偏移，先 0；切 IMU 后按实测调 */

    gimbal.pitch.max_torque_limit = 5.5f;//力矩限制从5.5下降
    gimbal.pitch.max_speed_limit = 6.0f;//最大角速度从6.0下降到3.0rad/s，现在已改回6.0f
  
    /* ====== 外环====== */
    gimbal.pitch.mech_angle_pid.kp = 28.0f;
    gimbal.pitch.mech_angle_pid.ki = 0.0f;
    gimbal.pitch.mech_angle_pid.kd = 0.0f;
    gimbal.pitch.mech_angle_pid.out_max = gimbal.pitch.max_speed_limit;
    gimbal.pitch.mech_angle_pid.integral_max = 0.0f;

    /* ====== 内环 ====== */
    /* 达妙 J4310 额定力矩为 3.5 ~ 7 N·m，输出限幅建议先设为 3.5 N·m */
    gimbal.pitch.mech_speed_pid.kp = 0.8f;  // 保持已验证的运动方向参数
    gimbal.pitch.mech_speed_pid.ki = 0.0f;  //保持小积分消摩擦。
    gimbal.pitch.mech_speed_pid.kd = 0.0f;
    gimbal.pitch.mech_speed_pid.out_max = gimbal.pitch.max_torque_limit;
    gimbal.pitch.mech_speed_pid.integral_max = 1.0f;
    // gimbal.pitch.imu_angle_pid = gimbal.pitch.mech_angle_pid;  /* 已弃用：IMU/机械必须独立调参 */
    // gimbal.pitch.imu_speed_pid = gimbal.pitch.mech_speed_pid;  /* 已弃用：IMU/机械必须独立调参 */

    /* ================ IMU 模式独立 PID=================== */
    /* 外环角度 PID：跟踪刚度，IMU 噪声大时 kp 应比机械模式略小 */
    gimbal.pitch.imu_angle_pid.kp = 8.0f;                    /* 起步同机械，IMU 调稳后再降 */
    gimbal.pitch.imu_angle_pid.ki = 0.0f;
    gimbal.pitch.imu_angle_pid.kd = 0.0f;
    gimbal.pitch.imu_angle_pid.out_max = gimbal.pitch.max_speed_limit;
    gimbal.pitch.imu_angle_pid.integral_max = 0.0f;

    /* 内环 */
    gimbal.pitch.imu_speed_pid.kp = 0.8f;                     /* 起步同机械，按 IMU 实测调 */
    gimbal.pitch.imu_speed_pid.ki = 0.0f;                     /* 起步留 0，前馈到位后再加 */
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
    gimbal.pitch.limit_offset = 0.0f;
    gimbal.pitch.limit_offset_valid = false;
    gimbal.pitch.held_target = 0.0f;
    gimbal.pitch.target_held = false;
    gimbal.pitch.held_at_max = false;
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
        /* 换闭环域：旧的限位参考和贴界锁存值一并作废。 */
        gimbal.pitch.limit_offset_valid = false;
        gimbal.pitch.target_held = false;
    }
    gimbal.pitch.use_imu = (board_rx_info.pitch_use_imu != 0U);

}

static void Gimbal_Mech_Target_Update(void)
{
    /* 从 0xD1 接收缓存中获取下层板计算出的 Pitch 目标角(rad)。
     * 下位机与编码器反馈统一为“向上为正”，不再反向解释目标角。 */
    gimbal.pitch.target_pitch = board_rx_info.pitch_encoder_tar;

    if (!gimbal.pitch.use_imu) {
        /* 机械模式边界是常量，直接限幅即稳定。 */
        if (gimbal.pitch.target_pitch > gimbal.pitch.mec_max_rad) {
            gimbal.pitch.target_pitch = gimbal.pitch.mec_max_rad;
        } else if (gimbal.pitch.target_pitch < gimbal.pitch.mec_min_rad) {
            gimbal.pitch.target_pitch = gimbal.pitch.mec_min_rad;
        }
        return;
    }

    /* IMU 域最终限幅（设计文档的 final authority）：下板负责操作手语义
     * （打满给死），上板负责物理可达性。坐标差 imu_pitch - feedback_pitch
     * 里混着两个传感器的更新时差，云台一抖边界就跟着抖，实时差正是锯齿
     * 的耦合路径，必须锁存：相对上次锁存累计变化超过阈值才跟随机身姿态
     * 刷新，两端再内缩同等余量。 */
    const float frame_offset = gimbal.pitch.imu_pitch - gimbal.pitch.feedback_pitch;
    if (!gimbal.pitch.limit_offset_valid ||
        fabsf(frame_offset - gimbal.pitch.limit_offset) > PITCH_LIMIT_FRAME_HYST_RAD) {
        gimbal.pitch.limit_offset = frame_offset;
        gimbal.pitch.limit_offset_valid = true;
    }
    const float imu_min = gimbal.pitch.limit_offset + gimbal.pitch.mec_min_rad + PITCH_LIMIT_MARGIN_RAD;
    const float imu_max = gimbal.pitch.limit_offset + gimbal.pitch.mec_max_rad - PITCH_LIMIT_MARGIN_RAD;

    /* 边界保持：贴界后锁存的是数值本身，不是边界公式。下板发来的目标
     * 继续外打则当拍丢弃增量；向内移动即释放，释放当拍按当前边界截断
     * 一次回到可达区间。车身大幅倾斜的可达性在释放那一刻收口。 */
    if (gimbal.pitch.target_held) {
        const float delta = gimbal.pitch.target_pitch - gimbal.pitch.held_target;
        const bool outward = gimbal.pitch.held_at_max ? (delta > 0.0f) : (delta < 0.0f);
        if (outward) {
            gimbal.pitch.target_pitch = gimbal.pitch.held_target;  /* 外打增量丢弃 */
        } else if (delta != 0.0f) {
            gimbal.pitch.target_held = false;    /* 目标向内移动：当拍释放 */
        }
        /* delta == 0：目标一个数位都不动 */
    }
    if (!gimbal.pitch.target_held) {
        if (gimbal.pitch.target_pitch > imu_max) {
            gimbal.pitch.target_pitch = imu_max;
            gimbal.pitch.held_target = imu_max;
            gimbal.pitch.held_at_max = true;
            gimbal.pitch.target_held = true;
        } else if (gimbal.pitch.target_pitch < imu_min) {
            gimbal.pitch.target_pitch = imu_min;
            gimbal.pitch.held_target = imu_min;
            gimbal.pitch.held_at_max = false;
            gimbal.pitch.target_held = true;
        }
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
    gimbal.pitch.give_torque = -speed_pid->out +
                               Pitch_Gravity_Compensation(feedback_pitch);
    

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
