/**
  ******************************************************************************
  * @file           : gimbal.c
  * @brief          : 上层板云台 Pitch 控制实现（达妙 J4310 双环 PID 闭环）
  * @details        : MECH（pitch_use_imu=0）：编码器机械角闭环（既有路径）。
  *                   GYRO/SPIN（pitch_use_imu=1）：本板 IMU 双环闭环，
  *                   目标在 IMU 世界域（下层板 0xD1 下发），软限位用
  *                   mec_now + (target_imu - imu_now) 折回机械行程。
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

static void Gimbal_Pitch_Clear_Pids(void)
{
    pid_clear(&gimbal.pitch.mec_angle_pid);
    pid_clear(&gimbal.pitch.mec_speed_pid);
    pid_clear(&gimbal.pitch.gyro_angle_pid);
    pid_clear(&gimbal.pitch.gyro_speed_pid);
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
 * Pitch 重力前馈补偿：
 * 幅值和相位通过 gimbal.pitch 成员在线调整；
 * 若补偿方向相反，将 gravity_max_torque 改为负值。
 *
 * 角度源按域选择（重力矩是世界系量，不是底盘相对量）：
 *  - MECH：编码器 mec。锁的是底盘相对角，平地上与世界角差一个常值，可并进 phase。
 *  - GYRO：本板 IMU 世界俯仰。上坡/下坡时底盘俯仰，闭环为保世界角会反向转
 *    编码器；若重力仍吃 encoder，前馈会跟着地形反着走，PID 硬扛稳态误差。
 * 幅值共用（同一套质心/杆长）；相位因零位参考不同必须分两套。
 */
static float Pitch_Gravity_Compensation(float pitch_rad, float phase_deg)
{
    return gimbal.pitch.gravity_max_torque *
           cosf(pitch_rad + phase_deg * DEG_TO_RAD);
}


void Gimbal_Init(void)
{
    Pitch_Motor.single_init(&Pitch_Motor);
    gimbal.pitch.gravity_max_torque = 1.5f;//重力前馈此处（幅值未标定，两域共用起点）
    gimbal.pitch.gravity_phase_deg = 0.0f;          // MECH：编码器零位相位，未标定
    gimbal.pitch.gyro_gravity_phase_deg = 0.0f;     // GYRO：IMU 世界零位相位，未标定，勿与 MECH 混调
    gimbal.pitch.max_torque_limit = 3.5f;//力矩限制从3.5下降至1.8防止暴走砸头
    gimbal.pitch.max_speed_limit = 6.0f;//MECH 最大角速度从8.0下降至6.0rad/s
    gimbal.pitch.gyro_max_speed_limit = 6.0f;//GYRO 外环限速，与 MECH 独立整定

    /* ====== MECH 外环角度 PID（纯 P）—— 编码器路径，保持已验证参数 ====== */
    gimbal.pitch.mec_angle_pid.kp = 20.0f;
    gimbal.pitch.mec_angle_pid.ki = 0.0f;
    gimbal.pitch.mec_angle_pid.kd = 0.0f;
    gimbal.pitch.mec_angle_pid.out_max = gimbal.pitch.max_speed_limit;
    gimbal.pitch.mec_angle_pid.integral_max = 0.0f;

    /* ====== MECH 内环速度 PID（PI）—— 保持已验证的运动方向参数 ====== */
    /* 达妙 J4310 额定力矩约 3.5 ~ 7 N·m，输出限幅建议先设 3.5 N·m */
    gimbal.pitch.mec_speed_pid.kp = 0.85f;
    gimbal.pitch.mec_speed_pid.ki = 0.1f;  //保持小积分消摩擦力
    gimbal.pitch.mec_speed_pid.kd = 0.0f;
    gimbal.pitch.mec_speed_pid.out_max = gimbal.pitch.max_torque_limit;
    gimbal.pitch.mec_speed_pid.integral_max = 1.5f;

    /* ====== GYRO 外环/内环 PID —— 与 MECH 独立，禁止直接抄 mec_* 上场 ======
       初值与 MECH 相同只为「能闭环」，不是 IMU 域标定结论。
       IMU 角度噪声、相位滞后和编码器不同，上机后必须单独整定这两套。 */
    gimbal.pitch.gyro_angle_pid.kp = 20.0f;
    gimbal.pitch.gyro_angle_pid.ki = 0.0f;
    gimbal.pitch.gyro_angle_pid.kd = 0.0f;
    gimbal.pitch.gyro_angle_pid.out_max = gimbal.pitch.gyro_max_speed_limit;
    gimbal.pitch.gyro_angle_pid.integral_max = 0.0f;

    gimbal.pitch.gyro_speed_pid.kp = 0.85f;
    gimbal.pitch.gyro_speed_pid.ki = 0.1f;
    gimbal.pitch.gyro_speed_pid.kd = 0.0f;
    gimbal.pitch.gyro_speed_pid.out_max = gimbal.pitch.max_torque_limit;
    gimbal.pitch.gyro_speed_pid.integral_max = 1.5f;

    /* 数据清零 */
    gimbal.pitch.target_pitch = 0.0f;
    gimbal.pitch.feedback_pitch = 0.0f;
    gimbal.pitch.feedback_speed = 0.0f;
    gimbal.pitch.use_imu = 0U;
    gimbal.pitch.imu_pitch = 0.0f;
    gimbal.pitch.imu_pitch_rate = 0.0f;
    /* Feedback convention: up is positive, down is negative. */
    //软限位：须与下层板 gimbal.h 的 PITCH_MIN_DEG / PITCH_MAX_DEG 保持一致。

    gimbal.pitch.mec_min_rad = -18.0f * DEG_TO_RAD;
    gimbal.pitch.mec_max_rad = 35.0f * DEG_TO_RAD;
    gimbal.pitch.give_torque = 0.0f;

    Gimbal_Pitch_Clear_Pids();
}


static void Gimbal_External_Date_Update(void)
{
    /*
     * 达妙 J4310 内部编码器始终解算，供：
     *  1) MECH 闭环反馈
     *  2) 重力前馈（机构相位）
     *  3) GYRO 域目标软限位（mec_now + Δimu）
     * motor_angle 已由驱动解码为 -PI~PI(rad)；参考工程使用
     * “编码器中值 + 方向取反”得到机械坐标。
     */
    float mec_angle = -(Pitch_Motor.rx_info->motor_angle - PITCH_MOTOR_ENCODER_MIDDLE);
    while (mec_angle > PITCH_MOTOR_HALF_RANGE) mec_angle -= 2.0f * PITCH_MOTOR_HALF_RANGE;
    while (mec_angle < -PITCH_MOTOR_HALF_RANGE) mec_angle += 2.0f * PITCH_MOTOR_HALF_RANGE;

    gimbal.pitch.feedback_pitch = mec_angle;
    gimbal.pitch.feedback_speed = -Pitch_Motor.rx_info->speed;

    /* IMU 反馈由 control_task 在 imu_sensor.update 之后写入 gimbal.pitch.imu_*，
       这里不直接摸 imu_sensor，保持云台模块只依赖 board_comm / 电机。 */
}

/**
  * @brief  目标域选择 + 软限位
  * @details MECH：0xD1 目标直接限在 mec_min/max。
  *          GYRO：目标是 IMU 域绝对角；软限位约束
  *                mec_now + (target_imu - imu_now) ∈ [mec_min, mec_max]，
  *                等价于把相对偏移量钳进机械行程，再折回 IMU 域。
  *          重力前馈与（若需要）调试显示始终用编码器 mec。
  */
static void Gimbal_Mech_Target_Update(void)
{
    const uint8_t use_imu = board_rx_info.pitch_use_imu ? 1U : 0U;
    /* 域切换：两套 PID 都清，防止另一域积分/输出残留带进新闭环 */
    if (use_imu != gimbal.pitch.use_imu) {
        Gimbal_Pitch_Clear_Pids();
    }
    gimbal.pitch.use_imu = use_imu;
    const float raw_target = board_rx_info.pitch_encoder_tar;

    if (gimbal.pitch.use_imu) {
        const float imu_now = gimbal.pitch.imu_pitch;
        float delta = raw_target - imu_now;
        float mec_tar = gimbal.pitch.feedback_pitch + delta;
        if (mec_tar > gimbal.pitch.mec_max_rad) {
            mec_tar = gimbal.pitch.mec_max_rad;
        } else if (mec_tar < gimbal.pitch.mec_min_rad) {
            mec_tar = gimbal.pitch.mec_min_rad;
        }
        gimbal.pitch.target_pitch = imu_now + (mec_tar - gimbal.pitch.feedback_pitch);
    } else {
        /* 下位机与编码器反馈统一为“向上为正”，不再反向解释目标角。 */
        gimbal.pitch.target_pitch = raw_target;
        if (gimbal.pitch.target_pitch > gimbal.pitch.mec_max_rad) {
            gimbal.pitch.target_pitch = gimbal.pitch.mec_max_rad;
        } else if (gimbal.pitch.target_pitch < gimbal.pitch.mec_min_rad) {
            gimbal.pitch.target_pitch = gimbal.pitch.mec_min_rad;
        }
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

    /* 每个控制周期同步限幅，使 Keil Watch 中的修改立即生效。
       两套 PID 的 out_max 各自跟自己的域限速/共用力矩顶。 */
    gimbal.pitch.mec_angle_pid.out_max = gimbal.pitch.max_speed_limit;
    gimbal.pitch.gyro_angle_pid.out_max = gimbal.pitch.gyro_max_speed_limit;
    gimbal.pitch.mec_speed_pid.out_max = gimbal.pitch.max_torque_limit;
    gimbal.pitch.gyro_speed_pid.out_max = gimbal.pitch.max_torque_limit;

    /* 反馈源：MECH 用编码器；GYRO 用本板 IMU（与 0x022 同符号）。
       增益按域选择，不共用；重力始终吃编码器 mec。 */
    pid_ctrl_t *angle_pid;
    pid_ctrl_t *speed_pid;
    float angle_fb;
    float speed_fb;
    if (gimbal.pitch.use_imu) {
        angle_pid = &gimbal.pitch.gyro_angle_pid;
        speed_pid = &gimbal.pitch.gyro_speed_pid;
        angle_fb  = gimbal.pitch.imu_pitch;
        speed_fb  = gimbal.pitch.imu_pitch_rate;
    } else {
        angle_pid = &gimbal.pitch.mec_angle_pid;
        speed_pid = &gimbal.pitch.mec_speed_pid;
        angle_fb  = gimbal.pitch.feedback_pitch;
        speed_fb  = gimbal.pitch.feedback_speed;
    }

    /* ====== 1. 外环角度 PID 计算 ====== */
    angle_pid->err = gimbal.pitch.target_pitch - angle_fb;
    single_pid_ctrl(angle_pid);

    /* 外环输出即为期望角速度 (rad/s) */
    float target_speed = angle_pid->out;

    /* ====== 2. 内环速度 PID 计算 ====== */
    speed_pid->target = target_speed;
    speed_pid->measure = speed_fb;
    pid_err_cal(speed_pid);
    single_pid_ctrl(speed_pid);

    /* ====== 3. 达妙 DM-J4310 电机力矩下发 ====== */
    /* PID 动态力矩叠加重力前馈，并限制最终输出范围 */
    /* 编码器反馈坐标已取反，电机力矩方向也需取反；重力前馈保持原有方向。 */
    /* 保持已验证的电机方向：反馈坐标取反后，PID 力矩需要取反；重力前馈方向不变。
       IMU 闭环沿用同一符号约定（PITCH_IMU_*_SIGN 上机校准）。 */
    /* 重力角度源按域：MECH=编码器机构角；GYRO=IMU 世界俯仰（见 Pitch_Gravity_Compensation 注释）。 */
    const float gravity_angle = gimbal.pitch.use_imu ? gimbal.pitch.imu_pitch
                                                     : gimbal.pitch.feedback_pitch;
    const float gravity_phase = gimbal.pitch.use_imu ? gimbal.pitch.gyro_gravity_phase_deg
                                                     : gimbal.pitch.gravity_phase_deg;
    gimbal.pitch.give_torque = -speed_pid->out +
                               Pitch_Gravity_Compensation(gravity_angle, gravity_phase);


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
    Gimbal_Mech_Target_Update();    /* 2. 读 0xD1 目标并按域软限位 */
    Gimbal_Pitch_Pid_Cal();         /* 3. 双环 PID（编码器或 IMU）并输出电机 */
}
