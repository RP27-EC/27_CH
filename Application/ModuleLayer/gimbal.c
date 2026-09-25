/**
  ******************************************************************************
  * @file           : gimbal.c
  * @brief          : 云台全模式实现（含 Yaw 轴陀螺仪双环 PID 闭环）
  * @details        : 实现 KT9025 编码器解算、遥控器空间视角积分、
  *                   陀螺仪模式/机械模式双环 PID 切换与电机力矩下发。
  ******************************************************************************
  */

#include "gimbal.h"
#include "rc_sensor.h"
#include "kt_yaw_motor.h"
#include "communicate.h"
#include <math.h>

#define PI 3.14159265358979323846f

Gimbal_t hero_gimbal;

/* ========================================================================= */
/*                           数学工具函数（防大回旋）                         */
/* ========================================================================= */

/**
 * @brief  将弧度规范化到 [-PI, PI]
 */
static float Angle_Normalize_PI(float rad) {
    while (rad > PI)  rad -= 2.0f * PI;
    while (rad < -PI) rad += 2.0f * PI;
    return rad;
}

/**
 * @brief  计算最短路径角度误差 (target - feedback)
 */
static float Calc_Shortest_Angle_Err(float target, float feedback) {
    return Angle_Normalize_PI(target - feedback);
}

/* ========================================================================= */
/*                          1. 云台初始化与复位流程                          */
/* ========================================================================= */

void Gimbal_Init(Gimbal_t *gimbal) {
    gimbal->mode = GIMBAL_MODE_SLEEP;
    gimbal->last_mode = GIMBAL_MODE_SLEEP;
    gimbal->reset_state = GIMBAL_RESET_PENDING;
    gimbal->init_timer = 0;

    /* Pitch 软限位设置 */
    gimbal->pitch.min_limit = PITCH_MIN_MEC_ANGLE;
    gimbal->pitch.max_limit = PITCH_MAX_MEC_ANGLE;
    gimbal->pitch.target_mec_angle = 0.0f;

    /* Yaw 轴外环角度 PID（纯 P 控制，建立对齐刚度） */
    gimbal->yaw.angle_pid.kp = 14.0f;
    gimbal->yaw.angle_pid.ki = 0.0f;
    gimbal->yaw.angle_pid.kd = 0.0f;
    gimbal->yaw.angle_pid.out_max = 12.0f;       /* 限制最大回正角速度 12 rad/s */
    gimbal->yaw.angle_pid.integral_max = 0.0f;
    gimbal->yaw.angle_pid.a = 1.0f;

    /* Yaw 轴内环速度 PID（PI 控制，提供高阻尼与抗扰） */
    gimbal->yaw.speed_pid.kp = 180.0f;
    gimbal->yaw.speed_pid.ki = 1.5f;
    gimbal->yaw.speed_pid.kd = 0.0f;
    gimbal->yaw.speed_pid.out_max = 2000.0f;     /* KT9025 电流输出上限 */
    gimbal->yaw.speed_pid.integral_max = 400.0f; /* 积分抗饱和限幅 */
    gimbal->yaw.speed_pid.a = 1.0f;

    /* 数据清零 */
    gimbal->yaw.target_mec_angle = 0.0f;
    gimbal->yaw.feedback_mec_angle = 0.0f;
    gimbal->yaw.feedback_mec_speed = 0.0f;

    gimbal->yaw.target_imu_angle = 0.0f;
    gimbal->yaw.imu_angle = 0.0f;
    gimbal->yaw.imu_speed = 0.0f;

    gimbal->yaw.give_current = 0;
    gimbal->yaw.pid_mode = GIMBAL_PID_MEC;

    pid_clear(&gimbal->yaw.angle_pid);
    pid_clear(&gimbal->yaw.speed_pid);
}

/**
 * @brief  上电自动寻零对齐车头
 */
static void Gimbal_Init_Step(Gimbal_t *gimbal) {
    gimbal->init_timer++;

    /* 机械零位对齐目标为 0.0 rad */
    gimbal->yaw.target_mec_angle = 0.0f;
    gimbal->pitch.target_mec_angle = 0.0f;

    /* 当角度误差极小且速度接近静止，或超过 1.5s (750 * 2ms)，复位完成 */
    float angle_err = fabsf(gimbal->yaw.feedback_mec_angle);
    if ((angle_err < 0.05f && fabsf(gimbal->yaw.feedback_mec_speed) < 0.2f) || (gimbal->init_timer >= 750)) {
        gimbal->reset_state = GIMBAL_RESET_DONE;
        gimbal->init_timer = 0;
        /* 复位完成瞬间，空间目标角同步为当前 IMU 角度，防止切模式甩头 */
        gimbal->yaw.target_imu_angle = gimbal->yaw.imu_angle;
    }
}

/* ========================================================================= */
/*                          2. 目标解算模块                                  */
/* ========================================================================= */

/**
 * @brief  机械模式目标计算：Yaw 轴锚定底盘车头正前方
 */
static void Gimbal_Mech_Update(Gimbal_t *gimbal) {
    gimbal->yaw.pid_mode = GIMBAL_PID_MEC;
    gimbal->yaw.target_mec_angle = 0.0f;  /* 相对底盘夹角死死锁在 0 弧度 */
}

/**
 * @brief  陀螺仪自稳模式目标计算：右摇杆 ch0 控制准星空间角速度
 */
static void Gimbal_Gyro_Update(Gimbal_t *gimbal) {
    gimbal->yaw.pid_mode = GIMBAL_PID_GYRO;

    /* 右摇杆 ch0: 左打为正(+)，右打为负(-) */
    float rc_yaw_input = -(float)rc_sensor.info->ch0;

    /* 对空间目标角进行积分累加 */
    gimbal->yaw.target_imu_angle += rc_yaw_input * RC_YAW_GYRO_SENSITIVITY;
    gimbal->yaw.target_imu_angle = Angle_Normalize_PI(gimbal->yaw.target_imu_angle);
}

/* ========================================================================= */
/*                     3. Yaw 轴陀螺仪 / 机械 双环 PID 闭环计算                */
/* ========================================================================= */

/**
 * @brief  Yaw 轴双环 PID 计算与输出下发
 * @details 核心分流：
 *          - GIMBAL_PID_GYRO: 外环用 target_imu_angle vs imu_angle, 内环用 imu_speed
 *          - GIMBAL_PID_MEC : 外环用 target_mec_angle vs feedback_mec_angle, 内环用 feedback_mec_speed
 */
static void Gimbal_Yaw_Pid_Cal(Gimbal_t *gimbal) {
    /* 休眠模式下立即清空 PID 并卸力 */
    if (gimbal->mode == GIMBAL_MODE_SLEEP) {
        pid_clear(&gimbal->yaw.angle_pid);
        pid_clear(&gimbal->yaw.speed_pid);
        gimbal->yaw.give_current = 0;
        kt_motor[0].base_info.motor_out = 0;
        KT_Yaw_SendIq(0);
        return;
    }

    /* 陀螺仪模式依赖上层 IMU；通信失联时禁止使用冻结姿态继续驱动。 */
    if (gimbal->mode == GIMBAL_MODE_GYRO && !board_rx_info.is_online) {
        pid_clear(&gimbal->yaw.angle_pid);
        pid_clear(&gimbal->yaw.speed_pid);
        gimbal->yaw.give_current = 0;
        kt_motor[0].base_info.motor_out = 0;
        KT_Yaw_SendIq(0);
        return;
    }

    float target_angle = 0.0f;
    float feedback_angle = 0.0f;
    float feedback_speed = 0.0f;

    /* ================= 步骤 1：根据 PID 模式选择反馈源 ================= */
    if (gimbal->yaw.pid_mode == GIMBAL_PID_GYRO) {
        /* 陀螺仪模式：大地坐标系闭环（空间自稳） */
        target_angle   = gimbal->yaw.target_imu_angle;
        feedback_angle = gimbal->yaw.imu_angle;
        feedback_speed = gimbal->yaw.imu_speed;   /* 直接使用上层板原生陀螺仪角速度 */
    } else {
        /* 机械/初始化模式：底盘相对坐标系闭环 */
        target_angle   = gimbal->yaw.target_mec_angle;
        feedback_angle = gimbal->yaw.feedback_mec_angle;
        feedback_speed = gimbal->yaw.feedback_mec_speed; /* 使用 KT 编码器速度 */
    }

    /* ================= 步骤 2：外环角度 PID 计算 ================= */
    /* 计算最短路径误差，防止过零大回旋 */
    float angle_err = Calc_Shortest_Angle_Err(target_angle, feedback_angle);
    gimbal->yaw.angle_pid.err = angle_err;
    single_pid_ctrl(&gimbal->yaw.angle_pid);

    /* 外环输出即为期望角速度 (rad/s) */
    float target_speed = gimbal->yaw.angle_pid.out;

    /* ================= 步骤 3：内环速度 PID 计算 ================= */
    gimbal->yaw.speed_pid.target = target_speed;
    gimbal->yaw.speed_pid.measure = feedback_speed;
    pid_err_cal(&gimbal->yaw.speed_pid);
    single_pid_ctrl(&gimbal->yaw.speed_pid);

    /* ================= 步骤 4：下发控制电流到 KT 电机 ================= */
    gimbal->yaw.give_current = (int16_t)gimbal->yaw.speed_pid.out;
    kt_motor[0].base_info.motor_out = gimbal->yaw.give_current;
    KT_Yaw_SendIq(gimbal->yaw.give_current);
}

/* ========================================================================= */
/*                          4. 云台主工作循环函数                            */
/* ========================================================================= */

/**
  * @brief  云台主控制更新函数（FreeRTOS 2ms 周期调用）
  * @param  gimbal   : 云台控制对象指针
  * @param  car_mode : 整车当前状态机模式
  */
void Gimbal_Update(Gimbal_t *gimbal, CarMode_e car_mode) {
    /*==========================================================================
     * 第 1 步：模式映射与安全保护
     *==========================================================================*/
    gimbal->last_mode = gimbal->mode;

    if (car_mode == CAR_MODE_SLEEP ||                           /* 整车休眠 */
        rc_sensor.work_state != DEV_ONLINE ||                   /* 遥控器离线 */
        rc_sensor.errno != NONE_ERR ||                          /* 遥控器错误 */
        kt_motor[0].KT_motor_info.state_info.work_state != M_ONLINE) {  /* KT 电机离线 */
        gimbal->mode = GIMBAL_MODE_SLEEP;
        gimbal->reset_state = GIMBAL_RESET_PENDING;
    }
    else if (car_mode == CAR_MODE_INIT || gimbal->reset_state == GIMBAL_RESET_PENDING) {
        gimbal->mode = GIMBAL_MODE_INIT;
    }
    else if (car_mode == CAR_MODE_GYRO || car_mode == CAR_MODE_SPIN) {
        gimbal->mode = GIMBAL_MODE_GYRO;  /* 陀螺仪自稳跟随模式 */
    }
    else {
        gimbal->mode = GIMBAL_MODE_MECH;  /* 机械坦克模式 */
    }

    /* 模式切换平滑过渡：切入 GYRO 瞬间，将目标同步为当前空间朝向，防止抽搐甩头 */
    if (gimbal->last_mode != GIMBAL_MODE_GYRO && gimbal->mode == GIMBAL_MODE_GYRO) {
        gimbal->yaw.target_imu_angle = gimbal->yaw.imu_angle;
        pid_clear(&gimbal->yaw.angle_pid);
        pid_clear(&gimbal->yaw.speed_pid);
    }

    /*==========================================================================
     * 第 2 步：读取下层板 KT9025 电机编码器（解算相对底盘夹角）
     *          注：无论在什么模式，feedback_mec_angle 都要实时计算，底盘跟随必须用！
     *==========================================================================*/
    float raw_encoder = (float)kt_motor[0].KT_motor_info.rx_info.encoder;
    float encoder_diff = raw_encoder - YAW_KT_ENCODER_MIDDLE;

    /* 65536 回绕约束到 [-32768, 32768] */
    while (encoder_diff > KT_ENCODER_HALF_RANGE)  encoder_diff -= KT_ENCODER_FULL_RANGE;
    while (encoder_diff < -KT_ENCODER_HALF_RANGE) encoder_diff += KT_ENCODER_FULL_RANGE;

    /* 转换为相对车头的标准弧度 [-PI, PI] */
    gimbal->yaw.feedback_mec_angle = encoder_diff * (PI / KT_ENCODER_HALF_RANGE);
    gimbal->yaw.feedback_mec_speed = (float)kt_motor[0].KT_motor_info.rx_info.speed * (PI / 180.0f);

    /*==========================================================================
     * 第 3 步：根据当前模式更新目标值
     *==========================================================================*/
    switch (gimbal->mode) {
        case GIMBAL_MODE_INIT:
            Gimbal_Init_Step(gimbal);
            break;

        case GIMBAL_MODE_MECH:
            Gimbal_Mech_Update(gimbal);
            break;

        case GIMBAL_MODE_GYRO:
            Gimbal_Gyro_Update(gimbal);
            break;

        case GIMBAL_MODE_SLEEP:
        default:
            break;
    }

    /*==========================================================================
     * 第 4 步：执行串级 PID 计算与下发力矩
     *==========================================================================*/
    Gimbal_Yaw_Pid_Cal(gimbal);
}
