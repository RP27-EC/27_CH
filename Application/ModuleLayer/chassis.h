/**
  ******************************************************************************
  * @file           : chassis.h
  * @brief          : 底盘控制模块头文件
  * @details        : 定义底盘控制所需的常量和函数接口
  ******************************************************************************
  */

#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdbool.h>
#include <stdint.h>
#include "car_control.h"
#include "config_chassis.h"

#define CHASSIS_RC_MAX_VALUE  660.0f
#define CHASSIS_FORWARD_DEADZONE  (6)
#define CHASSIS_ROTATE_DEADZONE   (30)  /* 右摇杆左右(ch0)中位死区，单位SBUS计数 */

/*M3508电机的额定转速约为8000RPM，这里限制为6000RPM（改为7000RPM）以保证安全*/
#define CHASSIS_MAX_MOTOR_RPM 7000.0f

/* 小陀螺归一化自旋速度；轮速约为 CHASSIS_MAX_MOTOR_RPM * 此值。 */
#define CHASSIS_SPIN_NORMALIZED (0.72f)  /* 旋转约RPM */

/* GYRO：云台夹角反馈 + 云台 IMU 实测角速度前馈。 */
#define CHASSIS_GYRO_ALIGN_FRONT_MAX_DEG (88.0f) /* 车尾跟随时，低于88度切回车头。 */
#define CHASSIS_GYRO_ALIGN_BACK_MIN_DEG  (91.0f) /* 车头跟随时，高于91度切换车尾。 */
#define CHASSIS_FOLLOW_DEADZONE (0.026f) /* 约1.5度的角度死区，抑制零位附近的抖动 */
#define FOLLOW_K_LINEAR         (0.80f)  /* 线性项系数：决定小角度时的跟随力度 */
#define FOLLOW_K_QUADRATIC      (0.0f)  /* 二次项系数：增大大角度时的追赶力度 */

/* 标准麦轮：motor_rpm = yaw_rad_s * (半轴距+半轮距) / 轮半径 * 减速比 * 60/(2*pi)。
 * 必须核对 config_chassis.h 中的实际轮径、减速比和轮距；只用于 GYRO 前馈换算。 */
#define CHASSIS_YAW_RPM_PER_RADS (60.0f * WHEEL_REDUCT_RATIO * \
    ((Car_Length + Car_Width) * 0.5f) / (6.28318530718f * WHEEL_RADIUS))

typedef struct {
    /* Watch 可调，遥控和键鼠共用。 */
    float ff_gain;                 /* 默认1，实测角速度前馈倍率。 */
    float rpm_per_rads;            /* rad/s -> 电机RPM，默认由上述几何参数计算。 */
    float lpf_alpha;               /* 默认0.2，控制周期2ms。 */
    float deadzone_rads;           /* 默认0.03 rad/s，抑制静止漂移。 */
    /* 本周期观察量，正值为逆时针。 */
    float imu_speed_rads;
    float filtered_speed_rads;
    float ff_wz;                   /* 归一化前馈。 */
    float follow_wz;               /* 夹角修正。 */
    float output_wz;               /* 前馈+修正，限幅到[-1,1]。 */
    bool ff_valid;                 /* 上板在线、速度及参数有效。 */
    AlignDir_e align_dir;           /* GYRO底盘对齐端：0=车头、1=车尾，独立于云台机械锁轴。 */
    bool align_valid;               /* 当前处于有效的GYRO对齐；离开GYRO后重新就近选择。 */
    float raw_angle_rads;          /* 真实云台相对底盘夹角，归一到[-pi,pi]。 */
    float follow_error_rads;       /* 对所选0/pi轴的最短角度误差，仅用于旋转跟随。 */
} ChassisYawFollow_t;

extern ChassisYawFollow_t chassis_yaw_follow;

float Chassis_Calc_Follow_Wz_Norm(float delta_angle);
float Chassis_Calc_Gyro_Wz(float delta_angle, float gimbal_yaw_speed_rads,
                          bool gimbal_yaw_valid);
void Chassis_Reset_Yaw_Feedforward(void); /* 停车或换输入源时清零低通历史。 */


void Chassis_Init(void);

/**
  * @param  forward     : 前后方向通道值，正值向前，范围[-660, 660]（云台坐标系）
  * @param  lateral     : 左右方向通道值，正值向左，范围[-660, 660]（云台坐标系）
  * @param  rotate      : 旋转方向通道值，正值逆时针，范围[-660, 660]
  * @param  delta_theta : 云台相对车头的夹角 (rad，由 KT 编码器解算，已归一到[-π,π])
  * @param  gimbal_yaw_speed_rads : 云台 IMU 空间角速度(rad/s)，逆时针为正
  * @param  gimbal_yaw_valid : 上层 Yaw IMU 通信在线标志
  * @param  align_dir   : 保留接口参数；GYRO 跟随不使用机械模式的锁轴方向
  * @param  car_mode    : 整车当前工作模式
  * @param  failsafe    : 失联保护标志，true时立即停止
  */
void Chassis_Update(int16_t forward, int16_t lateral, int16_t rotate,
                    float delta_theta, float gimbal_yaw_speed_rads,
                    bool gimbal_yaw_valid, AlignDir_e align_dir,
                    CarMode_e car_mode, bool failsafe);


void Chassis_Stop(void);



#endif /* CHASSIS_H */
