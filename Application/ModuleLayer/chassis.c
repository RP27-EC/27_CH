/**
  ******************************************************************************
  * @file           : chassis.c
  * @brief          : 底盘控制模块
  * @details        : 实现底盘的初始化、停止、更新控制等功能。
  *                   将遥控器输入的通道值转换为底盘运动指令，通过麦轮运动学
  *                   解算后控制四个电机的转速，实现全向运动。
  ******************************************************************************
  */

#include "chassis.h"

#include "chassis_kinematics.h"  /* 麦轮运动学解算模块 */
#include "chassis_motor.h"       /* 底盘电机驱动模块 */
#include "pid.h"                 /* PID控制器模块 */

#include <math.h>

#define CHASSIS_PI              (3.14159265358979323846f)
#define CHASSIS_SPIN_NORMALIZED (1.0f)

/**
  * @brief  四个轮子的方向系数数组
  * @details 由于电机安装位置和转向不同，需要通过方向系数进行补偿。
  *          正值表示电机正转时轮子正向运动，负值表示需要反转。
  *          具体方向由实际机械安装决定，需要调试确定。
  */
static const float chassis_wheel_direction[CHASSIS_WHEEL_COUNT] = {
    CHASSIS_FL_DIRECTION,  /* 前左轮方向系数 */
    CHASSIS_FR_DIRECTION,  /* 前右轮方向系数 */
    CHASSIS_BL_DIRECTION,  /* 后左轮方向系数 */
    CHASSIS_BR_DIRECTION,  /* 后右轮方向系数 */
};

/**
  * @brief  遥控器通道值限幅与归一化
  * @param  channel : 遥控器通道原始值，范围[-660, 660]
  * @retval 归一化后的浮点值，范围[-1.0, 1.0]
  */
static float Chassis_Clamp_Channel(int16_t channel)
{
    /* 上限限幅：不超过最大值 */
    if (channel > (int16_t)CHASSIS_RC_MAX_VALUE) {
        channel = (int16_t)CHASSIS_RC_MAX_VALUE;
    }
    /* 下限限幅：不低于负最大值 */
    else if (channel < -(int16_t)CHASSIS_RC_MAX_VALUE) {
        channel = -(int16_t)CHASSIS_RC_MAX_VALUE;
    }

    /* 归一化：除以最大值，转换到[-1, 1]范围 */
    return (float)channel / CHASSIS_RC_MAX_VALUE;
}

static float Chassis_Clamp_Normalized(float value)
{
    if (value > 1.0f) {
        return 1.0f;
    }
    if (value < -1.0f) {
        return -1.0f;
    }
    return value;
}

/**
  * @brief  底盘紧急停止
  * @details 立即停止所有轮子的运动，清零PID控制器状态和电机输出。
  *          用于失联保护、紧急情况等场景。
  *
  * @note   该函数会清零所有PID状态变量，确保电机完全停止
  * @retval None
  */
void Chassis_Stop(void)
{
    /* 遍历四个轮子，逐个清零PID控制器和电机输出 */
    for (unsigned int i = 0; i < CHASSIS_WHEEL_COUNT; ++i) {
        pid_ctrl_t *speed_pid = Chassis_Motor[i].ctrl->speed_ctrl;

        /* 清零PID控制器所有状态变量 */
        pid_clear(speed_pid);           /* 清零积分项和微分项 */
        speed_pid->target = 0.0f;       /* 目标值清零 */
        speed_pid->measure = 0.0f;      /* 测量值清零 */
        speed_pid->err = 0.0f;          /* 当前误差清零 */
        speed_pid->last_err = 0.0f;     /* 上次误差清零 */
        speed_pid->pout = 0.0f;         /* 比例输出清零 */
        speed_pid->iout = 0.0f;         /* 积分输出清零 */
        speed_pid->dout = 0.0f;         /* 微分输出清零 */
        speed_pid->last_dout = 0.0f;    /* 上次微分输出清零 */
        speed_pid->out = 0.0f;          /* 总输出清零 */

        /* 清零电机力矩输出 */
        Chassis_Motor[i].tx_info->torque = 0.0f;           /* 力矩清零 */
        Chassis_Motor[i].tx_info->torque_current = 0.0f;   /* 力矩电流清零 */
        Chassis_Motor[i].tx_info->torque_current_raw = 0;  /* 原始力矩电流清零 */
    }

    /* 通过CAN总线发送停止指令到所有电机 */
    Chassis_Motor_Group.group_set_torque(&Chassis_Motor_Group);
}

/**
  * @brief  底盘初始化
  * @details 调用上面的Chassis_Stop停止函数，确保上电后底盘处于静止状态
  * @retval None
  */
void Chassis_Init(void)
{
    Chassis_Stop();  /* 初始化时先停止底盘，确保安全 */
}

/**
  * @brief  底盘控制更新函数
  * @details 根据遥控器输入的三轴运动指令，通过麦轮运动学解算，
  *          计算并设置四个电机的目标转速，实现底盘全向运动。
  *
  *          控制流程：
  *          1. 检查是否需要失联保护
  *          2. 将遥控器通道值归一化到[-1, 1]
  *          3. 调用麦轮运动学解算，计算四个轮子的目标转速
  *          4. 应用轮子方向系数，设置电机速度目标
  *          5. 通过CAN总线发送控制指令
  *
  * @param  forward : 前后方向遥控器通道值，正值向前，范围[-660, 660]
  * @param  lateral : 左右方向遥控器通道值，正值向左，范围[-660, 660]
  * @param  rotate  : 旋转方向遥控器通道值，正值逆时针，范围[-660, 660]
  * @param  failsafe: 失联保护标志，true时立即停止底盘
  * @retval None
  */
void Chassis_Update(int16_t forward, int16_t lateral, int16_t rotate,
                    float delta_theta, CarMode_e car_mode, bool failsafe)
{
    float wheel_rpm[CHASSIS_WHEEL_COUNT];  /* 四个轮子的目标转速(RPM) */
    float vx = Chassis_Clamp_Channel(forward);
    float vy = Chassis_Clamp_Channel(lateral);
    float wz = Chassis_Clamp_Channel(rotate);

    /* 失联保护：立即停止底盘并返回 */
    if (failsafe) {
        Chassis_Stop();
        return;
    }

    if (car_mode == CAR_MODE_INIT || car_mode == CAR_MODE_SLEEP) {
        Chassis_Stop();
        return;
    }

    if (car_mode == CAR_MODE_GYRO || car_mode == CAR_MODE_LOB ||
        car_mode == CAR_MODE_SPIN) {
        const float cos_theta = cosf(delta_theta);
        const float sin_theta = sinf(delta_theta);
        const float gimbal_vx = vx;
        const float gimbal_vy = vy;

        vx = gimbal_vx * cos_theta - gimbal_vy * sin_theta;
        vy = gimbal_vx * sin_theta + gimbal_vy * cos_theta;

        if (car_mode == CAR_MODE_SPIN) {
            wz = CHASSIS_SPIN_NORMALIZED;
        } else {
            wz = Chassis_Clamp_Normalized(delta_theta / CHASSIS_PI);
        }
    }

    /*==========================================================================
     * 麦轮运动学解算
     * 将遥控器三轴输入归一化后，通过运动学公式计算四个轮子的目标转速
     *==========================================================================*/
    Chassis_Mecanum_Calculate(vx,                              /* 前后速度归一化 */
                              vy,                              /* 左右速度归一化 */
                              wz,                              /* 旋转速度归一化 */
                              CHASSIS_MAX_MOTOR_RPM,           /* 电机最大转速限制 */
                              wheel_rpm);                      /* 输出：四个轮子转速 */

    /*==========================================================================
     * 设置电机速度目标
     * 将运动学解算得到的轮子转速乘以方向系数后，设置为电机PID控制器的目标值
     *==========================================================================*/
    for (unsigned int i = 0; i < CHASSIS_WHEEL_COUNT; ++i) {
        /* 设置速度PID目标值：轮子转速 × 方向系数 */
        Chassis_Motor[i].ctrl->speed_ctrl->target =
            wheel_rpm[i] * chassis_wheel_direction[i];

        /* 调用单个电机的速度设置函数 */
        Chassis_Motor[i].single_set_speed(&Chassis_Motor[i]);
    }

    /* 通过CAN总线批量发送所有电机的控制指令 */
    Chassis_Motor_Group.group_set_torque(&Chassis_Motor_Group);
}
