/**
  ******************************************************************************
  * @file           : chassis.c
  * @brief          : 底盘控制模块
  * @details        : 本文件实现了麦克纳姆轮底盘的完整控制逻辑，包括：
  *                   遥控器输入 -> 归一化 -> 坐标旋转（可选）-> 麦轮解算 -> PID控制 -> CAN下发
  *
  ******************************************************************************
  */

#include "chassis.h"

#include "chassis_kinematics.h"  /* 麦轮运动学解算模块 */
#include "chassis_motor.h"       /* 底盘电机驱动模块 */
#include "pid.h"                 /* PID控制器模块 */

#include <math.h>

static const float chassis_wheel_direction[CHASSIS_WHEEL_COUNT] = {
    CHASSIS_FL_DIRECTION,  /* 前左轮方向系数 */
    CHASSIS_FR_DIRECTION,  /* 前右轮方向系数 */
    CHASSIS_BL_DIRECTION,  /* 后左轮方向系数 */
    CHASSIS_BR_DIRECTION,  /* 后右轮方向系数 */
};



static float chassis_yaw_ff_filtered = 0.0f;

static float Chassis_Clamp_Channel(int16_t channel)
{
    /* 上限限幅：不超过最大值660 */
    if (channel > (int16_t)CHASSIS_RC_MAX_VALUE) {
        channel = (int16_t)CHASSIS_RC_MAX_VALUE;
    }
    /* 下限限幅：不低于负最大值-660 */
    else if (channel < -(int16_t)CHASSIS_RC_MAX_VALUE) {
        channel = -(int16_t)CHASSIS_RC_MAX_VALUE;
    }

    /* 归一化：除以最大值，转换到[-1, 1]范围 */
    return (float)channel / CHASSIS_RC_MAX_VALUE;
}


/* 前后通道死区：死区内输出0，死区外重新映射，保证满杆仍为满量程。 */
static float Chassis_Clamp_Channel_Deadzone(int16_t channel, int16_t deadzone)
{
    if (channel > (int16_t)CHASSIS_RC_MAX_VALUE) {
        channel = (int16_t)CHASSIS_RC_MAX_VALUE;
    } else if (channel < -(int16_t)CHASSIS_RC_MAX_VALUE) {
        channel = -(int16_t)CHASSIS_RC_MAX_VALUE;
    }
    if (channel <= deadzone && channel >= -deadzone) {
        return 0.0f;
    }
    if (channel > 0) {
        channel -= deadzone;
    } else {
        channel += deadzone;
    }
    return (float)channel / (CHASSIS_RC_MAX_VALUE - (float)deadzone);
}

/*浮点值钳位到[-1.0, 1.0]范围 */
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

/* 非线性跟随：死区外采用线性项与带符号二次项，输出保持麦轮解算所需的归一化量纲。 */
float Chassis_Calc_Follow_Wz_Norm(float delta_angle)
{
    float err_eff;
    float wz_norm;

    /* 夹角很小时不跟转，避免车头在目标附近来回抖动。 */
    if (fabsf(delta_angle) < CHASSIS_FOLLOW_DEADZONE) {
        return 0.0f;
    }

    /* 扣除死区宽度，使指令在死区边界从0连续增大。 */
    if (delta_angle > 0.0f) {
        err_eff = delta_angle - CHASSIS_FOLLOW_DEADZONE;
    } else {
        err_eff = delta_angle + CHASSIS_FOLLOW_DEADZONE;
    }

    /* err_eff * |err_eff| 保留旋转方向，确保正负夹角的输出大小对称。 */
    wz_norm = (FOLLOW_K_LINEAR * err_eff) +
              (FOLLOW_K_QUADRATIC * err_eff * fabsf(err_eff));
    /* 麦轮解算使用归一化旋转指令，最终限制在[-1, 1]。 */
    return Chassis_Clamp_Normalized(wz_norm);
}


void Chassis_Stop(void)
{
    /* 遍历四个轮子，逐个清零PID控制器和电机输出 */
    for (unsigned int i = 0; i < CHASSIS_WHEEL_COUNT; ++i) {
        pid_ctrl_t *speed_pid = Chassis_Motor[i].ctrl->speed_ctrl;

        /* 清零PID控制器所有状态变量，确保不会残留上一次的积分和微分 */
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

    /* 通过CAN总线批量发送停止指令到所有电机 */
    Chassis_Motor_Group.group_set_torque(&Chassis_Motor_Group);
}


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
  *          1. 检查是否需要失联保护或处于休眠/初始化模式
  *          2. 将遥控器通道值归一化到[-1, 1]
  *          3. 在陀螺仪/小陀螺模式下进行坐标系旋转：
  *             将云台坐标系的输入旋转delta_theta角度到底盘坐标系
  *          4. 调用麦轮运动学解算，计算四个轮子的目标转速
  *          5. 应用轮子方向系数，设置电机速度目标
  *          6. 通过CAN总线批量发送控制指令
  *
  * @param  forward    : 前后方向遥控器通道值，正值向前，范围[-660, 660]
  * @param  lateral    : 左右方向遥控器通道值，正值向左，范围[-660, 660]
  * @param  rotate     : 旋转方向遥控器通道值，正值逆时针，范围[-660, 660]
  * @param  delta_theta: 云台相对车头的夹角（弧度），用于坐标系旋转
  * @param  car_mode   : 整车当前模式，决定是否进行坐标旋转
  * @param  failsafe   : 失联保护标志，true时立即停止底盘
  * @retval None
  */
void Chassis_Update(int16_t forward, int16_t lateral, int16_t rotate,
                    float delta_theta, CarMode_e car_mode, bool failsafe)
{
    float wheel_rpm[CHASSIS_WHEEL_COUNT];  /* 四个轮子的目标转速(RPM) */

    /* 将遥控器通道原始值归一化到[-1, 1] */
    float vx = Chassis_Clamp_Channel_Deadzone(forward, CHASSIS_FORWARD_DEADZONE);     /* 前后速度（带中位死区） */
    float vy = -Chassis_Clamp_Channel(lateral);   /* 左右速度 ，修改方向*/

    /* 机械模式：摇杆左打对应底盘逆时针，故反向旋转通道。 */
    /* 右摇杆左右(ch0)使用中位死区；死区外重新映射，保持满杆仍为满量程。 */
    float wz = -Chassis_Clamp_Channel_Deadzone(rotate, CHASSIS_ROTATE_DEADZONE);

    /* 失联保护：立即停止底盘并返回 */
    if (failsafe) {
        Chassis_Stop();
        return;
    }

    /* 初始化/休眠模式：停止底盘 */
    if (car_mode == CAR_MODE_INIT || car_mode == CAR_MODE_SLEEP) {
        Chassis_Stop();
        return;
    }

    //速度从云台坐标系转换到底盘坐标系
    if (car_mode == CAR_MODE_GYRO || car_mode == CAR_MODE_SPIN) {
        /* 预计算三角函数值 */
        const float cos_theta = cosf(delta_theta);
        const float sin_theta = sinf(delta_theta);

        /* 保存云台坐标系下的输入 */
        const float gimbal_vx = vx;
        const float gimbal_vy = vy;

        /* 2D旋转矩阵变换：云台坐标系 -> 底盘坐标系 */
        vx = gimbal_vx * cos_theta - gimbal_vy * sin_theta;
        vy = gimbal_vx * sin_theta + gimbal_vy * cos_theta;

        /* 小陀螺模式：底盘恒定以最大速度自旋，忽略操作手旋转输入 */
        if (car_mode == CAR_MODE_SPIN) {
            wz = Chassis_Clamp_Normalized(CHASSIS_SPIN_NORMALIZED);
            chassis_yaw_ff_filtered = 0.0f;

            /* 旋转优先*/
            const float trans_max = 1.0f - fabsf(wz);//剩给前进的速度
            const float trans_sum = fabsf(vx) + fabsf(vy); //遥控输入要多少
            if (trans_sum > trans_max && trans_sum > 0.001f) { //要且要的太多
                const float trans_scale = trans_max / trans_sum;
                vx *= trans_scale;
                vy *= trans_scale;
            }
        } else {
            /* 右摇杆回中时立即清空前馈，避免低通滤波残值造成松手后继续旋转。 */
            if (fabsf((float)rotate) <= (float)CHASSIS_ROTATE_DEADZONE) {
                chassis_yaw_ff_filtered = 0.0f;
                wz = 0.0f;
            } else {
                /* 非回中时才对右摇杆旋转输入进行低通平滑 */
                /* 一阶低通滤波（加权平均形式）：新值 = (1-α)×旧值 + α×摇杆输入 */
                chassis_yaw_ff_filtered =
                    (1.0f - CHASSIS_YAW_FF_LPF_ALPHA) * chassis_yaw_ff_filtered +
                    CHASSIS_YAW_FF_LPF_ALPHA * wz;
                    //得到摇杆前馈的值
            }
            wz = Chassis_Calc_Follow_Wz_Norm(delta_theta) + //偏移角度经过线性非线性误差叠加
                 CHASSIS_YAW_FF_GAIN * chassis_yaw_ff_filtered;//摇杆前馈乘以比例系数
            wz = Chassis_Clamp_Normalized(wz);
        }
    } else {
        chassis_yaw_ff_filtered = 0.0f;
    }


    Chassis_Mecanum_Calculate(vx,
                              vy,
                              wz,
                              CHASSIS_MAX_MOTOR_RPM,           /* 电机最大转速限制(6000RPM) */
                              wheel_rpm);                      /* 输出：四个轮子转速(RPM) */


    for (unsigned int i = 0; i < CHASSIS_WHEEL_COUNT; ++i) {
        /* 设置速度PID目标值：轮子转速(RPM) * 方向系数(+1/-1) */
        Chassis_Motor[i].ctrl->speed_ctrl->target =
            wheel_rpm[i] * chassis_wheel_direction[i];

        Chassis_Motor[i].single_set_speed(&Chassis_Motor[i]);
    }

    /* 一次CAN帧发送4个电机 */
    Chassis_Motor_Group.group_set_torque(&Chassis_Motor_Group);
}
