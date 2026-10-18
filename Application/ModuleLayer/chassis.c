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
#include "chassis_power.h"

#include "chassis_kinematics.h"  /* 麦轮运动学解算模块 */
#include "chassis_motor.h"       /* 底盘电机驱动模块 */
#include "PID.h"                 /* PID控制器模块 */

#include <math.h>

#if defined(__ARMCC_VERSION) && defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__
#error "Compile chassis.c with -ffp-mode=full so IMU finite-value guards remain effective."
#endif

static const float chassis_wheel_direction[CHASSIS_WHEEL_COUNT] = {
    CHASSIS_FL_DIRECTION,  /* 前左轮方向系数 */
    CHASSIS_FR_DIRECTION,  /* 前右轮方向系数 */
    CHASSIS_BL_DIRECTION,  /* 后左轮方向系数 */
    CHASSIS_BR_DIRECTION,  /* 后右轮方向系数 */
};

#define CHASSIS_PI (3.14159265358979323846f)
#define CHASSIS_DEG_TO_RAD (CHASSIS_PI / 180.0f)



ChassisYawFollow_t chassis_yaw_follow = {
    .ff_gain = 1.0f,
    .rpm_per_rads = CHASSIS_YAW_RPM_PER_RADS,
    .lpf_alpha = 0.20f,
    .deadzone_rads = 0.03f,
};

void Chassis_Reset_Yaw_Feedforward(void)
{
    /* 保留 Watch 调整的参数，只清观测量及滤波状态。 */
    chassis_yaw_follow.imu_speed_rads = 0.0f;
    chassis_yaw_follow.filtered_speed_rads = 0.0f;
    chassis_yaw_follow.ff_wz = 0.0f;
    chassis_yaw_follow.follow_wz = 0.0f;
    chassis_yaw_follow.output_wz = 0.0f;
    chassis_yaw_follow.ff_valid = false;
    chassis_yaw_follow.raw_angle_rads = 0.0f;
    chassis_yaw_follow.follow_error_rads = 0.0f;
    /* 输入源切换仅清前馈，不改变当前车头/车尾选择及迟滞状态。 */
}

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

    if (!isfinite(delta_angle)) {
        return 0.0f;
    }

    /* 夹角很小时不跟转，避免车头在目标附近来回抖动，tip这个视情况看是否取消 */
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

/* 输入为真实夹角，先归一，再选择较近的车头/车尾轴。
 * 首次进入（包括退出SPIN）按90度直接选；持续GYRO使用88/91度迟滞。 */
static float Chassis_Gyro_Align_Error(float angle)
{
    ChassisYawFollow_t *yaw = &chassis_yaw_follow;
    float abs_angle = fabsf(angle);
    if (!yaw->align_valid ||
        (yaw->align_dir != ALIGN_FRONT && yaw->align_dir != ALIGN_BACK)) {
        yaw->align_dir = abs_angle > CHASSIS_PI * 0.5f ? ALIGN_BACK : ALIGN_FRONT;
        yaw->align_valid = true;
    } else if (yaw->align_dir == ALIGN_FRONT &&
               abs_angle > CHASSIS_GYRO_ALIGN_BACK_MIN_DEG * CHASSIS_DEG_TO_RAD) {
        yaw->align_dir = ALIGN_BACK;
    } else if (yaw->align_dir == ALIGN_BACK &&
               abs_angle < CHASSIS_GYRO_ALIGN_FRONT_MAX_DEG * CHASSIS_DEG_TO_RAD) {
        yaw->align_dir = ALIGN_FRONT;
    }

    if (yaw->align_dir == ALIGN_BACK) {
        /* +150度对车尾的误差为-30度；-150度对应+30度。 */
        return angle >= 0.0f ? angle - CHASSIS_PI : angle + CHASSIS_PI;
    }
    return angle;
}

float Chassis_Calc_Gyro_Wz(float delta_angle, float gimbal_yaw_speed_rads,
                          bool gimbal_yaw_valid)
{
    ChassisYawFollow_t *yaw = &chassis_yaw_follow;
    float raw_angle = 0.0f;
    float follow_error = 0.0f;
    if (isfinite(delta_angle)) {
        raw_angle = fmodf(delta_angle, 2.0f * CHASSIS_PI);
        if (raw_angle > CHASSIS_PI) raw_angle -= 2.0f * CHASSIS_PI;
        if (raw_angle < -CHASSIS_PI) raw_angle += 2.0f * CHASSIS_PI;
        follow_error = Chassis_Gyro_Align_Error(raw_angle);
    } else {
        yaw->align_valid = false;
    }
    float follow_wz = Chassis_Calc_Follow_Wz_Norm(follow_error);

    /* 上板超时/异常数值时立即丢弃前馈和历史，不保留旧速度驱动底盘。 */
    if (!gimbal_yaw_valid || !isfinite(gimbal_yaw_speed_rads) ||
        !isfinite(delta_angle) || !isfinite(yaw->ff_gain) || yaw->ff_gain < 0.0f ||
        !isfinite(yaw->rpm_per_rads) || yaw->rpm_per_rads <= 0.0f ||
        !isfinite(yaw->lpf_alpha) || yaw->lpf_alpha <= 0.0f || yaw->lpf_alpha > 1.0f ||
        !isfinite(yaw->deadzone_rads) || yaw->deadzone_rads < 0.0f ||
        !isfinite(yaw->filtered_speed_rads)) {
        Chassis_Reset_Yaw_Feedforward();
    } else {
        float speed = gimbal_yaw_speed_rads;
        float max_speed = CHASSIS_MAX_MOTOR_RPM / yaw->rpm_per_rads;
        /* 进入低通前先限到满轮速对应的角速度，避免高速输入留下很长的滤波尾值。 */
        if (speed > max_speed) speed = max_speed;
        if (speed < -max_speed) speed = -max_speed;
        if (fabsf(speed) <= yaw->deadzone_rads) speed = 0.0f;
        yaw->imu_speed_rads = gimbal_yaw_speed_rads;
        yaw->filtered_speed_rads += yaw->lpf_alpha * (speed - yaw->filtered_speed_rads);
        /* 输出死区不清除滤波器状态，保证小速度能连续建立前馈。 */
        float ff_wz = (fabsf(yaw->filtered_speed_rads) <= yaw->deadzone_rads) ? 0.0f :
            yaw->filtered_speed_rads * (yaw->rpm_per_rads / CHASSIS_MAX_MOTOR_RPM) * yaw->ff_gain;
        if (!isfinite(ff_wz) || !isfinite(yaw->filtered_speed_rads)) {
            Chassis_Reset_Yaw_Feedforward();
        } else {
            yaw->ff_wz = Chassis_Clamp_Normalized(ff_wz);
            yaw->ff_valid = true;
        }
    }
    yaw->raw_angle_rads = raw_angle;
    yaw->follow_error_rads = follow_error;
    yaw->follow_wz = follow_wz;
    yaw->output_wz = Chassis_Clamp_Normalized(follow_wz + yaw->ff_wz);
    return yaw->output_wz;
}


void Chassis_Stop(void)
{
    Chassis_Reset_Yaw_Feedforward();
    chassis_yaw_follow.align_valid = false;
    chassis_yaw_follow.align_dir = ALIGN_FRONT;
    Chassis_Power_Clear_Output();
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
    Chassis_Power_Init();
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
  *          3. MECH/GYRO/SPIN 统一做坐标系旋转：
  *             操作手输入在云台坐标系（视线方向），按 delta_theta 投影到底盘系。
  *             头在车尾（Δθ≈180°）时 cos(180°)=-1，W 自动物理倒车，
  *             操作手视角仍为「向准星前方走」，不需要手写 W/S 反转。
  *          4. GYRO 独立选择较近车头/车尾跟随；平移始终使用真实云台夹角
  *          5. 调用麦轮运动学解算，计算四个轮子的目标转速
  *          6. 应用轮子方向系数，设置电机速度目标
  *          7. 通过CAN总线批量发送控制指令
  *
  * @param  forward    : 前后方向遥控器通道值，正值向前，范围[-660, 660]（云台坐标系）
  * @param  lateral    : 左右方向遥控器通道值，正值向左，范围[-660, 660]（云台坐标系）
  * @param  rotate     : 旋转方向遥控器通道值，正值逆时针，范围[-660, 660]
  * @param  delta_theta: 云台相对车头的夹角（弧度），用于坐标系旋转
  * @param  gimbal_yaw_speed_rads: 云台 IMU 实测空间角速度，rad/s，逆时针为正
  * @param  gimbal_yaw_valid: 上板 Yaw IMU 数据在线标志，超时只撤掉前馈
  * @param  align_dir  : 保留接口参数；机械锁轴由云台处理，GYRO 跟随不使用此值
  * @param  car_mode   : 整车当前模式，决定跟随/自旋/手动旋转
  * @param  failsafe   : 失联保护标志，true时立即停止底盘
  * @retval None
  */
void Chassis_Update(int16_t forward, int16_t lateral, int16_t rotate,
                    float delta_theta, float gimbal_yaw_speed_rads,
                    bool gimbal_yaw_valid, AlignDir_e align_dir,
                    CarMode_e car_mode, bool failsafe)
{
    float wheel_rpm[CHASSIS_WHEEL_COUNT];  /* 四个轮子的目标转速(RPM) */
    (void)align_dir;  /* GYRO底盘对齐端独立维护，不改变云台机械锁轴。 */

    /* 将遥控器通道原始值归一化到[-1, 1] */
    float vx = Chassis_Clamp_Channel_Deadzone(forward, CHASSIS_FORWARD_DEADZONE);     /* 前后速度（带中位死区） */
    float vy = -Chassis_Clamp_Channel(lateral);   /* 左右速度 ，修改方向*/

    /* 机械模式：摇杆左打对应底盘逆时针，故反向旋转通道。 */
    /* 右摇杆左右(ch0)使用中位死区；死区外重新映射，保持满杆仍为满量程。 */
    float wz = -Chassis_Clamp_Channel_Deadzone(rotate, CHASSIS_ROTATE_DEADZONE);

    /* 失联或夹角异常：立即停止，避免无效坐标变换传到轮速目标。 */
    if (failsafe || !isfinite(delta_theta)) {
        Chassis_Stop();
        return;
    }

    /* 初始化/休眠模式：停止底盘 */
    if (car_mode == CAR_MODE_INIT || car_mode == CAR_MODE_SLEEP) {
        Chassis_Stop();
        return;
    }

    if (car_mode != CAR_MODE_GYRO) {
        /* 自旋期间不切换对齐端；退出SPIN后的首个GYRO拍按当时夹角重新选择。
         * 保留align_dir供观察，但align_valid=false表示不沿用旧选择/机械冷却。 */
        chassis_yaw_follow.align_valid = false;
    }

    /* 速度从云台坐标系转换到底盘坐标系：MECH / GYRO / SPIN 统一投影。
     * 操作手输入在云台坐标系（视线方向），按 delta_theta 投影到底盘系。*/
    if (car_mode == CAR_MODE_GYRO || car_mode == CAR_MODE_SPIN ||
        car_mode == CAR_MODE_MECH) {
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
            Chassis_Reset_Yaw_Feedforward();

            /* 旋转优先*/
            const float trans_max = 1.0f - fabsf(wz);//剩给前进的速度
            const float trans_sum = fabsf(vx) + fabsf(vy); //遥控输入要多少
            if (trans_sum > trans_max && trans_sum > 0.001f) { //要且要的太多
                const float trans_scale = trans_max / trans_sum;
                vx *= trans_scale;
                vy *= trans_scale;
            }
        } else if (car_mode == CAR_MODE_GYRO) {
            /* 遥控/键鼠统一跟随云台实际运动；摇杆回中不再强制清前馈。
             * 跟随误差可折算到车尾；上面的平移投影仍使用真实delta_theta。 */
            wz = Chassis_Calc_Gyro_Wz(delta_theta, gimbal_yaw_speed_rads, gimbal_yaw_valid);
        } else {
            /* MECH：云台就近锁轴（0°或180°），ch0 手动转底盘，不做跟随。
               平移已经投影到云台坐标系——头锁在车尾时 W 自动倒车。 */
            Chassis_Reset_Yaw_Feedforward();
        }
    } else {
        Chassis_Reset_Yaw_Feedforward();
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

    /* 四轮 PID 已完成；功率限制必须在最终电流转换和 CAN 发送之前。 */
    Chassis_Power_Apply();

    /* 一次CAN帧发送4个电机 */
    Chassis_Motor_Group.group_set_torque(&Chassis_Motor_Group);
}
