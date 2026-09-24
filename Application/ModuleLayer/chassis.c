#include "chassis.h"

#include "chassis_kinematics.h"
#include "chassis_motor.h"
#include "pid.h"

static const float chassis_wheel_direction[CHASSIS_WHEEL_COUNT] = {
    CHASSIS_FL_DIRECTION,
    CHASSIS_FR_DIRECTION,
    CHASSIS_BL_DIRECTION,
    CHASSIS_BR_DIRECTION,
};

static float Chassis_Clamp_Channel(int16_t channel)
{
    if (channel > (int16_t)CHASSIS_RC_MAX_VALUE) {
        channel = (int16_t)CHASSIS_RC_MAX_VALUE;
    } else if (channel < -(int16_t)CHASSIS_RC_MAX_VALUE) {
        channel = -(int16_t)CHASSIS_RC_MAX_VALUE;
    }

    return (float)channel / CHASSIS_RC_MAX_VALUE;
}

void Chassis_Stop(void)
{
    for (unsigned int i = 0; i < CHASSIS_WHEEL_COUNT; ++i) {
        pid_ctrl_t *speed_pid = Chassis_Motor[i].ctrl->speed_ctrl;

        pid_clear(speed_pid);
        speed_pid->target = 0.0f;
        speed_pid->measure = 0.0f;
        speed_pid->err = 0.0f;
        speed_pid->last_err = 0.0f;
        speed_pid->pout = 0.0f;
        speed_pid->iout = 0.0f;
        speed_pid->dout = 0.0f;
        speed_pid->last_dout = 0.0f;
        speed_pid->out = 0.0f;
        Chassis_Motor[i].tx_info->torque = 0.0f;
        Chassis_Motor[i].tx_info->torque_current = 0.0f;
        Chassis_Motor[i].tx_info->torque_current_raw = 0;
    }

    Chassis_Motor_Group.group_set_torque(&Chassis_Motor_Group);
}

void Chassis_Init(void)
{
    Chassis_Stop();
}

void Chassis_Update(int16_t forward, int16_t lateral, int16_t rotate,
                    bool failsafe)
{
    float wheel_rpm[CHASSIS_WHEEL_COUNT];

    if (failsafe) {
        Chassis_Stop();
        return;
    }

    Chassis_Mecanum_Calculate(Chassis_Clamp_Channel(forward),
                              Chassis_Clamp_Channel(lateral),
                              Chassis_Clamp_Channel(rotate),
                              CHASSIS_MAX_MOTOR_RPM,
                              wheel_rpm);

    for (unsigned int i = 0; i < CHASSIS_WHEEL_COUNT; ++i) {
        Chassis_Motor[i].ctrl->speed_ctrl->target =
            wheel_rpm[i] * chassis_wheel_direction[i];
        Chassis_Motor[i].single_set_speed(&Chassis_Motor[i]);
    }

    Chassis_Motor_Group.group_set_torque(&Chassis_Motor_Group);
}
