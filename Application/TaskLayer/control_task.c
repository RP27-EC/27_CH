#include "control_task.h"

#include "chassis.h"
#include "gimbal_motor.h"
#include "rc_sensor.h"

void StartCtrlTask(void const *argument)
{
    (void)argument;
    Chassis_Init();

    for (;;) {
        const bool chassis_failsafe =
            (rc_sensor.work_state != DEV_ONLINE) ||
            (rc_sensor.errno != NONE_ERR);

        Yaw_Motor.single_sleep(&Yaw_Motor);
        Yaw_Motor.tx_info->torque = 0.0f;
        Yaw_Motor.single_set_torque(&Yaw_Motor);

        Chassis_Update(rc_sensor.info->ch3,
                       rc_sensor.info->ch2,
                       rc_sensor.info->ch0,
                       chassis_failsafe);

        osDelay(2);
    }
}
