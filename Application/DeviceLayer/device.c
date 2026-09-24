#include "device.h"
#include "kt_yaw_motor.h"

dev_list_t dev_list = {
    .rc_sen = &rc_sensor,
};

void DEVICE_Init(void)
{
    dev_list.rc_sen->init(dev_list.rc_sen);
    imu_sensor.init(&imu_sensor);

    Chassis_Motor_Group.group_init(&Chassis_Motor_Group);
    KT_Yaw_Init();

    My_Judge_Init();
    cap.init(&cap);
}
