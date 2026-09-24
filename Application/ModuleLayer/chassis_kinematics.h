#ifndef CHASSIS_KINEMATICS_H
#define CHASSIS_KINEMATICS_H

enum {
    CHASSIS_WHEEL_FL = 0,
    CHASSIS_WHEEL_FR,
    CHASSIS_WHEEL_BL,
    CHASSIS_WHEEL_BR,
    CHASSIS_WHEEL_COUNT
};

void Chassis_Mecanum_Calculate(float vx, float vy, float wz,
                               float max_rpm,
                               float wheel_rpm[CHASSIS_WHEEL_COUNT]);

#endif
