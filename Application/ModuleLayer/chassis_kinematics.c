#include "chassis_kinematics.h"

#include <math.h>

void Chassis_Mecanum_Calculate(float vx, float vy, float wz,
                               float max_rpm,
                               float wheel_rpm[CHASSIS_WHEEL_COUNT])
{
    float max_abs = 0.0f;
    float scale;

    if (wheel_rpm == 0) {
        return;
    }

    if (max_rpm <= 0.0f) {
        for (unsigned int i = 0; i < CHASSIS_WHEEL_COUNT; ++i) {
            wheel_rpm[i] = 0.0f;
        }
        return;
    }

    wheel_rpm[CHASSIS_WHEEL_FL] = vx - vy - wz;
    wheel_rpm[CHASSIS_WHEEL_FR] = vx + vy + wz;
    wheel_rpm[CHASSIS_WHEEL_BL] = vx + vy - wz;
    wheel_rpm[CHASSIS_WHEEL_BR] = vx - vy + wz;

    for (unsigned int i = 0; i < CHASSIS_WHEEL_COUNT; ++i) {
        const float magnitude = fabsf(wheel_rpm[i]);
        if (magnitude > max_abs) {
            max_abs = magnitude;
        }
    }

    scale = (max_abs > 1.0f) ? (max_rpm / max_abs) : max_rpm;
    for (unsigned int i = 0; i < CHASSIS_WHEEL_COUNT; ++i) {
        wheel_rpm[i] *= scale;
    }
}
