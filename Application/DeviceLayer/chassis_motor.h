#ifndef CHASSIS_MOTOR_H
#define CHASSIS_MOTOR_H

#include "RM_Motor.h"

#define CHASSIS_FL_DIRECTION  1.0f
#define CHASSIS_FR_DIRECTION -1.0f
#define CHASSIS_BL_DIRECTION  1.0f
#define CHASSIS_BR_DIRECTION -1.0f

extern Motor_RM_t Chassis_Motor[4];
extern Motor_RM_Group_t Chassis_Motor_Group;

#endif
