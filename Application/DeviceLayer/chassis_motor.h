#ifndef __CHASSIS_MOTOR_H
#define __CHASSIS_MOTOR_H

#include "DM_Motor.h"
#include "RM_Motor.h"
#include "drv_can.h"
#include "kalman_filter.h"

#define L_F_ORDER_CORRECT    -1.f
#define L_B_ORDER_CORRECT    -1.f

#define R_F_ORDER_CORRECT    -1.f
#define R_B_ORDER_CORRECT    -1.f

#define L_W_ORDER_CORRECT    -1.f
#define R_W_ORDER_CORRECT    1.f

extern Motor_RM_Group_t Steer_Group;
extern Motor_RM_Group_t Wheel_Group;
extern Motor_RM_Group_t Front_Group;
extern Motor_RM_Group_t Back_Group;

#endif
