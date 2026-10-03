/**
  ******************************************************************************
  * @file           : chassis.h
  * @brief          : 底盘控制模块头文件
  * @details        : 定义底盘控制所需的常量和函数接口
  ******************************************************************************
  */

#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdbool.h>
#include <stdint.h>
#include "car_control.h"

#define CHASSIS_RC_MAX_VALUE  660.0f
#define CHASSIS_FORWARD_DEADZONE  (6)
#define CHASSIS_ROTATE_DEADZONE   (30)  /* 右摇杆左右(ch0)中位死区，单位SBUS计数 */

/*M3508电机的额定转速约为8000RPM，这里限制为6000RPM以保证安全*/
#define CHASSIS_MAX_MOTOR_RPM 6000.0f

/* 小陀螺归一化自旋速度；轮速约为 CHASSIS_MAX_MOTOR_RPM * 此值。 */
#define CHASSIS_SPIN_NORMALIZED (0.75f)  /* 纯旋转约 4500 RPM */

/* 云台跟随参数：输入为云台相对车头的弧度角，输出为[-1, 1]的底盘旋转指令；当前未加入遥控器前馈。 */
#define CHASSIS_FOLLOW_DEADZONE (0.026f) /* 约1.5度的角度死区，抑制零位附近的抖动 */
#define FOLLOW_K_LINEAR         (0.80f)  /* 线性项系数：决定小角度时的跟随力度 */
#define FOLLOW_K_QUADRATIC      (0.0f)  /* 二次项系数：增大大角度时的追赶力度 */

float Chassis_Calc_Follow_Wz_Norm(float delta_angle);


void Chassis_Init(void);

/**
  * @param  forward     : 前后方向通道值，正值向前，范围[-660, 660]
  * @param  lateral     : 左右方向通道值，正值向左，范围[-660, 660]
  * @param  rotate      : 旋转方向通道值，正值逆时针，范围[-660, 660]
  * @param  delta_theta : 云台相对车头的夹角 (rad，由 KT 编码器解算)
  * @param  car_mode    : 整车当前工作模式
  * @param  failsafe    : 失联保护标志，true时立即停止
  */
void Chassis_Update(int16_t forward, int16_t lateral, int16_t rotate,
                    float delta_theta, CarMode_e car_mode, bool failsafe);


void Chassis_Stop(void);

#define CHASSIS_YAW_FF_LPF_ALPHA (0.20f)
#define CHASSIS_YAW_FF_GAIN      (0.0f)

#endif /* CHASSIS_H */
