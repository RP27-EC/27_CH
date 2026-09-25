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

/**
  * @brief  遥控器通道最大值
  * @details 遥控器通道值的范围为[-660, 660]，用于归一化处理
  */
#define CHASSIS_RC_MAX_VALUE  660.0f

/**
  * @brief  底盘电机最大转速(RPM)
  * @details 电机允许的最大转速，用于运动学解算后的输出限幅
  *          M3508电机的额定转速约为8000RPM，这里限制为6000RPM以保证安全
  */
#define CHASSIS_MAX_MOTOR_RPM 6000.0f

/**
  * @brief  底盘初始化函数
  * @details 初始化底盘，确保上电后处于静止状态
  * @retval None
  */
void Chassis_Init(void);

/**
  * @brief  底盘控制更新函数
  * @details 根据遥控器输入更新底盘运动状态
  * @param  forward     : 前后方向通道值，正值向前，范围[-660, 660]
  * @param  lateral     : 左右方向通道值，正值向左，范围[-660, 660]
  * @param  rotate      : 旋转方向通道值，正值逆时针，范围[-660, 660]
  * @param  delta_theta : 云台相对车头的夹角 (rad，由 KT 编码器解算)
  * @param  car_mode    : 整车当前工作模式
  * @param  failsafe    : 失联保护标志，true时立即停止
  * @retval None
  */
void Chassis_Update(int16_t forward, int16_t lateral, int16_t rotate,
                    float delta_theta, CarMode_e car_mode, bool failsafe);

/**
  * @brief  底盘紧急停止
  * @retval None
  */
void Chassis_Stop(void);

#endif /* CHASSIS_H */
