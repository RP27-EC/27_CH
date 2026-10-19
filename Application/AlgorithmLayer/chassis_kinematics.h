/**
  ******************************************************************************
  * @file           : chassis_kinematics.h
  * @brief          : 底盘麦轮运动学解算模块头文件
  * @details        : 定义麦轮底盘运动学解算所需的轮子索引枚举和函数接口
  ******************************************************************************
  */

#ifndef CHASSIS_KINEMATICS_H
#define CHASSIS_KINEMATICS_H

/**
  * @brief  底盘轮子索引枚举
  * @details 定义四个麦轮的索引，用于数组访问和标识
  */
enum {
    CHASSIS_WHEEL_FL = 0,  /* 前左轮(Front Left) - 索引0 */
    CHASSIS_WHEEL_FR,      /* 前右轮(Front Right) - 索引1 */
    CHASSIS_WHEEL_BL,      /* 后左轮(Back Left) - 索引2 */
    CHASSIS_WHEEL_BR,      /* 后右轮(Back Right) - 索引3 */
    CHASSIS_WHEEL_COUNT    /* 轮子总数 ,在这里就是刚好作为索引4，可用在数组大小填写 */
};

/**
  * @brief  麦克纳姆轮底盘运动学解算函数声明
  * @param  vx       : 前后方向速度分量，范围[-1, 1]，正值向前
  * @param  vy       : 左右方向速度分量，范围[-1, 1]，正值向左
  * @param  wz       : 旋转角速度分量，范围[-1, 1]，正值逆时针
  * @param  max_rpm  : 电机允许的最大转速(RPM)
  * @param  wheel_rpm: 输出数组，存储四个轮子的目标转速(RPM)
  * @retval None
  */
void Chassis_Mecanum_Calculate(float vx, float vy, float wz,
                               float max_rpm,
                               float wheel_rpm[CHASSIS_WHEEL_COUNT]);

#endif
