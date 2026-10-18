/**
  ******************************************************************************
  * @file           : chassis_motor.h
  * @brief          : 底盘电机驱动模块头文件
  * @details        : 定义底盘电机方向常量和外部接口
  ******************************************************************************
  */

#ifndef CHASSIS_MOTOR_H
#define CHASSIS_MOTOR_H

#include "RM_motor.h"  /* RoboMaster电机驱动库 */

/**
  * @brief  四个轮子的方向系数
  * @details 由于电机安装位置和转向不同，需要通过方向系数进行补偿。
  *          正值表示电机正转时轮子正向运动，负值表示需要反转。

  * @note   方向系数需要根据实际机械安装和电机转向进行调整
  */
#define CHASSIS_FL_DIRECTION  1.0f   /* 前左轮方向系数：正向 */
#define CHASSIS_FR_DIRECTION -1.0f   /* 前右轮方向系数：反向 */
#define CHASSIS_BL_DIRECTION  1.0f   /* 后左轮方向系数：正向 */
#define CHASSIS_BR_DIRECTION -1.0f   /* 后右轮方向系数：反向 */

/**
  * @brief  四个底盘电机单独操作
  * @details 数组顺序：[0]=前左, [1]=前右, [2]=后左, [3]=后右
  *          每个电机对象包含完整的配置、状态和控制信息 */
extern Motor_RM_t Chassis_Motor[4];

/*管理四个底盘电机的组操作，支持批量发送控制指令 */
extern Motor_RM_Group_t Chassis_Motor_Group;

#endif
