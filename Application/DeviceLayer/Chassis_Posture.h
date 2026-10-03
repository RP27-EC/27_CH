/**
  ******************************************************************************
  * @file           : Chassis_Posture.h
  * @brief          : 底盘姿态解算模块头文件
  * @details        : 定义底盘姿态信息的数据结构和接口
  ******************************************************************************
  */

#ifndef __CHASSIS_POSTURE_H
#define __CHASSIS_POSTURE_H

/* Includes ------------------------------------------------------------------*/
#include "imu_sensor.h"  /* IMU传感器模块 */

/* Exported macro ------------------------------------------------------------*/
/**
  * @brief  角度转弧度的转换系数
  * @details 1° = π/180 ≈ 0.017453 弧度
  */
#define Degree_to_rad 0.017453f

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  底盘姿态信息结构体
  * @details 存储底盘的姿态角、角速度、加速度等信息
  */
typedef struct Chassis_Posture_info_struct_t
{
	float pitch;      /* 俯仰角(弧度)，绕x轴旋转 */
	float yaw;        /* 航向角(弧度)，绕z轴旋转 */
	float roll;       /* 横滚角(弧度)，绕y轴旋转 */

	float pitch_v;    /* 俯仰角速度(弧度/秒) */
	float yaw_v;      /* 航向角速度(弧度/秒) */
	float roll_v;     /* 横滚角速度(弧度/秒) */

	float a_x;        /* x轴加速度(m/s²) */
	float a_y;        /* y轴加速度(m/s²) */
	float a_z;        /* z轴加速度(m/s²) */

	float x_world;    /* 世界坐标系x轴加速度(m/s²) */
	float y_world;    /* 世界坐标系y轴加速度(m/s²) */
	float z_world;    /* 世界坐标系z轴加速度(m/s²) */

	float slope_pitch;  /* 斜坡俯仰角(弧度)，用于坡道前馈补偿 */
	float slope_yaw;    /* 斜坡航向角(弧度)，用于坡道前馈补偿 */
}Chassis_Posture_info_t;

/**
  * @brief  底盘姿态对象结构体
  * @details 包含姿态信息和更新函数指针
  */
typedef struct Chassis_Posture_struct_t
{
	Chassis_Posture_info_t *info;  /* 姿态信息指针 */
	void (*data_update)(struct Chassis_Posture_struct_t* My_Chassis_Posture);  /* 更新函数 */
}Chassis_Posture_t;

/* Exported functions --------------------------------------------------------*/
/**
  * @brief  底盘姿态对象（外部声明）
  * @details 全局姿态对象，可通过 Chassis_Posture.data_update() 更新姿态
  */
extern Chassis_Posture_t Chassis_Posture;

#endif
