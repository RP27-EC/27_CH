/**
  ******************************************************************************
  * @file           : Chassis_Posture.c
  * @brief          : 底盘姿态解算模块
  * @details        : 从IMU传感器读取数据，解算底盘的姿态角（pitch、yaw、roll）、
  *                   角速度、加速度等信息，并计算斜坡角度用于前馈控制。
  ******************************************************************************
  */

#include "Chassis_Posture.h"
#include "arm_math.h"  /* ARM数学库，用于三角函数和平方根计算 */

/* 静态函数声明 */
static void Chassis_Posture_Update(Chassis_Posture_t* My_Chassis_Posture);
static void Chassis_Slope_Update(Chassis_Posture_t* My_Chassis_Posture, float yaw, float pitch, float roll);

/**
  * @brief  底盘姿态信息全局变量
  * @details 存储底盘姿态的原始数据和计算结果
  */
Chassis_Posture_info_t Chassis_Posture_info;

/**
  * @brief  底盘姿态对象
  * @details 包含姿态信息和更新函数指针
  */
Chassis_Posture_t Chassis_Posture = {
	.data_update = Chassis_Posture_Update,  /* 姿态更新函数 */
	.info = &Chassis_Posture_info,          /* 姿态信息指针 */
};

/**
  * @brief  底盘姿态更新函数
  * @details 从IMU传感器读取数据，解算底盘的姿态信息：
  *          1. 角度更新：将IMU的欧拉角转换为底盘坐标系
  *          2. 角速度更新：读取三轴角速度
  *          3. 加速度更新：读取三轴加速度
  *          4. 世界坐标加速度更新：读取世界坐标系下的加速度
  *          5. 斜坡角度更新：计算底盘所在斜坡的角度
  *
  * @param  My_Chassis_Posture: 底盘姿态对象指针
  * @retval None
  */
static void Chassis_Posture_Update(Chassis_Posture_t* My_Chassis_Posture)
{
	Chassis_Posture_info_t* info = My_Chassis_Posture->info;

	/*==========================================================================
	 * 角度更新
	 * IMU的roll轴对应底盘的pitch轴（需要坐标系转换）
	 * 根据roll的正负进行不同的偏移处理
	 *==========================================================================*/
	if(imu_sensor.info->base_info.roll > 0)
	{
		/* roll为正时，减去π（180°）进行坐标转换 */
		info->pitch = imu_sensor.info->base_info.roll * Degree_to_rad - PI;
	}
	else
	{
		/* roll为负时，加上π（180°）进行坐标转换 */
		info->pitch = imu_sensor.info->base_info.roll * Degree_to_rad + PI;
	}
	info->pitch *= -1.f;  /* 取反，调整正方向 */

	/* IMU的pitch轴对应底盘的roll轴，加上微小偏移补偿安装误差 */
	info->roll = imu_sensor.info->base_info.pitch * Degree_to_rad + 0.004f;

	/* yaw轴直接对应，角度转弧度 */
	info->yaw = imu_sensor.info->base_info.yaw * Degree_to_rad;

	/*==========================================================================
	 * 角速度更新
	 * 将IMU的角速度转换为底盘坐标系，并转换单位（度/秒 → 弧度/秒）
	 *==========================================================================*/
	info->roll_v = imu_sensor.info->base_info.ave_rate_pitch * Degree_to_rad;   /* roll角速度 */
	info->pitch_v = -imu_sensor.info->base_info.ave_rate_roll * Degree_to_rad;  /* pitch角速度（取反） */
	info->yaw_v = -imu_sensor.info->base_info.ave_rate_yaw * Degree_to_rad;     /* yaw角速度（取反） */

	/*==========================================================================
	 * 加速度更新
	 * 读取IMU原始加速度数据（需要坐标系转换）
	 *==========================================================================*/
	info->a_x = imu_sensor.info->raw_info.acc_y;  /* x轴加速度 */
	info->a_y = imu_sensor.info->raw_info.acc_x;  /* y轴加速度 */
	info->a_z = imu_sensor.info->raw_info.acc_z;  /* z轴加速度 */

	/*==========================================================================
	 * 世界坐标加速度更新
	 * 读取世界坐标系下的加速度，用于运动控制
	 *==========================================================================*/
	info->x_world = imu_sensor.info->base_info.accy;  /* 世界坐标x轴加速度 */
	info->y_world = imu_sensor.info->base_info.accx;  /* 世界坐标y轴加速度 */
	info->z_world = imu_sensor.info->base_info.accz;  /* 世界坐标z轴加速度 */

	/* 求解斜坡角度信息，用于前馈计算 */
	Chassis_Slope_Update(My_Chassis_Posture, info->yaw, info->pitch, info->roll);
}

/**
  * @brief  测试用yaw角变量
  * @note   仅用于调试，实际代码中应删除
  */
float test_yaw;

/**
  * @brief  求解底盘所在斜坡的pitch角和yaw角
  * @details 通过旋转矩阵将底盘坐标系转换到世界坐标系，
  *          计算底盘所在斜坡的角度信息，用于坡道行驶时的前馈补偿。
  *
  *          计算步骤：
  *          1. 计算底盘z轴在世界坐标系下的投影
  *          2. 通过投影计算斜坡的pitch角（倾斜角）
  *          3. 通过投影计算斜坡的yaw角（航向角）
  *
  * @param  My_Chassis_Posture: 底盘姿态对象指针
  * @param  yaw   : 底盘航向角(弧度)
  * @param  pitch : 底盘俯仰角(弧度)
  * @param  roll  : 底盘横滚角(弧度)
  * @retval None
  */
static void Chassis_Slope_Update(Chassis_Posture_t* My_Chassis_Posture, float yaw, float pitch, float roll)
{
	/*==========================================================================
	 * 计算底盘z轴在世界坐标系下的投影
	 * 通过旋转矩阵将底盘坐标系的z轴[0,0,1]转换到世界坐标系
	 *==========================================================================*/
	float z_world[3];
	/* x分量: sin(yaw)*sin(roll) + cos(yaw)*sin(pitch)*cos(roll) */
	z_world[0] = arm_sin_f32(yaw)*arm_sin_f32(roll) + arm_cos_f32(yaw)*arm_sin_f32(pitch)*arm_cos_f32(roll);
	/* y分量: -cos(yaw)*sin(roll) + sin(yaw)*sin(pitch)*cos(roll) */
	z_world[1] = -arm_cos_f32(yaw)*arm_sin_f32(roll) + arm_sin_f32(yaw)*arm_sin_f32(pitch)*arm_cos_f32(roll);
	/* z分量: cos(pitch)*cos(roll) */
	z_world[2] = arm_cos_f32(pitch)*arm_cos_f32(roll);

	/*==========================================================================
	 * 计算斜坡的pitch角（倾斜角）
	 * 通过z轴投影的水平分量和垂直分量计算倾斜角度
	 *==========================================================================*/
	float temp;
	/* 计算水平分量的模长: sqrt(x² + y²) */
	arm_sqrt_f32(z_world[0]*z_world[0]+z_world[1]*z_world[1], &temp);
	/* 计算倾斜角: atan2(水平分量, 垂直分量) */
	arm_atan2_f32(temp, z_world[2], &My_Chassis_Posture->info->slope_pitch);

	/*==========================================================================
	 * 计算斜坡的yaw角（航向角）
	 * 只有当倾斜角足够大时（>0.08rad ≈ 4.6°）才计算
	 *==========================================================================*/
	float sin_yaw, cos_yaw;
	if(My_Chassis_Posture->info->slope_pitch > 0.08f)
	{
		/* 倾斜角足够大，计算斜坡的航向角 */
		sin_yaw = z_world[1];  /* y分量 */
		cos_yaw = z_world[0];  /* x分量 */
		arm_atan2_f32(sin_yaw, cos_yaw, &My_Chassis_Posture->info->slope_yaw);
		test_yaw = yaw - My_Chassis_Posture->info->slope_yaw;  /* 测试用 */
	}
	else
	{
		/* 倾斜角太小，认为在平地，斜坡航向角等于底盘航向角 */
		My_Chassis_Posture->info->slope_yaw = yaw;
	}
}
