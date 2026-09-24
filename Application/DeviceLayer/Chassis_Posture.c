#include "Chassis_Posture.h"
#include "arm_math.h"
static void Chassis_Posture_Update(Chassis_Posture_t* My_Chassis_Posture);
static void Chassis_Slope_Update(Chassis_Posture_t* My_Chassis_Posture, float yaw, float pitch, float roll);
Chassis_Posture_info_t Chassis_Posture_info;

Chassis_Posture_t Chassis_Posture = {
	.data_update = Chassis_Posture_Update,
	.info = &Chassis_Posture_info,
};

static void Chassis_Posture_Update(Chassis_Posture_t* My_Chassis_Posture)
{
	Chassis_Posture_info_t* info = My_Chassis_Posture->info;
	
	//角度更新
	if(imu_sensor.info->base_info.roll > 0)
	{
		info->pitch = imu_sensor.info->base_info.roll * Degree_to_rad - PI;
	}
	else
	{
		info->pitch = imu_sensor.info->base_info.roll * Degree_to_rad + PI;
	}
	info->pitch *= -1.f;

	info->roll = imu_sensor.info->base_info.pitch * Degree_to_rad + 0.004f;


	info->yaw = imu_sensor.info->base_info.yaw * Degree_to_rad;
	
	//角速度更新
	info->roll_v = imu_sensor.info->base_info.ave_rate_pitch * Degree_to_rad;
	info->pitch_v = -imu_sensor.info->base_info.ave_rate_roll * Degree_to_rad;
	info->yaw_v = -imu_sensor.info->base_info.ave_rate_yaw * Degree_to_rad;
	
	//加速度更新
	info->a_x = imu_sensor.info->raw_info.acc_y;
	info->a_y = imu_sensor.info->raw_info.acc_x;
	info->a_z = imu_sensor.info->raw_info.acc_z;
	
	//世界加速度更新
	info->x_world = imu_sensor.info->base_info.accy;
	info->y_world = imu_sensor.info->base_info.accx;
	info->z_world = imu_sensor.info->base_info.accz;
	
	//求解斜坡角度信息，用于前馈计算
	Chassis_Slope_Update(My_Chassis_Posture, info->yaw, info->pitch, info->roll);
}
float test_yaw;
/**
  * @brief  求解底盘所在斜坡的pitch角和yaw角
  * @param  Chassis_Posture_t* My_Chassis_Posture
  * @retval None
  */
static void Chassis_Slope_Update(Chassis_Posture_t* My_Chassis_Posture, float yaw, float pitch, float roll)
{
	//求底盘在世界系下的z轴坐标
	float z_world[3];
	z_world[0] = arm_sin_f32(yaw)*arm_sin_f32(roll) + arm_cos_f32(yaw)*arm_sin_f32(pitch)*arm_cos_f32(roll);
	z_world[1] = -arm_cos_f32(yaw)*arm_sin_f32(roll) + arm_sin_f32(yaw)*arm_sin_f32(pitch)*arm_cos_f32(roll);
	z_world[2] = arm_cos_f32(pitch)*arm_cos_f32(roll);
	
	//求斜坡的pitch角
	float temp;
	arm_sqrt_f32(z_world[0]*z_world[0]+z_world[1]*z_world[1], &temp);
	arm_atan2_f32(temp, z_world[2], &My_Chassis_Posture->info->slope_pitch);
	
	//求斜坡的yaw角
	float sin_yaw, cos_yaw;
	if(My_Chassis_Posture->info->slope_pitch > 0.08f)
	{
		sin_yaw = z_world[1];
		cos_yaw = z_world[0];
		arm_atan2_f32(sin_yaw, cos_yaw, &My_Chassis_Posture->info->slope_yaw);
		test_yaw = yaw - My_Chassis_Posture->info->slope_yaw;
	}
	else
	{
		My_Chassis_Posture->info->slope_yaw = yaw;
	}
}
