/**
  ******************************************************************************
  * @file    observe_task.c
  * @brief   状态量观测任务
  *          1. 更新底盘各种状态变量
  ******************************************************************************
  */
#include "observe_task.h"
#include "imu_sensor.h"
#include "chassis_motor.h"
#include "gimbal_motor.h"

void StartUpdataTask(void const * argument)
{
	for(;;)
	{
		imu_sensor.update(&imu_sensor);
		
		 osDelay(1);
	}
}
