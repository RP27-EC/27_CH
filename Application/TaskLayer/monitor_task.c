/**
  ******************************************************************************
  * @file    monitor_task.c
  * @brief   监控任务
  *          1. 各模块心跳失联检测
  *          2. 监控遥控器状态，软件复位
  ******************************************************************************
  */
#include "monitor_task.h"
#include "device.h"
#include "gimbal_motor.h"
void StartMonitorTask(void const * argument)
{
	
	for(;;)
	{
		rc_sensor.heart_beat(&rc_sensor);
		imu_sensor.heart_beat(&imu_sensor.work_state);
		Yaw_Motor.single_heart_beat(&Yaw_Motor);
//		
//		My_Judge_Realtime_Task(&My_Judge);
//		cap.heartbeat(&cap);
		
		osDelay(1);
	}
}

