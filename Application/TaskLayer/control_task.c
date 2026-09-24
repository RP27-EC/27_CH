/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   ��������,����������̨�ͷ������
  ******************************************************************************
  */
#include "control_task.h"
#include "gimbal_motor.h"
#include "Judge.h"

void StartCtrlTask(void const * argument)
{

	for(;;)
	{
		Yaw_Motor.single_sleep(&Yaw_Motor);
		Yaw_Motor.tx_info->torque = 0.f;
		Yaw_Motor.single_set_torque(&Yaw_Motor);
//	Back_Group.group_set_torque(&Back_Group);
//	Front_Group.group_set_torque(&Front_Group);
		osDelay(1);
	}
}
