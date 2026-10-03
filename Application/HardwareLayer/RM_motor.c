/**
  ******************************************************************************
  * @file    RM_motor.c
  * @brief   RoboMaster标准电机驱动 - 支持3508/2006/6020等型号
  *          本文件实现了RoboMaster赛事标准电机的完整驱动，支持多种电机型号：
  *          1. M3508(带减速箱): 大扭矩直流无刷电机，减速比约3591/187
  *          2. M2006(带减速箱): 小扭矩直流无刷电机，减速比约36
  *          3. M3508(单电机):   无减速箱的3508电机
  *          4. GM6020:          电压控制型直流无刷电机（通常用于云台）
  *
  *          主要功能：
  *          1. 单电机控制：
  *             - 转矩控制（设置输出电流/转矩）
  *             - 速度PID控制
  *             - 角度PID串级控制（外环角度 + 内环速度）
  *             - 卸力（停止输出）
  *          2. 电机组控制：同一CAN总线最多4个电机同时控制
  *          3. 数据接收与解析：编码器角度、转速、转矩电流、温度
  *          4. 编码器数据处理：过零点检测、累计角度计算、弧度转换
  *          5. 心跳检测：离线计数与设备在线状态管理
  *
  *          CAN协议说明：
  *          - 控制帧ID: 0x200(电机1~4) / 0x1FF(电机5~8)
  *          - 反馈帧ID: 0x201~0x208(各电机独立)
  *          - 控制数据为2字节电流值，范围[-16384, 16384]对应[-20A, 20A]
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "rm_motor.h"
#include "pid.h"
#include "arm_math.h"
#include "rp_math.h"
static uint16_t CAN_01_GetMotorAngle(uint8_t *rxData);
static int16_t CAN_23_GetMotorSpeed(uint8_t *rxData);
static int16_t CAN_45_GetMotorCurrent(uint8_t *rxData);
static int16_t CAN_23_GetMotorTorque(uint8_t *rxData);
static int16_t CAN_45_GetMotorTorque(uint8_t *rxData);
static uint8_t CAN_6_GetMotorTemperature(uint8_t *rxData);
static void Torque_to_Raw_Current(Motor_RM_t *motor);
static void Angle_Sum_Cal(Motor_RM_t *motor);
static void Encoder_to_Motor_Angle(Motor_RM_t *motor);
static float RPM_to_Rads(Motor_RM_t *motor);
static void Raw_Current_to_Torque(Motor_RM_t* motor);
static void Encoder_Sum_Cal(Motor_RM_t *motor);
/* ========================================== 单电机控制函数 ========================================== */
/**
  * @brief          单电机控制输出转矩,含有CAN发送操作
  * @param[in]      Motor_RM_t *motor     电机本体
  * @retval         none
  */
static void Motor_Set_Torque(Motor_RM_t *motor)
{
	uint8_t Id = motor->born_info->rxId*2;
	Torque_to_Raw_Current(motor);
	motor->tx_info->tx_buff[Id] = (uint8_t)(motor->tx_info->torque_current_raw >> 8);
	motor->tx_info->tx_buff[Id+1] = (uint8_t)(motor->tx_info->torque_current_raw);
	CAN_SendData(motor->born_info->hcan, motor->born_info->stdId, motor->tx_info->tx_buff);
}

/**
  * @brief          单电机控制,含有CAN发送操作
  * @param[in]      Motor_RM_t *motor     电机本体
  * @retval         none
  */
static void Motor_Ctrl(Motor_RM_t *motor)
{
	uint8_t Id = motor->born_info->rxId*2;
	motor->tx_info->tx_buff[Id] = (uint8_t)(motor->tx_info->torque_current_raw >> 8);
	motor->tx_info->tx_buff[Id+1] = (uint8_t)(motor->tx_info->torque_current_raw);
	CAN_SendData(motor->born_info->hcan, motor->born_info->stdId, motor->tx_info->tx_buff);
}

/**
  * @brief          单电机卸力
  * @param[in]      Motor_RM_t *motor     电机本体
  * @retval         none
  */
static void Single_Motor_Sleep(Motor_RM_t *motor)
{
	motor->tx_info->torque = 0;
	motor->single_set_torque(motor);
}

/**
  * @brief          单电机控制速度(不含发送函数，需要再控制扭矩才可以控制)
  * @param[in]      Motor_RM_t *motor     
  * @retval         none
  */
static void Motor_Set_Speed(Motor_RM_t *motor)
{
	pid_ctrl_t* my_speed_ctrl = motor->ctrl->speed_ctrl;
	my_speed_ctrl->measure = motor->rx_info->encoder_speed;
	my_speed_ctrl->err=my_speed_ctrl->target-my_speed_ctrl->measure;
	single_pid_ctrl(my_speed_ctrl);
	motor->tx_info->torque = my_speed_ctrl->out;
}


/**
  * @brief          单电机控制角度(不含发送函数，需要再控制扭矩才可以控制)
  * @param[in]      Motor_RM_t *motor     
  * @retval         none
  */
static void Motor_Set_Angle(Motor_RM_t *motor)
{
	pid_ctrl_t* my_angle_ctrl = motor->ctrl->angle_ctrl_outer;
	pid_ctrl_t* my_speed_ctrl = motor->ctrl->angle_ctrl_inner;
	/*外环计算*/
	if(motor->ctrl->Angle_Input_Flag == false)
	{
	my_angle_ctrl->measure = motor->rx_info->encoder;
	}
	my_angle_ctrl->err=my_angle_ctrl->target-my_angle_ctrl->measure;
	if(motor->ctrl->Nearest_Return == true)
	{
		if(motor->ctrl->Angle_Input_Flag == false)
		{
			if(my_angle_ctrl->err < 0)
			{
				my_angle_ctrl->err += 8192;
			}
			if(my_angle_ctrl->err > 4096)
			{
				my_angle_ctrl->err -= 8192;
			}
		}
		else
		{
			if(my_angle_ctrl->err < -180.f)
			{
				my_angle_ctrl->err += 360.f;
			}
			if(my_angle_ctrl->err > 180.f)
			{
				my_angle_ctrl->err -= 360.f;
			}
		}
	}
	single_pid_ctrl(my_angle_ctrl);
	
	my_speed_ctrl->target = my_angle_ctrl->out;
	if(motor->ctrl->Speed_Input_Flag == false)
	my_speed_ctrl->measure = motor->rx_info->encoder_speed;
	my_speed_ctrl->err=my_speed_ctrl->target-my_speed_ctrl->measure;
	single_pid_ctrl(my_speed_ctrl);
	motor->tx_info->torque = my_speed_ctrl->out;
}

/**
 * @brief  电机心跳失联检测
 * @param  motor: 电机结构体
 * @retval 无
 */
static void rm_motor_heart_beat(Motor_RM_t *motor)
{
    Motor_RM_State_t *motor_state = motor->state;
    motor_state->offline_cnt++;
    if(motor_state->offline_cnt > motor_state->offline_cnt_max) 
	{
        motor_state->offline_cnt = motor_state->offline_cnt_max;
        motor_state->status = DEV_OFFLINE;
    }
    else 
	{
        if(motor_state->status == DEV_OFFLINE)
            motor_state->status = DEV_ONLINE;
    }
}

/**
  * @brief  解析RM标准电机反馈报文 - 更新角度、速度、转矩电流与温度
  * @param  rm_motor: 电机结构体指针
  * @param  rxBuf:    CAN接收的8字节数据缓冲区
  * @retval 无
  * @note   RM电机反馈报文格式(8字节)：
  *         Byte0-1: 编码器角度值 (0~8191对应0~360度)
  *         Byte2-3: 转子转速 (RPM，有符号)
  *         Byte4-5: 实际转矩电流 (有符号)
  *         Byte6:   电机温度 (摄氏度)
  *         Byte7:   保留
  *
  *         处理流程：
  *         1. 读取编码器角度 -> 累计角度和 -> 转换为弧度制
  *         2. 读取转速 -> 转换为弧度/秒
  *         3. 读取原始电流 -> 转换为实际转矩
  *         4. 读取温度
  *         5. 清零离线计数器
  */
static void rm_motor_update(Motor_RM_t *rm_motor, uint8_t *rxBuf)
{
    Motor_RM_Rx_Info_t *motor_info = rm_motor->rx_info;

    /* 从报文Byte[0:1]解析编码器角度值(0~8191) */
    motor_info->encoder = CAN_01_GetMotorAngle(rxBuf);
    /* 累计编码器角度和（处理过零点） */
    Encoder_Sum_Cal(rm_motor);
    /* 将编码器值转换为弧度制电机角度 */
    Encoder_to_Motor_Angle(rm_motor);

    /* 从报文Byte[2:3]解析转子转速(RPM) */
    motor_info->encoder_speed = CAN_23_GetMotorSpeed(rxBuf);
    /* 将RPM转换为弧度/秒(rad/s) */
    motor_info->speed = RPM_to_Rads(rm_motor);

    /* 从报文Byte[4:5]解析原始转矩电流值 */
    motor_info->torque_current_raw = CAN_45_GetMotorCurrent(rxBuf);
    /* 将原始电流值转换为实际转矩(N*m) */
    Raw_Current_to_Torque(rm_motor);

    /* 从报文Byte[6]解析电机温度 */
    motor_info->temperature = CAN_6_GetMotorTemperature(rxBuf);

    /* 清零离线计数器，表示刚收到有效数据 */
    rm_motor->state->offline_cnt = 0;
}

/**
 * @brief  电机初始化
 * @param  motor: 电机结构体
 * @retval 无
 */
void RM_Motor_Init(Motor_RM_t *motor)
{
	motor->single_set_torque = Motor_Set_Torque;
	motor->single_heart_beat = rm_motor_heart_beat;
	motor->single_sleep = Single_Motor_Sleep;
	motor->single_set_speed = Motor_Set_Speed;
	motor->single_set_angle = Motor_Set_Angle;
	motor->single_ctrl = Motor_Ctrl;
	motor->rx = rm_motor_update;
	motor->state->offline_cnt_max = 100;
}

/* ========================================== 多电机控制函数 ========================================== */
/**
  * @brief          多电机（1~4个）电机控制输出转矩(含转换),4个电机必须为同一类型；含有CAN发送操作
  * @param[in]      Motor_RM_Group_t *group     电机组
  * @retval         none
  */
static void Group_Motor_Set_Torque(Motor_RM_Group_t *group)
{	
		int16_t torque_current = 0;
		uint8_t Id;
		for(uint8_t i = 0; i < 4; i ++)
		{
			if(group->motor[i] != NULL)
			{
				Id = group->motor[i]->born_info->rxId*2;
				Torque_to_Raw_Current(group->motor[i]);
				torque_current = group->motor[i]->tx_info->torque_current_raw;
				group->tx_buff[Id] = (uint8_t)(torque_current >> 8);
				group->tx_buff[Id+1] = (uint8_t)(torque_current);
//				group->motor[i]->tx_info->torque = 0;
			}
		}
		
		CAN_SendData(group->hcan, group->stdId, group->tx_buff);
		
		memset(group->tx_buff, 0, 8);
}

/**
  * @brief          多电机（1~4个）电机控制输出,4个电机必须为同一类型；含有CAN发送操作
  * @param[in]      Motor_RM_Group_t *group     电机组
  * @retval         none
  */
static void Group_Motor_Ctrl(Motor_RM_Group_t *group)
{	
		int16_t torque_current = 0;
		uint8_t Id;
		for(uint8_t i = 0; i < 4; i ++)
		{
			if(group->motor[i] != NULL)
			{
				Id = group->motor[i]->born_info->rxId*2;
				torque_current = group->motor[i]->tx_info->torque_current_raw;
				group->tx_buff[Id] = (uint8_t)(torque_current >> 8);
				group->tx_buff[Id+1] = (uint8_t)(torque_current);
//				group->motor[i]->tx_info->torque = 0;
			}
		}
		
		CAN_SendData(group->hcan, group->stdId, group->tx_buff);
		
		memset(group->tx_buff, 0, 8);
}

/**
  * @brief          电机组卸力
  * @param[in]      Motor_RM_Group_t *group     电机组
  * @retval         none
  */
static void Group_Motor_Sleep(Motor_RM_Group_t *group)
{		
		
	for(uint8_t i = 0; i < 4; i ++)
		{
			if(group->motor[i] != NULL)
			{
				group->motor[i]->tx_info->torque = 0;
			}
		}
		
}


/**
  * @brief          电机组心跳包
  * @param[in]      Motor_RM_Group_t *group     电机组
  * @retval         none
  */
static void Group_Motor_Heartbeat(Motor_RM_Group_t *group)
{
	for(uint8_t i = 0; i < 4; i ++)
	{
		if(group->motor[i] != NULL)
		{
			group->motor[i]->single_heart_beat(group->motor[i]);
		}
	}
}

/**
  * @brief          电机组初始化
  * @param[in]      Motor_Ktech_Group_t *group     电机组
  * @retval         none
  */
void RM_Group_Motor_Init(Motor_RM_Group_t *group)
{
		for(uint8_t i = 0; i < 4; i ++)
		{
			if(group->motor[i] != NULL)
			{
				group->motor[i]->single_init = RM_Motor_Init;
				group->motor[i]->single_init(group->motor[i]);
			}
		}
	
	  group->group_set_torque = Group_Motor_Set_Torque;
		group->group_heartbeat = Group_Motor_Heartbeat;
	  group->group_sleep = Group_Motor_Sleep;
		group->group_ctrl = Group_Motor_Ctrl;
}

/* ========================================== 内部工具函数 ========================================== */
/**
 *	@brief	从CAN报文[0][1]中读取电机的位置反馈
 */
static uint16_t CAN_01_GetMotorAngle(uint8_t *rxData)
{
	uint16_t angle;
	angle = (uint16_t)(rxData[0] << 8| rxData[1]);
	return angle;
}

/**
 *	@brief	从CAN报文[2][3]中读取电机的转子转速反馈
 */
static int16_t CAN_23_GetMotorSpeed(uint8_t *rxData)
{
	int16_t speed;
	speed = (int16_t)(rxData[2] << 8| rxData[3]);
	return speed;
}

/**
 *	@brief	从CAN报文[4][5]中读取电机的实际转矩电流反馈
 */
static int16_t CAN_45_GetMotorCurrent(uint8_t *rxData)
{
	int16_t current;
	current = (int16_t)(rxData[4] << 8 | rxData[5]);
	return current;
}

/**
 *	@brief	从CAN报文[2][3]中读取电机的实际输出转矩
 */
static int16_t CAN_23_GetMotorTorque(uint8_t *rxData)
{
	int16_t torque;
	torque = ((uint16_t)rxData[2] << 8 | rxData[3]);
	return torque;
}

/**
 *	@brief	从CAN报文[4][5]中读取电机的实际输出转矩
 */
static int16_t CAN_45_GetMotorTorque(uint8_t *rxData)
{
	int16_t torque;
	torque = ((uint16_t)rxData[4] << 8 | rxData[5]);
	return torque;
}

/**
 *	@brief	从CAN报文[6]中读取电机的实际温度
 */
static uint8_t CAN_6_GetMotorTemperature(uint8_t *rxData)
{
	uint8_t temp;
	temp = rxData[6];
	return temp;
}

/**
  * @brief  将期望输出转矩转换为CAN发送所需的原始电流值
  * @param  motor: 电机结构体指针
  * @retval 无
  * @note   不同电机型号的转矩-电流转换关系不同：
  *
  *         _3508_Single（无减速箱3508）：
  *           转矩直接作为电流值使用，限幅[-16384, 16384]
  *           注：单3508没有稳定的转矩常数，torque值直接代表电流
  *
  *         _3508_Reduction（带减速箱3508）：
  *           电流 = 转矩 / 转矩常数(0.3)
  *           限幅: [-最大电流*0.9, 最大电流*0.9]（留10%余量）
  *           原始值 = (电流 / 最大电流) * 16384
  *
  *         _2006_Single（带减速箱2006）：
  *           转矩直接作为电流值使用，限幅[-10000, 10000]
  *
  *         _6020_Single（GM6020）：
  *           电流 = 转矩 / 转矩常数(0.741)
  *           限幅: [-最大电流, 最大电流]
  *           原始值 = 电流（直接使用）
  */
static void Torque_to_Raw_Current(Motor_RM_t *motor)
{
		/* 根据电机型号选择不同的转矩-电流转换算法 */
		switch(motor->born_info->type)
		{
			/* M3508无减速箱：转矩直接映射为电流值 */
			case _3508_Single:
			motor->tx_info->torque_current = motor->tx_info->torque;
			motor->tx_info->torque_current = constrain(motor->tx_info->torque_current, -16384, 16384);  /* 最大电流限幅 */
			/* 直接将转矩电流作为原始值 */
			motor->tx_info->torque_current_raw = (int16_t)((motor->tx_info->torque_current));
			break;

			/* M3508带减速箱：需要除以转矩常数，再归一化到[-16384, 16384] */
			case _3508_Reduction:
			motor->tx_info->torque_current = motor->tx_info->torque / _3508_TORQUE_CONSTANT;  /* 转矩 -> 电流(A) */
			motor->tx_info->torque_current = constrain(motor->tx_info->torque_current, -_3508_MAX_CURRENT * 0.9f, _3508_MAX_CURRENT * 0.9f);  /* 限幅90%最大电流 */
			/* 将电流值归一化到[-16384, 16384]范围 */
			motor->tx_info->torque_current_raw = (int16_t)((motor->tx_info->torque_current / _3508_MAX_CURRENT) * 16384.f);
			break;

			/* M2006带减速箱：转矩直接映射为电流值 */
			case _2006_Single:
			motor->tx_info->torque_current = motor->tx_info->torque;
			motor->tx_info->torque_current = constrain(motor->tx_info->torque_current, -10000, 10000);  /* 限幅 */
			/* 直接将转矩电流作为原始值 */
			motor->tx_info->torque_current_raw = (int16_t)((motor->tx_info->torque_current));
			break;

			/* GM6020：电压控制型电机，需要除以转矩常数 */
			case _6020_Single:
			motor->tx_info->torque_current = motor->tx_info->torque / _6020_TORQUE_CONSTANT;  /* 转矩 -> 电流(A) */
			motor->tx_info->torque_current = constrain(motor->tx_info->torque_current, -_6020_MAX_CURRENT, _6020_MAX_CURRENT);  /* 限幅 */
			/* 直接将电流值作为原始值 */
			motor->tx_info->torque_current_raw = (int16_t)((motor->tx_info->torque_current));
			break;
		}
}


/**
  * @brief  编码器累计角度和计算 - 处理编码器过零点问题
  * @param  motor: 电机结构体指针
  * @retval 无
  * @note   RM电机编码器范围为0~8191（对应电机机械角度0~360度）。
  *         当编码器值从0跳到8191（正向过零）或从8191跳到0（反向过零）时，
  *         需要正确补偿以保证累计角度的连续性。
  *
  *         过零点检测逻辑：
  *         - 编码器差值绝对值 > 4095（半圈）则认为发生了过零点
  *         - 正向过零(0->8191): 补偿 = -8191 + 当前差值
  *         - 反向过零(8191->0): 补偿 = 8191 + 当前差值
  *         - 首次启动时（上次角度和编码器和均为0）不计算误差
  */
static void Encoder_Sum_Cal(Motor_RM_t *motor)
{
	int16_t err;  /* 编码器值变化量 */
	Motor_RM_Rx_Info_t *motor_info = motor->rx_info;

	/* 首次启动检测：如果上一次角度和编码器和都为0，说明是首次运行 */
	if(motor_info->motor_angle_last == 0 && motor_info->encoder_sum == 0)
	{
		err = 0;  /* 首次启动，不计算误差 */
	}
	else
	{
		/* 计算本次编码器值与上次的差值 */
		err = motor_info->encoder - motor_info->encoder_last;
	}

	/* 过零点检测：差值绝对值超过半圈(4095)说明发生了跳变 */
	if(abs(err) > 4095)
	{
		/* 正向过零：编码器从0附近跳到8191附近，差值为正的大数 */
		if(err >= 0)
			motor_info->encoder_sum += -8191 + err;  /* 补偿-8191使累计连续 */
		/* 反向过零：编码器从8191附近跳到0附近，差值为负的大数 */
		else
			motor_info->encoder_sum += 8191 + err;   /* 补偿+8191使累计连续 */
	}
	/* 未过零点：直接累加差值 */
	else
	{
		motor_info->encoder_sum += err;
	}

	/* 保存本次编码器值，用于下次计算差值 */
	motor_info->encoder_last = motor_info->encoder;
}


/**
  * @brief  将编码器值转换为弧度制电机角度，并计算累计角度和
  * @param  motor: 电机结构体指针
  * @retval 无
  * @note   编码器值0~8191对应电机转子机械角度0~360度（2*PI弧度）。
  *         对于带减速箱的电机，还需要除以减速比得到输出轴角度。
  *
  *         转换公式：
  *         - 无减速箱: motor_angle = (encoder / 8191) * 2 * PI
  *         - 3508减速箱: motor_angle = (encoder / 8191) * 2 * PI / 减速比
  *         - 2006减速箱: motor_angle = (encoder / 8191) * 2 * PI / 减速比
  *
  *         转换后调用 Angle_Sum_Cal() 计算累计旋转角度（处理过零点）
  */
static void Encoder_to_Motor_Angle(Motor_RM_t *motor)
{
	/* 根据电机型号选择不同的角度转换公式 */
	if(motor->born_info->type == _3508_Reduction)
		/* 3508带减速箱：编码器角度 / 8191 * 2PI / 减速比 */
		motor->rx_info->motor_angle = ((float)motor->rx_info->encoder / 8191.f) * (float)PI * 2.f / _3508_REDUCT_RATIO;
	else if(motor->born_info->type == _2006_Single)
		/* 2006带减速箱：编码器角度 / 8191 * 2PI / 减速比 */
		motor->rx_info->motor_angle = ((float)motor->rx_info->encoder / 8191.f) * (float)PI * 2.f / _2006_REDUCT_RATIO;
	else
		/* 无减速箱电机（3508单电机、6020）：编码器角度 / 8191 * 2PI */
		motor->rx_info->motor_angle = ((float)motor->rx_info->encoder / 8191.f) * (float)PI * 2.f;

	/* 计算累计旋转角度（处理过零点问题） */
	Angle_Sum_Cal(motor);
}

/**
  * @brief  计算RM电机累计旋转角度 - 处理过零点问题
  * @param  motor: 电机结构体指针
  * @retval 无
  * @note   与DM电机的Angle_Sum_Cal类似，但需要考虑减速箱的影响。
  *         带减速箱的电机，输出轴旋转一圈对应编码器旋转多圈，
  *         因此过零点检测的阈值需要除以减速比。
  *
  *         过零点检测逻辑：
  *         - 无减速箱电机: 角度差 > PI 则认为过零点
  *         - 3508减速箱电机: 角度差 > PI/减速比 则认为过零点
  *         - order_correction: 用于修正电机正方向定义（1=默认，-1=反转）
  */
static void Angle_Sum_Cal(Motor_RM_t *motor)
{
	float err = 0.f;  /* 本次角度变化量 */

	/* 获取方向修正系数 */
	float order_correction = 0.f;
	if(motor->born_info->order_correction == 1 || motor->born_info->order_correction == -1)
	{
		order_correction = (float)motor->born_info->order_correction;
	}
	else
	{
		order_correction = 1.f;
	}

	/* 首次启动时不计算误差 */
	if(!motor->rx_info->motor_angle_last && !motor->rx_info->motor_angle_sum)
	{
		err = 0.f;
	}
	else
	{
		/* 计算本次角度与上次角度的差值 */
		err = motor->rx_info->motor_angle - motor->rx_info->motor_angle_last;
	}

	/* 过零点检测：需要根据是否带减速箱使用不同的阈值 */
	if((abs(err) > ((float)PI)) ||
	   (abs(err) > ((float)PI / _3508_REDUCT_RATIO) && motor->born_info->type == _3508_Reduction))
	{
		if(err > 0.f)
		{
			/* 正向过零点 */
			if(motor->born_info->type == _3508_Reduction)
				/* 3508减速箱：补偿 -2*PI/减速比 */
				motor->rx_info->motor_angle_sum += (-(float)PI * 2.f / _3508_REDUCT_RATIO + err) * order_correction;
			else
				/* 无减速箱：补偿 -2*PI */
				motor->rx_info->motor_angle_sum += (-(float)PI * 2.f + err) * order_correction;
		}
		else
		{
			/* 反向过零点 */
			if(motor->born_info->type == _3508_Reduction)
				/* 3508减速箱：补偿 +2*PI/减速比 */
				motor->rx_info->motor_angle_sum += ((float)PI * 2.f / _3508_REDUCT_RATIO + err) * order_correction;
			else
				/* 无减速箱：补偿 +2*PI */
				motor->rx_info->motor_angle_sum += ((float)PI * 2.f + err) * order_correction;
		}
	}
	else
	{
		/* 未过零点，直接累加角度变化量 */
		motor->rx_info->motor_angle_sum += err * order_correction;
	}

	/* 更新上一次角度值 */
	motor->rx_info->motor_angle_last = motor->rx_info->motor_angle;
}

/**
  * @brief  将电机转速从RPM(转/分钟)转换为弧度/秒(rad/s)
  * @param  motor: 电机结构体指针
  * @retval float: 转换后的角速度(rad/s)
  * @note   转换公式：rad/s = RPM / 60 * 2 * PI
  *         对于带减速箱的电机，还需要除以减速比得到输出轴角速度
  *
  *         三种转换：
  *         - 3508减速箱: rad/s = RPM / 60 * 2 * PI / 减速比
  *         - 2006减速箱: rad/s = RPM / 60 * 2 * PI / 减速比
  *         - 无减速箱:   rad/s = RPM / 60 * 2 * PI
  */
static float RPM_to_Rads(Motor_RM_t *motor)
{
	float ret;
	if(motor->born_info->type == _3508_Reduction)
		/* 3508减速箱: RPM -> rad/s，除以减速比 */
		ret = motor->rx_info->encoder_speed / 60.f * 2 * PI / _3508_REDUCT_RATIO;
	else if(motor->born_info->type == _2006_Single)
		/* 2006减速箱: RPM -> rad/s，除以减速比 */
		ret = motor->rx_info->encoder_speed / 60.f * 2 * PI / _2006_REDUCT_RATIO;
	else
		/* 无减速箱电机: RPM -> rad/s */
		ret = motor->rx_info->encoder_speed / 60.f * 2 * PI;
	return ret;
}


/**
  * @brief  将电机接收到的原始电流数据转换为实际输出转矩
  * @param  motor: 电机结构体指针
  * @retval 无
  * @note   转换过程分两步：
  *         1. 原始电流值 -> 实际电流(A):
  *            电流 = (原始值 / 16384) * 20A
  *            原始值范围[-16384, 16384]对应电流范围[-20A, 20A]
  *
  *         2. 实际电流(A) -> 输出转矩(N*m):
  *            转矩 = 电流 * 转矩常数(0.3 N*m/A for 3508)
  *
  *         注: 当前仅针对3508减速箱电机实现了完整的转换
  */
static void Raw_Current_to_Torque(Motor_RM_t* motor)
{
		/* 步骤1: 将原始电流值归一化并转换为实际电流(A) */
		/* 原始值范围[-16384, 16384] -> 电流范围[-20A, 20A] */
		motor->rx_info->torque_current = (motor->rx_info->torque_current_raw / 16384.f) * 20.f;

		/* 步骤2: 将电流转换为输出转矩(N*m) */
		/* 转矩 = 电流 * 转矩常数 */
		motor->rx_info->torque = motor->rx_info->torque_current * _3508_TORQUE_CONSTANT;
}
