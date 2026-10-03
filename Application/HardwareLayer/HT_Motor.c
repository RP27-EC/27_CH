/**
 * @file        HT_Motor.c
 * @author      2025_YZJ
 * @Version     V1.0
 * @date        8-February-2025
 * @brief       海泰(HT)电机驱动包 - 适用型号HT-03
 *              本文件实现了海泰HT-03电机的MIT控制协议驱动。
 *              HT-03通常用作轮毂电机，因此部分功能（如多圈角度累计）有所简化。
 *
 *              主要功能：
 *              1. 单电机控制：卸力、置零编码器、转矩控制、速度控制、角度控制
 *              2. 电机组控制：多电机批量控制（最多4个）
 *              3. 数据接收与解析：解析编码器值、速度、转矩电流
 *              4. 编码器到电机角度转换：处理过零点和累计角度
 *              5. 心跳检测：离线计数与设备在线状态管理
 *
 *              与DM电机的区别：
 *              - HT电机使用不同的参数范围（HT_P_MIN/HT_P_MAX等）
 *              - HT电机的速度乘以力矩常数得到实际转矩
 *              - HT电机使用积分法累加速度得到角度（motor_angle_sum_vi）
 *              - HT电机的编码器到角度转换使用度数单位而非弧度
 *
 * @note        尚未完全完善，由于主要作为轮毂电机使用，部分功能未实现
 */

/* Includes ------------------------------------------------------------------*/
#include "HT_Motor.h"

/* 内部静态变量 --------------------------------------------------------------*/
/**
  * 海泰电机命令报文缓冲区（8字节）
  * 与达秒电机类似的命令帧格式：前7字节为0xFF，最后1字节为命令码
  * 0xFC: 进入电机控制模式
  * 0xFD: 退出电机控制模式
  * 0xFE: 置零编码器位置
  */
static uint8_t Motor_Command[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

/* 内部函数前向声明 */
static void Motor_Send_Data(Motor_HT_t *motor, uint8_t* buf);
static void Motor_Send_Command(Motor_HT_t *motor, Motor_MIT_Command_e Command);
static void HT_Motor_SetControlPara(Motor_HT_t *motor);
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
static float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits);
static void Encoder_to_Motor_Angle(Motor_HT_t *motor);

/**
  * 接收标志位（用于调试或其他用途）
  */
uint8_t flag_rx;

/* ========================================== 单电机控制函数 ========================================== */
/**
  * @brief          单电机卸力
  * @param[in]      Motor_HT_t *motor     电机本体
  * @retval         none
  */
void HT_Single_Motor_Sleep(Motor_HT_t *motor)
{
	if(motor != NULL)
	{
//		if(motor->state->mode == Motor_UnControl)
//		{
//			Motor_Send_Command(motor, Enter_Motor_Mode);//开启电机控制
//			motor->state->mode = Motor_Control;
//		}
		
		Motor_HT_Tx_Info_t* motor_tx_info = motor->tx_info;
		motor_tx_info->torque = 0;
		motor_tx_info->Kd = 0;
		motor_tx_info->Kp = 0;
	}
}

/**
  * @brief          单电机置零编码器
  * @param[in]      Motor_HT_t *motor     电机本体
  * @retval         none
  */
void HT_Single_Motor_ZeroPosSensor(Motor_HT_t *motor)
{
	if(motor != NULL)
	{
		motor->single_sleep(motor);//先对电机卸力
		
		Motor_Send_Command(motor, Zero_Position_Sensor);
	}
}

/**
  * @brief          单电机控制输出转矩,含有CAN发送操作
  * @param[in]      Motor_HT_t *motor     电机本体
  * @retval         none
  */
void HT_Single_Motor_Set_Torque(Motor_HT_t *motor)
{
	  if(motor != NULL)
		{
			if(motor->state->mode == Motor_UnControl)
			{
				Motor_Send_Command(motor, Enter_Motor_Mode);//开启电机控制
				motor->state->mode = Motor_Control;
			}
			else
			{
				Motor_HT_Tx_Info_t* motor_tx_info = motor->tx_info;
				motor_tx_info->target_angle = 0;
				motor_tx_info->target_speed = 0;
				motor_tx_info->Kp = 0;
				motor_tx_info->Kd = 0;
				HT_Motor_SetControlPara(motor);
				Motor_Send_Data(motor, motor_tx_info->single_tx_buff);
			}
		}
}

/**
  * @brief          单电机控制速度,需要自行设置kd\target_speed\torque,含有CAN发送操作
  * @param[in]      Motor_HT_t *motor     电机本体
  * @retval         none
  */
void HT_Single_Motor_Set_Speed(Motor_HT_t *motor)
{
	  if(motor != NULL)
		{
			if(motor->state->mode == Motor_UnControl)
			{
				Motor_Send_Command(motor, Enter_Motor_Mode);//开启电机控制
				motor->state->mode = Motor_Control;
			}
			
			Motor_HT_Tx_Info_t* motor_tx_info = motor->tx_info;
			motor_tx_info->target_angle = 0;
			motor_tx_info->Kp = 0;
			HT_Motor_SetControlPara(motor);
		
			Motor_Send_Data(motor, motor_tx_info->single_tx_buff);
		}
}

/**
  * @brief          单电机控制角度,需要自行设置kp\target_angle\kd\target_speed\torque,含有CAN发送操作
  * @param[in]      Motor_HT_t *motor     电机本体
  * @retval         none
  */
void HT_Single_Motor_Set_Angle(Motor_HT_t *motor)
{
	  if(motor != NULL)
		{
			if(motor->state->mode == Motor_UnControl)
			{
				Motor_Send_Command(motor, Enter_Motor_Mode);//开启电机控制
				motor->state->mode = Motor_Control;
			}
			
			Motor_HT_Tx_Info_t* motor_tx_info = motor->tx_info;
			HT_Motor_SetControlPara(motor);
			Motor_Send_Data(motor, motor_tx_info->single_tx_buff);
		}
}

/**
  * @brief          海泰电机CAN中断接收数据处理 - 解析反馈报文
  * @param[in]      Motor_HT_t *motor:     电机本体结构体指针
  * @param[in]      uint8_t *rxBuf:        CAN接收的8字节数据缓冲区
  * @retval         无
  * @note   海泰电机反馈报文格式与达秒电机类似(8字节)：
  *         Byte0: [7:4]=错误码, [3:0]=编码器值高4位
  *         Byte1: [7:0]=编码器值高8位
  *         Byte2: [7:0]=编码器值低8位 (编码器共16位，范围[HT_P_MIN, HT_P_MAX])
  *         Byte3: [7:4]=速度高4位, [3:0]=编码器值低4位
  *         Byte4: [7:4]=速度低8位高4位, [3:0]=转矩电流高4位
  *         Byte5: [7:0]=转矩电流低8位 (速度和转矩电流各12位)
  *
  *         特殊处理：
  *         - 速度值乘以order_correction以修正正方向
  *         - 转矩电流乘以HT_TORQUE_CONSTANT得到实际输出转矩
  *         - 使用积分法累加速度得到电机角度和(motor_angle_sum_vi)
  */
void static Motor_ReceiveData(Motor_HT_t *motor, uint8_t *rxBuf)
{
	Motor_HT_Rx_Info_t* motor_rx_info = motor->rx_info;

	/* 解析编码器值: Byte[1:2]组合为16位值 */
	motor_rx_info->encoder = uint_to_float((uint16_t)((rxBuf[1] << 8) | rxBuf[2]), HT_P_MIN, HT_P_MAX, 16);

	/* 解析速度: Byte[3]高4位 + Byte[4]高4位 = 12位速度值，乘以方向修正系数 */
	motor_rx_info->speed = uint_to_float((uint16_t)((rxBuf[3] << 4) | (rxBuf[4] >> 4)), HT_V_MIN, HT_V_MAX, 12) * motor->born_info->order_correction;

	/* 解析转矩电流: Byte[4]低4位 + Byte[5] = 12位转矩电流值 */
	motor_rx_info->torque_current = uint_to_float((uint16_t)(((rxBuf[4] & 0x0F) << 8) | rxBuf[5]), HT_C_MIN, HT_C_MAX, 12);

	/* 将转矩电流转换为实际输出转矩: 转矩 = 转矩电流 * 力矩常数 */
	motor_rx_info->torque = motor_rx_info->torque_current * HT_TORQUE_CONSTANT;

	/* 将编码器值转换为电机角度（度数），处理过零点 */
	Encoder_to_Motor_Angle(motor);

	/* 积分法累加速度得到角度和（速度 * 时间步长） */
	motor_rx_info->motor_angle_sum_vi += motor_rx_info->speed * TIME_STEP;

	/* 清零离线计数器 */
	motor->state->offline_cnt = 0;
}

/**
  * @brief          单电机心跳包
  * @param[in]      Motor_HT_t *motor    电机本体
  * @retval         none
  */
void HT_Motor_Hearbeat(Motor_HT_t *motor)
{
	motor->state->offline_cnt++;
	
	if(motor->state->offline_cnt > motor->state->offline_cnt_max) 
	{
//		motor->state->offline_cnt = motor->state->offline_cnt_max;
		motor->state->status = DEV_OFFLINE;
		motor->state->mode = Motor_UnControl;
	}
	else 
	{
		if(motor->state->status == DEV_OFFLINE)
			motor->state->status = DEV_ONLINE;
	}
}

/**
  * @brief          单电机初始化
  * @param[in]      Motor_HT_t *motor     电机本体
  * @retval         none
  */
void HT_Single_Motor_Init(Motor_HT_t *motor)
{
	motor->state->mode = Motor_UnControl;
	motor->single_sleep = HT_Single_Motor_Sleep;
	motor->zero_position = HT_Single_Motor_ZeroPosSensor;
	motor->single_set_torque = HT_Single_Motor_Set_Torque;
	motor->single_set_speed  = HT_Single_Motor_Set_Speed;
	motor->single_set_angle  = HT_Single_Motor_Set_Angle;
	motor->rx = Motor_ReceiveData;
	motor->single_heart_beat = HT_Motor_Hearbeat;
	/*开启电机控制*/
	motor->single_sleep(motor);
	motor->state->offline_cnt_max = 500;
	motor->rx_info->motor_angle_sum = 0;
}

/* ========================================== 多电机控制函数 ========================================== */

/**
  * @brief          多电机（1~4个）电机控制输出转矩,4个电机必须为同一类型；含有CAN发送操作
  * @param[in]      Motor_HT_Group_t *group     电机组
  * @retval         none
  */
static void Group_Motor_Set_Torque(Motor_HT_Group_t *group)
{	
	
		if(group->motor[0] != NULL)
		{
			group->motor[0]->single_set_angle(group->motor[0]);
		}
		if(group->motor[1] != NULL)
		{
			group->motor[1]->single_set_angle(group->motor[1]);
		}
		if(group->motor[2] != NULL)
		{
			group->motor[2]->single_set_angle(group->motor[2]);
		}
		if(group->motor[3] != NULL)
		{
			group->motor[3]->single_set_angle(group->motor[3]);
		}
}

/**
  * @brief          多电机（1~4个）电机控制卸力；不含有CAN发送操作
  * @param[in]      Motor_HT_Group_t *group     电机组
  * @retval         none
  */
static void Group_Motor_Sleep(Motor_HT_Group_t *group)
{	
		if(group->motor[0] != NULL)
		{
			group->motor[0]->single_sleep(group->motor[0]);
		}
		if(group->motor[1] != NULL)
		{
			group->motor[1]->single_sleep(group->motor[1]);
		}
		if(group->motor[2] != NULL)
		{
			group->motor[2]->single_sleep(group->motor[2]);
		}
		if(group->motor[3] != NULL)
		{
			group->motor[3]->single_sleep(group->motor[3]);
		}
}

/**
  * @brief          电机组心跳包
  * @param[in]      Motor_HT_Group_t *group     电机组
  * @retval         none
  */
static void Group_Motor_Heartbeat(Motor_HT_Group_t *group)
{
	if(group->motor[0] != NULL)
	{
		group->motor[0]->single_heart_beat(group->motor[0]);
	}
	  
	if(group->motor[1] != NULL)
	{
		group->motor[1]->single_heart_beat(group->motor[1]);
	}

  if(group->motor[2] != NULL)
	{
		group->motor[2]->single_heart_beat(group->motor[2]);
	}
	  
	if(group->motor[3] != NULL)
	{
		group->motor[3]->single_heart_beat(group->motor[3]);
	}
}


/**
  * @brief          电机组初始化
  * @param[in]      Motor_HT_Group_t *group     电机组
  * @retval         none
  */
void HT_Group_Motor_Init(Motor_HT_Group_t *group)
{
	  if(group->motor[0] != NULL)
		{
			group->motor[0]->single_init = HT_Single_Motor_Init;
	    group->motor[0]->single_init(group->motor[0]);
		}
	  
		if(group->motor[1] != NULL)
		{
			group->motor[1]->single_init = HT_Single_Motor_Init;
	    group->motor[1]->single_init(group->motor[1]);
		}

    if(group->motor[2] != NULL)
		{
			group->motor[2]->single_init = HT_Single_Motor_Init;
	    group->motor[2]->single_init(group->motor[2]);
		}
	  
		if(group->motor[3] != NULL)
		{
			group->motor[3]->single_init = HT_Single_Motor_Init;
	    group->motor[3]->single_init(group->motor[3]);
		}
	  
	
	  group->group_set_torque = Group_Motor_Set_Torque;
		group->group_heartbeat = Group_Motor_Heartbeat;
	  group->group_sleep = Group_Motor_Sleep;
}

/* ========================================== 内部工具函数 ========================================== */
/**
  * @brief          整合并发送电机命令报文
  * @param          Motor_HT_t *motor
  * @param[in]      Motor_HT_Command_e Command
  * @retval         none
  */
static void Motor_Send_Command(Motor_HT_t *motor, Motor_MIT_Command_e Command)
{
	switch(Command)
	{
		case Enter_Motor_Mode:
		Motor_Command[7] = 0xFC;
		break;
		case Exit_Motor_Mode:
		Motor_Command[7] = 0xFD;
		break;
		case Zero_Position_Sensor:
		Motor_Command[7] = 0xFE;
		break;
		default:
		break;
	}
	Motor_Send_Data(motor, Motor_Command);
}

/**
  * @brief          根据结构体信息发送报文
  * @param          Motor_HT_t *motor
  * @param          uint8_t* buf 要发送的报文信息
  * @retval         none
  */
static void Motor_Send_Data(Motor_HT_t *motor, uint8_t* buf)
{
	Motor_HT_Born_Info_t* motor_born_info = motor->born_info;
	
	CAN_SendData(motor_born_info->hcan, motor_born_info->stdId, buf);
}

/**
  * @brief          根据发送的报文信息设置报文
  * @param          Motor_HT_t *motor
  * @retval         none
  */
static void HT_Motor_SetControlPara(Motor_HT_t *motor)
{
	Motor_HT_Tx_Info_t* motor_tx_info = motor->tx_info;
	uint16_t p, v, kp, kd, t;
  uint8_t* buf = motor_tx_info->single_tx_buff;
	
	/* 限制输入的参数在定义的范围内 */
	motor_tx_info->target_angle = constrain(motor_tx_info->target_angle, HT_P_MIN, HT_P_MAX);
	motor_tx_info->target_speed = constrain(motor_tx_info->target_speed, HT_V_MIN, HT_V_MAX);
	motor_tx_info->Kp = constrain(motor_tx_info->Kp, HT_KP_MIN, HT_KP_MAX);
	motor_tx_info->Kd = constrain(motor_tx_info->Kd, HT_KD_MIN, HT_KD_MAX);
	motor_tx_info->torque = constrain(motor_tx_info->torque, -8, 8);
	
	/* 根据协议，对float参数进行转换 */
	p = float_to_uint(motor_tx_info->target_angle,      HT_P_MIN,  HT_P_MAX,  16);            
	v = float_to_uint(motor_tx_info->target_speed,      HT_V_MIN,  HT_V_MAX,  12);
	kp = float_to_uint(motor_tx_info->Kp,    HT_KP_MIN, HT_KP_MAX, 12);
	kd = float_to_uint(motor_tx_info->Kd,    HT_KD_MIN, HT_KD_MAX, 12);
	t = float_to_uint(motor_tx_info->torque,      HT_T_MIN,  HT_T_MAX,  12);
	
	/* 根据传输协议，把数据转换为CAN命令数据字段 */
	buf[0] = p>>8;
	buf[1] = p&0xFF;
	buf[2] = v>>4;
	buf[3] = ((v&0xF)<<4)|(kp>>8);
	buf[4] = kp&0xFF;
	buf[5] = kd>>4;
	buf[6] = ((kd&0xF)<<4)|(t>>8);
	buf[7] = t&0xff;
}

/**
  * @brief  将float转为uint，并对正负做处理,与通信协议保持一致
  * @param
  * @retval 
  */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    
    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
  * @brief  将uint转为float，并对正负做处理
  * @param
  * @retval 
  */
static float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}


/**
  * @brief          将编码器值转换为电机角度(度数)并累计角度和
  * @param[in]      Motor_HT_t *motor: 电机本体结构体指针
  * @retval         无
  * @note   编码器值范围为[HT_P_MIN, HT_P_MAX]，对应电机旋转一圈的角度。
  *         本函数处理以下逻辑：
  *         1. 计算本次编码器值与上次的差值（考虑过零点修正）
  *         2. 过零点处理：差值超过180度时进行补偿
  *         3. 累加编码器角度差到总角度和
  *         4. 将总角度和转换为[0, 2*PI]范围内的角度
  *         5. 更新上一次编码器值
  */
static void Encoder_to_Motor_Angle(Motor_HT_t *motor)
{
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

	/* 编码器过零点检测与差值计算 */
	/* 首次启动（上次编码器值和角度和均为0）时不计算误差 */
	if(!motor->rx_info->encoder_last && !motor->rx_info->motor_angle_sum)
	{
		motor->rx_info->encoder_err = 0.f;
	}
	else
	{
		/* 计算本次与上次编码器值的差值 */
		motor->rx_info->encoder_err = -motor->rx_info->encoder_last + motor->rx_info->encoder;
	}

	/* 过零点修正：编码器值从最大跳到最小（正向过零） */
	if(motor->rx_info->encoder_err > 180.f)
	{
		motor->rx_info->encoder_err -= 2 * HT_P_MAX;  /* 补偿-360度 */
	}
	/* 过零点修正：编码器值从最小跳到最大（反向过零） */
	else if(motor->rx_info->encoder_err < -180.f)
	{
		motor->rx_info->encoder_err += 2 * HT_P_MAX;  /* 补偿+360度 */
	}

	/* 累加角度差到总角度和 */
	motor->rx_info->motor_angle_sum += motor->rx_info->encoder_err * order_correction;

	/* 将总角度和映射到[0, 2*PI]范围内 */
	motor->rx_info->motor_angle = motor->rx_info->motor_angle_sum - (int16_t)(motor->rx_info->motor_angle_sum / (2.f * PI)) * 2.f * PI;

	/* 保存本次编码器值，用于下次计算差值 */
	motor->rx_info->encoder_last = motor->rx_info->encoder;
}

/*示例代码*/
/*-------------电机变量创建-------------*/
/*

Motor_HT_Born_Info_t L_Wheel_Born_Info = 
{	
	.stdId = 0x009,//电机控制报文ID
	
	.hcan = &hcan2,//使用的Can总线
	
	.order_correction = 0,//电机总角度的正方向为顺时针
	
};

Motor_HT_Rx_Info_t L_Wheel_Rx_Info_t;

Motor_HT_Tx_Info_t L_Wheel_Tx_Info_t;

Motor_HT_State_t L_Wheel_State_t;

Motor_HT_t L_Wheel = 
{
	.born_info = &L_Wheel_Born_Info,
	
	.rx_info = &L_Wheel_Rx_Info_t,
	
	.tx_info = &L_Wheel_Tx_Info_t,
	
	.state = &L_Wheel_State_t,
	
	.single_init = &HT_Single_Motor_Init,
*/

/*-------------初始化-------------*/
/*
L_Wheel.single_init(&L_Wheel);
*/

/*-------------接收函数-------------*/
/*
void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case 0x000://接收ID
		L_Wheel.rx(&L_Wheel, rxBuf);
		break;
		default:
			break;
	}
}
*/

/*-------------任务执行-------------*/
/*
  * @file    monitor_task.c
  * @brief   监控任务
  *          1. 各模块心跳失联检测
  *          2. 监控遥控器状态，软件复位
void StartMonitorTask(void const * argument)//
{
	
	for(;;)
	{
		L_Wheel.heartbeat(&L_Wheel);
		
		osDelay(1);
	}
}

  * @file    monitor_task.c
  * @brief   电机控制任务
  *          1. 给电机发送控制报文
  *          2. 对状态标志位进行响应
void StartMonitorTask(void const * argument)//
{
	
	for(;;)
	{
		//发送控制报文，控制电机输出扭矩为0.5N*m
		L_Wheel.tx_info->torque = 0.5f;
		L_Wheel.single_set_torque(&L_Wheel);
		
		osDelay(1);
	}
}

*/
