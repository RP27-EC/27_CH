/**
 * @file        DM_Motor.c
 * @author      2025_YZJ
 * @Version     V1.0
 * @date        8-February-2025
 * @brief       达秒(DM) MIT控制模式电机驱动包
 *              本文件实现了达秒(DM)系列电机的完整驱动，基于MIT控制协议。
 *              支持单电机和电机组(最多4个)两种控制模式。
 *
 *              主要功能：
 *              1. 单电机控制：卸力、置零编码器、转矩控制、速度控制、角度控制
 *              2. 电机组控制：多电机批量控制（最多4个电机同一CAN总线）
 *              3. 数据接收与解析：解析电机反馈的角度、速度、转矩电流
 *              4. 心跳检测：离线计数与设备状态管理
 *              5. 错误检测：电机故障码解析（过压、欠压、过流、过温等）
 *
 *              MIT协议控制参数说明：
 *              - 目标角度(target_angle): 通过KP增益产生角度误差力矩
 *              - 目标速度(target_speed): 通过KD增益产生阻尼力矩
 *              - KP: 比例增益，控制角度响应强度
 *              - KD: 微分增益，控制速度阻尼强度
 *              - 扭矩(torque): 前馈力矩
 *              最终输出力矩 = KP * (目标角度 - 当前角度) + KD * (目标速度 - 当前速度) + 扭矩
 *
 *              故障恢复说明：
 *              - 0xFC 进入MIT模式（仅 Motor_Unenable 发送）
 *              - 0xFB 清除电机错误标志（其它故障态发送，如 Commun_Loss）
 *              - 0xFD 退出MIT / 0xFE 置零编码器
 *              状态以电机反馈错误码为准，不在发 0xFC 时本地预置 Motor_Enable
 */

/* Includes ------------------------------------------------------------------*/
#include "DM_Motor.h"
#include "can.h"

static Motor_DM_Born_Info_t Pitch_Motor_Born = {
    .stdId = 0x001U,
    .hcan = &hcan2,
    .order_correction = 1,
};
static Motor_DM_Rx_Info_t Pitch_Motor_Rx;
static Motor_DM_Tx_Info_t Pitch_Motor_Tx;
static Motor_DM_State_t Pitch_Motor_State;
Motor_DM_t Pitch_Motor = {
    .born_info = &Pitch_Motor_Born,
    .rx_info = &Pitch_Motor_Rx,
    .tx_info = &Pitch_Motor_Tx,
    .state = &Pitch_Motor_State,
    .single_init = DM_Single_Motor_Init,
};

/**
  * 达秒电机命令报文缓冲区（8字节）
  * 达秒MIT协议的命令帧格式: 前7字节为0xFF，最后1字节为命令码
  * 0xFC: 进入电机控制模式 (Enter Motor Mode)
  * 0xFD: 退出电机控制模式 (Exit Motor Mode)
  * 0xFE: 置零编码器位置 (Zero Position Sensor)
  */
static uint8_t Motor_Command[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

static void Motor_Send_Data(Motor_DM_t *motor, uint8_t* buf);
static void Motor_SetControlPara(Motor_DM_t *motor);
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
static float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits);
static void Motor_Send_Command(Motor_DM_t *motor, Motor_MIT_Command_e Command);
static void Angle_Sum_Cal(Motor_DM_t *motor);
static void Motor_ERR_Check(Motor_DM_t *motor, uint8_t err_word);
static void Group_Motor_Heartbeat(Motor_DM_Group_t *group);

/* ========================================== 单电机控制函数 ========================================== */
/**
  * @brief          单电机卸力
  * @param[in]      Motor_DM_t *motor     电机本体
  * @retval         none
  */
void DM_Single_Motor_Sleep(Motor_DM_t *motor)
{
	if(motor != NULL)
	{
		motor->tx_info->torque = 0;
		motor->tx_info->Kd = 0;
		motor->tx_info->Kp = 0;
		if (motor->state->motor_state == Motor_Enable) {
			Motor_Send_Command(motor, Exit_Motor_Mode);
		}
		motor->state->motor_state = Motor_Unenable;
	}
}

/**
  * @brief          单电机置零编码器
  * @param[in]      Motor_DM_t *motor     电机本体
  * @retval         none
  */
void DM_Single_Motor_ZeroPosSensor(Motor_DM_t *motor)
{
	if(motor != NULL)
	{
		motor->single_sleep(motor);//先对电机卸力
		
		Motor_Send_Command(motor, Zero_Position_Sensor);
	}
}

/**
  * @brief          单电机控制输出转矩,含有CAN发送操作
  * @param[in]      Motor_DM_t *motor     电机本体
  * @retval         none
  * @note           状态机对齐 RP26：
  *                 Motor_Enable    -> 打包 MIT 力矩帧
  *                 Motor_Unenable  -> 发 0xFC 进 MIT（状态由 RX 错误码确认，不本地预置 Enable）
  *                 其它故障态      -> 发 0xFB Clear_Err，避免对故障电机空打 MIT
  */
void DM_Single_Motor_Set_Torque(Motor_DM_t *motor)
{
	  if(motor != NULL)
		{
			if(motor->state->motor_state != Motor_Enable)
			{
				if(motor->state->motor_state == Motor_Unenable)
					Motor_Send_Command(motor, Enter_Motor_Mode);
				else
					Motor_Send_Command(motor, Clear_Err);
			}
			else
			{
				Motor_DM_Tx_Info_t* motor_tx_info = motor->tx_info;
				motor_tx_info->Kp = 0;
				motor_tx_info->Kd = 0;
				Motor_SetControlPara(motor);
				motor->tx_info->torque = 0;
			}
		}
}

/**
  * @brief          单电机控制速度,需要自行设置kd\target_speed\torque,含有CAN发送操作
  * @param[in]      Motor_DM_t *motor     电机本体
  * @retval         none
  */
void DM_Single_Motor_Set_Speed(Motor_DM_t *motor)
{
	  if(motor != NULL)
		{
			if(motor->state->motor_state == Motor_Unenable)
			{
				motor->state->motor_state = Motor_Enable;
				Motor_Send_Command(motor, Enter_Motor_Mode);
			}
			else
			{
				Motor_DM_Tx_Info_t* motor_tx_info = motor->tx_info;
				motor_tx_info->Kp = 0;
				Motor_SetControlPara(motor);
			}

		}
}

/**
  * @brief          单电机控制角度,需要自行设置kp\target_angle\kd\target_speed\torque,含有CAN发送操作
  * @param[in]      Motor_DM_t *motor     电机本体
  * @retval         none
  */
void DM_Single_Motor_Set_Angle(Motor_DM_t *motor)
{
	  if(motor != NULL)
		{
			if(motor->state->motor_state == Motor_Unenable)
			{
				motor->state->motor_state = Motor_Enable;
				Motor_Send_Command(motor, Enter_Motor_Mode);
			}
			else
			{
				motor->tx_info->target_speed = 0;
				Motor_SetControlPara(motor);
			}
		}
}

/**
  * @brief          电机CAN中断接收数据处理 - 解析达秒电机反馈报文
  * @param[in]      Motor_DM_t *motor:      电机本体结构体指针
  * @param[in]      uint8_t *rxBuf:         CAN接收的8字节数据缓冲区
  * @retval         无
  * @note   达秒电机反馈报文格式(8字节)：
  *         Byte0: [7:4]=错误码, [3:0]=位置高4位
  *         Byte1: [7:0]=位置高8位
  *         Byte2: [7:0]=位置低8位 (位置共16位)
  *         Byte3: [7:4]=速度高4位, [3:0]=位置低4位
  *         Byte4: [7:4]=速度低8位高4位, [3:0]=转矩高4位
  *         Byte5: [7:0]=转矩低8位 (速度和转矩各12位)
  *         Byte6: [7:0]=温度
  *         Byte7: [7:0]=保留
  */
static void Motor_ReceiveData(Motor_DM_t *motor, uint8_t *rxBuf)
{
	Motor_DM_Rx_Info_t* motor_rx_info = motor->rx_info;

	/* 解析错误码: Byte0的高4位为电机状态/错误码 */
	Motor_ERR_Check(motor, rxBuf[0] >> 4);

	/* 解析电机角度: Byte[1:2]组合为16位无符号整数，转换为浮点角度值 */
	motor_rx_info->motor_angle = uint_to_float((uint16_t)((rxBuf[1] << 8) | rxBuf[2]), P_MIN, P_MAX, 16);

	/* 解析电机速度: Byte[3]高4位 + Byte[4]高4位 = 12位速度值 */
	motor_rx_info->speed = uint_to_float((uint16_t)((rxBuf[3] << 4) | (rxBuf[4] >> 4)), V_MIN, V_MAX, 12);

	/* 速度零漂修正: 当速度值极小时视为零速（消除浮点计算误差） */
	if(abs(motor_rx_info->speed) == 0.0109901428f)
	{
		motor_rx_info->speed = 0;
	}

	/* 解析转矩电流: Byte[4]低4位 + Byte[5] = 12位转矩值 */
	motor_rx_info->torque = uint_to_float((uint16_t)(((rxBuf[4] & 0x0F) << 8) | rxBuf[5]), C_MIN, C_MAX, 12);

	/* 计算电机累计旋转角度（处理过零点） */
	Angle_Sum_Cal(motor);

	/* 清零离线计数器，表示刚收到有效数据 */
	motor->state->offline_cnt = 0;
}

/**
  * @brief          单电机心跳包
  * @param[in]      Motor_HT_t *motor    电机本体
  * @retval         none
  */
static void DM_Motor_Hearbeat(Motor_DM_t *motor)
{
	motor->state->offline_cnt++;
	
	if(motor->state->offline_cnt > motor->state->offline_cnt_max) 
	{
		motor->state->offline_cnt = motor->state->offline_cnt_max;
		motor->state->status = DEV_OFFLINE;
		motor->state->motor_state = Motor_Unenable;
	}
	else 
	{
		if(motor->state->status == DEV_OFFLINE)
			motor->state->status = DEV_ONLINE;
	}
}

/**
  * @brief          单电机初始化
  * @param[in]      Motor_DM_t *motor     电机本体
  * @retval         none
  */
void DM_Single_Motor_Init(Motor_DM_t *motor)
{
	motor->single_sleep = DM_Single_Motor_Sleep;
	motor->single_set_torque = DM_Single_Motor_Set_Torque;
	motor->single_set_speed  = DM_Single_Motor_Set_Speed;
	motor->single_set_angle  = DM_Single_Motor_Set_Angle;
	motor->rx = Motor_ReceiveData;
	motor->single_heart_beat = DM_Motor_Hearbeat;
	/*开启电机控制*/
	motor->state->motor_state = Motor_Unenable;
	motor->state->last_motor_state = Motor_Unenable;
	motor->state->offline_cnt_max = 100;
	motor->rx_info->motor_angle_sum = 0;
}
/* ========================================== 电机组控制函数 ========================================== */
/**
  * @brief          多电机（1~4个）电机控制输出转矩
  * @param[in]      Motor_DM_Group_t *group     电机组
  * @retval         none
  */
static void Group_Motor_Set_Torque(Motor_DM_Group_t *group)
{
	static uint8_t rx_num = 0;
	
	if(group->motor[rx_num] != NULL)
	{
		group->motor[rx_num]->single_set_torque(group->motor[rx_num]);
		rx_num ++;
	}
	if(rx_num >= group->motor_num)
	{
		rx_num = 0;
	}

}

/**
  * @brief          电机组卸力
  * @param[in]      Motor_DM_Group_t *group     电机组
  * @retval         none
  */
static void Group_Motor_Sleep(Motor_DM_Group_t *group)
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
  * @param[in]      Motor_DM_Group_t *group     电机组
  * @retval         none
  */
static void Group_Motor_Heartbeat(Motor_DM_Group_t *group)
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
  * @param[in]      Motor_DM_Group_t *group     电机组
  * @retval         none
  */
void Group_Motor_Init(Motor_DM_Group_t *group)
{
	uint8_t num_init = 0;
	if(group->motor[0] != NULL)
	{
		group->motor[0]->single_init = DM_Single_Motor_Init;
		group->motor[0]->single_init(group->motor[0]);
		num_init++;
	}
	
	if(group->motor[1] != NULL)
	{
		group->motor[1]->single_init = DM_Single_Motor_Init;
		group->motor[1]->single_init(group->motor[1]);
		num_init++;
	}

  if(group->motor[2] != NULL)
	{
		group->motor[2]->single_init = DM_Single_Motor_Init;
		group->motor[2]->single_init(group->motor[2]);
		num_init++;
	}
	  
	if(group->motor[3] != NULL)
	{
		group->motor[3]->single_init = DM_Single_Motor_Init;
		group->motor[3]->single_init(group->motor[3]);
		num_init++;
	}
	  
		group->motor_num = num_init;
	  group->group_set_torque = Group_Motor_Set_Torque;
		group->group_heartbeat = Group_Motor_Heartbeat;
	  group->group_sleep = Group_Motor_Sleep;
}

/* ========================================== 内部工具函数 ========================================== */
/**
  * @brief          整合并发送电机命令报文
  * @param          Motor_DM_t *motor
  * @param[in]      Motor_DM_Command_e Command
  * @retval         none
  */
static void Motor_Send_Command(Motor_DM_t *motor, Motor_MIT_Command_e Command)
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
		case Clear_Err:
		Motor_Command[7] = 0xFB;/* 清除电机错误标志 */
		break;
		default:
		break;
	}
	Motor_Send_Data(motor, Motor_Command);
}

/**
  * @brief          根据结构体信息发送报文
  * @param          Motor_DM_t *motor
  * @param          uint8_t* buf 要发送的报文信息
  * @retval         none
  */
static void Motor_Send_Data(Motor_DM_t *motor, uint8_t* buf)
{
	Motor_DM_Born_Info_t* motor_born_info = motor->born_info;
	
	CAN_SendData(motor_born_info->hcan, motor_born_info->stdId, buf);
}

/**
  * @brief          MIT协议控制参数编码与发送 - 核心函数
  * @param[in]      Motor_DM_t *motor: 电机本体结构体指针
  * @retval         无
  * @note   MIT协议CAN数据帧编码格式(8字节)：
  *         Byte0: [7:0]=position[15:8]     目标角度高8位(16位)
  *         Byte1: [7:0]=position[7:0]      目标角度低8位
  *         Byte2: [7:4]=velocity[11:4]     目标速度高8位(12位)
  *         Byte3: [7:4]=velocity[3:0]      目标速度低4位
  *                [3:0]=Kp[11:8]           KP增益高4位(12位)
  *         Byte4: [7:0]=Kp[7:0]            KP增益低8位
  *         Byte5: [7:4]=Kd[11:4]           KD增益高8位(12位)
  *         Byte6: [7:4]=Kd[3:0]            KD增益低4位
  *                [3:0]=torque[11:8]       扭矩高4位(12位)
  *         Byte7: [7:0]=torque[7:0]        扭矩低8位
  */
static void Motor_SetControlPara(Motor_DM_t *motor)
{
	Motor_DM_Tx_Info_t* motor_tx_info = motor->tx_info;
	uint16_t p, v, kp, kd, t;  /* 编码后的整型控制参数 */
  uint8_t* buf = motor_tx_info->single_tx_buff;  /* 发送缓冲区指针 */

	/* 限制输入的参数在协议定义的范围内，防止编码溢出 */
	motor_tx_info->target_angle = constrain(motor_tx_info->target_angle, P_MIN, P_MAX);
	motor_tx_info->target_speed = constrain(motor_tx_info->target_speed, V_MIN, V_MAX);
	motor_tx_info->Kp = constrain(motor_tx_info->Kp, KP_MIN, KP_MAX);
	motor_tx_info->Kd = constrain(motor_tx_info->Kd, KD_MIN, KD_MAX);
	motor_tx_info->torque = constrain(motor_tx_info->torque, T_MIN, T_MAX);

	/* 根据MIT协议，将浮点参数转换为无符号整数进行编码 */
	p  = float_to_uint(motor_tx_info->target_angle, P_MIN,  P_MAX,  16);   /* 16位编码 */
	v  = float_to_uint(motor_tx_info->target_speed, V_MIN,  V_MAX,  12);   /* 12位编码 */
	kp = float_to_uint(motor_tx_info->Kp,           KP_MIN, KP_MAX, 12);  /* 12位编码 */
	kd = float_to_uint(motor_tx_info->Kd,           KD_MIN, KD_MAX, 12);  /* 12位编码 */
	t  = float_to_uint(motor_tx_info->torque,       T_MIN,  T_MAX,  12);  /* 12位编码 */

	/* 按照MIT协议将各参数打包到8字节CAN数据帧中 */
	buf[0] = p >> 8;                                    /* 位置高8位 */
	buf[1] = p & 0xFF;                                  /* 位置低8位 */
	buf[2] = v >> 4;                                    /* 速度高8位 */
	buf[3] = ((v & 0xF) << 4) | (kp >> 8);             /* 速度低4位 + Kp高4位 */
	buf[4] = kp & 0xFF;                                 /* Kp低8位 */
	buf[5] = kd >> 4;                                    /* Kd高8位 */
	buf[6] = ((kd & 0xF) << 4) | (t >> 8);             /* Kd低4位 + 扭矩高4位 */
	buf[7] = t & 0xFF;                                  /* 扭矩低8位 */

	/* 通过CAN总线发送编码后的控制数据 */
	Motor_Send_Data(motor, buf);
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
  * @brief  计算达秒 (DM) 电机连续累计角度（处理 [-PI, +PI] 边界跳变）
  * @param  motor: 达秒电机结构体指针
  */
static void Angle_Sum_Cal(Motor_DM_t *motor)
{
    /* 首次上电未初始化时，直接对齐当前角度，跳过累加 */
    if (motor->rx_info->motor_angle_last == 0.0f && motor->rx_info->motor_angle_sum == 0.0f)
    {
        motor->rx_info->motor_angle_last = motor->rx_info->motor_angle;
        return;
    }

    /* 计算error */
    float diff = motor->rx_info->motor_angle - motor->rx_info->motor_angle_last;

    /* 边界处理*/
    while (diff >  (float)PI)  diff -= 2.0f * (float)PI;
    while (diff < -(float)PI)  diff += 2.0f * (float)PI;

    /* 获取电机正反转修正系数 (1 或 -1) */
    float order_correction = (motor->born_info->order_correction == -1) ? -1.0f : 1.0f;

    /* 累加本次真实的物理旋转增量 */
    motor->rx_info->motor_angle_sum += diff * order_correction;

    /*保留当前时刻的值为上一时刻 */
    motor->rx_info->motor_angle_last = motor->rx_info->motor_angle;
}



/**
  * @brief          电机错误码检测与状态更新
  * @param[in]      Motor_DM_t *motor: 电机本体结构体指针
  * @param[in]      uint8_t err_word:  接收到的错误码（Byte0的高4位）
  * @retval         无
  * @note   错误码对照表（达秒MIT协议）：
  *         0  - Motor_Unenable:  电机未使能
  *         1  - Motor_Enable:    电机已使能（正常工作）
  *         8  - Over_Voltage:    过压保护
  *         9  - Lack_Voltage:    欠压保护
  *         10 - Over_Current:    过流保护
  *         11 - MOS_OverTemp:    MOS管过温保护
  *         12 - Motor_OverTemp:  电机本体过温保护
  *         13 - Commun_Loss:     通信丢失
  *         其他 - Unknow_Err:    未知错误
  */
static void Motor_ERR_Check(Motor_DM_t *motor, uint8_t err_word)
{
	Motor_DM_State_t* my_state = motor->state;

	/* 根据错误码更新电机工作状态 */
	switch(err_word)
	{
		case 0:
		my_state->motor_state = Motor_Unenable;    /* 电机未使能状态 */
		break;
		case 1:
		my_state->motor_state = Motor_Enable;      /* 电机正常使能 */
		break;
		case 8:
		my_state->motor_state = Over_Voltage;      /* 过压保护触发 */
		break;
		case 9:
		my_state->motor_state = Lack_Voltage;      /* 欠压保护触发 */
		break;
		case 10:
		my_state->motor_state = Over_Current;      /* 过流保护触发 */
		break;
		case 11:
		my_state->motor_state = MOS_OverTemp;      /* MOS管过温保护 */
		break;
		case 12:
		my_state->motor_state = Motor_OverTemp;    /* 电机本体过温保护 */
		break;
		case 13:
		my_state->motor_state = Commun_Loss;       /* 通信丢失 */
		break;
		default:
		my_state->motor_state = Unknow_Err;        /* 未知错误 */
		break;
	};

	/* 记录非正常状态（非使能和非未使能状态视为异常），用于错误追踪 */
	if(my_state->motor_state != Motor_Enable &&
		my_state->motor_state != Motor_Unenable)
	{
		my_state->last_motor_state = my_state->motor_state;
	}
}

/*示例代码*/

/*-------------电机变量创建-------------*/
/*
Motor_DM_Born_Info_t Yaw_Born_Info =
{
	.stdId = 0x001,//电机控制报文ID
	
	.hcan = &hfdcan2,//使用的Can总线

};

Motor_DM_Rx_Info_t Yaw_Rx_Info_t;

Motor_DM_Tx_Info_t Yaw_Tx_Info_t;

Motor_DM_State_t Yaw_State_t;

Motor_DM_t Yaw_Motor = 
{
	.born_info = &Yaw_Born_Info,
	
	.rx_info = &Yaw_Rx_Info_t,
	
	.tx_info = &Yaw_Tx_Info_t,
	
	.state = &Yaw_State_t,
	
	.single_init = &DM_Single_Motor_Init,
};
*/

/*-------------初始化-------------*/
/*
Yaw_Motor.single_init(&Yaw_Motor);
*/

/*-------------接收函数-------------*/
/*
void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case 0x000://接收ID
		Yaw_Motor.rx(&Yaw_Motor, rxBuf);
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
		Yaw_Motor.heartbeat(&Yaw_Motor);
		
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
		Yaw_Motor.tx_info->torque = 0.5f;
		Yaw_Motor.single_set_torque(&Yaw_Motor);
		
		osDelay(1);
	}
}

*/
