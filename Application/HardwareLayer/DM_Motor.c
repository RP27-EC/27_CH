/**
 ******************************************************************************
 * @file        DM_Motor.c
 * @author      2025_YZJ
 * @Version     V1.0
 * @date        8-February-2025
 * @brief       达秒（DM）电机MIT控制协议驱动包
 ******************************************************************************
 * @attention
 *
 * 该文件实现了达秒(DM)系列电机的完整驱动，采用MIT控制协议。
 * 支持单电机和电机组（最多4个电机）两种控制模式。
 *
 * MIT协议说明：
 *   - 使用CAN总线通信，标准CAN数据帧（8字节）
 *   - 控制参数包括：目标角度(P)、目标速度(V)、角度增益(Kp)、速度增益(Kd)、力矩(T)
 *   - 参数通过线性映射转换为整数后打包发送
 *   - 电机反馈包含：当前角度、速度、力矩、错误状态
 *
 * 控制模式：
 *   - 力矩控制：直接输出指定力矩
 *   - 速度控制：通过Kd增益控制速度
 *   - 角度控制：通过Kp和Kd增益控制角度
 *
 * 通信协议：
 *   - 命令报文：0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF [CMD]
 *   - 控制报文：[P_H] [P_L] [V_H:KP_L] [KP_H:KD_L] [KD_H:T_L] [T_H]
 *
 * Copyright 2025 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "DM_Motor.h"

/* Private macro -------------------------------------------------------------*/

/**
 * @brief 电机命令报文（固定格式）
 *        前7字节固定为0xFF，第8字节为命令字
 *        0xFC: 进入电机模式
 *        0xFD: 退出电机模式
 *        0xFE: 置零编码器
 */
static uint8_t Motor_Command[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

/* Private function prototypes -----------------------------------------------*/
static void Motor_Send_Data(Motor_DM_t *motor, uint8_t* buf);
static void Motor_SetControlPara(Motor_DM_t *motor);
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
static float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits);
static void Motor_Send_Command(Motor_DM_t *motor, Motor_MIT_Command_e Command);
static void Angle_Sum_Cal(Motor_DM_t *motor);
static void Motor_ERR_Check(Motor_DM_t *motor, uint8_t err_word);
static void Group_Motor_Heartbeat(Motor_DM_Group_t *group);

/*============================================================================*/
/*                         单电机控制函数                                      */
/*============================================================================*/

/**
 * @brief  单电机卸力（将所有控制参数清零）
 * @param  motor: DM电机结构体指针
 * @retval 无
 * @note   将力矩、Kd、Kp全部置零，电机失去输出力矩
 *         用于紧急停机或安全保护场景
 */
void DM_Single_Motor_Sleep(Motor_DM_t *motor)
{
  if(motor != NULL)
  {
    motor->tx_info->torque = 0;  /* 清零力矩输出 */
    motor->tx_info->Kd = 0;      /* 清零速度增益 */
    motor->tx_info->Kp = 0;      /* 清零角度增益 */
  }
}

/**
 * @brief  单电机置零编码器
 * @param  motor: DM电机结构体指针
 * @retval 无
 * @note   先卸力，再发送置零命令
 *         置零后编码器当前值变为0
 */
void DM_Single_Motor_ZeroPosSensor(Motor_DM_t *motor)
{
  if(motor != NULL)
  {
    motor->single_sleep(motor);  /* 先卸力，确保安全 */
    /* 发送置零编码器命令 */
    Motor_Send_Command(motor, Zero_Position_Sensor);
  }
}

/**
 * @brief  单电机力矩控制
 * @param  motor: DM电机结构体指针
 * @retval 无
 * @note   首次调用时自动进入电机控制模式（发送Enter_Motor_Mode命令）
 *         后续调用设置Kp=0, Kd=0, 力矩=0（力矩控制模式）
 *         使用前需设置 motor->tx_info->torque 值
 */
void DM_Single_Motor_Set_Torque(Motor_DM_t *motor)
{
  if(motor != NULL)
  {
    if(motor->state->motor_state == Motor_Unenable)
    {
      /* 电机未使能，先发送使能命令 */
      motor->state->motor_state = Motor_Enable;
      Motor_Send_Command(motor, Enter_Motor_Mode);
    }
    else
    {
      /* 电机已使能，设置力矩控制参数 */
      Motor_DM_Tx_Info_t* motor_tx_info = motor->tx_info;
      motor_tx_info->Kp = 0;       /* 力矩模式下不需要角度增益 */
      motor_tx_info->Kd = 0;       /* 力矩模式下不需要速度增益 */
      Motor_SetControlPara(motor);  /* 打包并发送控制报文 */
      motor->tx_info->torque = 0;   /* 发送后清零（下次使用前重新设置） */
    }
  }
}

/**
 * @brief  单电机速度控制
 * @param  motor: DM电机结构体指针
 * @retval 无
 * @note   首次调用自动进入电机模式
 *         速度控制需要自行设置Kd、target_speed和torque参数
 *         本函数设置Kp=0（纯速度控制，无位置环）
 */
void DM_Single_Motor_Set_Speed(Motor_DM_t *motor)
{
  if(motor != NULL)
  {
    if(motor->state->motor_state == Motor_Unenable)
    {
      /* 电机未使能，先发送使能命令 */
      motor->state->motor_state = Motor_Enable;
      Motor_Send_Command(motor, Enter_Motor_Mode);
    }
    else
    {
      /* 电机已使能，设置速度控制参数 */
      Motor_DM_Tx_Info_t* motor_tx_info = motor->tx_info;
      motor_tx_info->Kp = 0;       /* 速度模式下不需要位置增益 */
      Motor_SetControlPara(motor);  /* 打包并发送控制报文 */
    }
  }
}

/**
 * @brief  单电机角度控制
 * @param  motor: DM电机结构体指针
 * @retval 无
 * @note   首次调用自动进入电机模式
 *         角度控制需要自行设置Kp、target_angle、Kd、target_speed和torque参数
 *         本函数将target_speed设为0（纯角度位置控制）
 */
void DM_Single_Motor_Set_Angle(Motor_DM_t *motor)
{
  if(motor != NULL)
  {
    if(motor->state->motor_state == Motor_Unenable)
    {
      /* 电机未使能，先发送使能命令 */
      motor->state->motor_state = Motor_Enable;
      Motor_Send_Command(motor, Enter_Motor_Mode);
    }
    else
    {
      /* 电机已使能，设置角度控制参数 */
      motor->tx_info->target_speed = 0;  /* 角度模式下目标速度设为0 */
      Motor_SetControlPara(motor);        /* 打包并发送控制报文 */
    }
  }
}

/**
 * @brief  电机CAN中断接收数据处理
 * @param  motor: DM电机结构体指针
 * @param  rxBuf: CAN接收数据缓冲区（8字节）
 * @retval 无
 * @note   解析DM电机MIT协议的反馈数据：
 *         [0]: 错误状态（高4位）和电机使能状态（低4位）
 *         [1][2]: 电机角度（16位，映射范围P_MIN~P_MAX）
 *         [3][4高4位]: 速度（12位，映射范围V_MIN~V_MAX）
 *         [4低4位][5]: 力矩电流（12位，映射范围C_MIN~C_MAX）
 *         同时进行多圈角度累计计算
 *         收到有效数据后清零离线计数器
 */
static void Motor_ReceiveData(Motor_DM_t *motor, uint8_t *rxBuf)
{
  Motor_DM_Rx_Info_t* motor_rx_info = motor->rx_info;

  /* 解析错误码（高4位）并更新电机状态 */
  Motor_ERR_Check(motor, rxBuf[0] >> 4);

  /* 解析电机角度：将16位整数转换为浮点数（范围P_MIN~P_MAX） */
  motor_rx_info->motor_angle = uint_to_float((uint16_t)((rxBuf[1] << 8) | rxBuf[2]), P_MIN, P_MAX, 16);

  /* 解析电机速度：12位数据，范围V_MIN~V_MAX */
  motor_rx_info->speed = uint_to_float((uint16_t)((rxBuf[3] << 4) | (rxBuf[4] >> 4)), V_MIN, V_MAX, 12);

  /* 速度精度误差修正：当速度值接近0时强制置零 */
  if(abs(motor_rx_info->speed) == 0.0109901428f)
  {
    motor_rx_info->speed = 0;
  }

  /* 解析力矩电流：12位数据，范围C_MIN~C_MAX */
  motor_rx_info->torque = uint_to_float((uint16_t)(((rxBuf[4] & 0x0F) << 8) | rxBuf[5]), C_MIN, C_MAX, 12);

  /* 计算多圈累计角度 */
  Angle_Sum_Cal(motor);

  /* 收到有效数据，清零离线计数器 */
  motor->state->offline_cnt = 0;
}

/**
 * @brief  单电机心跳包（离线检测）
 * @param  motor: DM电机结构体指针
 * @retval 无
 * @note   每次调用离线计数器加1
 *         超过阈值后标记设备离线，并将电机状态设为未使能
 */
static void DM_Motor_Hearbeat(Motor_DM_t *motor)
{
  motor->state->offline_cnt++;

  if(motor->state->offline_cnt > motor->state->offline_cnt_max)
  {
    /* 超过最大阈值，标记离线 */
    motor->state->offline_cnt = motor->state->offline_cnt_max;
    motor->state->status = DEV_OFFLINE;
    motor->state->motor_state = Motor_Unenable;  /* 电机未使能 */
  }
  else
  {
    /* 未超阈值，如果之前离线则恢复在线 */
    if(motor->state->status == DEV_OFFLINE)
      motor->state->status = DEV_ONLINE;
  }
}

/**
 * @brief  单电机初始化
 * @param  motor: DM电机结构体指针
 * @retval 无
 * @note   绑定所有函数指针并设置默认参数：
 *         - single_sleep: 卸力
 *         - single_set_torque: 力矩控制
 *         - single_set_speed: 速度控制
 *         - single_set_angle: 角度控制
 *         - rx: 数据接收解析
 *         - single_heart_beat: 心跳检测
 *         初始状态设为未使能，离线阈值100
 */
void DM_Single_Motor_Init(Motor_DM_t *motor)
{
  /* 绑定函数指针 */
  motor->single_sleep = DM_Single_Motor_Sleep;           /* 卸力函数 */
  motor->single_set_torque = DM_Single_Motor_Set_Torque; /* 力矩控制 */
  motor->single_set_speed  = DM_Single_Motor_Set_Speed;  /* 速度控制 */
  motor->single_set_angle  = DM_Single_Motor_Set_Angle;  /* 角度控制 */
  motor->rx = Motor_ReceiveData;                          /* 数据接收 */
  motor->single_heart_beat = DM_Motor_Hearbeat;          /* 心跳检测 */

  /* 初始化状态 */
  motor->state->motor_state = Motor_Unenable;       /* 电机未使能 */
  motor->state->last_motor_state = Motor_Unenable;  /* 上一次状态也为未使能 */
  motor->state->offline_cnt_max = 100;              /* 离线检测阈值：100次 */
  motor->rx_info->motor_angle_sum = 0;              /* 多圈角度清零 */
}

/*============================================================================*/
/*                         电机组控制函数                                      */
/*============================================================================*/

/**
 * @brief  多电机（1~4个）力矩控制
 * @param  group: 电机组结构体指针
 * @retval 无
 * @note   逐个遍历电机组中的电机，依次调用单电机力矩控制
 *         使用静态变量rx_num记录当前控制的电机索引
 *         每次调用只控制一个电机，4次调用控制完整个电机组
 */
static void Group_Motor_Set_Torque(Motor_DM_Group_t *group)
{
  static uint8_t rx_num = 0;  /* 当前控制的电机索引 */

  /* 控制当前索引的电机 */
  if(group->motor[rx_num] != NULL)
  {
    group->motor[rx_num]->single_set_torque(group->motor[rx_num]);
    rx_num++;
  }

  /* 索引到达电机数量上限时归零 */
  if(rx_num >= group->motor_num)
  {
    rx_num = 0;
  }
}

/**
 * @brief  电机组卸力
 * @param  group: 电机组结构体指针
 * @retval 无
 * @note   遍历电机组中最多4个电机，逐个调用卸力函数
 *         如果某个电机指针为NULL则跳过
 */
static void Group_Motor_Sleep(Motor_DM_Group_t *group)
{
  if(group->motor[0] != NULL)
    group->motor[0]->single_sleep(group->motor[0]);  /* 电机0卸力 */

  if(group->motor[1] != NULL)
    group->motor[1]->single_sleep(group->motor[1]);  /* 电机1卸力 */

  if(group->motor[2] != NULL)
    group->motor[2]->single_sleep(group->motor[2]);  /* 电机2卸力 */

  if(group->motor[3] != NULL)
    group->motor[3]->single_sleep(group->motor[3]);  /* 电机3卸力 */
}

/**
 * @brief  电机组心跳包
 * @param  group: 电机组结构体指针
 * @retval 无
 * @note   遍历电机组中最多4个电机，逐个调用心跳检测函数
 */
static void Group_Motor_Heartbeat(Motor_DM_Group_t *group)
{
  if(group->motor[0] != NULL)
    group->motor[0]->single_heart_beat(group->motor[0]);  /* 电机0心跳 */

  if(group->motor[1] != NULL)
    group->motor[1]->single_heart_beat(group->motor[1]);  /* 电机1心跳 */

  if(group->motor[2] != NULL)
    group->motor[2]->single_heart_beat(group->motor[2]);  /* 电机2心跳 */

  if(group->motor[3] != NULL)
    group->motor[3]->single_heart_beat(group->motor[3]);  /* 电机3心跳 */
}

/**
 * @brief  电机组初始化
 * @param  group: 电机组结构体指针
 * @retval 无
 * @note   遍历电机组中最多4个非空电机：
 *         1. 为每个电机绑定初始化函数并调用
 *         2. 统计初始化的电机数量
 *         3. 绑定电机组的控制函数
 */
void Group_Motor_Init(Motor_DM_Group_t *group)
{
  uint8_t num_init = 0;  /* 已初始化的电机计数 */

  if(group->motor[0] != NULL)
  {
    group->motor[0]->single_init = DM_Single_Motor_Init;  /* 绑定初始化函数 */
    group->motor[0]->single_init(group->motor[0]);         /* 初始化电机0 */
    num_init++;
  }

  if(group->motor[1] != NULL)
  {
    group->motor[1]->single_init = DM_Single_Motor_Init;
    group->motor[1]->single_init(group->motor[1]);         /* 初始化电机1 */
    num_init++;
  }

  if(group->motor[2] != NULL)
  {
    group->motor[2]->single_init = DM_Single_Motor_Init;
    group->motor[2]->single_init(group->motor[2]);         /* 初始化电机2 */
    num_init++;
  }

  if(group->motor[3] != NULL)
  {
    group->motor[3]->single_init = DM_Single_Motor_Init;
    group->motor[3]->single_init(group->motor[3]);         /* 初始化电机3 */
    num_init++;
  }

  /* 设置电机组参数 */
  group->motor_num = num_init;              /* 记录初始化的电机数量 */
  group->group_set_torque = Group_Motor_Set_Torque;  /* 绑定组力矩控制 */
  group->group_heartbeat = Group_Motor_Heartbeat;    /* 绑定组心跳检测 */
  group->group_sleep = Group_Motor_Sleep;            /* 绑定组卸力 */
}

/*============================================================================*/
/*                         工具函数                                            */
/*============================================================================*/

/**
 * @brief  整合并发送电机命令报文
 * @param  motor:   DM电机结构体指针
 * @param  Command: 命令类型枚举值
 * @retval 无
 * @note   根据命令类型设置Motor_Command的第8字节：
 *         - Enter_Motor_Mode:   0xFC（进入控制模式）
 *         - Exit_Motor_Mode:    0xFD（退出控制模式）
 *         - Zero_Position_Sensor: 0xFE（置零编码器）
 *         然后发送固定8字节命令报文
 */
static void Motor_Send_Command(Motor_DM_t *motor, Motor_MIT_Command_e Command)
{
  switch(Command)
  {
    case Enter_Motor_Mode:
      Motor_Command[7] = 0xFC;  /* 进入电机控制模式 */
      break;
    case Exit_Motor_Mode:
      Motor_Command[7] = 0xFD;  /* 退出电机控制模式 */
      break;
    case Zero_Position_Sensor:
      Motor_Command[7] = 0xFE;  /* 置零编码器 */
      break;
    default:
      break;
  }
  /* 发送命令报文 */
  Motor_Send_Data(motor, Motor_Command);
}

/**
 * @brief  根据电机结构体信息通过CAN总线发送数据
 * @param  motor: DM电机结构体指针
 * @param  buf:   8字节数据缓冲区
 * @retval 无
 * @note   从电机结构体中获取CAN实例和标准ID，调用CAN_SendData发送
 */
static void Motor_Send_Data(Motor_DM_t *motor, uint8_t* buf)
{
  Motor_DM_Born_Info_t* motor_born_info = motor->born_info;
  /* 通过CAN总线发送数据帧 */
  CAN_SendData(motor_born_info->hcan, motor_born_info->stdId, buf);
}

/**
 * @brief  根据MIT协议打包控制参数并发送
 * @param  motor: DM电机结构体指针
 * @retval 无
 * @note   打包流程：
 *         1. 将各参数限制在有效范围内
 *         2. 将float参数通过线性映射转换为整数
 *         3. 按照MIT协议格式将整数打包到8字节缓冲区
 *         4. 发送控制报文
 *
 *         数据打包格式（8字节）：
 *         buf[0] = P >> 8          (角度高8位)
 *         buf[1] = P & 0xFF        (角度低8位)
 *         buf[2] = V >> 4          (速度高8位)
 *         buf[3] = (V&0xF)<<4 | kp>>8  (速度低4位 + Kp高4位)
 *         buf[4] = kp & 0xFF       (Kp低8位)
 *         buf[5] = kd >> 4         (Kd高8位)
 *         buf[6] = (kd&0xF)<<4 | t>>8  (Kd低4位 + 力矩高4位)
 *         buf[7] = t & 0xFF        (力矩低8位)
 */
static void Motor_SetControlPara(Motor_DM_t *motor)
{
  Motor_DM_Tx_Info_t* motor_tx_info = motor->tx_info;
  uint16_t p, v, kp, kd, t;       /* 转换后的整数参数 */
  uint8_t* buf = motor_tx_info->single_tx_buff;  /* 发送缓冲区 */

  /* 限制输入参数在定义的有效范围内 */
  motor_tx_info->target_angle = constrain(motor_tx_info->target_angle, P_MIN, P_MAX);
  motor_tx_info->target_speed = constrain(motor_tx_info->target_speed, V_MIN, V_MAX);
  motor_tx_info->Kp = constrain(motor_tx_info->Kp, KP_MIN, KP_MAX);
  motor_tx_info->Kd = constrain(motor_tx_info->Kd, KD_MIN, KD_MAX);
  motor_tx_info->torque = constrain(motor_tx_info->torque, T_MIN, T_MAX);

  /* 将float参数通过线性映射转换为uint整数 */
  p = float_to_uint(motor_tx_info->target_angle, P_MIN, P_MAX, 16);       /* 角度：16位 */
  v = float_to_uint(motor_tx_info->target_speed, V_MIN, V_MAX, 12);       /* 速度：12位 */
  kp = float_to_uint(motor_tx_info->Kp, KP_MIN, KP_MAX, 12);              /* Kp：12位 */
  kd = float_to_uint(motor_tx_info->Kd, KD_MIN, KD_MAX, 12);              /* Kd：12位 */
  t = float_to_uint(motor_tx_info->torque, T_MIN, T_MAX, 12);             /* 力矩：12位 */

  /* 按照MIT协议格式打包到CAN数据帧（8字节） */
  buf[0] = p >> 8;                                    /* 角度高8位 */
  buf[1] = p & 0xFF;                                  /* 角度低8位 */
  buf[2] = v >> 4;                                    /* 速度高8位 */
  buf[3] = ((v & 0xF) << 4) | (kp >> 8);             /* 速度低4位 | Kp高4位 */
  buf[4] = kp & 0xFF;                                 /* Kp低8位 */
  buf[5] = kd >> 4;                                    /* Kd高8位 */
  buf[6] = ((kd & 0xF) << 4) | (t >> 8);             /* Kd低4位 | 力矩高4位 */
  buf[7] = t & 0xFF;                                   /* 力矩低8位 */

  /* 发送控制报文 */
  Motor_Send_Data(motor, buf);
}

/**
 * @brief  float转uint16（线性映射，带符号处理）
 * @param  x:     要转换的浮点值
 * @param  x_min: 映射范围最小值
 * @param  x_max: 映射范围最大值
 * @param  bits:  目标整数的位宽（12或16）
 * @retval 转换后的uint16值
 * @note   线性映射公式：result = (x - x_min) * (2^bits - 1) / (x_max - x_min)
 *         与DM电机通信协议保持一致
 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
  float span = x_max - x_min;       /* 映射范围跨度 */
  float offset = x_min;              /* 偏移量 */

  return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief  uint16转float（线性映射的逆运算）
 * @param  x_int:  要转换的uint16整数值
 * @param  x_min:  映射范围最小值
 * @param  x_max:  映射范围最大值
 * @param  bits:   源整数的位宽
 * @retval 转换后的浮点值
 * @note   逆映射公式：result = x_int * span / (2^bits - 1) + x_min
 */
static float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief  计算电机旋转角度和（多圈累计）
 * @param  motor: DM电机结构体指针
 * @retval 无
 * @note   通过比较当前角度与上次角度的差值来累计总旋转角度
 *         过零点处理：当角度差超过PI时，说明发生了跨越零点的旋转
 *         - 正向过零：差值为正但超过PI，实际是反向越过零点
 *         - 反向过零：差值为负但绝对值超过PI，实际是正向越过零点
 *         order_correction参数支持正反方向修正（1或-1）
 */
static void Angle_Sum_Cal(Motor_DM_t *motor)
{
  float err = 0.f;                  /* 角度差值 */
  float order_correction = 0.f;     /* 方向修正系数 */

  /* 确定方向修正系数 */
  if(motor->born_info->order_correction == 1 || motor->born_info->order_correction == -1)
  {
    order_correction = (float)motor->born_info->order_correction;
  }
  else
  {
    order_correction = 1.f;  /* 默认不修正 */
  }

  /* 电机刚启动时（上次角度和累计角度都为0），不计算误差 */
  if(!motor->rx_info->motor_angle_last && !motor->rx_info->motor_angle_sum)
  {
    err = 0.f;
  }
  else
  {
    /* 计算本次与上次的角度差 */
    err = motor->rx_info->motor_angle - motor->rx_info->motor_angle_last;
  }

  /* 过零点检测与处理 */
  if(abs(err) > (float)PI)
  {
    if(err > 0.f)
    {
      /* 正向过零：修正为实际反向旋转 */
      motor->rx_info->motor_angle_sum += (-(float)PI * 2.f + err) * order_correction;
    }
    else
    {
      /* 反向过零：修正为实际正向旋转 */
      motor->rx_info->motor_angle_sum += ((float)PI * 2.f + err) * order_correction;
    }
  }
  else
  {
    /* 未过零点，直接累加角度差 */
    motor->rx_info->motor_angle_sum += err * order_correction;
  }

  /* 更新上次角度为当前角度 */
  motor->rx_info->motor_angle_last = motor->rx_info->motor_angle;
}

/**
 * @brief  电机错误码检查
 * @param  motor:    DM电机结构体指针
 * @param  err_word: 错误码（接收数据高4位）
 * @retval 无
 * @note   根据错误码更新电机工作状态：
 *         0: 未使能
 *         1: 使能（正常）
 *         8: 过压
 *         9: 欠压
 *         10: 过流
 *         11: MOS过温
 *         12: 电机过温
 *         13: 通信丢失
 *         其他: 未知错误
 *         如果发生非正常/非使能状态，记录为上次状态（用于故障诊断）
 */
static void Motor_ERR_Check(Motor_DM_t *motor, uint8_t err_word)
{
  Motor_DM_State_t* my_state = motor->state;

  switch(err_word)
  {
    case 0:
      my_state->motor_state = Motor_Unenable;  /* 未使能 */
      break;
    case 1:
      my_state->motor_state = Motor_Enable;    /* 正常使能 */
      break;
    case 8:
      my_state->motor_state = Over_Voltage;    /* 过压保护 */
      break;
    case 9:
      my_state->motor_state = Lack_Voltage;    /* 欠压保护 */
      break;
    case 10:
      my_state->motor_state = Over_Current;    /* 过流保护 */
      break;
    case 11:
      my_state->motor_state = MOS_OverTemp;    /* MOS管过温 */
      break;
    case 12:
      my_state->motor_state = Motor_OverTemp;  /* 电机过温 */
      break;
    case 13:
      my_state->motor_state = Commun_Loss;     /* 通信丢失 */
      break;
    default:
      my_state->motor_state = Unknow_Err;      /* 未知错误 */
      break;
  };

  /* 记录故障状态（仅记录非使能和非正常的状态） */
  if(my_state->motor_state != Motor_Enable &&
     my_state->motor_state != Motor_Unenable)
  {
    my_state->last_motor_state = my_state->motor_state;
  }
}

/*-----------------------------示例代码---------------------------------------*/
/* 以下为使用示例，展示了如何创建电机变量、初始化、接收和控制电机 */

/*-------------电机变量创建-------------*/
/*
Motor_DM_Born_Info_t Yaw_Born_Info =
{
  .stdId = 0x001,       // 电机控制报文ID
  .hcan = &hfdcan2,     // 使用的CAN总线
};

Motor_DM_Rx_Info_t Yaw_Rx_Info_t;   // 接收信息结构体
Motor_DM_Tx_Info_t Yaw_Tx_Info_t;   // 发送信息结构体
Motor_DM_State_t Yaw_State_t;       // 电机状态结构体

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
    case 0x000:   // 接收ID
      Yaw_Motor.rx(&Yaw_Motor, rxBuf);
      break;
    default:
      break;
  }
}
*/

/*-------------任务执行-------------*/
/*
void StartMonitorTask(void const * argument)
{
  for(;;)
  {
    // 发送控制报文，控制电机输出扭矩为0.5N*m
    Yaw_Motor.tx_info->torque = 0.5f;
    Yaw_Motor.single_set_torque(&Yaw_Motor);
    osDelay(1);
  }
}
*/
