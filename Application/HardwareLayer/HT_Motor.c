/**
 ******************************************************************************
 * @file        HT_Motor.c
 * @author      2025_YZJ
 * @Version     V1.0
 * @date        8-February-2025
 * @brief       海泰（HT）电机驱动包 - 适用型号HT-03
 ******************************************************************************
 * @attention
 *
 * 该文件实现了海泰HT-03电机的驱动功能，采用MIT控制协议。
 * 支持单电机和电机组（最多4个）两种控制模式。
 *
 * HT-03电机特点：
 *   - 作为轮毂电机使用，具有减速比
 *   - 使用CAN总线通信，MIT控制协议
 *   - 编码器值范围：HT_P_MIN ~ HT_P_MAX（单位：度）
 *   - 具有力矩常数TORQUE_CONSTANT，用于电流到力矩的转换
 *
 * 与DM电机的区别：
 *   - 编码器值单位为度（而非弧度）
 *   - 具有order_correction方向修正参数
 *   - 多圈角度通过速度积分计算（motor_angle_sum_vi）
 *
 * Copyright 2025 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "HT_Motor.h"

/* Private macro -------------------------------------------------------------*/

/**
 * @brief 电机命令报文（固定格式）
 *        前7字节固定为0xFF，第8字节为命令字
 *        注意：与DM电机不同，HT电机的命令报文第8字节为0xFD
 */
static uint8_t Motor_Command[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

/* Private function prototypes -----------------------------------------------*/
static void Motor_Send_Data(Motor_HT_t *motor, uint8_t* buf);
static void Motor_Send_Command(Motor_HT_t *motor, Motor_MIT_Command_e Command);
static void HT_Motor_SetControlPara(Motor_HT_t *motor);
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
static float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits);
static void Encoder_to_Motor_Angle(Motor_HT_t *motor);

/**
 * @brief 接收状态标志（用于调试）
 */
uint8_t flag_rx;

/*============================================================================*/
/*                         单电机控制函数                                      */
/*============================================================================*/

/**
 * @brief  单电机卸力
 * @param  motor: HT电机结构体指针
 * @retval 无
 * @note   将力矩、Kd、Kp全部清零，电机失去输出力矩
 */
void HT_Single_Motor_Sleep(Motor_HT_t *motor)
{
  if(motor != NULL)
  {
    Motor_HT_Tx_Info_t* motor_tx_info = motor->tx_info;
    motor_tx_info->torque = 0;  /* 清零力矩 */
    motor_tx_info->Kd = 0;      /* 清零速度增益 */
    motor_tx_info->Kp = 0;      /* 清零角度增益 */
  }
}

/**
 * @brief  单电机置零编码器
 * @param  motor: HT电机结构体指针
 * @retval 无
 * @note   先卸力，再发送置零编码器命令
 */
void HT_Single_Motor_ZeroPosSensor(Motor_HT_t *motor)
{
  if(motor != NULL)
  {
    motor->single_sleep(motor);  /* 先卸力，确保安全 */
    Motor_Send_Command(motor, Zero_Position_Sensor);  /* 发送置零命令 */
  }
}

/**
 * @brief  单电机力矩控制
 * @param  motor: HT电机结构体指针
 * @retval 无
 * @note   首次调用自动进入电机控制模式
 *         设置Kp=0, Kd=0, target_angle=0, target_speed=0
 *         纯力矩输出模式
 */
void HT_Single_Motor_Set_Torque(Motor_HT_t *motor)
{
  if(motor != NULL)
  {
    if(motor->state->mode == Motor_UnControl)
    {
      /* 电机未控制，先发送使能命令 */
      Motor_Send_Command(motor, Enter_Motor_Mode);
      motor->state->mode = Motor_Control;
    }
    else
    {
      /* 电机已使能，设置力矩控制参数 */
      Motor_HT_Tx_Info_t* motor_tx_info = motor->tx_info;
      motor_tx_info->target_angle = 0;  /* 力矩模式下角度目标为0 */
      motor_tx_info->target_speed = 0;  /* 力矩模式下速度目标为0 */
      motor_tx_info->Kp = 0;            /* 力矩模式下角度增益为0 */
      motor_tx_info->Kd = 0;            /* 力矩模式下速度增益为0 */
      HT_Motor_SetControlPara(motor);   /* 打包并发送控制报文 */
      Motor_Send_Data(motor, motor_tx_info->single_tx_buff);  /* 发送数据 */
    }
  }
}

/**
 * @brief  单电机速度控制
 * @param  motor: HT电机结构体指针
 * @retval 无
 * @note   首次调用自动进入电机控制模式
 *         设置Kp=0, target_angle=0，纯速度控制
 *         需要自行设置Kd和target_speed
 */
void HT_Single_Motor_Set_Speed(Motor_HT_t *motor)
{
  if(motor != NULL)
  {
    if(motor->state->mode == Motor_UnControl)
    {
      Motor_Send_Command(motor, Enter_Motor_Mode);  /* 使能电机 */
      motor->state->mode = Motor_Control;
    }

    Motor_HT_Tx_Info_t* motor_tx_info = motor->tx_info;
    motor_tx_info->target_angle = 0;  /* 速度模式下角度目标为0 */
    motor_tx_info->Kp = 0;            /* 速度模式下不需要位置增益 */
    HT_Motor_SetControlPara(motor);   /* 打包并发送 */
    Motor_Send_Data(motor, motor_tx_info->single_tx_buff);
  }
}

/**
 * @brief  单电机角度控制
 * @param  motor: HT电机结构体指针
 * @retval 无
 * @note   首次调用自动进入电机控制模式
 *         角度控制需要自行设置Kp、target_angle、Kd、target_speed、torque
 */
void HT_Single_Motor_Set_Angle(Motor_HT_t *motor)
{
  if(motor != NULL)
  {
    if(motor->state->mode == Motor_UnControl)
    {
      Motor_Send_Command(motor, Enter_Motor_Mode);  /* 使能电机 */
      motor->state->mode = Motor_Control;
    }

    Motor_HT_Tx_Info_t* motor_tx_info = motor->tx_info;
    HT_Motor_SetControlPara(motor);                 /* 打包并发送 */
    Motor_Send_Data(motor, motor_tx_info->single_tx_buff);
  }
}

/**
 * @brief  电机CAN中断接收数据处理
 * @param  motor: HT电机结构体指针
 * @param  rxBuf: CAN接收数据缓冲区（8字节）
 * @retval 无
 * @note   解析HT电机MIT协议反馈数据：
 *         [1][2]: 编码器值（16位，范围HT_P_MIN~HT_P_MAX，单位：度）
 *         [3][4高4位]: 速度（12位，范围HT_V_MIN~HT_V_MAX）
 *         [4低4位][5]: 扭矩电流（12位，范围HT_C_MIN~HT_C_MAX）
 *         扭矩 = 扭矩电流 * 力矩常数
 *         同时进行编码器到角度的转换和多圈角度积分
 */
void static Motor_ReceiveData(Motor_HT_t *motor, uint8_t *rxBuf)
{
  Motor_HT_Rx_Info_t* motor_rx_info = motor->rx_info;

  /* 解析编码器值（范围HT_P_MIN~HT_P_MAX，单位：度） */
  motor_rx_info->encoder = uint_to_float((uint16_t)((rxBuf[1] << 8) | rxBuf[2]), HT_P_MIN, HT_P_MAX, 16);

  /* 解析速度（12位），乘以方向修正系数 */
  motor_rx_info->speed = uint_to_float((uint16_t)((rxBuf[3] << 4) | (rxBuf[4] >> 4)), HT_V_MIN, HT_V_MAX, 12)
                         * motor->born_info->order_correction;

  /* 解析扭矩电流（12位） */
  motor_rx_info->torque_current = uint_to_float((uint16_t)(((rxBuf[4] & 0x0F) << 8) | rxBuf[5]), HT_C_MIN, HT_C_MAX, 12);

  /* 计算实际输出扭矩 = 扭矩电流 * 力矩常数 */
  motor_rx_info->torque = motor_rx_info->torque_current * TORQUE_CONSTANT;

  /* 编码器值转换为弧度制角度 */
  Encoder_to_Motor_Angle(motor);

  /* 通过速度积分计算多圈累计角度（用于需要多圈位置的场景） */
  motor_rx_info->motor_angle_sum_vi += motor_rx_info->speed * TIME_STEP;

  /* 收到有效数据，清零离线计数器 */
  motor->state->offline_cnt = 0;
}

/**
 * @brief  单电机心跳包（离线检测）
 * @param  motor: HT电机结构体指针
 * @retval 无
 * @note   每次调用离线计数器加1
 *         超过阈值后标记设备离线，电机状态设为未控制
 */
void HT_Motor_Hearbeat(Motor_HT_t *motor)
{
  motor->state->offline_cnt++;

  if(motor->state->offline_cnt > motor->state->offline_cnt_max)
  {
    /* 超过阈值，标记离线 */
    motor->state->status = DEV_OFFLINE;
    motor->state->mode = Motor_UnControl;  /* 设为未控制状态 */
  }
  else
  {
    if(motor->state->status == DEV_OFFLINE)
      motor->state->status = DEV_ONLINE;  /* 恢复在线 */
  }
}

/**
 * @brief  单电机初始化
 * @param  motor: HT电机结构体指针
 * @retval 无
 * @note   绑定所有函数指针并设置默认参数：
 *         - 卸力、置零、力矩/速度/角度控制函数
 *         - 数据接收、心跳检测函数
 *         初始状态为未控制，离线阈值500
 */
void HT_Single_Motor_Init(Motor_HT_t *motor)
{
  motor->state->mode = Motor_UnControl;  /* 初始状态：未控制 */

  /* 绑定函数指针 */
  motor->single_sleep = HT_Single_Motor_Sleep;           /* 卸力 */
  motor->zero_position = HT_Single_Motor_ZeroPosSensor;  /* 置零 */
  motor->single_set_torque = HT_Single_Motor_Set_Torque; /* 力矩控制 */
  motor->single_set_speed  = HT_Single_Motor_Set_Speed;  /* 速度控制 */
  motor->single_set_angle  = HT_Single_Motor_Set_Angle;  /* 角度控制 */
  motor->rx = Motor_ReceiveData;                          /* 数据接收 */
  motor->single_heart_beat = HT_Motor_Hearbeat;          /* 心跳检测 */

  /* 初始卸力 */
  motor->single_sleep(motor);

  /* 设置默认参数 */
  motor->state->offline_cnt_max = 500;    /* 离线阈值：500次 */
  motor->rx_info->motor_angle_sum = 0;    /* 多圈角度清零 */
}

/*============================================================================*/
/*                         电机组控制函数                                      */
/*============================================================================*/

/**
 * @brief  多电机（1~4个）角度控制
 * @param  group: HT电机组结构体指针
 * @retval 无
 * @note   逐个遍历电机组中的电机，依次调用单电机角度控制
 *         4个电机必须为同一类型
 */
static void Group_Motor_Set_Torque(Motor_HT_Group_t *group)
{
  /* 逐个控制电机组中的电机 */
  if(group->motor[0] != NULL)
    group->motor[0]->single_set_angle(group->motor[0]);  /* 电机0角度控制 */

  if(group->motor[1] != NULL)
    group->motor[1]->single_set_angle(group->motor[1]);  /* 电机1角度控制 */

  if(group->motor[2] != NULL)
    group->motor[2]->single_set_angle(group->motor[2]);  /* 电机2角度控制 */

  if(group->motor[3] != NULL)
    group->motor[3]->single_set_angle(group->motor[3]);  /* 电机3角度控制 */
}

/**
 * @brief  电机组卸力
 * @param  group: HT电机组结构体指针
 * @retval 无
 * @note   逐个遍历电机组中的电机，调用卸力函数
 *         不包含CAN发送操作，仅设置参数
 */
static void Group_Motor_Sleep(Motor_HT_Group_t *group)
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
 * @param  group: HT电机组结构体指针
 * @retval 无
 * @note   遍历电机组中最多4个电机，逐个调用心跳检测函数
 */
static void Group_Motor_Heartbeat(Motor_HT_Group_t *group)
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
 * @param  group: HT电机组结构体指针
 * @retval 无
 * @note   遍历电机组中最多4个非空电机：
 *         1. 为每个电机绑定初始化函数并调用
 *         2. 绑定电机组的控制/心跳/卸力函数
 */
void HT_Group_Motor_Init(Motor_HT_Group_t *group)
{
  /* 逐个初始化电机 */
  if(group->motor[0] != NULL)
  {
    group->motor[0]->single_init = HT_Single_Motor_Init;
    group->motor[0]->single_init(group->motor[0]);  /* 初始化电机0 */
  }

  if(group->motor[1] != NULL)
  {
    group->motor[1]->single_init = HT_Single_Motor_Init;
    group->motor[1]->single_init(group->motor[1]);  /* 初始化电机1 */
  }

  if(group->motor[2] != NULL)
  {
    group->motor[2]->single_init = HT_Single_Motor_Init;
    group->motor[2]->single_init(group->motor[2]);  /* 初始化电机2 */
  }

  if(group->motor[3] != NULL)
  {
    group->motor[3]->single_init = HT_Single_Motor_Init;
    group->motor[3]->single_init(group->motor[3]);  /* 初始化电机3 */
  }

  /* 绑定电机组操作函数 */
  group->group_set_torque = Group_Motor_Set_Torque;  /* 组控制 */
  group->group_heartbeat = Group_Motor_Heartbeat;    /* 组心跳 */
  group->group_sleep = Group_Motor_Sleep;            /* 组卸力 */
}

/*============================================================================*/
/*                         工具函数                                            */
/*============================================================================*/

/**
 * @brief  整合并发送电机命令报文
 * @param  motor:   HT电机结构体指针
 * @param  Command: 命令类型枚举值
 * @retval 无
 * @note   根据命令类型设置Motor_Command的第8字节：
 *         - Enter_Motor_Mode:   0xFC（进入控制模式）
 *         - Exit_Motor_Mode:    0xFD（退出控制模式）
 *         - Zero_Position_Sensor: 0xFE（置零编码器）
 */
static void Motor_Send_Command(Motor_HT_t *motor, Motor_MIT_Command_e Command)
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
 * @param  motor: HT电机结构体指针
 * @param  buf:   8字节数据缓冲区
 * @retval 无
 */
static void Motor_Send_Data(Motor_HT_t *motor, uint8_t* buf)
{
  Motor_HT_Born_Info_t* motor_born_info = motor->born_info;
  /* 通过CAN总线发送数据帧 */
  CAN_SendData(motor_born_info->hcan, motor_born_info->stdId, buf);
}

/**
 * @brief  根据MIT协议打包控制参数并发送
 * @param  motor: HT电机结构体指针
 * @retval 无
 * @note   与DM电机的MIT协议打包格式相同
 *         将float参数限制范围后转换为整数，按协议打包到8字节缓冲区
 */
static void HT_Motor_SetControlPara(Motor_HT_t *motor)
{
  Motor_HT_Tx_Info_t* motor_tx_info = motor->tx_info;
  uint16_t p, v, kp, kd, t;
  uint8_t* buf = motor_tx_info->single_tx_buff;

  /* 限制输入参数在HT电机定义的有效范围内 */
  motor_tx_info->target_angle = constrain(motor_tx_info->target_angle, HT_P_MIN, HT_P_MAX);
  motor_tx_info->target_speed = constrain(motor_tx_info->target_speed, HT_V_MIN, HT_V_MAX);
  motor_tx_info->Kp = constrain(motor_tx_info->Kp, HT_KP_MIN, HT_KP_MAX);
  motor_tx_info->Kd = constrain(motor_tx_info->Kd, HT_KD_MIN, HT_KD_MAX);
  motor_tx_info->torque = constrain(motor_tx_info->torque, -8, 8);  /* 力矩限制在-8~8 */

  /* 将float参数通过线性映射转换为整数 */
  p = float_to_uint(motor_tx_info->target_angle, HT_P_MIN, HT_P_MAX, 16);
  v = float_to_uint(motor_tx_info->target_speed, HT_V_MIN, HT_V_MAX, 12);
  kp = float_to_uint(motor_tx_info->Kp, HT_KP_MIN, HT_KP_MAX, 12);
  kd = float_to_uint(motor_tx_info->Kd, HT_KD_MIN, HT_KD_MAX, 12);
  t = float_to_uint(motor_tx_info->torque, HT_T_MIN, HT_T_MAX, 12);

  /* 按照MIT协议格式打包到CAN数据帧（8字节） */
  buf[0] = p >> 8;
  buf[1] = p & 0xFF;
  buf[2] = v >> 4;
  buf[3] = ((v & 0xF) << 4) | (kp >> 8);
  buf[4] = kp & 0xFF;
  buf[5] = kd >> 4;
  buf[6] = ((kd & 0xF) << 4) | (t >> 8);
  buf[7] = t & 0xFF;
}

/**
 * @brief  float转uint16（线性映射）
 * @param  x:     要转换的浮点值
 * @param  x_min: 映射范围最小值
 * @param  x_max: 映射范围最大值
 * @param  bits:  目标整数的位宽
 * @retval 转换后的uint16值
 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
  float span = x_max - x_min;
  float offset = x_min;

  return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief  uint16转float（线性映射的逆运算）
 * @param  x_int:  要转换的uint16整数值
 * @param  x_min:  映射范围最小值
 * @param  x_max:  映射范围最大值
 * @param  bits:   源整数的位宽
 * @retval 转换后的浮点值
 */
static float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}


/**
 * @brief  将编码器值转化为弧度制角度（0~2PI）
 * @param  motor: HT电机结构体指针
 * @retval 无
 * @note   编码器原始值（度）转换为弧度：
 *         1. 计算本次与上次的角度差（过零点处理）
 *         2. 应用方向修正系数
 *         3. 累计多圈角度
 *         4. 将累计角度归一化到0~2PI范围
 */
static void Encoder_to_Motor_Angle(Motor_HT_t *motor)
{
  float order_correction = 0.f;

  /* 确定方向修正系数 */
  if(motor->born_info->order_correction == 1 || motor->born_info->order_correction == -1)
  {
    order_correction = (float)motor->born_info->order_correction;
  }
  else
  {
    order_correction = 1.f;
  }

  /* 计算编码器角度差（过零点判断） */
  if(!motor->rx_info->encoder_last && !motor->rx_info->motor_angle_sum)
  {
    /* 电机刚启动，不计算误差 */
    motor->rx_info->encoder_err = 0.f;
  }
  else
  {
    /* 计算本次与上次的角度差 */
    motor->rx_info->encoder_err = -motor->rx_info->encoder_last + motor->rx_info->encoder;
  }

  /* 过零点处理（编码器范围为-180~180度） */
  if(motor->rx_info->encoder_err > 180.f)
  {
    motor->rx_info->encoder_err -= 2 * HT_P_MAX;  /* 正向过零修正 */
  }
  else if(motor->rx_info->encoder_err < -180.f)
  {
    motor->rx_info->encoder_err += 2 * HT_P_MAX;  /* 反向过零修正 */
  }

  /* 累计多圈角度（应用方向修正） */
  motor->rx_info->motor_angle_sum += motor->rx_info->encoder_err * order_correction;

  /* 将累计角度归一化到0~2PI范围 */
  motor->rx_info->motor_angle = motor->rx_info->motor_angle_sum
    - (int16_t)(motor->rx_info->motor_angle_sum / (2.f * PI)) * 2.f * PI;

  /* 更新上次编码器值 */
  motor->rx_info->encoder_last = motor->rx_info->encoder;
}

/*-----------------------------示例代码---------------------------------------*/
/* 以下为使用示例 */

/*-------------电机变量创建-------------*/
/*
Motor_HT_Born_Info_t L_Wheel_Born_Info =
{
  .stdId = 0x009,               // 电机控制报文ID
  .hcan = &hcan2,               // 使用的CAN总线
  .order_correction = 0,        // 电机总角度的正方向为顺时针
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
};
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
    case 0x000:
      L_Wheel.rx(&L_Wheel, rxBuf);
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
    L_Wheel.tx_info->torque = 0.5f;
    L_Wheel.single_set_torque(&L_Wheel);
    osDelay(1);
  }
}
*/
