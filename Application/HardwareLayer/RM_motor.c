/**
 ******************************************************************************
 * @file    RM_motor.c
 * @brief   RoboMaster（RM）标准电机控制驱动文件
 ******************************************************************************
 * @attention
 *
 * 该文件实现了RoboMaster标准电机的完整驱动，支持以下电机类型：
 * - 3508电机（带减速箱）：大扭矩，常用于底盘驱动
 * - 3508电机（单电机/无减速）：不常用
 * - 2006电机（带减速箱）：中等扭矩，常用于发射机构
 * - 6020电机（无减速）：高精度角度控制，常用于云台
 *
 * 通信协议：
 *   - 使用CAN总线通信，标准CAN数据帧（8字节）
 *   - 控制报文：每个电机占用2字节（电流值），4个电机共8字节
 *   - 反馈报文：位置(2字节) + 速度(2字节) + 电流(2字节) + 温度(1字节)
 *
 * 控制模式：
 *   - 力矩（电流）控制：直接输出指定电流
 *   - 速度控制：通过PID控制速度
 *   - 角度控制：串级PID（外环角度 + 内环速度）
 *
 * 电机参数：
 *   - 3508减速箱：减速比 _3508_REDUCT_RATIO，力矩常数 _3508_TORQUE_CONSTANT
 *   - 2006减速箱：减速比 _2006_REDUCT_RATIO
 *   - 6020电机：力矩常数 _6020_TORQUE_CONSTANT
 *   - 编码器分辨率：8192（13位）
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "rm_motor.h"
#include "pid.h"         /* PID控制算法 */
#include "arm_math.h"    /* ARM DSP数学库 */
#include "rp_math.h"     /* 数学工具函数 */

/* Private function prototypes -----------------------------------------------*/
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

/*============================================================================*/
/*                         单电机控制函数                                      */
/*============================================================================*/

/**
 * @brief  单电机力矩控制（含CAN发送操作）
 * @param  motor: RM电机结构体指针
 * @retval 无
 * @note   将力矩值转换为原始电流数据，打包到发送缓冲区对应位置
 *         通过CAN总线发送控制报文
 *         每个电机在报文中占用2字节，位置由rxId决定
 */
static void Motor_Set_Torque(Motor_RM_t *motor)
{
  uint8_t Id = motor->born_info->rxId * 2;  /* 计算在报文中的字节偏移（每个电机占2字节） */

  /* 将力矩转换为原始电流值 */
  Torque_to_Raw_Current(motor);

  /* 将电流值打包到发送缓冲区（大端字节序：高字节在前） */
  motor->tx_info->tx_buff[Id] = (uint8_t)(motor->tx_info->torque_current_raw >> 8);    /* 高字节 */
  motor->tx_info->tx_buff[Id + 1] = (uint8_t)(motor->tx_info->torque_current_raw);     /* 低字节 */

  /* 通过CAN总线发送8字节控制报文（包含最多4个电机的电流数据） */
  CAN_SendData(motor->born_info->hcan, motor->born_info->stdId, motor->tx_info->tx_buff);
}

/**
 * @brief  单电机直接控制（不含力矩转换，含CAN发送操作）
 * @param  motor: RM电机结构体指针
 * @retval 无
 * @note   与Motor_Set_Torque不同，直接使用已设置的torque_current_raw值
 *         适用于外部已经计算好电流值的场景
 */
static void Motor_Ctrl(Motor_RM_t *motor)
{
  uint8_t Id = motor->born_info->rxId * 2;

  /* 直接使用已设置的原始电流值打包 */
  motor->tx_info->tx_buff[Id] = (uint8_t)(motor->tx_info->torque_current_raw >> 8);
  motor->tx_info->tx_buff[Id + 1] = (uint8_t)(motor->tx_info->torque_current_raw);

  CAN_SendData(motor->born_info->hcan, motor->born_info->stdId, motor->tx_info->tx_buff);
}

/**
 * @brief  单电机卸力
 * @param  motor: RM电机结构体指针
 * @retval 无
 * @note   将力矩设为0后调用力矩控制函数，发送零电流指令
 */
static void Single_Motor_Sleep(Motor_RM_t *motor)
{
  motor->tx_info->torque = 0;        /* 清零力矩 */
  motor->single_set_torque(motor);   /* 发送零电流控制报文 */
}

/**
 * @brief  单电机速度控制
 * @param  motor: RM电机结构体指针
 * @retval 无
 * @note   速度PID控制，输出为力矩值
 *         不含CAN发送操作，需要再次调用set_torque才会发送
 *         控制流程：
 *         1. 设置速度PID反馈值（编码器速度）
 *         2. 计算误差
 *         3. 执行PID计算
 *         4. 输出为力矩值
 */
static void Motor_Set_Speed(Motor_RM_t *motor)
{
  pid_ctrl_t* my_speed_ctrl = motor->ctrl->speed_ctrl;  /* 获取速度PID控制器 */

  my_speed_ctrl->measure = motor->rx_info->encoder_speed;  /* 设置反馈值：编码器速度 */
  pid_err_cal(my_speed_ctrl);     /* 计算误差 */
  single_pid_ctrl(my_speed_ctrl); /* 执行PID计算 */
  motor->tx_info->torque = my_speed_ctrl->out;  /* 输出为力矩值 */
}


/**
 * @brief  单电机角度控制（串级PID：外环角度 + 内环速度）
 * @param  motor: RM电机结构体指针
 * @retval 无
 * @note   串级PID控制架构：
 *         外环：角度PID，输出为速度目标值
 *         内环：速度PID，输出为力矩值
 *
 *         特殊功能：
 *         - Nearest_Return：最近路径返回（过零点处理）
 *         - Angle_Input_Flag：角度输入模式标志
 *           false: 使用编码器值（单位：编码器计数）
 *           true:  使用角度值（单位：度）
 *         - Speed_Input_Flag：速度输入模式标志
 *
 *         不含CAN发送操作，需要再次调用set_torque才会发送
 */
static void Motor_Set_Angle(Motor_RM_t *motor)
{
  pid_ctrl_t* my_angle_ctrl = motor->ctrl->angle_ctrl_outer;  /* 外环角度PID控制器 */
  pid_ctrl_t* my_speed_ctrl = motor->ctrl->angle_ctrl_inner;  /* 内环速度PID控制器 */

  /* ====== 外环：角度PID ====== */
  if(motor->ctrl->Angle_Input_Flag == false)
  {
    /* 使用编码器值作为角度反馈（编码器模式） */
    my_angle_ctrl->measure = motor->rx_info->encoder;
  }
  pid_err_cal(my_angle_ctrl);  /* 计算角度误差 */

  /* 最近路径返回处理（过零点优化） */
  if(motor->ctrl->Nearest_Return == true)
  {
    if(motor->ctrl->Angle_Input_Flag == false)
    {
      /* 编码器模式：编码器范围0~8191，过零点处理 */
      if(my_angle_ctrl->err < 0)
        my_angle_ctrl->err += 8192;   /* 负误差修正（反向过零） */
      if(my_angle_ctrl->err > 4096)
        my_angle_ctrl->err -= 8192;   /* 正误差修正（正向过零） */
    }
    else
    {
      /* 角度模式：范围-180~180度，过零点处理 */
      if(my_angle_ctrl->err < -180.f)
        my_angle_ctrl->err += 360.f;  /* 反向过零修正 */
      if(my_angle_ctrl->err > 180.f)
        my_angle_ctrl->err -= 360.f;  /* 正向过零修正 */
    }
  }

  single_pid_ctrl(my_angle_ctrl);  /* 外环PID计算，输出为速度目标值 */

  /* ====== 内环：速度PID ====== */
  my_speed_ctrl->target = my_angle_ctrl->out;  /* 外环输出作为内环目标 */

  if(motor->ctrl->Speed_Input_Flag == false)
    my_speed_ctrl->measure = motor->rx_info->encoder_speed;  /* 设置速度反馈 */

  pid_err_cal(my_speed_ctrl);     /* 计算速度误差 */
  single_pid_ctrl(my_speed_ctrl); /* 内环PID计算，输出为力矩值 */

  motor->tx_info->torque = my_speed_ctrl->out;  /* 最终输出为力矩值 */
}


/**
 * @brief  电机心跳失联检测
 * @param  motor: RM电机结构体指针
 * @retval 无
 * @note   每次调用离线计数器加1
 *         超过阈值后标记设备离线
 */
static void rm_motor_heart_beat(Motor_RM_t *motor)
{
  Motor_RM_State_t *motor_state = motor->state;
  motor_state->offline_cnt++;

  if(motor_state->offline_cnt > motor_state->offline_cnt_max)
  {
    /* 超过阈值，标记离线 */
    motor_state->offline_cnt = motor_state->offline_cnt_max;
    motor_state->status = DEV_OFFLINE;
  }
  else
  {
    /* 未超阈值，如果之前离线则恢复在线 */
    if(motor_state->status == DEV_OFFLINE)
      motor_state->status = DEV_ONLINE;
  }
}

/**
 * @brief  RM标准电机数据更新函数
 * @param  rm_motor: RM电机结构体指针
 * @param  rxBuf:    CAN接收数据缓冲区（8字节）
 * @retval 无
 * @note   解析RM电机反馈数据：
 *         [0][1]: 编码器位置（13位，范围0~8191）
 *         [2][3]: 转子转速（RPM）
 *         [4][5]: 实际转矩电流
 *         [6]:    温度（摄氏度）
 *         同时计算编码器累计值、弧度角度、实际力矩
 */
static void rm_motor_update(Motor_RM_t *rm_motor, uint8_t *rxBuf)
{
  Motor_RM_Rx_Info_t *motor_info = rm_motor->rx_info;

  /* 解析编码器位置值（13位，范围0~8191） */
  motor_info->encoder = CAN_01_GetMotorAngle(rxBuf);

  /* 计算编码器累计值（处理过零点） */
  Encoder_Sum_Cal(rm_motor);

  /* 将编码器值转换为弧度制角度 */
  Encoder_to_Motor_Angle(rm_motor);

  /* 解析转子转速（RPM） */
  motor_info->encoder_speed = CAN_23_GetMotorSpeed(rxBuf);

  /* 将转速转换为弧度/秒 */
  motor_info->speed = RPM_to_Rads(rm_motor);

  /* 解析转矩电流原始值 */
  motor_info->torque_current_raw = CAN_45_GetMotorCurrent(rxBuf);

  /* 将原始电流值转换为实际力矩 */
  Raw_Current_to_Torque(rm_motor);

  /* 解析温度 */
  motor_info->temperature = CAN_6_GetMotorTemperature(rxBuf);

  /* 收到有效数据，清零离线计数器 */
  rm_motor->state->offline_cnt = 0;
}

/**
 * @brief  RM电机初始化
 * @param  motor: RM电机结构体指针
 * @retval 无
 * @note   绑定所有函数指针并设置默认参数：
 *         - 力矩控制、速度控制、角度控制、直接控制
 *         - 卸力、心跳检测、数据接收
 *         离线阈值100
 */
void RM_Motor_Init(Motor_RM_t *motor)
{
  /* 绑定函数指针 */
  motor->single_set_torque = Motor_Set_Torque;    /* 力矩控制（含力矩转换） */
  motor->single_heart_beat = rm_motor_heart_beat;  /* 心跳检测 */
  motor->single_sleep = Single_Motor_Sleep;        /* 卸力 */
  motor->single_set_speed = Motor_Set_Speed;       /* 速度控制 */
  motor->single_set_angle = Motor_Set_Angle;       /* 角度控制（串级PID） */
  motor->single_ctrl = Motor_Ctrl;                 /* 直接控制（不含转换） */
  motor->rx = rm_motor_update;                     /* 数据接收解析 */

  /* 设置默认参数 */
  motor->state->offline_cnt_max = 100;  /* 离线阈值：100次 */
}

/*============================================================================*/
/*                         多电机控制函数                                      */
/*============================================================================*/

/**
 * @brief  多电机（1~4个）力矩控制（含力矩转换和CAN发送）
 * @param  group: RM电机组结构体指针
 * @retval 无
 * @note   将4个电机的电流数据打包到同一个8字节缓冲区中发送
 *         每个电机占用2字节，4个电机共8字节
 *         4个电机必须为同一CAN总线和同一标准ID
 */
static void Group_Motor_Set_Torque(Motor_RM_Group_t *group)
{
  int16_t torque_current = 0;
  uint8_t Id;

  /* 遍历4个电机槽位 */
  for(uint8_t i = 0; i < 4; i++)
  {
    if(group->motor[i] != NULL)
    {
      Id = group->motor[i]->born_info->rxId * 2;  /* 计算字节偏移 */

      /* 力矩转换为原始电流值 */
      Torque_to_Raw_Current(group->motor[i]);
      torque_current = group->motor[i]->tx_info->torque_current_raw;

      /* 打包到缓冲区对应位置 */
      group->tx_buff[Id] = (uint8_t)(torque_current >> 8);    /* 高字节 */
      group->tx_buff[Id + 1] = (uint8_t)(torque_current);     /* 低字节 */
    }
  }

  /* 一次性发送8字节控制报文（包含4个电机数据） */
  CAN_SendData(group->hcan, group->stdId, group->tx_buff);

  /* 清零发送缓冲区 */
  memset(group->tx_buff, 0, 8);
}

/**
 * @brief  多电机（1~4个）直接控制（不含力矩转换，含CAN发送）
 * @param  group: RM电机组结构体指针
 * @retval 无
 * @note   与Group_Motor_Set_Torque不同，直接使用已设置的torque_current_raw值
 */
static void Group_Motor_Ctrl(Motor_RM_Group_t *group)
{
  int16_t torque_current = 0;
  uint8_t Id;

  for(uint8_t i = 0; i < 4; i++)
  {
    if(group->motor[i] != NULL)
    {
      Id = group->motor[i]->born_info->rxId * 2;

      /* 直接使用已设置的原始电流值 */
      torque_current = group->motor[i]->tx_info->torque_current_raw;
      group->tx_buff[Id] = (uint8_t)(torque_current >> 8);
      group->tx_buff[Id + 1] = (uint8_t)(torque_current);
    }
  }

  CAN_SendData(group->hcan, group->stdId, group->tx_buff);
  memset(group->tx_buff, 0, 8);
}

/**
 * @brief  电机组卸力
 * @param  group: RM电机组结构体指针
 * @retval 无
 * @note   将所有电机的力矩值清零
 *         不包含CAN发送操作，需要调用set_torque才会生效
 */
static void Group_Motor_Sleep(Motor_RM_Group_t *group)
{
  for(uint8_t i = 0; i < 4; i++)
  {
    if(group->motor[i] != NULL)
    {
      group->motor[i]->tx_info->torque = 0;  /* 清零力矩 */
    }
  }
}


/**
 * @brief  电机组心跳包
 * @param  group: RM电机组结构体指针
 * @retval 无
 * @note   遍历电机组中最多4个电机，逐个调用心跳检测函数
 */
static void Group_Motor_Heartbeat(Motor_RM_Group_t *group)
{
  for(uint8_t i = 0; i < 4; i++)
  {
    if(group->motor[i] != NULL)
    {
      group->motor[i]->single_heart_beat(group->motor[i]);
    }
  }
}

/**
 * @brief  电机组初始化
 * @param  group: RM电机组结构体指针
 * @retval 无
 * @note   遍历电机组中最多4个非空电机：
 *         1. 为每个电机绑定初始化函数并调用
 *         2. 绑定电机组的控制/心跳/卸力/直接控制函数
 */
void RM_Group_Motor_Init(Motor_RM_Group_t *group)
{
  for(uint8_t i = 0; i < 4; i++)
  {
    if(group->motor[i] != NULL)
    {
      group->motor[i]->single_init = RM_Motor_Init;  /* 绑定初始化函数 */
      group->motor[i]->single_init(group->motor[i]);  /* 初始化电机 */
    }
  }

  /* 绑定电机组操作函数 */
  group->group_set_torque = Group_Motor_Set_Torque;  /* 组力矩控制 */
  group->group_heartbeat = Group_Motor_Heartbeat;    /* 组心跳检测 */
  group->group_sleep = Group_Motor_Sleep;            /* 组卸力 */
  group->group_ctrl = Group_Motor_Ctrl;              /* 组直接控制 */
}

/*============================================================================*/
/*                         工具函数                                            */
/*============================================================================*/

/**
 * @brief  从CAN报文[0][1]中读取电机角度反馈
 * @param  rxData: CAN接收数据缓冲区
 * @retval 编码器位置值（13位，范围0~8191）
 */
static uint16_t CAN_01_GetMotorAngle(uint8_t *rxData)
{
  uint16_t angle;
  angle = (uint16_t)(rxData[0] << 8 | rxData[1]);  /* 大端字节序拼接 */
  return angle;
}

/**
 * @brief  从CAN报文[2][3]中读取电机转子转速反馈
 * @param  rxData: CAN接收数据缓冲区
 * @retval 转子转速（RPM，有符号）
 */
static int16_t CAN_23_GetMotorSpeed(uint8_t *rxData)
{
  int16_t speed;
  speed = (int16_t)(rxData[2] << 8 | rxData[3]);  /* 大端字节序拼接 */
  return speed;
}

/**
 * @brief  从CAN报文[4][5]中读取电机实际转矩电流反馈
 * @param  rxData: CAN接收数据缓冲区
 * @retval 转矩电流值（有符号）
 */
static int16_t CAN_45_GetMotorCurrent(uint8_t *rxData)
{
  int16_t current;
  current = (int16_t)(rxData[4] << 8 | rxData[5]);
  return current;
}

/**
 * @brief  从CAN报文[2][3]中读取电机实际输出转矩
 * @param  rxData: CAN接收数据缓冲区
 * @retval 输出转矩值（有符号）
 */
static int16_t CAN_23_GetMotorTorque(uint8_t *rxData)
{
  int16_t torque;
  torque = ((uint16_t)rxData[2] << 8 | rxData[3]);
  return torque;
}

/**
 * @brief  从CAN报文[4][5]中读取电机实际输出转矩
 * @param  rxData: CAN接收数据缓冲区
 * @retval 输出转矩值（有符号）
 */
static int16_t CAN_45_GetMotorTorque(uint8_t *rxData)
{
  int16_t torque;
  torque = ((uint16_t)rxData[4] << 8 | rxData[5]);
  return torque;
}

/**
 * @brief  从CAN报文[6]中读取电机实际温度
 * @param  rxData: CAN接收数据缓冲区
 * @retval 温度值（摄氏度）
 */
static uint8_t CAN_6_GetMotorTemperature(uint8_t *rxData)
{
  uint8_t temp;
  temp = rxData[6];
  return temp;
}

/**
 * @brief  将电机期望输出扭矩转换为原始电流数据
 * @param  motor: RM电机结构体指针
 * @retval 无
 * @note   根据电机类型进行不同的转换：
 *
 *         3508单电机（无减速箱）：
 *           电流值 = 力矩值（直接使用）
 *           限幅：-16384 ~ 16384
 *
 *         3508减速箱电机：
 *           电流值 = 力矩值 / 转矩常数
 *           限幅：最大电流的90%
 *           原始电流 = (电流值 / 最大电流) * 16384
 *
 *         2006电机：
 *           电流值 = 力矩值（直接使用）
 *           限幅：-10000 ~ 10000
 *           原始电流 = 电流值
 *
 *         6020电机：
 *           电流值 = 力矩值 / 转矩常数
 *           限幅：最大电流
 *           原始电流 = 电流值
 */
static void Torque_to_Raw_Current(Motor_RM_t *motor)
{
  switch(motor->born_info->type)
  {
    /* 3508单电机（无减速箱） */
    case _3508_Single:
      motor->tx_info->torque_current = motor->tx_info->torque;
      /* 最大电流限幅（直接使用力矩值） */
      motor->tx_info->torque_current = constrain(motor->tx_info->torque_current, -16384, 16384);
      /* 单3508转矩电流直接作为原始电流值 */
      motor->tx_info->torque_current_raw = (int16_t)(motor->tx_info->torque_current);
      break;

    /* 3508减速箱电机 */
    case _3508_Reduction:
      /* 通过转矩常数将力矩转换为电流值（安培） */
      motor->tx_info->torque_current = motor->tx_info->torque / _3508_TORQUE_CONSTANT;
      /* 限幅为最大电流的90%（保护电机） */
      motor->tx_info->torque_current = constrain(motor->tx_info->torque_current, -_3508_MAX_CURRENT * 0.9f, _3508_MAX_CURRENT * 0.9f);
      /* 电流值转换为原始电流数值（0~16384对应0~最大电流） */
      motor->tx_info->torque_current_raw = (int16_t)((motor->tx_info->torque_current / _3508_MAX_CURRENT) * 16384.f);
      break;

    /* 2006电机 */
    case _2006_Single:
      motor->tx_info->torque_current = motor->tx_info->torque;
      /* 限幅：-10000 ~ 10000 */
      motor->tx_info->torque_current = constrain(motor->tx_info->torque_current, -10000, 10000);
      /* 2006转矩电流直接作为原始电流值 */
      motor->tx_info->torque_current_raw = (int16_t)(motor->tx_info->torque_current);
      break;

    /* 6020电机 */
    case _6020_Single:
      /* 通过转矩常数将力矩转换为电流值 */
      motor->tx_info->torque_current = motor->tx_info->torque / _6020_TORQUE_CONSTANT;
      /* 限幅：最大电流 */
      motor->tx_info->torque_current = constrain(motor->tx_info->torque_current, -_6020_MAX_CURRENT, _6020_MAX_CURRENT);
      /* 6020转矩电流直接作为原始电流值 */
      motor->tx_info->torque_current_raw = (int16_t)(motor->tx_info->torque_current);
      break;
  }
}


/**
 * @brief  计算RM电机编码器累计值（处理过零点）
 * @param  motor: RM电机结构体指针
 * @retval 无
 * @note   编码器范围为0~8191（13位）
 *         当差值超过4095时认为发生了过零点：
 *         - 正向过零（0->8191）：差值为正但超过4095，修正为-8191+差值
 *         - 反向过零（8191->0）：差值为负且绝对值超过4095，修正为8191+差值
 *         电机刚启动时（上次值和累计值都为0），不计算误差
 */
static void Encoder_Sum_Cal(Motor_RM_t *motor)
{
  int16_t err;
  Motor_RM_Rx_Info_t *motor_info = motor->rx_info;

  /* 电机刚启动时不计算误差 */
  if(motor_info->motor_angle_last == 0 && motor_info->encoder_sum == 0)
  {
    err = 0;
  }
  else
  {
    /* 计算本次与上次编码器值的差 */
    err = motor_info->encoder - motor_info->encoder_last;
  }

  /* 过零点检测与处理 */
  if(abs(err) > 4095)
  {
    /* 发生过零点 */
    if(err >= 0)
      motor_info->encoder_sum += -8191 + err;   /* 正向过零（0↓->8191） */
    else
      motor_info->encoder_sum += 8191 + err;    /* 反向过零（8191↑->0） */
  }
  else
  {
    /* 未过零点，直接累加 */
    motor_info->encoder_sum += err;
  }

  /* 更新上次编码器值 */
  motor_info->encoder_last = motor_info->encoder;
}


/**
 * @brief  将编码器值转换为弧度制，并计算电机角度和
 * @param  motor: RM电机结构体指针
 * @retval 无
 * @note   根据电机类型使用不同的减速比进行转换：
 *         - 3508减速箱：角度 = 编码器值 / 8191 * 2PI / 减速比
 *         - 2006减速箱：角度 = 编码器值 / 8191 * 2PI / 减速比
 *         - 其他电机：  角度 = 编码器值 / 8191 * 2PI
 *         然后调用Angle_Sum_Cal累计多圈角度
 */
static void Encoder_to_Motor_Angle(Motor_RM_t *motor)
{
  if(motor->born_info->type == _3508_Reduction)
    /* 3508减速箱：除以减速比 */
    motor->rx_info->motor_angle = ((float)motor->rx_info->encoder / 8191.f) * (float)PI * 2.f / _3508_REDUCT_RATIO;
  else if(motor->born_info->type == _2006_Single)
    /* 2006减速箱：除以减速比 */
    motor->rx_info->motor_angle = ((float)motor->rx_info->encoder / 8191.f) * (float)PI * 2.f / _2006_REDUCT_RATIO;
  else
    /* 其他电机（如6020）：不除减速比 */
    motor->rx_info->motor_angle = ((float)motor->rx_info->encoder / 8191.f) * (float)PI * 2.f;

  /* 计算多圈累计角度 */
  Angle_Sum_Cal(motor);
}

/**
 * @brief  计算电机旋转角度和（多圈累计）
 * @param  motor: RM电机结构体指针
 * @retval 无
 * @note   与DM_Motor的Angle_Sum_Cal类似：
 *         通过比较当前角度与上次角度的差值来累计总旋转角度
 *         过零点处理针对不同电机类型有不同阈值：
 *         - 3508减速箱：过零阈值为PI/减速比
 *         - 其他电机：过零阈值为PI
 *         order_correction参数支持方向修正
 */
static void Angle_Sum_Cal(Motor_RM_t *motor)
{
  float err = 0.f;
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

  /* 电机刚启动时不计算误差 */
  if(!motor->rx_info->motor_angle_last && !motor->rx_info->motor_angle_sum)
  {
    err = 0.f;
  }
  else
  {
    /* 计算本次与上次的角度差 */
    err = motor->rx_info->motor_angle - motor->rx_info->motor_angle_last;
  }

  /* 过零点检测与处理（针对不同电机类型使用不同阈值） */
  if((abs(err) > ((float)PI)) ||
     (abs(err) > ((float)PI / _3508_REDUCT_RATIO) && motor->born_info->type == _3508_Reduction))
  {
    if(err > 0.f)
    {
      /* 正向过零修正 */
      if(motor->born_info->type == _3508_Reduction)
        motor->rx_info->motor_angle_sum += (-(float)PI * 2.f / _3508_REDUCT_RATIO + err) * order_correction;
      else
        motor->rx_info->motor_angle_sum += (-(float)PI * 2.f + err) * order_correction;
    }
    else
    {
      /* 反向过零修正 */
      if(motor->born_info->type == _3508_Reduction)
        motor->rx_info->motor_angle_sum += ((float)PI * 2.f / _3508_REDUCT_RATIO + err) * order_correction;
      else
        motor->rx_info->motor_angle_sum += ((float)PI * 2.f + err) * order_correction;
    }
  }
  else
  {
    /* 未过零点，直接累加 */
    motor->rx_info->motor_angle_sum += err * order_correction;
  }

  /* 更新上次角度 */
  motor->rx_info->motor_angle_last = motor->rx_info->motor_angle;
}

/**
 * @brief  将电机旋转速度从RPM转换为rad/s
 * @param  motor: RM电机结构体指针
 * @retval 速度值（rad/s）
 * @note   根据电机类型使用不同的减速比：
 *         - 3508减速箱：rad/s = RPM / 60 * 2PI / 减速比
 *         - 2006减速箱：rad/s = RPM / 60 * 2PI / 减速比
 *         - 其他电机：  rad/s = RPM / 60 * 2PI
 */
static float RPM_to_Rads(Motor_RM_t *motor)
{
  float ret;

  if(motor->born_info->type == _3508_Reduction)
    ret = motor->rx_info->encoder_speed / 60.f * 2 * PI / _3508_REDUCT_RATIO;  /* 3508减速箱 */
  else if(motor->born_info->type == _2006_Single)
    ret = motor->rx_info->encoder_speed / 60.f * 2 * PI / _2006_REDUCT_RATIO;  /* 2006减速箱 */
  else
    ret = motor->rx_info->encoder_speed / 60.f * 2 * PI;  /* 其他电机 */

  return ret;
}


/**
 * @brief  将电机接收的原始电流数据转换为实际转矩
 * @param  motor: RM电机结构体指针
 * @retval 无
 * @note   转换公式：
 *         实际电流(A) = 原始电流值 / 16384 * 20A（最大电流）
 *         实际转矩 = 实际电流 * 转矩常数
 *         目前仅针对3508电机的转矩常数计算
 */
static void Raw_Current_to_Torque(Motor_RM_t* motor)
{
  /* 将原始电流值转换为实际电流（安培） */
  motor->rx_info->torque_current = (motor->rx_info->torque_current_raw / 16384.f) * 20.f;

  /* 通过转矩常数将电流转换为力矩 */
  motor->rx_info->torque = motor->rx_info->torque_current * _3508_TORQUE_CONSTANT;
}
