/**
 ******************************************************************************
 * @file    KT_motor.c
 * @brief   KTech（KT）电机驱动文件
 ******************************************************************************
 * @attention
 *
 * 该文件实现了KTech系列电机的完整驱动功能，支持以下控制模式：
 * - 转矩开环控制：控制输出功率
 * - 转矩闭环控制：控制扭矩电流（iq）
 * - 速度闭环控制：控制转速
 * - 角度闭环控制（多种模式）：
 *   - 多圈闭环角度（无/有速度限制）
 *   - 单圈闭环角度（有方向，无/有速度限制）
 *   - 角度增量控制（无/有速度限制）
 *
 * 通信协议：
 *   - 使用CAN总线通信，标准CAN数据帧（8字节）
 *   - 写参数命令：根据命令ID打包不同参数
 *   - 读参数命令：主动查询电机状态
 *   - 多电机控制：最多4个电机通过一个报文同时控制
 *
 * 电机反馈数据：
 *   - 温度、电流、速度、编码器值
 *   - 多圈绝对角度、单圈角度
 *   - 电机状态和错误标志位
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "KT_motor.h"

/* Exported variables --------------------------------------------------------*/
/* 外部CAN句柄声明 */
extern CAN_HandleTypeDef hfdcan1;
extern CAN_HandleTypeDef hfdcan2;

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief 心跳检测函数
 */
void KT_motor_class_heartbeat(KT_motor_t *motor);

/**
 * @brief PID参数初始化函数
 */
void kt_motor_class_pid_init(KT_motor_t *motor);

/**
 * @brief 接收电机信息并更新结构体
 */
void get_kt_motor_info(KT_motor_t *motor, uint8_t *rxBuf);

/**
 * @brief 发送写参数命令
 */
void tx_kt_motor_W_command(KT_motor_t *motor, uint8_t command);

/**
 * @brief 发送读参数命令
 */
void tx_kt_motor_R_command(KT_motor_t *motor, uint8_t command);

/* 参数设置函数前置声明 */
void write_kt_motor_pid_param(KT_motor_t *motor, uint8_t* buff);
void write_kt_motor_accel_param(KT_motor_t *motor, int32_t accel);
void write_kt_motor_encoderOffset_param(KT_motor_t *motor, uint16_t encoderOffset);
void write_kt_motor_powerControl_param(KT_motor_t *motor, int16_t powerControl);
void write_kt_motor_iqControl_param(KT_motor_t *motor, int16_t iqControl);
void write_kt_motor_speedControl_param(KT_motor_t *motor, int32_t speedControl);
void write_kt_motor_angle_sum_Control_param(KT_motor_t *motor, int32_t angle_sum_Control, uint16_t angle_sum_Control_maxSpeed);
void write_kt_motor_angle_single_Control_param(KT_motor_t *motor, uint16_t angle_single_Control, uint8_t angle_single_Control_spinDirection, uint16_t angle_single_Control_maxSpeed);
void write_kt_motor_angle_add_Control_param(KT_motor_t *motor, int32_t angle_add_Control, uint16_t angle_add_Control_maxSpeed);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  KTech电机初始化
 * @param  motor: KTech电机结构体指针
 * @retval 无
 * @note   执行以下初始化操作：
 *         1. 清零发送缓冲区和发送参数结构体
 *         2. 初始化状态信息（离线计数、自保护计数等）
 *         3. 绑定所有函数指针：
 *            - heartbeat: 心跳检测
 *            - get_info: 接收信息
 *            - tx_W_cmd: 写参数命令
 *            - tx_R_cmd: 读参数命令
 *            - W_pid: 写PID参数
 *            - W_accel: 写加速度参数
 *            - W_encoderOffset: 写编码器零点
 *            - W_powerControl: 开环功率控制
 *            - W_iqControl: 闭环转矩控制
 *            - W_speedControl: 速度控制
 *            - W_angle_sum_Control: 多圈角度控制
 *            - W_angle_single_Control: 单圈角度控制
 *            - W_angle_add_Control: 角度增量控制
 */
void KT_motor_class_init(KT_motor_t *motor)
{
  if(motor == NULL)
    return;

  /* 清零发送缓冲区和发送参数 */
  memset((uint8_t*)motor->tx_buff, 0, 8);
  memset(&motor->KT_motor_info.tx_info, 0, 8);

  /* 初始化状态信息 */
  motor->KT_motor_info.state_info.init_flag = M_INIT;                        /* 标记已初始化 */
  motor->KT_motor_info.state_info.offline_cnt_max = OFFLINE_LINE_CNT_MAX;    /* 离线计数最大值 */
  motor->KT_motor_info.state_info.selfprotect_cnt_max = SELFPROTECT_CNT_MAX; /* 自保护计数最大值 */
  motor->KT_motor_info.state_info.offline_cnt = 0;                           /* 离线计数清零 */
  motor->KT_motor_info.state_info.selfprotect_cnt = 0;                       /* 自保护计数清零 */
  motor->KT_motor_info.state_info.work_state = M_OFFLINE;                    /* 初始状态：离线 */
  motor->KT_motor_info.state_info.selfprotect_flag = M_PROTECT_OFF;          /* 自保护标志：关闭 */

  /* 绑定函数指针 */
  motor->heartbeat  = KT_motor_class_heartbeat;  /* 心跳检测 */
  motor->get_info = get_kt_motor_info;            /* 接收信息 */
  motor->tx_W_cmd = tx_kt_motor_W_command;        /* 写参数命令 */
  motor->tx_R_cmd = tx_kt_motor_R_command;        /* 读参数命令 */

  /* 绑定参数设置函数 */
  motor->W_pid                  = write_kt_motor_pid_param;                /* 写PID参数 */
  motor->W_accel                = write_kt_motor_accel_param;              /* 写加速度参数 */
  motor->W_encoderOffset        = write_kt_motor_encoderOffset_param;      /* 写编码器零点 */
  motor->W_powerControl         = write_kt_motor_powerControl_param;       /* 开环功率控制 */
  motor->W_iqControl            = write_kt_motor_iqControl_param;          /* 闭环转矩控制 */
  motor->W_speedControl         = write_kt_motor_speedControl_param;       /* 速度控制 */
  motor->W_angle_sum_Control    = write_kt_motor_angle_sum_Control_param;  /* 多圈角度控制 */
  motor->W_angle_single_Control = write_kt_motor_angle_single_Control_param; /* 单圈角度控制 */
  motor->W_angle_add_Control    = write_kt_motor_angle_add_Control_param;  /* 角度增量控制 */
}

/**
 * @brief  KTech电机心跳检测
 * @param  motor: KTech电机结构体指针
 * @retval 无
 * @note   功能：
 *         1. 离线计数：每次调用加1，超过阈值标记离线
 *         2. 自保护检测：如果连续多次电流值相同，可能进入电流保护状态
 *         3. 状态恢复：离线后收到数据时会自动恢复（由get_kt_motor_info清零计数）
 */
void KT_motor_class_heartbeat(KT_motor_t *motor)
{
  static int16_t current_last;  /* 上一次的电流值（静态变量） */

  if(motor == NULL)
    return;

  KT_motor_state_info_t *state_info = &motor->KT_motor_info.state_info;

  /* 检查是否已初始化 */
  if(state_info->init_flag == M_DEINIT)
  {
    state_info->work_state = M_INIT_ERR;  /* 未初始化，报错 */
    return;
  }

  /* 离线计数递增 */
  state_info->offline_cnt++;

  /* 自保护检测：如果电流值连续不变，可能进入电流保护 */
  if(motor->KT_motor_info.rx_info.current == current_last)
  {
    state_info->selfprotect_cnt++;
  }
  current_last = motor->KT_motor_info.rx_info.current;  /* 更新上次电流值 */

  /* 离线判断 */
  if(state_info->offline_cnt > state_info->offline_cnt_max)
  {
    state_info->offline_cnt = state_info->offline_cnt_max;
    state_info->work_state = M_OFFLINE;  /* 标记离线 */
  }
  else
  {
    if(state_info->work_state == M_OFFLINE)
      state_info->work_state = M_ONLINE;  /* 恢复在线 */
  }

  /* 自保护判断 */
  if(state_info->selfprotect_cnt > state_info->selfprotect_cnt_max)
  {
    state_info->selfprotect_cnt = state_info->selfprotect_cnt_max;
    state_info->selfprotect_flag = M_PROTECT_ON;  /* 开启自保护 */
  }
  else
  {
    if(state_info->selfprotect_flag == M_PROTECT_ON)
    {
      state_info->selfprotect_flag = M_PROTECT_OFF;  /* 关闭自保护 */
      state_info->selfprotect_cnt = 0;
    }
  }
}


/**
 * @brief  KTech电机PID结构体初始化
 * @param  motor: KTech电机结构体指针
 * @retval 无
 * @note   初始化PID发送和接收结构体的默认值
 *         发送默认值：angleKp=10, angleKi=1, speedKp=10, speedKi=1, iqKp=1, iqKi=1
 *         接收值全部初始化为0（等待电机返回实际值）
 */
void kt_motor_class_pid_init(KT_motor_t *motor)
{
  if(motor == NULL)
    return;

  KT_motor_pid_t *pid = &motor->KT_motor_info.pid_info;

  pid->init_flag = M_INIT;  /* 标记PID已初始化 */

  /* 接收PID参数清零（等待电机返回） */
  pid->rx.angleKp = 0;
  pid->rx.angleKi = 0;
  pid->rx.speedKp = 0;
  pid->rx.speedKi = 0;
  pid->rx.iqKp = 0;
  pid->rx.iqKi = 0;

  /* 发送PID默认值 */
  pid->tx.angleKp = 10;    /* 角度环比例增益 */
  pid->tx.angleKi = 1;     /* 角度环积分增益 */
  pid->tx.speedKp = 10;    /* 速度环比例增益 */
  pid->tx.speedKi = 1;     /* 速度环积分增益 */
  pid->tx.iqKp = 1;        /* 电流环比例增益 */
  pid->tx.iqKi = 1;        /* 电流环积分增益 */
}


/*============================================================================*/
/*                         多电机控制                                          */
/*============================================================================*/

/**
 * @brief  多电机转矩电流控制（最多4个电机）
 * @param  iqControl:   转矩电流数组指针，包含最多4个电机的电流值
 * @param  kt_motor_num: 实际电机数量（1~4）
 * @param  drive_type:   CAN总线类型（M_CAN1或M_CAN2）
 * @retval 无
 * @note   功能说明：
 *         1. 首先检查所有电流值是否在有效范围内（-KT_TX_IQ_CONTROL_MAX~+KT_TX_IQ_CONTROL_MAX）
 *         2. 如果实际电机数小于4，将多余位置清零
 *         3. 将4个电机的电流值按小端字节序打包到8字节缓冲区
 *         4. 通过指定的CAN总线发送
 *
 *         数据格式（8字节）：
 *         [0][1]: 电机0电流（16位，小端）
 *         [2][3]: 电机1电流（16位，小端）
 *         [4][5]: 电机2电流（16位，小端）
 *         [6][7]: 电机3电流（16位，小端）
 */
void kt_motor_multi_control(int16_t* iqControl, char kt_motor_num, motor_drive_e drive_type)
{
  /* 检查所有电流值是否在有效范围内 */
  for(int i = 0; i < kt_motor_num; i++)
  {
    if(within_or_not(iqControl[i], -KT_TX_IQ_CONTROL_MAX, KT_TX_IQ_CONTROL_MAX) == Flase)
      return;  /* 超出范围，直接返回不发送 */
  }

  /* 将未使用的电机位置清零 */
  for(int i = kt_motor_num; i < 4; i++)
  {
    if(iqControl[i] != 0)
      iqControl[i] = 0;
  }

  /* 打包8字节数据（小端字节序） */
  uint8_t tx_buff[8] = {0};
  tx_buff[0] = (uint8_t) iqControl[0];          /* 电机0低字节 */
  tx_buff[1] = (uint8_t) (iqControl[0] >> 8);   /* 电机0高字节 */
  tx_buff[2] = (uint8_t) iqControl[1];          /* 电机1低字节 */
  tx_buff[3] = (uint8_t) (iqControl[1] >> 8);   /* 电机1高字节 */
  tx_buff[4] = (uint8_t) iqControl[2];          /* 电机2低字节 */
  tx_buff[5] = (uint8_t) (iqControl[2] >> 8);   /* 电机2高字节 */
  tx_buff[6] = (uint8_t) iqControl[3];          /* 电机3低字节 */
  tx_buff[7] = (uint8_t) (iqControl[3] >> 8);   /* 电机3高字节 */

  /* 根据CAN总线类型发送数据 */
  if(drive_type == M_CAN1)
  {
    CAN_SendData(&hfdcan1, KT_MULTI_TX_ID, tx_buff);  /* 通过CAN1发送 */
  }
  else if(drive_type == M_CAN2)
  {
    CAN_SendData(&hfdcan2, KT_MULTI_TX_ID, tx_buff);  /* 通过CAN2发送 */
  }
  else
    return;  /* 无效的CAN总线类型 */
}


/*============================================================================*/
/*                         单电机控制                                          */
/*============================================================================*/

/**
 * @brief  发送写参数/控制模式命令（单电机）
 * @param  motor:   KTech电机结构体指针
 * @param  command: 命令ID
 * @retval 无
 * @note   支持的命令类型：
 *         - PID_TX_RAM_ID: 写PID参数到RAM
 *         - PID_TX_ROM_ID: 写PID参数到ROM
 *         - ACCEL_TX_ID:   写加速度到RAM
 *         - ZERO_ENCODER_TX_ID: 写编码器值到ROM作为零点
 *         - ZERO_POSNOW_TX_ID:  写当前位置到ROM作为零点
 *         - MOTOR_CLOSE_ID: 关闭电机
 *         - MOTOR_STOP_ID:  停止电机
 *         - MOTOR_RUN_ID:   运行电机
 *         - TORQUE_OPEN_LOOP_ID:  开环转矩控制
 *         - TORQUE_CLOSE_LOOP_ID: 闭环转矩控制
 *         - SPEED_CLOSE_LOOP_ID:  速度闭环控制
 *         - POSI_CLOSE_LOOP_ID1~6: 各种位置闭环控制
 *         根据命令类型打包不同参数到8字节缓冲区
 *         通过指定的CAN总线发送
 */
void tx_kt_motor_W_command(KT_motor_t *motor, uint8_t command)
{
  if(motor == NULL)
    return;

  KT_motor_pid_t     *pid     = &motor->KT_motor_info.pid_info;     /* PID参数 */
  KT_motor_tx_info_t *tx_info = &motor->KT_motor_info.tx_info;      /* 发送参数 */

  memset((uint8_t*)motor->tx_buff, 0, 8);  /* 清零发送缓冲区 */

  switch(command)
  {
    case PID_TX_RAM_ID:  /* 写PID参数到RAM（掉电丢失） */
      motor->tx_buff[0] = PID_TX_RAM_ID;
      motor->tx_buff[1] = 0x00;
      motor->tx_buff[2] = pid->tx.angleKp;   /* 角度环Kp */
      motor->tx_buff[3] = pid->tx.angleKi;   /* 角度环Ki */
      motor->tx_buff[4] = pid->tx.speedKp;   /* 速度环Kp */
      motor->tx_buff[5] = pid->tx.speedKi;   /* 速度环Ki */
      motor->tx_buff[6] = pid->tx.iqKp;      /* 电流环Kp */
      motor->tx_buff[7] = pid->tx.iqKi;      /* 电流环Ki */
    break;

    case PID_TX_ROM_ID:  /* 写PID参数到ROM（掉电保存） */
      motor->tx_buff[0] = PID_TX_RAM_ID;
      motor->tx_buff[1] = 0x00;
      motor->tx_buff[2] = pid->tx.angleKp;
      motor->tx_buff[3] = pid->tx.angleKi;
      motor->tx_buff[4] = pid->tx.speedKp;
      motor->tx_buff[5] = pid->tx.speedKi;
      motor->tx_buff[6] = pid->tx.iqKp;
      motor->tx_buff[7] = pid->tx.iqKi;
    break;

    case ACCEL_TX_ID:  /* 写加速度到RAM */
      motor->tx_buff[0] = ACCEL_TX_ID;
      motor->tx_buff[1] = 0x00;
      motor->tx_buff[2] = 0x00;
      motor->tx_buff[3] = 0x00;
      /* 加速度为32位有符号整数，小端字节序 */
      motor->tx_buff[4] = (uint8_t) tx_info->accel;
      motor->tx_buff[5] = (uint8_t) (tx_info->accel >> 8);
      motor->tx_buff[6] = (uint8_t) (tx_info->accel >> 16);
      motor->tx_buff[7] = (uint8_t) (tx_info->accel >> 24);
    break;

    case ZERO_ENCODER_TX_ID:  /* 写编码器值到ROM作为电机零点 */
      motor->tx_buff[0] = ZERO_ENCODER_TX_ID;
      motor->tx_buff[1] = 0x00;
      motor->tx_buff[2] = 0x00;
      motor->tx_buff[3] = 0x00;
      motor->tx_buff[4] = 0x00;
      motor->tx_buff[5] = 0x00;
      /* 编码器偏移值为16位，小端字节序 */
      motor->tx_buff[6] = (uint8_t) tx_info->encoderOffset;
      motor->tx_buff[7] = (uint8_t) (tx_info->encoderOffset >> 8);
    break;

    case ZERO_POSNOW_TX_ID:  /* 写当前位置到ROM作为零点（减少使用） */
      motor->tx_buff[0] = ZERO_POSNOW_TX_ID;
      motor->tx_buff[1] = 0x00;
      motor->tx_buff[2] = 0x00;
      motor->tx_buff[3] = 0x00;
      motor->tx_buff[4] = 0x00;
      motor->tx_buff[5] = 0x00;
      motor->tx_buff[6] = (uint8_t) tx_info->encoderOffset;
      motor->tx_buff[7] = (uint8_t) (tx_info->encoderOffset >> 8);
    break;

    case MOTOR_CLOSE_ID:  /* 电机关闭命令：清除运行状态和之前收到的指令 */
      motor->tx_buff[0] = MOTOR_CLOSE_ID;
    break;

    case MOTOR_STOP_ID:  /* 电机停止命令：清除运行状态和之前收到的指令 */
      motor->tx_buff[0] = MOTOR_STOP_ID;
    break;

    case MOTOR_RUN_ID:  /* 电机运行命令：从停止中恢复 */
      motor->tx_buff[0] = MOTOR_RUN_ID;
    break;

    case TORQUE_OPEN_LOOP_ID:  /* 开环转矩控制：控制输出功率 */
      motor->tx_buff[0] = TORQUE_OPEN_LOOP_ID;
      motor->tx_buff[1] = 0x00;
      motor->tx_buff[2] = 0x00;
      motor->tx_buff[3] = 0x00;
      /* 功率控制值为16位有符号整数 */
      motor->tx_buff[4] = (uint8_t) tx_info->powerControl;
      motor->tx_buff[5] = (uint8_t) (tx_info->powerControl >> 8);
      motor->tx_buff[6] = 0x00;
      motor->tx_buff[7] = 0x00;
    break;

    case TORQUE_CLOSE_LOOP_ID:  /* 闭环转矩控制：控制扭矩电流（iq） */
      motor->tx_buff[0] = TORQUE_CLOSE_LOOP_ID;
      motor->tx_buff[1] = 0x00;
      motor->tx_buff[2] = 0x00;
      motor->tx_buff[3] = 0x00;
      /* iq控制值为16位有符号整数 */
      motor->tx_buff[4] = *(uint8_t*) (&tx_info->iqControl);
      motor->tx_buff[5] = *((uint8_t*)(&tx_info->iqControl) + 1);
      motor->tx_buff[6] = 0x00;
      motor->tx_buff[7] = 0x00;
    break;

    case SPEED_CLOSE_LOOP_ID:  /* 速度闭环控制 */
      motor->tx_buff[0] = SPEED_CLOSE_LOOP_ID;
      motor->tx_buff[1] = 0x00;
      motor->tx_buff[2] = 0x00;
      motor->tx_buff[3] = 0x00;
      /* 速度控制值为32位有符号整数 */
      motor->tx_buff[4] = (uint8_t) tx_info->speedControl;
      motor->tx_buff[5] = (uint8_t) (tx_info->speedControl >> 8);
      motor->tx_buff[6] = (uint8_t) (tx_info->speedControl >> 16);
      motor->tx_buff[7] = (uint8_t) (tx_info->speedControl >> 24);
    break;

    case POSI_CLOSE_LOOP_ID1:  /* 多圈角度闭环，无速度限制 */
      motor->tx_buff[0] = POSI_CLOSE_LOOP_ID1;
      motor->tx_buff[1] = 0x00;
      motor->tx_buff[2] = 0x00;
      motor->tx_buff[3] = 0x00;
      motor->tx_buff[4] = (uint8_t) tx_info->angle_sum_Control;
      motor->tx_buff[5] = (uint8_t) (tx_info->angle_sum_Control >> 8);
      motor->tx_buff[6] = (uint8_t) (tx_info->angle_sum_Control >> 16);
      motor->tx_buff[7] = (uint8_t) (tx_info->angle_sum_Control >> 24);
    break;

    case POSI_CLOSE_LOOP_ID2:  /* 多圈角度闭环，有速度限制 */
      motor->tx_buff[0] = POSI_CLOSE_LOOP_ID2;
      motor->tx_buff[1] = 0x00;
      /* 最大速度限制（16位）+ 多圈角度（32位） */
      motor->tx_buff[2] = (uint8_t) tx_info->angle_sum_Control_maxSpeed;
      motor->tx_buff[3] = (uint8_t) (tx_info->angle_sum_Control_maxSpeed >> 8);
      motor->tx_buff[4] = (uint8_t) tx_info->angle_sum_Control;
      motor->tx_buff[5] = (uint8_t) (tx_info->angle_sum_Control >> 8);
      motor->tx_buff[6] = (uint8_t) (tx_info->angle_sum_Control >> 16);
      motor->tx_buff[7] = (uint8_t) (tx_info->angle_sum_Control >> 24);
    break;

    case POSI_CLOSE_LOOP_ID3:  /* 单圈角度，有方向 */
      motor->tx_buff[0] = POSI_CLOSE_LOOP_ID3;
      motor->tx_buff[1] = (uint8_t) tx_info->angle_single_Control_spinDirection;  /* 旋转方向 */
      motor->tx_buff[2] = 0x00;
      motor->tx_buff[3] = 0x00;
      motor->tx_buff[4] = (uint8_t) tx_info->angle_single_Control;     /* 目标角度（16位） */
      motor->tx_buff[5] = (uint8_t) (tx_info->angle_single_Control >> 8);
      motor->tx_buff[6] = 0x00;
      motor->tx_buff[7] = 0x00;
    break;

    case POSI_CLOSE_LOOP_ID4:  /* 单圈角度，有方向，有最高转速 */
      motor->tx_buff[0] = POSI_CLOSE_LOOP_ID4;
      motor->tx_buff[1] = (uint8_t) tx_info->angle_single_Control_spinDirection;  /* 旋转方向 */
      motor->tx_buff[2] = (uint8_t) tx_info->angle_single_Control_maxSpeed;      /* 最大转速（16位） */
      motor->tx_buff[3] = (uint8_t) (tx_info->angle_single_Control_maxSpeed >> 8);
      motor->tx_buff[4] = (uint8_t) tx_info->angle_single_Control;     /* 目标角度（16位） */
      motor->tx_buff[5] = (uint8_t) (tx_info->angle_single_Control >> 8);
      motor->tx_buff[6] = 0x00;
      motor->tx_buff[7] = 0x00;
    break;

    case POSI_CLOSE_LOOP_ID5:  /* 角度增量控制，无速度限制 */
      motor->tx_buff[0] = POSI_CLOSE_LOOP_ID5;
      motor->tx_buff[1] = 0x00;
      motor->tx_buff[2] = 0x00;
      motor->tx_buff[3] = 0x00;
      motor->tx_buff[4] = (uint8_t) tx_info->angle_add_Control;        /* 角度增量（32位） */
      motor->tx_buff[5] = (uint8_t) (tx_info->angle_add_Control >> 8);
      motor->tx_buff[6] = (uint8_t) (tx_info->angle_add_Control >> 16);
      motor->tx_buff[7] = (uint8_t) (tx_info->angle_add_Control >> 24);
    break;

    case POSI_CLOSE_LOOP_ID6:  /* 角度增量控制，有速度限制 */
      motor->tx_buff[0] = POSI_CLOSE_LOOP_ID6;
      motor->tx_buff[1] = 0x00;
      /* 最大速度限制（16位）+ 角度增量（32位） */
      motor->tx_buff[2] = (uint8_t) tx_info->angle_add_Control_maxSpeed;
      motor->tx_buff[3] = (uint8_t) (tx_info->angle_add_Control_maxSpeed >> 8);
      motor->tx_buff[4] = (uint8_t) tx_info->angle_add_Control;
      motor->tx_buff[5] = (uint8_t) (tx_info->angle_add_Control >> 8);
      motor->tx_buff[6] = (uint8_t) (tx_info->angle_add_Control >> 16);
      motor->tx_buff[7] = (uint8_t) (tx_info->angle_add_Control >> 24);
    break;

    default:
      break;
  }

  /* 根据CAN总线类型发送命令 */
  if(motor->KT_motor_info.id.drive_type == M_CAN1)
  {
    CAN_SendData(&hfdcan1, motor->KT_motor_info.id.tx_id, motor->tx_buff);
  }
  else if(motor->KT_motor_info.id.drive_type == M_CAN2)
  {
    CAN_SendData(&hfdcan2, motor->KT_motor_info.id.tx_id, motor->tx_buff);
  }
  else
    return;
}


/**
 * @brief  发送读参数命令（单电机）
 * @param  motor:   KTech电机结构体指针
 * @param  command: 读取命令ID
 * @retval 无
 * @note   支持的读取命令：
 *         - PID_RX_ID:       读取PID参数
 *         - ACCEL_RX_ID:     读取加速度参数
 *         - ENCODER_RX_ID:   读取编码器数据
 *         - MOTOR_ANGLE_ID:  读取多圈绝对角度
 *         - CIRCLE_ANGLE_ID: 读取单圈角度
 *         - STATE1_ID:       读取电机状态1和错误标志位
 *         - STATE2_ID:       读取电机状态2
 *         - STATE3_ID:       读取电机状态3
 *         命令报文只需第1字节为命令ID即可
 */
void tx_kt_motor_R_command(KT_motor_t *motor, uint8_t command)
{
  if(motor == NULL)
    return;

  memset((uint8_t*)motor->tx_buff, 0, 8);  /* 清零发送缓冲区 */

  switch(command)
  {
    case PID_RX_ID:        /* 读取PID参数 */
      motor->tx_buff[0] = PID_RX_ID;
    break;

    case ACCEL_RX_ID:      /* 读取加速度参数 */
      motor->tx_buff[0] = ACCEL_RX_ID;
    break;

    case ENCODER_RX_ID:    /* 读取编码器数据 */
      motor->tx_buff[0] = ENCODER_RX_ID;
    break;

    case MOTOR_ANGLE_ID:   /* 读取电机多圈绝对角度 */
      motor->tx_buff[0] = MOTOR_ANGLE_ID;
    break;

    case CIRCLE_ANGLE_ID:  /* 读取电机单圈角度 */
      motor->tx_buff[0] = CIRCLE_ANGLE_ID;
    break;

    case STATE1_ID:        /* 读取电机状态1和错误标志位 */
      motor->tx_buff[0] = STATE1_ID;
    break;

    case STATE2_ID:        /* 读取电机状态2 */
      motor->tx_buff[0] = STATE2_ID;
    break;

    case STATE3_ID:        /* 读取电机状态3 */
      motor->tx_buff[0] = STATE3_ID;
    break;

    default:
      break;
  }

  /* 根据CAN总线类型发送命令 */
  if(motor->KT_motor_info.id.drive_type == M_CAN1)
  {
    CAN_SendData(&hfdcan1, motor->KT_motor_info.id.tx_id, motor->tx_buff);
  }
  else if(motor->KT_motor_info.id.drive_type == M_CAN2)
  {
    CAN_SendData(&hfdcan2, motor->KT_motor_info.id.tx_id, motor->tx_buff);
  }
  else
    return;
}



/**
 * @brief  接收电机信息并自动更新结构体
 * @param  motor: KTech电机结构体指针
 * @param  rxBuf: CAN接收数据缓冲区（8字节）
 * @retval 无
 * @note   根据接收数据的命令ID（rxBuf[0]）解析不同类型的数据：
 *         - PID相关：解析PID参数
 *         - 加速度/编码器：解析相应参数
 *         - 多圈/单圈角度：解析角度值
 *         - 状态1/2/3：解析温度、电压、电流、速度、编码器、错误码
 *         - 控制命令反馈：解析温度、电流、速度、编码器
 *         每次收到有效数据后清零离线计数器
 */
void get_kt_motor_info(KT_motor_t *motor, uint8_t *rxBuf)
{
  if(motor == NULL || rxBuf == NULL)
  {
    motor->KT_motor_info.state_info.work_state = M_DATA_ERR;  /* 数据错误 */
    return;
  }

  uint8_t ID = rxBuf[0];  /* 获取命令ID */

  KT_motor_pid_rx_info_t *pid_rx_info = &motor->KT_motor_info.pid_info.rx;  /* PID接收信息 */
  KT_motor_rx_info_t     *rx_info     = &motor->KT_motor_info.rx_info;       /* 接收信息 */
  KT_motor_state_info_t  *state_info  = &motor->KT_motor_info.state_info;    /* 状态信息 */

  /* 收到有效数据，清零离线计数器，标记在线 */
  state_info->offline_cnt = 0;
  state_info->work_state = M_ONLINE;

  switch (ID)
  {
    case PID_RX_ID:  /* 主动读取PID参数返回 */
      pid_rx_info->angleKp = rxBuf[2];
      pid_rx_info->angleKi = rxBuf[3];
      pid_rx_info->speedKp = rxBuf[4];
      pid_rx_info->speedKi = rxBuf[5];
      pid_rx_info->iqKp    = rxBuf[6];
      pid_rx_info->iqKi    = rxBuf[7];
    break;

    case PID_TX_RAM_ID:  /* 发送PID参数到RAM时返回 */
    case PID_TX_ROM_ID:  /* 发送PID参数到ROM时返回 */
      pid_rx_info->angleKp = rxBuf[2];
      pid_rx_info->angleKi = rxBuf[3];
      pid_rx_info->speedKp = rxBuf[4];
      pid_rx_info->speedKi = rxBuf[5];
      pid_rx_info->iqKp    = rxBuf[6];
      pid_rx_info->iqKi    = rxBuf[7];
    break;

    case ACCEL_RX_ID:  /* 主动读取加速度返回 */
    case ACCEL_TX_ID:  /* 发送加速度到RAM时返回 */
      /* 加速度为32位有符号整数，小端字节序拼接 */
      rx_info->accel  = (int32_t)rxBuf[7];
      rx_info->accel <<= 8;
      rx_info->accel |= (int32_t)rxBuf[6];
      rx_info->accel <<= 8;
      rx_info->accel |= (int32_t)rxBuf[5];
      rx_info->accel <<= 8;
      rx_info->accel |= (int32_t)rxBuf[4];
      rx_info->accel <<= 8;
    break;

    case ENCODER_RX_ID:  /* 主动读取编码器数据返回 */
      /* 编码器值、原始值、偏移值均为16位 */
      rx_info->encoder = (uint16_t)rxBuf[3];
      rx_info->encoder <<= 8;
      rx_info->encoder |= (uint16_t)rxBuf[2];
      rx_info->encoderRaw = (uint16_t)rxBuf[5];
      rx_info->encoderRaw <<= 8;
      rx_info->encoderRaw |= (uint16_t)rxBuf[4];
      rx_info->encoderOffset = (uint16_t)rxBuf[7];
      rx_info->encoderOffset <<= 8;
      rx_info->encoderOffset |= (uint16_t)rxBuf[6];
    break;

    case ZERO_ENCODER_TX_ID:  /* 写编码器值到ROM返回 */
    case ZERO_POSNOW_TX_ID:  /* 写当前位置到ROM返回 */
      rx_info->encoderOffset = (uint16_t)rxBuf[7];
      rx_info->encoderOffset <<= 8;
      rx_info->encoderOffset |= (uint16_t)rxBuf[6];
    break;


    case MOTOR_ANGLE_ID:  /* 读取多圈绝对角度返回 */
      /* 多圈角度为64位有符号整数，小端字节序拼接 */
      rx_info->motorAngle  = (int64_t)rxBuf[7];
      rx_info->motorAngle <<= 8;
      rx_info->motorAngle |= (int64_t)rxBuf[6];
      rx_info->motorAngle <<= 8;
      rx_info->motorAngle |= (int64_t)rxBuf[5];
      rx_info->motorAngle <<= 8;
      rx_info->motorAngle |= (int64_t)rxBuf[4];
      rx_info->motorAngle <<= 8;
      rx_info->motorAngle |= (int64_t)rxBuf[3];
      rx_info->motorAngle <<= 8;
      rx_info->motorAngle |= (int64_t)rxBuf[2];
      rx_info->motorAngle <<= 8;
      rx_info->motorAngle |= (int64_t)rxBuf[1];
    break;

    case CIRCLE_ANGLE_ID:  /* 读取单圈角度返回 */
      /* 单圈角度为32位无符号整数 */
      rx_info->circleAngle  = (uint32_t)rxBuf[7];
      rx_info->circleAngle <<= 8;
      rx_info->circleAngle |= (uint32_t)rxBuf[6];
      rx_info->circleAngle <<= 8;
      rx_info->circleAngle |= (uint32_t)rxBuf[5];
      rx_info->circleAngle <<= 8;
      rx_info->circleAngle |= (uint32_t)rxBuf[4];
    break;

    case STATE1_ID:  /* 读取电机状态1：温度、电压、错误标志位 */
      rx_info->temperature = (int8_t)rxBuf[1];      /* 温度（8位有符号） */
      rx_info->voltage = (uint16_t)rxBuf[4];         /* 电压（16位无符号） */
      rx_info->voltage <<= 8;
      rx_info->voltage |= (uint16_t)rxBuf[3];
      rx_info->errorState = rxBuf[7];                /* 错误状态标志 */
    break;

    case STATE2_ID:  /* 读取电机状态2：温度、电流、速度、编码器 */
      rx_info->temperature = (int8_t)rxBuf[1];
      rx_info->current = (int16_t)rxBuf[3];          /* 电流（16位有符号） */
      rx_info->current <<= 8;
      rx_info->current |= (int16_t)rxBuf[2];
      rx_info->speed = (int16_t)rxBuf[5];            /* 速度（16位有符号） */
      rx_info->speed <<= 8;
      rx_info->speed |= (int16_t)rxBuf[4];
      rx_info->encoder = (uint16_t)rxBuf[7];         /* 编码器值（16位无符号） */
      rx_info->encoder <<= 8;
      rx_info->encoder |= (uint16_t)rxBuf[6];
    break;

    case STATE3_ID:  /* 读取电机状态3：温度、三相电流 */
      rx_info->temperature = (int8_t)rxBuf[1];
      rx_info->current_A = (int16_t)rxBuf[3];        /* A相电流 */
      rx_info->current_A <<= 8;
      rx_info->current_A |= (int16_t)rxBuf[2];
      rx_info->current_B = (int16_t)rxBuf[5];        /* B相电流 */
      rx_info->current_B <<= 8;
      rx_info->current_B |= (int16_t)rxBuf[4];
      rx_info->current_C = (int16_t)rxBuf[7];        /* C相电流 */
      rx_info->current_C <<= 8;
      rx_info->current_C |= (int16_t)rxBuf[6];
    break;

    case TORQUE_OPEN_LOOP_ID:  /* 扭矩开环控制返回 */
      rx_info->temperature = (int8_t)rxBuf[1];
      rx_info->powerControl = (int16_t)rxBuf[3];     /* 功率控制值 */
      rx_info->powerControl <<= 8;
      rx_info->powerControl |= (int16_t)rxBuf[2];
      rx_info->speed = (int16_t)rxBuf[5];
      rx_info->speed <<= 8;
      rx_info->speed |= (int16_t)rxBuf[4];
      rx_info->encoder = (uint16_t)rxBuf[7];
      rx_info->encoder <<= 8;
      rx_info->encoder |= (uint16_t)rxBuf[6];
    break;

    /* 以下控制模式返回的数据格式相同：温度、电流、速度、编码器 */
    case TORQUE_CLOSE_LOOP_ID:   /* 扭矩闭环控制返回 */
    case SPEED_CLOSE_LOOP_ID:    /* 速度闭环控制返回 */
    case POSI_CLOSE_LOOP_ID1:    /* 多圈角度闭环返回 */
    case POSI_CLOSE_LOOP_ID2:
    case POSI_CLOSE_LOOP_ID3:    /* 单圈角度闭环返回 */
    case POSI_CLOSE_LOOP_ID4:
    case POSI_CLOSE_LOOP_ID5:    /* 角度增量控制返回 */
    case POSI_CLOSE_LOOP_ID6:
      rx_info->temperature = (int8_t)rxBuf[1];
      rx_info->current = (int16_t)rxBuf[3];          /* 扭矩电流 */
      rx_info->current <<= 8;
      rx_info->current |= (int16_t)rxBuf[2];
      rx_info->speed = (int16_t)rxBuf[5];            /* 转速 */
      rx_info->speed <<= 8;
      rx_info->speed |= (int16_t)rxBuf[4];
      rx_info->encoder = (uint16_t)rxBuf[7];         /* 编码器值 */
      rx_info->encoder <<= 8;
      rx_info->encoder |= (uint16_t)rxBuf[6];
    break;

    default:
      break;
  }
}

/**
 * @brief  写电机PID发送参数（通过缓冲区）
 * @param  motor: KTech电机结构体指针
 * @param  buff:  PID参数数组 [angleKp, angleKi, speedKp, speedKi, iqKp, iqKi]
 * @retval 无
 * @note   将缓冲区中的6个PID参数写入电机发送PID结构体
 */
void write_kt_motor_pid_param(KT_motor_t *motor, uint8_t* buff)
{
  if(motor == NULL || buff == NULL)
    return;

  KT_motor_pid_t *pid = &motor->KT_motor_info.pid_info;

  /* 逐个设置PID参数 */
  pid->tx.angleKp = buff[0];    /* 角度环比例增益 */
  pid->tx.angleKi = buff[1];    /* 角度环积分增益 */
  pid->tx.speedKp = buff[2];    /* 速度环比例增益 */
  pid->tx.speedKi = buff[3];    /* 速度环积分增益 */
  pid->tx.iqKp    = buff[4];    /* 电流环比例增益 */
  pid->tx.iqKi    = buff[5];    /* 电流环积分增益 */
}


/**
 * @brief  写电机加速度参数
 * @param  motor: KTech电机结构体指针
 * @param  accel: 加速度值（32位有符号整数）
 * @retval 无
 */
void write_kt_motor_accel_param(KT_motor_t *motor, int32_t accel)
{
  if(motor == NULL)
    return;

  motor->KT_motor_info.tx_info.accel = accel;
}


/**
 * @brief  写电机编码器零点参数
 * @param  motor:         KTech电机结构体指针
 * @param  encoderOffset: 编码器偏移值（16位无符号）
 * @retval 无
 * @note   编码器偏移值超过KT_TX_ENCODER_OFFSET_MAX时拒绝设置
 */
void write_kt_motor_encoderOffset_param(KT_motor_t *motor, uint16_t encoderOffset)
{
  if(motor == NULL)
    return;

  /* 无符号数据不能使用within_or_not，直接比较上限 */
  if(encoderOffset > KT_TX_ENCODER_OFFSET_MAX)
    return;

  motor->KT_motor_info.tx_info.encoderOffset = encoderOffset;
}

/**
 * @brief  写电机开环功率控制参数
 * @param  motor:        KTech电机结构体指针
 * @param  powerControl: 功率控制值（16位有符号，范围-KT_TX_POWER_CONTROL_MAX~+KT_TX_POWER_CONTROL_MAX）
 * @retval 无
 */
void write_kt_motor_powerControl_param(KT_motor_t *motor, int16_t powerControl)
{
  if(motor == NULL)
    return;

  /* 范围检查 */
  if(within_or_not(powerControl, -KT_TX_POWER_CONTROL_MAX, KT_TX_POWER_CONTROL_MAX) == Flase)
    return;

  motor->KT_motor_info.tx_info.powerControl = powerControl;
}

/**
 * @brief  写电机闭环转矩电流控制参数
 * @param  motor:     KTech电机结构体指针
 * @param  iqControl: 扭矩电流值（16位有符号，范围-KT_TX_IQ_CONTROL_MAX~+KT_TX_IQ_CONTROL_MAX）
 * @retval 无
 * @note   使用constrain函数限制在有效范围内
 */
void write_kt_motor_iqControl_param(KT_motor_t *motor, int16_t iqControl)
{
  if(motor == NULL)
    return;

  iqControl = constrain(iqControl, -KT_TX_IQ_CONTROL_MAX, KT_TX_IQ_CONTROL_MAX);

  motor->KT_motor_info.tx_info.iqControl = iqControl;
}


/**
 * @brief  写电机速度控制参数
 * @param  motor:       KTech电机结构体指针
 * @param  speedControl: 速度目标值（32位有符号）
 * @retval 无
 */
void write_kt_motor_speedControl_param(KT_motor_t *motor, int32_t speedControl)
{
  if(motor == NULL)
    return;

  motor->KT_motor_info.tx_info.speedControl = speedControl;
}

/**
 * @brief  写电机多圈角度控制参数
 * @param  motor:                    KTech电机结构体指针
 * @param  angle_sum_Control:        多圈角度目标值（32位有符号）
 * @param  angle_sum_Control_maxSpeed: 最大速度限制（16位无符号）
 * @retval 无
 */
void write_kt_motor_angle_sum_Control_param(KT_motor_t *motor,
                                            int32_t angle_sum_Control,
                                            uint16_t angle_sum_Control_maxSpeed)
{
  if(motor == NULL)
    return;

  motor->KT_motor_info.tx_info.angle_sum_Control = angle_sum_Control;
  motor->KT_motor_info.tx_info.angle_sum_Control_maxSpeed = angle_sum_Control_maxSpeed;
}


/**
 * @brief  写电机单圈角度控制参数
 * @param  motor:    KTech电机结构体指针
 * @param  angle_single_Control: 目标角度值（16位无符号）
 * @param  angle_single_Control_spinDirection: 旋转方向
 * @param  angle_single_Control_maxSpeed: 最大速度限制（16位无符号）
 * @retval 无
 */
void write_kt_motor_angle_single_Control_param(KT_motor_t *motor,
                                               uint16_t angle_single_Control,
                                               uint8_t angle_single_Control_spinDirection,
                                               uint16_t angle_single_Control_maxSpeed)
{
  motor->KT_motor_info.tx_info.angle_single_Control = angle_single_Control;
  motor->KT_motor_info.tx_info.angle_single_Control_spinDirection = angle_single_Control_spinDirection;
  motor->KT_motor_info.tx_info.angle_single_Control_maxSpeed = angle_single_Control_maxSpeed;
}

/**
 * @brief  写电机角度增量控制参数
 * @param  motor:    KTech电机结构体指针
 * @param  angle_add_Control: 角度增量目标值（32位有符号）
 * @param  angle_add_Control_maxSpeed: 最大速度限制（16位无符号）
 * @retval 无
 */
void write_kt_motor_angle_add_Control_param(KT_motor_t *motor,
                                            int32_t angle_add_Control,
                                            uint16_t angle_add_Control_maxSpeed)
{
  if(motor == NULL)
    return;

  motor->KT_motor_info.tx_info.angle_add_Control = angle_add_Control;
  motor->KT_motor_info.tx_info.angle_add_Control_maxSpeed = angle_add_Control_maxSpeed;
}
