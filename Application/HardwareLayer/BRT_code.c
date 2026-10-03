/**
 ******************************************************************************
 * @file    BRT_code.c
 * @brief   BRT系列CAN总线绝对值编码器驱动文件
 ******************************************************************************
 * @attention
 *
 * 该文件实现了BRT系列CAN总线绝对值编码器的完整驱动功能，包括：
 * - 编码器角度值读取（单圈/多圈/角速度）
 * - 编码器参数配置（ID、波特率、零点、方向等）
 * - 编码器数据解析与更新
 * - 心跳检测与离线判断
 *
 * BRT编码器通过CAN总线通信，采用自定义协议：
 * - 发送缓冲区格式：[数据长度] [0x01] [命令码] [参数...] [保留]
 * - 接收数据包含编码器的当前角度、速度、多圈计数等信息
 *
 * 支持的命令：
 *   Read_encoder      - 读取编码器值
 *   Read_speed        - 读取角速度
 *   Read_sum_encoder  - 读取虚拟多圈值
 *   Read_Turn         - 读取虚拟圈数
 *   Set_Id            - 设置编码器ID
 *   Set_Baud          - 设置通信波特率
 *   Set_Zero_Pole     - 设置零点
 *   Set_Mid_Pole      - 设置中点
 *   Set_Num_Pole      - 设置当前编码器值
 *   Set_Code_Mode     - 设置编码器工作模式
 *   Set_Dire          - 设置递增方向
 *   Set_Sample_Time   - 设置采样速率
 *   Set_Receive_Time  - 设置主动回传时间
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "brt_code.h"
#include "rp_math.h"     /* 数学工具函数（如constrain等） */
#include "arm_math.h"    /* ARM DSP数学库（PI常量等） */

/* Private function prototypes -----------------------------------------------*/
static void Code_Set_Command(Code_BRT_t* code, Code_BRT_Command_e command);
static void Code_Send_data(Code_BRT_t* code);

/**
 * @brief  主动读取编码器角度值
 * @param  code: BRT编码器结构体指针
 * @retval 无
 * @note   通过CAN总线发送读取编码器命令
 *         使用command_flag防止重复发送：
 *         - flag=0：可以发送新命令
 *         - flag=1：已发送，等待响应
 *         超时机制：100次调用未收到响应则重置flag
 */
static void Code_Read_Angle(Code_BRT_t* code)
{
  static uint8_t over_time = 0;  /* 超时计数器 */

  /* 如果当前没有等待响应的读取命令，则发送新命令 */
  if(code->tx_info->command_flag[Read_encoder] != 1)
  {
    /* 设置命令类型 */
    Code_Set_Command(code, Read_encoder);
    /* 第4字节为保留参数，设为0x00 */
    code->tx_info->tx_buff[3] = 0x00;
    /* 标记命令已发送，等待响应 */
    code->tx_info->command_flag[Read_encoder] = 1;
    /* 通过CAN总线发送数据 */
    Code_Send_data(code);
  }
  else
  {
    /* 已发送命令但未收到响应，累加超时计数 */
    over_time++;
    if(over_time > 100)  /* 超时阈值：100次调用 */
    {
      over_time = 0;
      /* 重置命令标志，允许重新发送 */
      code->tx_info->command_flag[Read_encoder] = 0;
    }
  }
}

/**
 * @brief  主动读取编码器角速度值
 * @param  code: BRT编码器结构体指针
 * @retval 无
 * @note   与Code_Read_Angle类似的超时等待机制
 */
static void Code_Read_Speed(Code_BRT_t* code)
{
  static uint8_t over_time = 0;

  if(code->tx_info->command_flag[Read_speed] != 1)
  {
    Code_Set_Command(code, Read_speed);
    code->tx_info->tx_buff[3] = 0x00;
    code->tx_info->command_flag[Read_speed] = 1;
    Code_Send_data(code);
  }
  else
  {
    over_time++;
    if(over_time > 100)
    {
      over_time = 0;
      code->tx_info->command_flag[Read_speed] = 0;
    }
  }
}

/**
 * @brief  主动读取编码器虚拟多圈值
 * @param  code: BRT编码器结构体指针
 * @retval 无
 * @note   读取编码器累计的多圈角度值（有符号32位整数）
 *         用于需要精确多圈位置信息的应用场景
 */
static void Code_Read_Sum_Angle(Code_BRT_t* code)
{
  static uint8_t over_time = 0;

  if(code->tx_info->command_flag[Read_sum_encoder] != 1)
  {
    Code_Set_Command(code, Read_sum_encoder);
    code->tx_info->tx_buff[3] = 0x00;
    code->tx_info->command_flag[Read_sum_encoder] = 1;
    Code_Send_data(code);
  }
  else
  {
    over_time++;
    if(over_time > 100)
    {
      over_time = 0;
      code->tx_info->command_flag[Read_sum_encoder] = 0;
    }
  }
}


/**
 * @brief  主动读取编码器虚拟圈数
 * @param  code: BRT编码器结构体指针
 * @retval 无
 * @note   读取编码器累计的转动圈数
 */
static void Code_Read_Turn(Code_BRT_t* code)
{
  static uint8_t over_time = 0;

  if(code->tx_info->command_flag[Read_Turn] != 1)
  {
    Code_Set_Command(code, Read_Turn);
    code->tx_info->tx_buff[3] = 0x00;
    code->tx_info->command_flag[Read_Turn] = 1;
    Code_Send_data(code);
  }
  else
  {
    over_time++;
    if(over_time > 100)
    {
      over_time = 0;
      code->tx_info->command_flag[Read_Turn] = 0;
    }
  }
}

/**
 * @brief  设置编码器主动回传时间
 * @param  code: BRT编码器结构体指针
 * @param  Time: 回传时间，单位为微秒(us)，取值范围：50~65535
 * @retval 无
 * @note   设置后编码器会以指定间隔自动发送角度数据
 *         无需主机主动查询，降低通信延迟
 */
static void Code_Set_Receive_Time(Code_BRT_t* code, uint16_t Time)
{
  static uint8_t over_time = 0;

  if(code->tx_info->command_flag[Set_Receive_Time] == 0)
  {
    Code_Set_Command(code, Set_Receive_Time);
    /* 将时间限制在有效范围内 */
    Time = constrain(Time, 50, 65535);
    /* 将16位时间值拆分为两个字节（低字节在前，高字节在后） */
    code->tx_info->tx_buff[3] = (uint8_t)(Time);         /* 低8位 */
    code->tx_info->tx_buff[4] = (uint8_t)(Time >> 8);    /* 高8位 */
    code->tx_info->command_flag[Set_Receive_Time] = 1;
    Code_Send_data(code);
  }
  else if(code->tx_info->command_flag[Set_Receive_Time] == 1)
  {
    over_time++;
    if(over_time > 100)
    {
      over_time = 0;
      code->tx_info->command_flag[Set_Receive_Time] = 0;
    }
  }
}


/**
 * @brief  设置编码器ID
 * @param  code: BRT编码器结构体指针
 * @param  id:   新的编码器ID，取值范围：1~255
 * @retval 无
 * @note   修改编码器的CAN通信ID
 *         设置后需要等待编码器返回确认消息
 *         确认后更新born_info中的stdId
 */
static void Code_Set_ID(Code_BRT_t* code, uint8_t id)
{
  static uint8_t over_time = 0;

  if(code->tx_info->command_flag[Set_Id] == 0)
  {
    Code_Set_Command(code, Set_Id);
    /* 将ID限制在有效范围内 */
    id = constrain(id, 1, 255);
    code->tx_info->tx_buff[3] = (uint8_t)(id);
    code->tx_info->command_flag[Set_Id] = 1;
    /* 保存新ID，等待确认后生效 */
    code->tx_info->new_id = id;
    Code_Send_data(code);
  }
  else if(code->tx_info->command_flag[Set_Id] == 1)
  {
    /* 等待确认中... */
    over_time++;
    if(over_time > 100)
    {
      over_time = 0;
      code->tx_info->command_flag[Set_Id] = 0;
    }
  }
  else if(code->tx_info->command_flag[Set_Id] == 2)
  {
    /* 收到确认，更新编码器ID */
    code->born_info->stdId = code->tx_info->new_id;
  }
  else
  {
    /* 其他状态，设为3（异常状态） */
    code->tx_info->command_flag[Set_Id] = 3;
  }
}

/**
 * @brief  设置编码器CAN通信波特率
 * @param  code: BRT编码器结构体指针
 * @param  Baud: 波特率选择，取值范围：0~4
 *               0: 500Kbps
 *               1: 1Mbps
 *               2: 250Kbps
 *               3: 125Kbps
 *               4: 100Kbps
 * @retval 无
 * @note   修改波特率后需要主机也相应修改波特率才能继续通信
 */
static void Code_Set_Baud(Code_BRT_t* code, uint8_t Baud)
{
  static uint8_t over_time = 0;

  if(code->tx_info->command_flag[Set_Baud] == 0)
  {
    Code_Set_Command(code, Set_Baud);
    Baud = constrain(Baud, 0, 4);
    code->tx_info->tx_buff[3] = Baud;
    code->tx_info->command_flag[Set_Baud] = 1;
    Code_Send_data(code);
  }
  else if(code->tx_info->command_flag[Set_Baud] == 1)
  {
    over_time++;
    if(over_time > 100)
    {
      over_time = 0;
      code->tx_info->command_flag[Set_Baud] = 0;
    }
  }
}

/**
 * @brief  设置编码器零点位置
 * @param  code: BRT编码器结构体指针
 * @retval 无
 * @note   将编码器当前位置设置为零点（角度归零）
 *         通常在机械安装完成后执行一次
 */
static void Code_Set_Zero_Point(Code_BRT_t* code)
{
  static uint8_t over_time = 0;

  if(code->tx_info->command_flag[Set_Zero_Pole] == 0)
  {
    Code_Set_Command(code, Set_Zero_Pole);
    code->tx_info->tx_buff[3] = 0;
    code->tx_info->command_flag[Set_Zero_Pole] = 1;
    Code_Send_data(code);
  }
  else if(code->tx_info->command_flag[Set_Zero_Pole] == 1)
  {
    over_time++;
    if(over_time > 100)
    {
      over_time = 0;
      code->tx_info->command_flag[Set_Zero_Pole] = 0;
    }
  }
}

/**
 * @brief  设置编码器中点位置
 * @param  code: BRT编码器结构体指针
 * @retval 无
 * @note   将编码器当前位置设置为中点
 *         与零点不同，中点用于特定应用场景
 */
static void Code_Set_Mid_Point(Code_BRT_t* code)
{
  static uint8_t over_time = 0;

  if(code->tx_info->command_flag[Set_Mid_Pole] == 0)
  {
    Code_Set_Command(code, Set_Mid_Pole);
    code->tx_info->tx_buff[3] = 0;
    code->tx_info->command_flag[Set_Mid_Pole] = 1;
    Code_Send_data(code);
  }
  else if(code->tx_info->command_flag[Set_Mid_Pole] == 1)
  {
    over_time++;
    if(over_time > 100)
    {
      over_time = 0;
      code->tx_info->command_flag[Set_Mid_Pole] = 0;
    }
  }
}

/**
 * @brief  设置编码器当前值
 * @param  code:     BRT编码器结构体指针
 * @param  encoder:  要设置的编码器值，取值范围：0~65535
 * @retval 无
 * @note   直接将编码器设为指定数值
 *         高低字节顺序：[6]=低字节, [5]=高字节
 */
static void Code_Set_Num_Point(Code_BRT_t* code, uint16_t encoder)
{
  static uint8_t over_time = 0;

  if(code->tx_info->command_flag[Set_Num_Pole] == 0)
  {
    Code_Set_Command(code, Set_Num_Pole);
    encoder = constrain(encoder, 0, 65535);
    /* 注意：高字节在[5]，低字节在[6]，字节顺序与协议一致 */
    code->tx_info->tx_buff[6] = (uint8_t)encoder;         /* 低8位 */
    code->tx_info->tx_buff[5] = (uint8_t)(encoder >> 8);   /* 高8位 */
    code->tx_info->command_flag[Set_Num_Pole] = 1;
    Code_Send_data(code);
  }
  else if(code->tx_info->command_flag[Set_Num_Pole] == 1)
  {
    over_time++;
    if(over_time > 100)
    {
      over_time = 0;
      code->tx_info->command_flag[Set_Num_Pole] = 0;
    }
  }
}

/**
 * @brief  设置编码器工作模式
 * @param  code: BRT编码器结构体指针
 * @param  Mode: 工作模式，取值范围：0~2
 *               0: 查询模式（主机主动查询）
 *               1: 自动返回角速度值模式（模式1目前不可用）
 *               2: 自动返回编码器值模式
 * @retval 无
 * @note   查询模式下需要主机主动发送读取命令
 *         自动模式下编码器按设定时间间隔自动回传数据
 */
static void Code_Set_Mode(Code_BRT_t* code, uint8_t Mode)
{
  static uint8_t over_time = 0;

  if(code->tx_info->command_flag[Set_Code_Mode] == 0)
  {
    Code_Set_Command(code, Set_Code_Mode);
    Mode = constrain(Mode, 0, 2);
    /* 将模式编号转换为协议定义的模式代码 */
    switch(Mode)
    {
      case 0:
        Mode = 0x00;    /* 查询模式 */
        break;
      case 1:
        Mode = 0x01;    /* 自动返回角速度（暂不可用） */
        break;
      case 2:
        Mode = 0xAA;    /* 自动返回编码器值 */
        break;
    }
    code->tx_info->tx_buff[3] = Mode;
    code->tx_info->command_flag[Set_Code_Mode] = 1;
    Code_Send_data(code);
  }
  else if(code->tx_info->command_flag[Set_Code_Mode] == 1)
  {
    over_time++;
    if(over_time > 100)
    {
      over_time = 0;
      code->tx_info->command_flag[Set_Code_Mode] = 0;
    }
  }
}

/**
 * @brief  设置编码器值递增方向
 * @param  code: BRT编码器结构体指针
 * @param  dire: 方向设置
 *               0: 顺时针递增
 *               1: 逆时针递增
 * @retval 无
 * @note   根据实际安装方向调整编码器递增方向
 */
static void Code_Set_Dire(Code_BRT_t* code, uint8_t dire)
{
  static uint8_t over_time = 0;

  if(code->tx_info->command_flag[Set_Dire] == 0)
  {
    Code_Set_Command(code, Set_Dire);
    code->tx_info->tx_buff[3] = dire;
    code->tx_info->command_flag[Set_Dire] = 1;
    Code_Send_data(code);
  }
  else if(code->tx_info->command_flag[Set_Dire] == 1)
  {
    over_time++;
    if(over_time > 100)
    {
      over_time = 0;
      code->tx_info->command_flag[Set_Dire] = 0;
    }
  }
}

/**
 * @brief  设置编码器角速度采样速率
 * @param  code: BRT编码器结构体指针
 * @param  time: 采样时间参数，取值范围：0~65535
 * @retval 无
 * @note   控制编码器角速度的采样频率
 *         高字节在[4]，低字节在[3]
 */
static void Code_Set_Sample_Time(Code_BRT_t* code, uint16_t time)
{
  static uint8_t over_time = 0;

  if(code->tx_info->command_flag[Set_Sample_Time] == 0)
  {
    Code_Set_Command(code, Set_Sample_Time);
    code->tx_info->tx_buff[3] = (uint8_t)(time);        /* 低8位 */
    code->tx_info->tx_buff[4] = (uint8_t)(time >> 8);   /* 高8位 */
    code->tx_info->command_flag[Set_Sample_Time] = 1;
    Code_Send_data(code);
  }
  else if(code->tx_info->command_flag[Set_Sample_Time] == 1)
  {
    over_time++;
    if(over_time > 100)
    {
      over_time = 0;
      code->tx_info->command_flag[Set_Sample_Time] = 0;
    }
  }
}

/**
 * @brief  通过CAN总线发送编码器数据
 * @param  code: BRT编码器结构体指针
 * @retval 无
 * @note   调用CAN_SendData发送8字节数据帧
 *         发送完成后清零发送缓冲区
 */
static void Code_Send_data(Code_BRT_t* code)
{
  /* 通过CAN总线发送数据：使用编码器的CAN实例、标准ID和发送缓冲区 */
  CAN_SendData(code->born_info->hcan, code->born_info->stdId, code->tx_info->tx_buff);
  /* 发送完成后清零发送缓冲区，避免残留数据影响下次发送 */
  memset(code->tx_info->tx_buff, 0, 8);
}


/**
 * @brief  编码器接收数据更新函数
 * @param  code:  BRT编码器结构体指针
 * @param  rxBuf: CAN接收数据缓冲区（8字节）
 * @retval 无
 * @note   根据响应数据的命令码（rxBuf[2]）解析不同类型的数据：
 *         - Read_encoder: 解析单圈角度值（16位）
 *         - Read_sum_encoder: 解析多圈累计角度（32位有符号）
 *         - Read_Turn: 解析累计圈数（32位）
 *         - Read_speed: 解析角速度（32位）
 *         - Set_Id: 确认ID设置是否成功
 *         - 其他命令：检查返回数据判断操作是否成功
 *         每次成功接收后清零离线计数器
 */
static void Code_Update(Code_BRT_t* code, uint8_t *rxBuf)
{
  switch(rxBuf[2])
  {
    case Read_encoder:
      /* 解析编码器角度值：高字节在前，低字节在后 */
      code->rx_info->angle_raw = (uint16_t)((rxBuf[4] << 8) | rxBuf[3]);
      /* 标记命令响应已收到（flag=2表示成功接收） */
      code->tx_info->command_flag[rxBuf[2]] = 2;
      break;

    case Read_sum_encoder:
      /* 解析多圈累计角度（32位有符号整数） */
      memcpy(&code->rx_info->sum_angle_raw, &rxBuf[3], 4);
      /* 有符号处理：超过最大正值时减去偏移量转换为有符号数 */
      if(code->rx_info->sum_angle_raw > 1073741823)
        code->rx_info->sum_encoder = code->rx_info->sum_angle_raw - 2147483647;
      else
        code->rx_info->sum_encoder = code->rx_info->sum_angle_raw;
      /* 将编码器值转换为弧度（一圈4096个编码值，转换为2*PI弧度） */
      code->rx_info->sum_angle = code->rx_info->sum_encoder * 2.f * PI / 4096;
      code->tx_info->command_flag[rxBuf[2]] = 2;
      break;

    case Read_Turn:
      /* 解析累计圈数（32位无符号整数） */
      memcpy(&code->rx_info->sum_turn, &rxBuf[3], 4);
      /* 有符号处理 */
      if(code->rx_info->sum_turn > 262143)
        code->rx_info->sum_encoder = code->rx_info->sum_turn - 524287;
      else
        code->rx_info->sum_encoder = code->rx_info->sum_turn;
      code->tx_info->command_flag[rxBuf[2]] = 2;
      break;

    case Read_speed:
      /* 解析角速度（32位） */
      memcpy(&code->rx_info->speed_raw, &rxBuf[3], 4);
      code->tx_info->command_flag[rxBuf[2]] = 2;
      break;

    case Set_Id:
      /* 确认ID设置：检查返回的ID是否与设置的新ID一致 */
      if(rxBuf[1] == code->tx_info->new_id)
        code->tx_info->command_flag[rxBuf[2]] = 2;  /* 成功 */
      else
        code->tx_info->command_flag[rxBuf[2]] = 3;  /* 失败 */
      break;

    default:
      /* 其他命令：检查返回数据判断操作是否成功 */
      if(rxBuf[3] == 0)
        code->tx_info->command_flag[rxBuf[2]] = 2;  /* 成功 */
      else
        code->tx_info->command_flag[rxBuf[2]] = 3;  /* 失败 */
      break;
  }

  /* 收到有效数据，清零离线计数器 */
  code->state->offline_cnt = 0;
}

/**
 * @brief  编码器心跳失联检测
 * @param  code: BRT编码器结构体指针
 * @retval 无
 * @note   每次调用离线计数器加1
 *         超过最大阈值后标记设备为离线状态
 *         收到有效数据时会清零离线计数器（在Code_Update中）
 */
static void BRT_Code_Heart_Beat(Code_BRT_t *code)
{
  Code_BRT_State_t *code_state = code->state;

  /* 离线计数器递增 */
  code_state->offline_cnt++;

  if(code_state->offline_cnt > code_state->offline_cnt_max)
  {
    /* 超过最大阈值，锁定在最大值，标记为离线 */
    code_state->offline_cnt = code_state->offline_cnt_max;
    code_state->status = DEV_OFFLINE;
  }
  else
  {
    /* 未超阈值，如果之前是离线状态则恢复为在线 */
    if(code_state->status == DEV_OFFLINE)
      code_state->status = DEV_ONLINE;
  }
}

/**
 * @brief  BRT编码器初始化函数
 * @param  code: BRT编码器结构体指针
 * @retval 无
 * @note   绑定所有函数指针，设置默认参数：
 *         - rx: 数据解析回调
 *         - receive_encoder: 读取角度
 *         - receive_speed: 读取速度
 *         - receive_sum_encoder: 读取多圈角度
 *         - receive_sum_turn: 读取圈数
 *         - set_*: 各种参数设置函数
 *         - single_heart_beat: 心跳检测
 *         离线计数器最大值设为100
 */
void BRT_Code_Init(Code_BRT_t* code)
{
  /* 绑定数据接收解析函数 */
  code->rx = Code_Update;

  /* 绑定读取函数 */
  code->receive_encoder = Code_Read_Angle;       /* 读取单圈角度 */
  code->receive_speed = Code_Read_Speed;          /* 读取角速度 */
  code->receive_sum_encoder = Code_Read_Sum_Angle; /* 读取多圈角度 */
  code->receive_sum_turn = Code_Read_Turn;        /* 读取圈数 */

  /* 绑定设置函数 */
  code->set_receive_time = Code_Set_Receive_Time; /* 设置回传时间 */
  code->set_mode = Code_Set_Mode;                 /* 设置工作模式 */
  code->set_id = Code_Set_ID;                     /* 设置ID */
  code->set_baud = Code_Set_Baud;                 /* 设置波特率 */
  code->set_zero = Code_Set_Zero_Point;           /* 设置零点 */
  code->set_dire = Code_Set_Dire;                 /* 设置方向 */
  code->set_sample_time = Code_Set_Sample_Time;   /* 设置采样时间 */
  code->set_encoder = Code_Set_Num_Point;         /* 设置编码器值 */
  code->set_mid = Code_Set_Mid_Point;             /* 设置中点 */

  /* 设置默认参数 */
  code->state->offline_cnt_max = 100;  /* 离线检测最大阈值 */

  /* 绑定心跳检测函数 */
  code->single_heart_beat = BRT_Code_Heart_Beat;
}


/*---------------------工具函数----------------------*/

/**
 * @brief  根据命令类型设置发送缓冲区的命令头
 * @param  code:    BRT编码器结构体指针
 * @param  command: 命令类型枚举值
 * @retval 无
 * @note   每个命令对应固定的缓冲区格式：
 *         [0] = 数据长度（4或5或7字节）
 *         [1] = 固定值0x01（设备地址/标志）
 *         [2] = 命令码（不同命令对应不同值）
 *         [3]~[7] = 命令参数（由具体命令决定）
 */
static void Code_Set_Command(Code_BRT_t* code, Code_BRT_Command_e command)
{
  switch(command)
  {
    case Read_encoder:
      /* 读取编码器值：数据长度=4 */
      code->tx_info->tx_buff[0] = 0x04;
      code->tx_info->tx_buff[1] = 0x01;
      code->tx_info->tx_buff[2] = 0x01;  /* 命令码：0x01 */
      break;

    case Set_Id:
      /* 设置ID：数据长度=4 */
      code->tx_info->tx_buff[0] = 0x04;
      code->tx_info->tx_buff[1] = 0x01;
      code->tx_info->tx_buff[2] = 0x02;  /* 命令码：0x02 */
      break;

    case Set_Baud:
      /* 设置波特率：数据长度=4 */
      code->tx_info->tx_buff[0] = 0x04;
      code->tx_info->tx_buff[1] = 0x01;
      code->tx_info->tx_buff[2] = 0x03;  /* 命令码：0x03 */
      break;

    case Set_Code_Mode:
      /* 设置编码器模式：数据长度=4 */
      code->tx_info->tx_buff[0] = 0x04;
      code->tx_info->tx_buff[1] = 0x01;
      code->tx_info->tx_buff[2] = 0x04;  /* 命令码：0x04 */
      break;

    case Set_Receive_Time:
      /* 设置回传时间：数据长度=5（含2字节时间参数） */
      code->tx_info->tx_buff[0] = 0x05;
      code->tx_info->tx_buff[1] = 0x01;
      code->tx_info->tx_buff[2] = 0x05;  /* 命令码：0x05 */
      break;

    case Set_Zero_Pole:
      /* 设置零点：数据长度=4 */
      code->tx_info->tx_buff[0] = 0x04;
      code->tx_info->tx_buff[1] = 0x01;
      code->tx_info->tx_buff[2] = 0x06;  /* 命令码：0x06 */
      break;

    case Set_Dire:
      /* 设置方向：数据长度=4 */
      code->tx_info->tx_buff[0] = 0x04;
      code->tx_info->tx_buff[1] = 0x01;
      code->tx_info->tx_buff[2] = 0x07;  /* 命令码：0x07 */
      break;

    case Read_sum_encoder:
      /* 读取多圈角度：数据长度=4 */
      code->tx_info->tx_buff[0] = 0x04;
      code->tx_info->tx_buff[1] = 0x01;
      code->tx_info->tx_buff[2] = 0x08;  /* 命令码：0x08 */
      break;

    case Read_Turn:
      /* 读取圈数：数据长度=4 */
      code->tx_info->tx_buff[0] = 0x04;
      code->tx_info->tx_buff[1] = 0x01;
      code->tx_info->tx_buff[2] = 0x09;  /* 命令码：0x09 */
      break;

    case Read_speed:
      /* 读取角速度：数据长度=4 */
      code->tx_info->tx_buff[0] = 0x04;
      code->tx_info->tx_buff[1] = 0x01;
      code->tx_info->tx_buff[2] = 0x0A;  /* 命令码：0x0A */
      break;

    case Set_Sample_Time:
      /* 设置采样时间：数据长度=5（含2字节时间参数） */
      code->tx_info->tx_buff[0] = 0x05;
      code->tx_info->tx_buff[1] = 0x01;
      code->tx_info->tx_buff[2] = 0x0B;  /* 命令码：0x0B */
      break;

    case Set_Mid_Pole:
      /* 设置中点：数据长度=4 */
      code->tx_info->tx_buff[0] = 0x04;
      code->tx_info->tx_buff[1] = 0x01;
      code->tx_info->tx_buff[2] = 0x0C;  /* 命令码：0x0C */
      break;

    case Set_Num_Pole:
      /* 设置编码器值：数据长度=7（含2字节编码器参数） */
      code->tx_info->tx_buff[0] = 0x07;
      code->tx_info->tx_buff[1] = 0x01;
      code->tx_info->tx_buff[2] = 0x0D;  /* 命令码：0x0D */
      break;

    default:
      /* 未知命令，不做处理 */
      break;
  };
}
