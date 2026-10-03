/**
  ******************************************************************************
  * @file           : chassis_motor.c
  * @brief          : 底盘电机驱动模块
  * @details        : 配置和管理四个M3508底盘驱动电机，包括电机参数、
  *                   PID控制器、控制信息等的初始化和定义。
  *                   四个电机通过FDCAN1总线通信，使用标准ID 0x200-0x203。
  ******************************************************************************
  */

#include "chassis_motor.h"

/**
  * @brief  四个底盘电机的出厂信息数组
  * @details 定义每个电机的基本参数，包括：
  *          - order_correction: 电机序号修正（用于多电机级联时的ID映射）
  *          - rxId: 接收ID，用于识别电机反馈数据
  *          - stdId: 标准发送ID，所有底盘电机共用0x200
  *          - type: 电机类型，M3508单电机模式
  *          - hcan: CAN总线句柄，使用FDCAN1
  */
static Motor_RM_Born_Info_t chassis_motor_born[4] = {
    /* 前左轮(FL) - 电机ID 0，标准发送ID 0x200 */
    {.order_correction = 0, .rxId = 0, .stdId = 0x200, .type = _3508_Single, .hcan = &hfdcan1},
    /* 前右轮(FR) - 电机ID 1，标准发送ID 0x200 */
    {.order_correction = 0, .rxId = 1, .stdId = 0x200, .type = _3508_Single, .hcan = &hfdcan1},
    /* 后左轮(BL) - 电机ID 2，标准发送ID 0x200 */
    {.order_correction = 0, .rxId = 2, .stdId = 0x200, .type = _3508_Single, .hcan = &hfdcan1},
    /* 后右轮(BR) - 电机ID 3，标准发送ID 0x200 */
    {.order_correction = 0, .rxId = 3, .stdId = 0x200, .type = _3508_Single, .hcan = &hfdcan1},
};

/* 存储从CAN总线接收到的电机反馈数据（角度、速度、力矩等）*/
static Motor_RM_Rx_Info_t chassis_motor_rx[4];

/*存储要发送到电机的控制指令（力矩电流等） */
static Motor_RM_Tx_Info_t chassis_motor_tx[4];

/**
  * @brief  四个底盘电机的状态信息
  * @details 存储电机的运行状态（在线/离线、错误码等）
  */
static Motor_RM_State_t chassis_motor_state[4];

/**
  * @brief  四个底盘电机的速度PID控制器
  * @details 每个电机配备独立的速度PID控制器，参数为：
  *          - kp = 6.0: 比例系数，决定响应速度
  *          - ki = 0.0: 积分系数，用于消除稳态误差（当前未使用）
  *          - kd = 0.0: 微分系数，用于抑制超调（当前未使用）
  *          - a = 1.0: 滤波系数
  *          - integral_max = 6000.0: 积分限幅，防止积分饱和
  *          - out_max = 10000.0: 输出限幅，对应电机最大力矩电流
  *
  * @note   M3508电机的力矩电流范围为[-16384, 16384]，这里限制为10000以保护电机
  */
static pid_ctrl_t chassis_motor_speed_pid[4] = {
    {.kp = 6.0f, .ki = 0.0f, .kd = 0.0f, .a = 1.0f, .integral_max = 6000.0f, .out_max = 10000.0f},  /* 前左轮PID */
    {.kp = 6.0f, .ki = 0.0f, .kd = 0.0f, .a = 1.0f, .integral_max = 6000.0f, .out_max = 10000.0f},  /* 前右轮PID */
    {.kp = 6.0f, .ki = 0.0f, .kd = 0.0f, .a = 1.0f, .integral_max = 6000.0f, .out_max = 10000.0f},  /* 后左轮PID */
    {.kp = 6.0f, .ki = 0.0f, .kd = 0.0f, .a = 1.0f, .integral_max = 6000.0f, .out_max = 10000.0f},  /* 后右轮PID */
};

/**
  * @brief  四个底盘电机的控制信息
  * @details 包含速度/角度输入标志和对应的PID控制器指针
  *          Speed_Input_Flag: 速度输入使能标志
  *          Angle_Input_Flag: 角度输入使能标志
  *          speed_ctrl: 指向速度PID控制器的指针
  */
static Motor_RM_Ctrl_Info_t chassis_motor_ctrl[4] = {
    {.Speed_Input_Flag = false, .Angle_Input_Flag = false, .speed_ctrl = &chassis_motor_speed_pid[0]},  /* 前左轮控制 */
    {.Speed_Input_Flag = false, .Angle_Input_Flag = false, .speed_ctrl = &chassis_motor_speed_pid[1]},  /* 前右轮控制 */
    {.Speed_Input_Flag = false, .Angle_Input_Flag = false, .speed_ctrl = &chassis_motor_speed_pid[2]},  /* 后左轮控制 */
    {.Speed_Input_Flag = false, .Angle_Input_Flag = false, .speed_ctrl = &chassis_motor_speed_pid[3]},  /* 后右轮控制 */
};

/**
  * @brief  四个底盘电机对象数组
  * @details 每个电机对象包含完整的配置和控制信息：
  *          - born_info: 出厂信息（CAN ID、电机类型等）
  *          - rx_info: 接收数据缓冲区
  *          - tx_info: 发送数据缓冲区
  *          - state: 电机状态
  *          - ctrl: 控制信息（PID控制器等）
  *          - single_init: 单电机初始化函数指针
  *
  * @note   数组顺序：[0]=前左, [1]=前右, [2]=后左, [3]=后右
  */
Motor_RM_t Chassis_Motor[4] = {
    {.born_info = &chassis_motor_born[0], .rx_info = &chassis_motor_rx[0], .tx_info = &chassis_motor_tx[0], .state = &chassis_motor_state[0], .ctrl = &chassis_motor_ctrl[0], .single_init = RM_Motor_Init},  /* 前左轮 */
    {.born_info = &chassis_motor_born[1], .rx_info = &chassis_motor_rx[1], .tx_info = &chassis_motor_tx[1], .state = &chassis_motor_state[1], .ctrl = &chassis_motor_ctrl[1], .single_init = RM_Motor_Init},  /* 前右轮 */
    {.born_info = &chassis_motor_born[2], .rx_info = &chassis_motor_rx[2], .tx_info = &chassis_motor_tx[2], .state = &chassis_motor_state[2], .ctrl = &chassis_motor_ctrl[2], .single_init = RM_Motor_Init},  /* 后左轮 */
    {.born_info = &chassis_motor_born[3], .rx_info = &chassis_motor_rx[3], .tx_info = &chassis_motor_tx[3], .state = &chassis_motor_state[3], .ctrl = &chassis_motor_ctrl[3], .single_init = RM_Motor_Init},  /* 后右轮 */
};

/**
  * @brief  底盘电机组对象
  * @details 管理四个底盘电机的组操作，包括：
  *          - motor: 指向四个电机对象的指针数组
  *          - stdId: 组发送的标准ID (0x200)
  *          - hcan: CAN总线句柄 (FDCAN1)
  *          - group_init: 组初始化函数指针
  *
  * @note   使用组操作可以一次性发送所有电机的控制指令，提高通信效率
  */
Motor_RM_Group_t Chassis_Motor_Group = {
    .motor = {&Chassis_Motor[0], &Chassis_Motor[1], &Chassis_Motor[2], &Chassis_Motor[3]},
    .stdId = 0x200,           /* 底盘电机组的标准发送ID */
    .hcan = &hfdcan1,         /* 使用FDCAN1总线 */
    .group_init = RM_Group_Motor_Init,  /* 组初始化函数 */
};
