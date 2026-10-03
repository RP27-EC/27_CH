/**
  ******************************************************************************
  * @file           : shoot_motor.c
  * @brief          : 拨弹电机驱动模块
  * @details        : 配置拨弹RM3508电机的参数、PID控制器、控制信息等。
  *                   电机通过FDCAN2总线通信，反馈ID 0x205，电流帧0x1FF。
  *                   控制类型与底盘轮子相同（_3508_Single）：
  *                   速度目标单位为转子RPM，角度目标单位为编码器计数。
  ******************************************************************************
  */

#include "shoot_motor.h"

/**
  * @brief  拨弹电机出厂信息
  * @details 定义电机的基本参数：
  *          - order_correction: 电机序号修正（单电机固定为1）
  *          - rxId: 报文字节序号，决定0x1FF帧里电流占哪2字节。
  *                 电机5在帧最前2字节，故rxId=0（不是电调ID=5）
  *          - stdId: 标准发送ID 0x1FF（电机5~8电流帧）
  *          - type: 电机类型，M3508单电机模式
  *          - hcan: CAN总线句柄，使用FDCAN2
  */
static Motor_RM_Born_Info_t shoot_motor_born = {
    .order_correction = 1,
    .rxId = 0,                       /* 0x1FF帧内字节偏移，对应电机5 */
    .stdId = SHOOT_MOTOR_TX_ID,      /* 0x1FF */
    .type = _3508_Single,
    .hcan = &hfdcan2,
};

/* 存储从CAN总线接收到的电机反馈数据（角度、速度、力矩等） */
static Motor_RM_Rx_Info_t shoot_motor_rx;

/* 存储要发送到电机的控制指令（力矩电流等） */
static Motor_RM_Tx_Info_t shoot_motor_tx;


static Motor_RM_State_t shoot_motor_state;

/**
  *  拨弹电机的速度PID控制器（连发用）
  *          - out_max = 10000.0: 输出限幅，对应电机最大力矩电流
  M3508电机的力矩电流范围为[-16384, 16384]，这里限制为10000以保护电机
  */
static pid_ctrl_t shoot_motor_speed_pid = {
    .kp = 15.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .a = 1.0f,
    .integral_max = 0.0f,
    .out_max = 10000.0f,
};

/**
  * @brief  拨弹电机的角度外环PID（单发/锁位用）
  * @details 目标与反馈均为encoder_sum（多圈编码器计数）。
  *          kp=0.15：一格弹约31481计数，外环输出作为内环速度目标，
  *          量级与速度环rpm目标匹配，上车后按实际响应再调。
  */
static pid_ctrl_t shoot_motor_angle_pid = {
    .kp = 0.15f,
    .ki = 0.0f,
    .kd = 0.0f,
    .a = 1.0f,
    .integral_max = 0.0f,
    .out_max = 3000.0f,
};

/**
  * @brief  拨弹电机角度环内层速度PID
  * @details 与连发速度环共用同一组参数，反馈为encoder_speed(rpm)
  */
static pid_ctrl_t shoot_motor_angle_speed_pid = {
    .kp = 15.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .a = 1.0f,
    .integral_max = 0.0f,
    .out_max = 10000.0f,
};

/**
  * @brief  拨弹电机的控制信息
  * @details 包含速度/角度输入标志和对应的PID控制器指针
  *          Angle_Input_Flag=true：角度反馈由模块层填encoder_sum，
  *          不走RM库内部的单圈encoder覆盖逻辑
  */
static Motor_RM_Ctrl_Info_t shoot_motor_ctrl = {
    .Speed_Input_Flag = false,
    .Angle_Input_Flag = true,           /* 角度反馈用多圈encoder_sum */
    .Nearest_Return = false,
    .angle_ctrl_outer = &shoot_motor_angle_pid,
    .angle_ctrl_inner = &shoot_motor_angle_speed_pid,
    .speed_ctrl = &shoot_motor_speed_pid,
};

/**
  * @brief  拨弹电机对象
  * @details 将所有子模块组合成完整的电机控制对象
  */
Motor_RM_t Shoot_Motor = {
    .born_info = &shoot_motor_born,
    .rx_info = &shoot_motor_rx,
    .tx_info = &shoot_motor_tx,
    .state = &shoot_motor_state,
    .ctrl = &shoot_motor_ctrl,
    .single_init = RM_Motor_Init,
};

/**
  * @brief  拨弹电机初始化
  * @details 绑定函数指针并调用RM库初始化。
  *          上电先按在线处理；若无CAN反馈，心跳累计后自动置离线。
  * @retval None
  */
void Shoot_Motor_Init(void)
{
    Shoot_Motor.single_init = RM_Motor_Init;
    Shoot_Motor.single_init(&Shoot_Motor);

    /* 上电先按在线处理；无CAN反馈时由心跳在offline_cnt_max后拉离线 */
    shoot_motor_state.status = DEV_ONLINE;
    shoot_motor_state.offline_cnt = 0;
}

/**
  * @brief  拨弹电机卸力
  * @details 清零三个PID的状态与目标，力矩清零后下发一帧零电流，
  *          用于初始化、总开关关闭或失联保护。
  * @retval None
  */
void Shoot_Motor_Stop(void)
{
    pid_clear(&shoot_motor_speed_pid);
    pid_clear(&shoot_motor_angle_pid);
    pid_clear(&shoot_motor_angle_speed_pid);

    shoot_motor_speed_pid.target = 0.0f;
    shoot_motor_angle_pid.target = 0.0f;

    shoot_motor_tx.torque = 0.0f;
    shoot_motor_tx.torque_current = 0.0f;
    shoot_motor_tx.torque_current_raw = 0;

    Shoot_Motor.single_set_torque(&Shoot_Motor);
}
