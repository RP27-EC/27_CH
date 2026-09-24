#include "chassis_motor.h"

static Motor_RM_Born_Info_t chassis_motor_born[4] = {
    {.order_correction = 0, .rxId = 0, .stdId = 0x200, .type = _3508_Single, .hcan = &hfdcan1},
    {.order_correction = 0, .rxId = 1, .stdId = 0x200, .type = _3508_Single, .hcan = &hfdcan1},
    {.order_correction = 0, .rxId = 2, .stdId = 0x200, .type = _3508_Single, .hcan = &hfdcan1},
    {.order_correction = 0, .rxId = 3, .stdId = 0x200, .type = _3508_Single, .hcan = &hfdcan1},
};

static Motor_RM_Rx_Info_t chassis_motor_rx[4];
static Motor_RM_Tx_Info_t chassis_motor_tx[4];
static Motor_RM_State_t chassis_motor_state[4];

static pid_ctrl_t chassis_motor_speed_pid[4] = {
    {.kp = 6.0f, .ki = 0.0f, .kd = 0.0f, .a = 1.0f, .integral_max = 6000.0f, .out_max = 10000.0f},
    {.kp = 6.0f, .ki = 0.0f, .kd = 0.0f, .a = 1.0f, .integral_max = 6000.0f, .out_max = 10000.0f},
    {.kp = 6.0f, .ki = 0.0f, .kd = 0.0f, .a = 1.0f, .integral_max = 6000.0f, .out_max = 10000.0f},
    {.kp = 6.0f, .ki = 0.0f, .kd = 0.0f, .a = 1.0f, .integral_max = 6000.0f, .out_max = 10000.0f},
};

static Motor_RM_Ctrl_Info_t chassis_motor_ctrl[4] = {
    {.Speed_Input_Flag = false, .Angle_Input_Flag = false, .speed_ctrl = &chassis_motor_speed_pid[0]},
    {.Speed_Input_Flag = false, .Angle_Input_Flag = false, .speed_ctrl = &chassis_motor_speed_pid[1]},
    {.Speed_Input_Flag = false, .Angle_Input_Flag = false, .speed_ctrl = &chassis_motor_speed_pid[2]},
    {.Speed_Input_Flag = false, .Angle_Input_Flag = false, .speed_ctrl = &chassis_motor_speed_pid[3]},
};

Motor_RM_t Chassis_Motor[4] = {
    {.born_info = &chassis_motor_born[0], .rx_info = &chassis_motor_rx[0], .tx_info = &chassis_motor_tx[0], .state = &chassis_motor_state[0], .ctrl = &chassis_motor_ctrl[0], .single_init = RM_Motor_Init},
    {.born_info = &chassis_motor_born[1], .rx_info = &chassis_motor_rx[1], .tx_info = &chassis_motor_tx[1], .state = &chassis_motor_state[1], .ctrl = &chassis_motor_ctrl[1], .single_init = RM_Motor_Init},
    {.born_info = &chassis_motor_born[2], .rx_info = &chassis_motor_rx[2], .tx_info = &chassis_motor_tx[2], .state = &chassis_motor_state[2], .ctrl = &chassis_motor_ctrl[2], .single_init = RM_Motor_Init},
    {.born_info = &chassis_motor_born[3], .rx_info = &chassis_motor_rx[3], .tx_info = &chassis_motor_tx[3], .state = &chassis_motor_state[3], .ctrl = &chassis_motor_ctrl[3], .single_init = RM_Motor_Init},
};

Motor_RM_Group_t Chassis_Motor_Group = {
    .motor = {&Chassis_Motor[0], &Chassis_Motor[1], &Chassis_Motor[2], &Chassis_Motor[3]},
    .stdId = 0x200,
    .hcan = &hfdcan1,
    .group_init = RM_Group_Motor_Init,
};
