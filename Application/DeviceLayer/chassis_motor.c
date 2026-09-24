#include "Chassis_motor.h"
#include "RM_Motor.h"
/*舵向电机组*/
/*-----------------------左前舵向电机-------------------------*/
Motor_RM_Born_Info_t L_F_Se_Born ={
	.order_correction = 0,
	
	.rxId = 0,
	
	.stdId = 0x200,
	
	.type = _3508_Reduction,
	
	.hcan = &hfdcan1,
	
};
Motor_RM_Rx_Info_t L_F_Se_Rxinfo;

pid_ctrl_t L_F_Se_Ctrlout ={
	.kp = 22.f,//
	.ki = 0.f,
	.kd = 0.f,
	.a = 1.f,
	.integral_max = 0.f,
	.out_max = 1500.f,//
};
pid_ctrl_t L_F_Se_Ctrlin ={
	.kp = 0.003f,//
	.ki = 0.00005f,
	.kd = 0.f,
	.a = 1.f,
	.integral_max = 10000.f,
	.out_max = 8000.f,//
};
Motor_RM_Ctrl_Info_t L_F_Se_Ctrl ={
	.Speed_Input_Flag = 0,
	.Angle_Input_Flag = 1,
	.Nearest_Return = 1,
	.angle_ctrl_inner = &L_F_Se_Ctrlin,
	.angle_ctrl_outer = &L_F_Se_Ctrlout,
};

Motor_RM_Tx_Info_t L_F_Se_Txinfo;

Motor_RM_State_t L_F_Se_State;

Motor_RM_t L_F_Se = {
	.born_info = &L_F_Se_Born,
	
	.rx_info = &L_F_Se_Rxinfo,
	
	.tx_info = &L_F_Se_Txinfo,
	
	.state = &L_F_Se_State,
	
	.ctrl = &L_F_Se_Ctrl,
	
	.single_init = RM_Motor_Init,
	
};
/*-----------------------右前舵向电机-------------------------*/
Motor_RM_Born_Info_t R_F_Se_Born ={
	.order_correction = 0,
	
	.rxId = 1,
	
	.stdId = 0x200,
	
	.type = _3508_Reduction,
	
	.hcan = &hfdcan1,
	
};
Motor_RM_Rx_Info_t R_F_Se_Rxinfo;

pid_ctrl_t R_F_Se_Ctrlout ={
	.kp = 22.f,//
	.ki = 0.f,
	.kd = 0.f,
	.a = 1.f,
	.integral_max = 0.f,
	.out_max = 1500.f,//
};;
pid_ctrl_t R_F_Se_Ctrlin ={
	.kp = 0.003f,//
	.ki = 0.00005f,
	.kd = 0.f,
	.a = 1.f,
	.integral_max = 10000.f,
	.out_max = 8000.f,//
};
Motor_RM_Ctrl_Info_t R_F_Se_Ctrl ={
	.Speed_Input_Flag = 0,
	.Angle_Input_Flag = 1,
	.Nearest_Return = 1,
	.angle_ctrl_inner = &R_F_Se_Ctrlin,
	.angle_ctrl_outer = &R_F_Se_Ctrlout,
};

Motor_RM_Tx_Info_t R_F_Se_Txinfo;

Motor_RM_State_t R_F_Se_State;

Motor_RM_t R_F_Se = {
	.born_info = &R_F_Se_Born,
	
	.rx_info = &R_F_Se_Rxinfo,
	
	.tx_info = &R_F_Se_Txinfo,
	
	.state = &R_F_Se_State,
	
	.ctrl = &R_F_Se_Ctrl,
	
	.single_init = RM_Motor_Init,
	
};
/*-----------------------左后舵向电机-------------------------*/
Motor_RM_Born_Info_t L_B_Se_Born ={
	.order_correction = 0,
	
	.rxId = 1,
	
	.stdId = 0x200,
	
	.type = _3508_Reduction,
	
	.hcan = &hfdcan2,
	
};
Motor_RM_Rx_Info_t L_B_Se_Rxinfo;

pid_ctrl_t L_B_Se_Ctrlout ={
	.kp = 22.f,//
	.ki = 0.f,
	.kd = 0.f,
	.a = 1.f,
	.integral_max = 0.f,
	.out_max = 1500.f,//
};
pid_ctrl_t L_B_Se_Ctrlin ={
	.kp = 0.0003f,//
	.ki = 0.00001f,
	.kd = 0.f,
	.a = 1.f,
	.integral_max = 10000.f,
	.out_max = 8000.f,//
};
Motor_RM_Ctrl_Info_t L_B_Se_Ctrl ={
	.Speed_Input_Flag = 0,
	.Angle_Input_Flag = 1,
	.Nearest_Return = 1,
	.angle_ctrl_inner = &L_B_Se_Ctrlin,
	.angle_ctrl_outer = &L_B_Se_Ctrlout,
};

Motor_RM_Tx_Info_t L_B_Se_Txinfo;

Motor_RM_State_t L_B_Se_State;

Motor_RM_t L_B_Se = {
	.born_info = &L_B_Se_Born,
	
	.rx_info = &L_B_Se_Rxinfo,
	
	.tx_info = &L_B_Se_Txinfo,
	
	.state = &L_B_Se_State,
	
	.ctrl = &L_B_Se_Ctrl,
	
	.single_init = RM_Motor_Init,
	
};
/*-----------------------右后舵向电机-------------------------*/
Motor_RM_Born_Info_t R_B_Se_Born ={
	.order_correction = 0,
	
	.rxId = 0,
	
	.stdId = 0x200,
	
	.type = _3508_Reduction,
	
	.hcan = &hfdcan2,
	
};
Motor_RM_Rx_Info_t R_B_Se_Rxinfo;

pid_ctrl_t R_B_Se_Ctrlout ={
	.kp = 22.f,//
	.ki = 0.f,
	.kd = 0.f,
	.a = 1.f,
	.integral_max = 0.f,
	.out_max = 1500.f,//
};
pid_ctrl_t R_B_Se_Ctrlin ={
	.kp = 0.003f,//
	.ki = 0.0001f,
	.kd = 0.f,
	.a = 1.f,
	.integral_max = 10000.f,
	.out_max = 8000.f,//
};
Motor_RM_Ctrl_Info_t R_B_Se_Ctrl ={
	.Speed_Input_Flag = 0,
	.Angle_Input_Flag = 1,
	.Nearest_Return = 1,
	.angle_ctrl_inner = &R_B_Se_Ctrlin,
	.angle_ctrl_outer = &R_B_Se_Ctrlout,
};

Motor_RM_Tx_Info_t R_B_Se_Txinfo;

Motor_RM_State_t R_B_Se_State;

Motor_RM_t R_B_Se = {
	.born_info = &R_B_Se_Born,
	
	.rx_info = &R_B_Se_Rxinfo,
	
	.tx_info = &R_B_Se_Txinfo,
	
	.state = &R_B_Se_State,
	
	.ctrl = &R_B_Se_Ctrl,
	
	.single_init = RM_Motor_Init,
	
};

/*航向电机组*/
/*-----------------------左前航向电机-------------------------*/
Motor_RM_Born_Info_t L_F_Wheel_Born ={
	.order_correction = 0,
	
	.rxId = 3,
	
	.stdId = 0x200,
	
	.type = _3508_Single,
	
	.hcan = &hfdcan1,
	
};
Motor_RM_Rx_Info_t L_F_Wheel_Rxinfo;

pid_ctrl_t L_F_Wheel_Ctrlspeed = 
{
	.kp = 6.f,//
	.ki = 0.f,
	.kd = 0.f,
	.a = 1.f,
	.integral_max = 6000.f,
	.out_max = 10000.f,//
};
Motor_RM_Ctrl_Info_t L_F_Wheel_Ctrl ={
	.Speed_Input_Flag = 0,
	.Angle_Input_Flag = 0,
	.speed_ctrl = &L_F_Wheel_Ctrlspeed,
};

Motor_RM_Tx_Info_t L_F_Wheel_Txinfo;

Motor_RM_State_t L_F_Wheel_State;

Motor_RM_t L_F_Wheel = {
	.born_info = &L_F_Wheel_Born,
	
	.rx_info = &L_F_Wheel_Rxinfo,
	
	.tx_info = &L_F_Wheel_Txinfo,
	
	.state = &L_F_Wheel_State,
	
	.ctrl = &L_F_Wheel_Ctrl,
	
	.single_init = RM_Motor_Init,
	
};
/*-----------------------右前航向电机-------------------------*/
Motor_RM_Born_Info_t R_F_Wheel_Born ={
	.order_correction = 0,
	
	.rxId = 2,
	
	.stdId = 0x200,
	
	.type = _3508_Single,
	
	.hcan = &hfdcan1,
	
};
Motor_RM_Rx_Info_t R_F_Wheel_Rxinfo;

pid_ctrl_t R_F_Wheel_Ctrlspeed = 
{
	.kp = 6.f,//
	.ki = 0.f,
	.kd = 0.f,
	.a = 1.f,
	.integral_max = 6000.f,
	.out_max = 10000.f,//
};
Motor_RM_Ctrl_Info_t R_F_Wheel_Ctrl ={
	.Speed_Input_Flag = 0,
	.Angle_Input_Flag = 0,
	.speed_ctrl = &R_F_Wheel_Ctrlspeed,
};

Motor_RM_Tx_Info_t R_F_Wheel_Txinfo;

Motor_RM_State_t R_F_Wheel_State;

Motor_RM_t R_F_Wheel = {
	.born_info = &R_F_Wheel_Born,
	
	.rx_info = &R_F_Wheel_Rxinfo,
	
	.tx_info = &R_F_Wheel_Txinfo,
	
	.state = &R_F_Wheel_State,
	
	.ctrl = &R_F_Wheel_Ctrl,
	
	.single_init = RM_Motor_Init,
	
};
/*-----------------------左后航向电机-------------------------*/
Motor_RM_Born_Info_t L_B_Wheel_Born ={
	.order_correction = 0,
	
	.rxId = 2,
	
	.stdId = 0x200,
	
	.type = _3508_Single,
	
	.hcan = &hfdcan2,
	
};
Motor_RM_Rx_Info_t L_B_Wheel_Rxinfo;

pid_ctrl_t L_B_Wheel_Ctrlspeed = 
{
	.kp = 6.f,//
	.ki = 0.f,
	.kd = 0.f,
	.a = 1.f,
	.integral_max = 6000.f,
	.out_max = 10000.f,//
};
Motor_RM_Ctrl_Info_t L_B_Wheel_Ctrl ={
	.Speed_Input_Flag = 0,
	.Angle_Input_Flag = 0,
	.speed_ctrl = &L_B_Wheel_Ctrlspeed,
};

Motor_RM_Tx_Info_t L_B_Wheel_Txinfo;

Motor_RM_State_t L_B_Wheel_State;

Motor_RM_t L_B_Wheel = {
	.born_info = &L_B_Wheel_Born,
	
	.rx_info = &L_B_Wheel_Rxinfo,
	
	.tx_info = &L_B_Wheel_Txinfo,
	
	.state = &L_B_Wheel_State,
	
	.ctrl = &L_B_Wheel_Ctrl,
	
	.single_init = RM_Motor_Init,
	
};
/*-----------------------右后航向电机-------------------------*/
Motor_RM_Born_Info_t R_B_Wheel_Born ={
	.order_correction = 0,
	
	.rxId = 3,
	
	.stdId = 0x200,
	
	.type = _3508_Single,
	
	.hcan = &hfdcan2,
	
};
Motor_RM_Rx_Info_t R_B_Wheel_Rxinfo;

pid_ctrl_t R_B_Wheel_Ctrlspeed = 
{
	.kp = 6.f,//
	.ki = 0.f,
	.kd = 0.f,
	.a = 1.f,
	.integral_max = 6000.f,
	.out_max = 10000.f,//
};
Motor_RM_Ctrl_Info_t R_B_Wheel_Ctrl ={
	.Speed_Input_Flag = 0,
	.Angle_Input_Flag = 0,
	.speed_ctrl = &R_B_Wheel_Ctrlspeed,
};

Motor_RM_Tx_Info_t R_B_Wheel_Txinfo;

Motor_RM_State_t R_B_Wheel_State;

Motor_RM_t R_B_Wheel = {
	.born_info = &R_B_Wheel_Born,
	
	.rx_info = &R_B_Wheel_Rxinfo,
	
	.tx_info = &R_B_Wheel_Txinfo,
	
	.state = &R_B_Wheel_State,
	
	.ctrl = &R_B_Wheel_Ctrl,
	
	.single_init = RM_Motor_Init,
	
};
/*-----------------------前向电机组-------------------------*/
Motor_RM_Group_t Front_Group = {
	.motor[0] = &L_F_Se,
	.motor[1] = &R_F_Se,
	.motor[2] = &L_F_Wheel,
	.motor[3] = &R_F_Wheel,
	
	.stdId = 0x200,
	.hcan = &hfdcan1,
	
	.group_init = RM_Group_Motor_Init,
};


/*-----------------------后向电机组-------------------------*/
Motor_RM_Group_t Back_Group = {
	.motor[0] = &L_B_Se,
	.motor[1] = &R_B_Se,
	.motor[2] = &L_B_Wheel,
	.motor[3] = &R_B_Wheel,
	
	.stdId = 0x200,
	.hcan = &hfdcan2,
	
	.group_init = RM_Group_Motor_Init,
};
