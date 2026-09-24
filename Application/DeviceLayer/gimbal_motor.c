#include "gimbal_motor.h"
#include "DM_Motor.h"
/*yawµç»ú*/
Motor_DM_Born_Info_t Yaw_Born_Info =
{
	.stdId = 0x002,
	
	.hcan = &hfdcan1,

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