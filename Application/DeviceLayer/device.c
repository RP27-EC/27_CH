/**
 * @file  device.c
 */
 
/* Includes ------------------------------------------------------------------*/
#include "device.h"


/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
dev_list_t dev_list = {
	.rc_sen     = &rc_sensor,
};

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DEVICE_Init(void)
{
	dev_list.rc_sen->init(dev_list.rc_sen);
	imu_sensor.init(&imu_sensor);
//	
	/*电机初始化*/
	Front_Group.group_init(&Front_Group);
	Back_Group.group_init(&Back_Group);
	Yaw_Motor.single_init(&Yaw_Motor);

	/*裁判系统初始化*/
	My_Judge_Init();;
	cap.init(&cap);
}
