/**
  ******************************************************************************
  * @file           : cap.c\h
	* @author         : czf
	* @date           : 2022.4.28
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */
	
#include "cap.h"
#include "drv_can.h"
void Cap_Init(cap_t* my_cap);
static void Cap_HeartBeat(cap_t* my_cap);
cap_status_t cap_status;
cap_t cap = 
{
	.Y_O_N = 0,
	.record_Y_O_N = 0,
	.info = &cap_receive_data,
	.init = Cap_Init,
	.status = &cap_status,
};

void Cap_Init(cap_t* my_cap)
{
	my_cap->status->offline_cnt_max = 0;
	my_cap->rx = cap_update;
	my_cap->heartbeat = Cap_HeartBeat;
	
}

static void Cap_HeartBeat(cap_t* my_cap)
{
	my_cap->status->offline_cnt ++;
	
	if(my_cap->status->offline_cnt >= my_cap->status->offline_cnt_max)
	{
		my_cap->status->offline_cnt = my_cap->status->offline_cnt_max;
		my_cap->status->status = DEV_OFFLINE;
	}
	else
	{
		my_cap->status->status = DEV_ONLINE;
	}
}
