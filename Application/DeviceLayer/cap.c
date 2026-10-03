/**
  ******************************************************************************
  * @file           : cap.c
	* @author         : czf
	* @date           : 2022.4.28
  * @brief          : 电容模块驱动文件，负责管理超级电容的通信与状态监控
	* @details        : 通过CAN总线与电容板通信，获取电容电压、功率等信息，
	*                   并维护电容设备的在线/离线心跳检测机制。
	* @history        : 2022.4.28 初始版本
  ******************************************************************************
  */

/* 包含电容模块头文件和CAN驱动头文件 */
#include "cap.h"
#include "drv_can.h"

/* 函数前向声明 */
void Cap_Init(cap_t* my_cap);      /* 电容初始化函数 */
static void Cap_HeartBeat(cap_t* my_cap);  /* 电容心跳检测函数（仅文件内部使用） */

/* 电容状态全局变量，记录电容在线/离线等状态信息 */
cap_status_t cap_status;

/**
  * @brief  电容模块全局对象定义
  * @details 初始化电容对象的各成员：
  *          - Y_O_N: 电容开关状态（0=关闭）
  *          - record_Y_O_N: 上次电容开关状态记录
  *          - info: 指向电容接收数据的指针，存储电压、功率等信息
  *          - init: 初始化函数指针
  *          - status: 指向电容状态信息的指针
  */
cap_t cap =
{
	.Y_O_N = 0,                /* 电容默认关闭 */
	.record_Y_O_N = 0,         /* 上次开关状态记录，初始为0 */
	.info = &cap_receive_data, /* 指向电容CAN接收数据缓冲区 */
	.init = Cap_Init,          /* 绑定初始化函数 */
	.status = &cap_status,     /* 绑定状态信息 */
};

/**
  * @brief  电容模块初始化
  * @param  my_cap: 电容对象指针
  * @retval None
  * @details 初始化电容的离线计数最大值、数据接收回调和心跳回调函数
  */
void Cap_Init(cap_t* my_cap)
{
	/* 离线计数最大值设为0，表示不进行离线检测（电容板可能无心跳机制） */
	my_cap->status->offline_cnt_max = 0;

	/* 绑定电容数据更新回调函数，当收到CAN数据时调用 */
	my_cap->rx = cap_update;

	/* 绑定心跳检测函数，周期性调用以检测设备在线状态 */
	my_cap->heartbeat = Cap_HeartBeat;
}

/**
  * @brief  电容心跳检测函数
  * @param  my_cap: 电容对象指针
  * @retval None
  * @details 每次调用时离线计数器加1，当计数超过最大值时判定设备离线，
  *          否则判定设备在线。用于检测电容板通信是否正常。
  */
static void Cap_HeartBeat(cap_t* my_cap)
{
	/* 离线计数器递增 */
	my_cap->status->offline_cnt ++;

	/* 判断是否超过离线阈值 */
	if(my_cap->status->offline_cnt >= my_cap->status->offline_cnt_max)
	{
		/* 超过阈值，将计数器钳位在最大值，标记设备离线 */
		my_cap->status->offline_cnt = my_cap->status->offline_cnt_max;
		my_cap->status->status = DEV_OFFLINE;
	}
	else
	{
		/* 未超过阈值，标记设备在线 */
		my_cap->status->status = DEV_ONLINE;
	}
}
