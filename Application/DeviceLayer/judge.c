/**
  ******************************************************************************
  * @file           : judge.c
  * @brief          : 裁判系统数据处理模块
  * @details        : 接收和解析RoboMaster比赛裁判系统的数据，包括：
  *                   - 比赛状态信息（游戏状态、比赛阶段等）
  *                   - 机器人状态（血量、装甲伤害等）
  *                   - 功率和热量数据（底盘功率限制、枪管热量等）
  *                   - 射击数据（弹丸速度、弹丸剩余量等）
  *                   - RFID状态、血量数据等
  *                   通过CAN总线接收裁判系统主机发送的数据帧。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "judge.h"
#include "imu_sensor.h"
#include "cap.h"
#include "drv_tick.h"

/* 裁判系统处理后的信息（供上层算法使用） */
Judge_Info_t Judge_Info;

/* 裁判系统原始数据（从CAN直接解析的结构体） */
Judge_Org_Info_t Judge_Org_Info;

/**
  * @brief  裁判系统状态信息
  * @details offline_cnt_max: 最大离线计数阈值，超过此值判定裁判系统离线
  */
Judge_Status_t Judge_Status =
{
	.offline_cnt_max = JUDGE_OFFLINE_CNT_MAX,
};

/**
  * @brief  裁判系统完整对象
  * @details 组合原始数据、处理后数据和状态信息
  */
My_Judge_t My_Judge =
{
	.org_info = &Judge_Org_Info,   /* 指向原始数据缓冲区 */
	.info = &Judge_Info,           /* 指向处理后的数据 */
	.status = &Judge_Status,       /* 指向状态信息 */
};


/**
  * @brief  裁判系统初始化
  * @retval None
  * @details 将离线计数器设为最大值，标记为离线状态
  *          等待收到第一帧裁判系统数据后会自动恢复在线
  */
void My_Judge_Init()
{
	/* 离线计数器设为最大值，初始判定为离线 */
	My_Judge.status->offline_cnt = My_Judge.status->offline_cnt_max;
	My_Judge.status->status = DEV_OFFLINE;
}

/**
  * @brief  裁判系统实时任务（周期性调用）
  * @retval None
  * @details 功能：
  *          1. 离线检测：累加离线计数器，超过阈值则重新初始化
  *          2. 数据更新：将原始数据转换为上层可用的格式
  * @note   需要周期性调用以维持心跳检测和数据同步
  */
void My_Judge_Realtime_Task(My_Judge_t* my_judge)
{
	/* 离线计数器递增 */
	my_judge->status->offline_cnt++;

	/* 超过离线阈值，重新初始化裁判系统 */
	if(my_judge->status->offline_cnt >= my_judge->status->offline_cnt_max)
	{
		My_Judge_Init();
	}

	/* 更新处理后的数据 */
	My_Judge_Update(my_judge);
}

/**
  * @brief  裁判系统数据更新（将原始数据转换为简化格式）
  * @param  my_judge: 裁判系统对象指针
  * @retval None
  * @details 从原始数据结构体中提取关键信息到简化结构体，供上层算法使用：
  *          - remain_HP_now: 当前剩余血量
  *          - car_color: 机器人ID（用于判断红蓝方）
  *          - chassis_power_buffer: 底盘功率缓冲区
  *          - chassis_power_limit: 底盘功率限制值
  *          - shooter_cooling_limit: 枪管热量冷却上限
  *          - shooting_speed: 弹丸射速
  *          - shooter_cooling_heat: 当前枪管热量
  */
void My_Judge_Update(My_Judge_t * my_judge)
{
	/* 更新当前剩余血量 */
	my_judge->info->remain_HP_now = my_judge->info->remain_HP;

	/* 获取机器人ID（判断红蓝方：1~7为红方，101~107为蓝方） */
	my_judge->info->car_color = my_judge->org_info->game_robot_status.robot_id;

	/* 获取底盘功率缓冲区（功率缓冲区>0时可以短暂超功率运行） */
	my_judge->info->chassis_power_buffer = my_judge->org_info->power_heat_data.chassis_power_buffer;

	/* 获取底盘功率限制值（底盘总功率上限，单位：W） */
	my_judge->info->chassis_power_limit = my_judge->org_info->game_robot_status.chassis_power_limit;

	/* 获取枪管热量冷却限制值 */
	my_judge->info->shooter_cooling_limit = my_judge->org_info->game_robot_status.shooter_barrel_heat_limit;

	/* 获取弹丸射速（单位：m/s） */
	my_judge->info->shooting_speed = my_judge->org_info->shoot_data.bullet_speed;

	/* 获取当前17mm枪管热量 */
	my_judge->info->shooter_cooling_heat = my_judge->org_info->power_heat_data.shooter_id1_17mm_cooling_heat;
}

/* 射击计数器和警告标志（调试用） */
uint16_t shoot_cnt11 = 0;
uint8_t warning;

/**
  * @brief  裁判系统CAN数据接收回调函数
  * @param  id: CAN消息ID，标识不同的数据帧类型
  * @param  rxBuf: 接收到的原始数据缓冲区
  * @retval None
  * @details 根据消息ID将数据拷贝到对应的原始数据结构体中，
  *          并重置离线计数器，标记设备在线。
  *          支持的数据帧类型：
  *          - ID_rfid_status: RFID状态（触发特殊效果）
  *          - ID_game_state: 比赛状态（开始/结束/倒计时等）
  *          - ID_power_heat_data: 功率和热量数据（触发超级电容发送）
  *          - ID_game_robot_state: 机器人状态（血量、功率限制等）
  *          - ID_shoot_data: 射击数据（弹速等）
  *          - ID_game_robot_HP: 机器人血量数据
  *          - ID_bullet_remaining: 弹丸剩余量
  */
void judge_update(uint16_t id, uint8_t *rxBuf)
{
	switch(id)
	{
		case ID_rfid_status:
			/* RFID状态数据，用于触发地图上的特殊效果 */
			memcpy(&My_Judge.org_info->rfid_status, rxBuf, LEN_rfid_status);
			My_Judge.status->offline_cnt = 0;           /* 重置离线计数器 */
			My_Judge.status->status = DEV_ONLINE;       /* 标记在线 */
			break;

		case ID_game_state:
			/* 比赛状态数据（比赛阶段、剩余时间等） */
			memcpy(&My_Judge.org_info->game_status, rxBuf, LEN_game_state);
			My_Judge.status->offline_cnt = 0;
			My_Judge.status->status = DEV_ONLINE;
			break;

		case ID_power_heat_data:
			/* 功率和热量数据（底盘功率、枪管热量等） */
			memcpy(&My_Judge.org_info->power_heat_data, rxBuf, LEN_power_heat_data);
			My_Judge.status->offline_cnt = 0;
			My_Judge.status->status = DEV_ONLINE;
			cap_send_2E();  /* 收到功率数据后触发超级电容数据发送 */
			break;

		case ID_game_robot_state:
			/* 机器人状态数据（血量、功率限制、冷却限制等） */
			memcpy(&My_Judge.org_info->game_robot_status, rxBuf, LEN_game_robot_state);
			My_Judge.status->offline_cnt = 0;
			My_Judge.status->status = DEV_ONLINE;
			break;

		case ID_shoot_data:
			/* 射击数据（弹丸编号、弹速等） */
			memcpy(&My_Judge.org_info->shoot_data, rxBuf, LEN_shoot_data);
			My_Judge.status->offline_cnt = 0;
			My_Judge.status->status = DEV_ONLINE;
			break;

		case ID_game_robot_HP:
			/* 机器人血量数据（自身和对手血量） */
			memcpy(&My_Judge.org_info->ext_game_robot_HP, rxBuf, LEN_game_robot_HP);
			My_Judge.status->offline_cnt = 0;
			My_Judge.status->status = DEV_ONLINE;
			break;

		case ID_robot_hurt:
			/* 机器人伤害数据（当前未启用） */
			break;

		case ID_bullet_remaining:
			/* 弹丸剩余量数据 */
			memcpy(&My_Judge.org_info->ext_bullet_remaining, rxBuf, LEN_bullet_remaining);
			My_Judge.status->offline_cnt = 0;
			My_Judge.status->status = DEV_ONLINE;
			break;

		default:
			break;
	}
}

