#ifndef __JUDGE_H
#define __JUDGE_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "rp_config.h"
#include <stdbool.h>
#include "judge_protocol.h"
/* Exported macro ------------------------------------------------------------*/
#define JUDGE_OFFLINE_CNT_MAX 1000

typedef struct Judge_Org_Info_struct_t
{
	ext_rfid_status_t rfid_status;
	ext_game_status_t game_status;
	ext_game_robot_status_t game_robot_status;
	ext_power_heat_data_t power_heat_data;
	ext_shoot_data_t shoot_data;
	ext_game_robot_pos_t game_robot_pos;
	ext_robot_hurt_t ext_robot_hurt;
	ext_game_robot_HP_t ext_game_robot_HP;
  ext_bullet_remaining_t ext_bullet_remaining;
}Judge_Org_Info_t;

typedef struct
{
	int16_t chassis_power_buffer;           //底盘缓存功率
	int32_t chassis_out_put_max;            //底盘最大输出
	uint16_t shooter_cooling_limit;					//机器人 17mm 枪口热量上限
	uint16_t shooter_cooling_heat; 					//机器人 17mm 枪口热量
	uint8_t car_color;                      //2蓝色 1红色
	uint8_t hurt_type;                      //伤害种类
	uint16_t chassis_power_limit;           //底盘功率限制
	uint16_t shooter_id1_17mm_speed_limit;  //射速上限
	uint16_t remain_HP;                     //剩余血量
	uint8_t game_status;                    //比赛状态
	uint16_t remain_HP_now;
	uint16_t remain_HP_last;
	uint8_t rfid;
	float shooting_speed;
}Judge_Info_t;

/* 0x0201 已通过 CRC 后的处理结果，供 Keil Watch 查看。 */
typedef enum
{
	JUDGE_ROBOT_STATE_NONE = 0,
	JUDGE_ROBOT_STATE_ACCEPTED = 1,
	JUDGE_ROBOT_STATE_BAD_LENGTH = 2,
	JUDGE_ROBOT_STATE_ZERO_LIMIT = 3
} Judge_Robot_State_Result_t;

typedef struct
{
	uint16_t offline_cnt_max;
	uint8_t status;
	uint16_t offline_cnt;
	volatile bool limit_received, buffer_received;
	volatile uint32_t limit_rx_count, limit_update_ms, buffer_rx_count, buffer_update_ms;
	/* 以下记录所有 CRC 正确的 0x0201 尝试；limit_* 只记录被接纳的有效上限。 */
	volatile uint32_t robot_state_rx_count, robot_state_update_ms;
	volatile uint16_t robot_state_length, robot_state_limit_raw;
	volatile uint8_t robot_state_result; /* 对应 Judge_Robot_State_Result_t。 */
}Judge_Status_t;

typedef struct
{
	Judge_Org_Info_t* org_info;
	Judge_Info_t* info;
	Judge_Status_t* status;
}My_Judge_t;

extern My_Judge_t My_Judge;

void My_Judge_Init(void);
void My_Judge_Realtime_Task(My_Judge_t* my_judge);
void My_Judge_Update(My_Judge_t * my_judge);
void judge_update(uint16_t id, const uint8_t *rxBuf, uint16_t length);

#endif
