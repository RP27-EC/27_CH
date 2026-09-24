/**
  ******************************************************************************
  * @file           : cap_protocol.c\h
	* @author         : czf
	* @date           : 2022-4-22 15:41:14
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */
	
#include "cap_protocol.h"
#include "string.h"
#include "drv_can.h"
#include "cap.h"
#include "Judge.h"

cap_receive_data_t cap_receive_data;
cap_rx_info_t cap_rx_info;
cap_transmit_data_t cap_tx_info;
uint8_t cap_tx_buf[8]; //0x222

void cap_send_2E(void)
{
	cap_tx_info.chassis_power_buffer = My_Judge.org_info->power_heat_data.chassis_power_buffer;
	cap_tx_info.chassis_power_limit = My_Judge.org_info->game_robot_status.chassis_power_limit;
	cap_tx_info.cap_power_in_limit = 300;
	cap_tx_info.cap_power_out_limit = -300;
	cap_tx_info.bit_control.turbo_mode = 0;
	
	if(cap.Y_O_N == 1)
	{
		cap_tx_info.bit_control.cap_switch = 1;
	}
	else
	{
		cap_tx_info.bit_control.cap_switch = 1;
	}
	memcpy(cap_tx_buf, &cap_tx_info, sizeof(cap_transmit_data_t));
	
	CAN_SendData(&hfdcan3, 0x222, cap_tx_buf);
}


void cap_update(cap_t* my_cap, uint8_t *rxBuf)
{
	memcpy(&cap_rx_info, rxBuf, sizeof(cap_rx_info_t));
	
	my_cap->info->chassis_power = cap_rx_info.now_chassis_power;
	my_cap->info->cap_I = int16_to_float(cap_rx_info.now_cap_I, 32000, -32000, 16, -16);
	my_cap->info->cap_Ucr = int16_to_float(cap_rx_info.now_cap_V, 32000, -32000, 25, 0);
	my_cap->info->ability = cap_rx_info.bit_state.ability;
	
	cap.status->offline_cnt = 0;
}

float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min)
{
    int32_t a_32 = a, a_max_32 = a_max, a_min_32 = a_min;
    int32_t diff_a = a_max_32 - a_min_32;
    
    if (diff_a == 0) return (b_max + b_min) / 2.0f; // 处理除零
    
    float ratio = (float)(a_32 - a_min_32) / (float)diff_a;
    return ratio * (b_max - b_min) + b_min;
}

int16_t float_to_int16(float b, float b_max, float b_min, int16_t a_max, int16_t a_min)
{
    // 处理除零和无效输入
    if (b_max == b_min) return (int16_t)((a_max + a_min) / 2);
    
    // 计算比例并映射到整数范围
    float ratio = (b - b_min) / (b_max - b_min);
    
    // 提升计算范围避免溢出
    int32_t a = (int32_t)(ratio * (a_max - a_min) + a_min + 0.5f); // 四舍五入
    
    // 钳位到目标范围
    a = (a < a_min) ? a_min : (a > a_max) ? a_max : a;
    
    return (int16_t)a;
}
