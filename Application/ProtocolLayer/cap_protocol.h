#ifndef __CAP_PROTOCOL_H
#define __CAP_PROTOCOL_H

#include "main.h"
typedef struct __attribute__((packed)) cap_rx_info_struct {
    
    int16_t now_chassis_power;              // 当前底盘消耗功率
    int16_t now_cap_V;                      // 当前电容组电压
    int16_t now_cap_I;                      // 当前电容组电流
    
    struct __attribute__((packed)) bit_state_struct
    {
        uint8_t ability             : 1;    // 电容是否有放电能力，0为无，1为有
        uint8_t unuse               : 7;    // 暂时未使用
    }bit_state;
    
} cap_rx_info_t;

typedef struct
{
		int16_t chassis_power;
    float cap_Ucr;    //电容两端电压Ucr，0~30V
    float cap_I;    //电容电流I，-20~20A
    
		uint8_t ability;// 电容是否有放电能力，0为无，1为有
}cap_receive_data_t;

typedef struct __attribute__((packed))cap_transmit_data_struct {
    
    uint8_t  chassis_power_buffer;          // 底盘能量缓冲
    uint16_t chassis_power_limit ;          // 机器人底盘功率限制上限
    int16_t  cap_power_out_limit ;          // 电容放电功率限制，定义为负值
    uint16_t cap_power_in_limit  ;          // 电容充电功率限制，定义为正值
    
    struct __attribute__((packed)) bit_control_struct
    {
        uint8_t cap_switch : 1;             // 电容开关，1为开，0为关
        uint8_t turbo_mode : 1;             // 是否使用缓冲能量来充电，0为不用，1为用
        uint8_t unuse      : 6;             // 暂时未使用
    }bit_control;
    
}cap_transmit_data_t;

typedef struct
{
	uint16_t offline_cnt_max;
	uint8_t status;
	uint16_t offline_cnt;
}cap_status_t;

typedef struct cap_struct_t
{
	uint8_t Y_O_N;
	uint8_t record_Y_O_N;
	cap_status_t* status;
	cap_receive_data_t* info;
	
	void (*heartbeat)(struct cap_struct_t *my_cap);
	void (*rx)(struct cap_struct_t* my_cap, uint8_t *rxBuf);
	void (*init)(struct cap_struct_t *my_cap);
}cap_t;

extern cap_receive_data_t cap_receive_data;

void cap_send_2E(void);
void cap_send_2F(void);
void cap_update(cap_t* my_cap, uint8_t *rxBuf);
int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min);
float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min);

#endif



