#ifndef __BUZZER_H
#define __BUZZER_H

#include "tim.h"
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct buzzer_config_struct {
	TIM_HandleTypeDef* tim;
	uint32_t channel;
	
	uint32_t max_tim_arr;
	uint32_t tim_freq;
	
	float min_pwm_duty;
	float max_pwm_duty;
	float	 min_volume;
	float 	 max_volume;
}buzzer_config_t;


// 音量-音符-时长结构体：绑定每个音符的频率和播放时长
typedef struct note_duration {
	float volume;				/* 音量 */
    float freq;     			/* 音符频率（Hz） */
    uint32_t duration; 			/* 播放时长（毫秒） */
}note_duration_t;

/* 目标结构体 */
typedef struct buzzer_input_info_struct {
	float volume;				 
    float freq;   		
}buzzer_input_info_t;

typedef struct buzzer_status_struct {
	buzzer_input_info_t input_info;
	uint16_t tim_presc;
	float duty;
  	uint16_t CCR;
	float ARR_raw;
	uint16_t ARR;
}buzzer_base_info_t;

typedef struct buzzer_struct
{
	buzzer_config_t config;
	buzzer_base_info_t base_info;

	void (*init)(struct buzzer_struct* buz_str);
	void (*work)(struct buzzer_struct* buz_str);
} buzzer_t;


/* Exported function --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern buzzer_t buzzer;


#endif

