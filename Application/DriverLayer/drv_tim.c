/**
 ******************************************************************************
 * @file    drv_tim.c
 * @brief   定时器底层驱动 - 提供定时器初始化和PWM输出功能
 *          本文件实现了TIM1和TIM4定时器的初始化及应用：
 *          1. TIM1: 用于PWM输出（如舵机控制、云台电机控制等）
 *          2. TIM4: 用于产生定时中断和蜂鸣器PWM输出
 *          TIM1通道1输出PWM信号，TIM4通道3用于蜂鸣器驱动
 ******************************************************************************
 * @attention
 *
 * Copyright 2024 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_tim.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/**
  * 舵机/云台控制CCR值（已注释，根据实际硬件调整）
  * val_up:    倍镜抬起时的比较值
  * val_down:  倍镜放下时的比较值
  * val_slep:  倍镜卸力时的比较值（关闭PWM输出）
  */
//#ifdef IS_SIX_FRIC
//uint16_t val_up = 2500;//倍镜抬起CCR
//uint16_t val_down = 1350;//倍镜放下CCR
//#else
//uint16_t val_up = 1200;//倍镜抬起CCR
//uint16_t val_down = 760;//倍镜放下CCR
//#endif
//uint16_t val_slep = 0;//倍镜卸力

/* Exported variables --------------------------------------------------------*/
/**
  * TIM1和TIM4的HAL句柄（在CubeMX中生成）
  * htim1: 定时器1，用于PWM输出
  * htim4: 定时器4，用于蜂鸣器和定时中断
  */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  TIM1初始化 - 启动定时器基础功能并开启PWM输出
  * @param  无
  * @retval 无
  * @note   TIM1通道1的PWM输出通常用于舵机控制或云台电机
  *         PWM频率和占空比由CubeMX配置决定
  */
void TIM1_Init(void)
{
	HAL_TIM_Base_Init(&htim1);           /* 初始化TIM1基础参数（预分频、计数周期等） */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  /* 启动TIM1通道1的PWM输出 */
}

/**
  * @brief  TIM4初始化 - 启动定时器基础功能并开启定时中断
  * @param  无
  * @retval 无
  * @note   TIM4用于产生周期性定时中断（如系统节拍、电机控制周期等）
  *         同时TIM4通道3预留用于蜂鸣器PWM输出
  */
void TIM4_Init(void)
{
	HAL_TIM_Base_Init(&htim4);                    /* 初始化TIM4基础参数 */
	HAL_TIM_Base_Start_IT(&htim4);                /* 启动TIM4并使能更新中断 */
}

/* Servo/蜂鸣器 控制函数 -----------------------------------------------------*/

/**
  * @brief  开启蜂鸣器 - 通过PWM驱动蜂鸣器发声
  * @param  psc:  预分频器值，用于调节蜂鸣器音调（频率）
  *               预分频值越小，PWM频率越高，音调越高
  * @param  pwm:  比较寄存器CCR值，用于调节蜂鸣器音量（占空比）
  *               值越大占空比越高，音量越大
  * @retval 无
  * @note   使用TIM4通道3输出PWM信号驱动蜂鸣器
  *         动态修改预分频器可以在运行时改变蜂鸣器音调
  */
void buzzer_on(uint16_t psc, uint16_t pwm)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);  /* 启动TIM4通道3的PWM输出 */
    __HAL_TIM_PRESCALER(&htim4, psc);            /* 动态设置预分频器，改变PWM频率(音调) */
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);  /* 设置比较值，改变占空比(音量) */
}

/**
  * @brief  关闭蜂鸣器 - 将PWM比较值设为0，停止蜂鸣器发声
  * @param  无
  * @retval 无
  * @note   通过将CCR设为0使占空比为0%，PWM输出为低电平，蜂鸣器停止
  */
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);  /* CCR=0，占空比为0，蜂鸣器静音 */
}

/* 舵机/倍镜控制函数（已注释） ------------------------------------------------*/
/* 以下函数用于控制舵机或倍镜机构的升降，PWM值范围500~2500对应0~180度 */
// 500<CCR<2500，对应0~180°
//void Telescope_Up(void)
//{
//	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, val_up);  // 设置PWM比较值使舵机抬起
//}
//
//void Telescope_Down(void)
//{
//	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, val_down);  // 设置PWM比较值使舵机放下
//}
//
//void Telescope_Sleep(void)
//{
//	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, val_slep);  // 设置CCR=0，舵机卸力
//}


