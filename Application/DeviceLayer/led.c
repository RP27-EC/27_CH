/**
 * @file    led.c
 * @brief   C板 LED 灯控制模块
 *
 * 本文件实现了 C 板（开发板）板载 RGB LED 的控制逻辑。
 * C 板上有一个 RGB LED，由红（R）、蓝（B）、绿（G）三个独立的 LED 组成。
 * 同一时刻只能点亮一种颜色，对应一种显示状态（常亮、闪烁、关闭）。
 *
 * 使用方式：
 *   1. 修改 led 变量的 colour 和 state 字段
 *   2. 在主循环或定时任务中调用 led_work(&led) 更新 LED 状态
 *
 * 支持的颜色：红色、蓝色、绿色
 * 支持的状态：常亮(ON)、闪烁(BLINK)、关闭(OFF)
 *
 * @note   闪烁频率通过 led->blink_fre 设置，单位为 Hz
 * @version V1.0
 * @date    2026-06-11
 */

#include "led.h"  /* LED 驱动头文件，包含 LED GPIO 宏定义和 led_t 结构体定义 */


/**
 * @brief LED 全局实例定义
 * 初始状态为红色关闭，闪烁频率 5Hz
 * 可在其他模块中通过 extern 直接使用此全局变量
 */
led_t led = {
	.state = LED_OFF,           /* 初始状态：关闭 */
	.colour = LED_colour_red,   /* 初始颜色：红色 */
	.blink_fre = 5,             /* 闪烁频率：5Hz（每秒闪烁5次） */
};

/**
 * @brief  C板 LED 总控制函数
 *
 * 根据 led 结构体中的 colour（颜色）和 state（状态）字段，
 * 控制 RGB LED 的亮灭和闪烁。
 *
 * 工作原理：
 *   - 先判断是否处于 OFF 状态，若 OFF 则熄灭所有 LED
 *   - 根据颜色选择对应的 RGB 组合
 *   - 再根据状态决定是常亮还是闪烁
 *   - 闪烁通过 HAL_GetTick() 实现非阻塞定时翻转
 *
 * @param[in,out] led  LED 控制结构体指针，包含颜色、状态和闪烁频率信息
 *
 * @note   每次调用只会显示一种颜色的灯，三色互斥
 * @note   闪烁使用静态变量记录上次翻转时间，实现非阻塞闪烁
 * @note   需要在主循环或定时任务中周期性调用此函数以维持闪烁效果
 */
void led_work(led_t *led)
{
	/* 防止闪烁频率为 0 导致除零错误，设置最低频率为 1Hz */
	if(led->blink_fre == 0)
	{
		led->blink_fre = 1;  /* 最低闪烁频率设为 1Hz */
	}

	/**
	 * 计算闪烁翻转间隔（毫秒）
	 * 公式：翻转间隔 = (1 / 频率) * 1000 / 2
	 * 例如：频率 = 5Hz，则周期 = 200ms，翻转间隔 = 100ms
	 */
	uint32_t toggle_tick_cnt = 1 / led->blink_fre * 1000 / 2;
	/* 静态变量：记录上次 LED 翻转的时间戳，用于非阻塞定时 */
	static uint32_t last_toggle_tick;

	/* 若状态为关闭，则熄灭所有颜色的 LED */
	if(led->state == LED_OFF)
	{
		LED_RED_OFF;    /* 关闭红色 LED */
		LED_BLUE_OFF;   /* 关闭蓝色 LED */
		LED_GREEN_OFF;  /* 关闭绿色 LED */
	}

	/* 根据颜色分支处理 */
	switch (led->colour)
	{
		/* ==================== 红色 LED 控制 ==================== */
		case LED_colour_red:
		{
			switch (led->state)
			{
				/* 红色常亮：点亮红色，关闭蓝色和绿色 */
				case LED_ON:
				{
					LED_RED_ON;     /* 点亮红色 LED */
					LED_BLUE_OFF;   /* 确保蓝色 LED 关闭 */
					LED_GREEN_OFF;  /* 确保绿色 LED 关闭 */
					break;
				}

				/* 红色闪烁：按设定频率翻转红色 LED，保持其他颜色关闭 */
				case LED_BLINK:
				{
					/* 判断是否到达翻转时间 */
					if(HAL_GetTick() - last_toggle_tick > toggle_tick_cnt)
					{
						LED_RED_Toggle;              /* 翻转红色 LED 电平（亮->灭 或 灭->亮） */
						last_toggle_tick = HAL_GetTick();  /* 更新上次翻转时间戳 */
					}
					LED_BLUE_OFF;   /* 确保蓝色 LED 关闭 */
					LED_GREEN_OFF;  /* 确保绿色 LED 关闭 */
					break;
				}

				default:
					break;
			}
			break;
		}

		/* ==================== 蓝色 LED 控制 ==================== */
		case LED_colour_blue:
		{
			switch (led->state)
			{
				/* 蓝色常亮：点亮蓝色，关闭红色和绿色 */
				case LED_ON:
				{
					LED_RED_OFF;    /* 确保红色 LED 关闭 */
					LED_BLUE_ON;    /* 点亮蓝色 LED */
					LED_GREEN_OFF;  /* 确保绿色 LED 关闭 */
					break;
				}

				/* 蓝色闪烁：按设定频率翻转蓝色 LED */
				case LED_BLINK:
				{
					/* 判断是否到达翻转时间 */
					if(HAL_GetTick() - last_toggle_tick > toggle_tick_cnt)
					{
						LED_BLUE_Toggle;              /* 翻转蓝色 LED 电平 */
						last_toggle_tick = HAL_GetTick();  /* 更新上次翻转时间戳 */
					}
					LED_RED_OFF;    /* 确保红色 LED 关闭 */
					LED_GREEN_OFF;  /* 确保绿色 LED 关闭 */
					break;
				}

				default:
					break;
			}
			break;
		}

		/* ==================== 绿色 LED 控制 ==================== */
		case LED_colour_green:
		{
			switch (led->state)
			{
				/* 绿色常亮：点亮绿色，关闭红色和蓝色 */
				case LED_ON:
				{
					LED_RED_OFF;    /* 确保红色 LED 关闭 */
					LED_BLUE_OFF;   /* 确保蓝色 LED 关闭 */
					LED_GREEN_ON;   /* 点亮绿色 LED */
					break;
				}

				/* 绿色闪烁：按设定频率翻转绿色 LED */
				case LED_BLINK:
				{
					/* 判断是否到达翻转时间 */
					if(HAL_GetTick() - last_toggle_tick > toggle_tick_cnt)
					{
						LED_GREEN_Toggle;             /* 翻转绿色 LED 电平 */
						last_toggle_tick = HAL_GetTick();  /* 更新上次翻转时间戳 */
					}
					LED_RED_OFF;    /* 确保红色 LED 关闭 */
					LED_BLUE_OFF;   /* 确保蓝色 LED 关闭 */
					break;
				}

				default:
					break;
			}
			break;
		}

		default:
			break;
	}
}
