/**
  ******************************************************************************
  * @file        rp_math.c
  * @author      RobotPilots
  * @Version     v1.1
  * @brief       RobotPilots机器人数学库
  * @update
  *              v1.0(11-September-2020)  - 初始版本，包含基础数学函数
  *              v1.1(13-November-2021)   - 增加位操作函数
  ******************************************************************************
  * @details
  * 本文件是RobotPilots机器人的核心数学库，提供常用的数学工具函数。
  * 这些函数被算法层的其他模块广泛使用，包括滤波、PID控制、功率限制等。
  *
  * 包含的功能模块：
  *   - 低通滤波函数（Lowpass）
  *   - 角度过半圈处理（motor_half_cycle）
  *   - 斜坡/渐变函数（RampInt / RampFloat）
  *   - 死区处理函数（DeathZoom）
  *   - 循环内定时触发器（Time_Trigger_inloop）
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "rp_math.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 * @brief  一阶低通滤波器（与 low_pass_filter.c 中的函数功能相同）
 * @param  X_last: 上一次的滤波输出值（历史值）
 * @param  X_new:  当前时刻的原始输入值
 * @param  K:      滤波系数，取值范围 (0, 1)
 *                 - K 越大，滤波效果越弱，输出越接近新输入，响应越快
 *                 - K 越小，滤波效果越强，输出越平滑，但延迟越大
 */
float Lowpass(float X_last, float X_new, float K)
{
	/*K越大越信任新数据，(1-K)越大越信任历史数据*/
	return (X_last + (X_new - X_last) * K);
}

/**
 * @brief  单圈编码器误差“就近原则”解算（最短路径，防止转 350° 大回旋）
 * @param  err: 原始误差 (target - measure)，范围通常在 [-max, +max]
 * @param  max: 编码器一圈的总数值（如 13位大疆编码器为 8192.0f，16位为 65536.0f）
 * @retval float 归一化到 [-max/2, +max/2] 的最短路径误差
 */
float motor_half_cycle(float err, float max)
{
    float half_max = max * 0.5f;

    /* 超过正半圈，说明反向走更近（减去一整圈） */
    if (err > half_max)
    {
        err -= max;
    }
    /* 超过负半圈，说明正向走更近（加上一整圈） */
    else if (err < -half_max)
    {
        err += max;
    }

    return err; /* 返回就近的最短误差 */
}

/**
 * @brief  整数斜坡/渐变函数（限速变化）
 * @param  final: 目标值
 * @param  now:   当前值
 * @param  ramp:  每次调用允许的最大变化量（步进值）
 * @retval int16_t 经过步进限制后的当前值
 * @note   本函数使 now 值以不超过 ramp 的步进量向 final 逼近。
 *         每次调用最多变化 ramp 大小，若剩余差距小于 ramp 则直接到达目标。
 *
 *         典型应用：
 *         - 电机启动/停止时的速度渐变，避免突变导致的冲击
 *         - 云台角度的平滑过渡
 *
 *         示例（ramp=100）：
 *           now=1000, final=1200 -> 返回 1100（步进100）
 *           now=1150, final=1200 -> 返回 1200（剩余50小于步进，直接到达）
 *           now=1200, final=1000 -> 返回 1100（反方向步进-100）
 */
int16_t RampInt(int16_t final, int16_t now, int16_t ramp)
{
	int32_t buffer = 0;

	buffer = final - now;  /* 计算当前值与目标值的差值 */

	if (buffer > 0)
	{
		/* 目标值比当前值大，需要正向增加 */
		if (buffer > ramp)
			now += ramp;      /* 差值大于步进量，按步进量增加 */
		else
			now += buffer;    /* 差值小于步进量，直接到达目标值 */
	}
	else
	{
		/* 目标值比当前值小，需要反向减小 */
		if (buffer < -ramp)
			now += -ramp;     /* 差值的绝对值大于步进量，按步进量减小 */
		else
			now += buffer;    /* 差值的绝对值小于步进量，直接到达目标值 */
	}

	return now;  /* 返回步进限制后的当前值 */
}

/**
 * @brief  浮点数斜坡/渐变函数（限速变化）
 * @param  final: 目标值
 * @param  now:   当前值
 * @param  ramp:  每次调用允许的最大变化量（步进值）
 * @retval float  经过步进限制后的当前值
 * @note   功能与 RampInt 相同，但使用浮点数运算，精度更高。
 *         适用于需要小数精度的控制场景。
 *
 *         使用示例：
 *         target_speed = RampFloat(target_speed, current_speed, 0.5f);
 */
float RampFloat(float final, float now, float ramp)
{
	float buffer = 0;

	buffer = final - now;  /* 计算当前值与目标值的差值 */

	if (buffer > 0)
	{
		/* 目标值比当前值大，正向增加 */
		if (buffer > ramp)
			now += ramp;      /* 按步进量增加 */
		else
			now += buffer;    /* 直接到达目标值 */
	}
	else
	{
		/* 目标值比当前值小，反向减小 */
		if (buffer < -ramp)
			now += -ramp;     /* 按步进量减小 */
		else
			now += buffer;    /* 直接到达目标值 */
	}

	return now;  /* 返回步进限制后的当前值 */
}


/**
 * @brief  死区处理函数（Dead Zone）
 * @param  input:  输入值
 * @param  center: 死区中心点（通常为0）
 * @param  death:  死区半径（从中心点向两侧延伸的范围）
 * @retval float   处理后的输出值
 * @note   死区处理用于消除小范围内的噪声或抖动：
 *         - 若 |input - center| < death（输入在死区内），返回 center
 *         - 若 |input - center| >= death（输入在死区外），返回 input
 *
 *         典型应用：
 *         - 摇杆遥控器的居中死区，避免轻微偏移导致的误动作
 *         - 电位器零点附近的噪声抑制
 *
 *         示例（center=0, death=100）：
 *           input=50   -> |50-0|<100   -> 返回 0（在死区内）
 *           input=200  -> |200-0|>=100 -> 返回 200（在死区外）
 *           input=-80  -> |-80-0|<100  -> 返回 0（在死区内）
 */
float DeathZoom(float input, float center, float death)
{
	if (abs(input - center) < death)
		return center;   /* 输入在死区内，返回中心值（消除抖动） */
	return input;         /* 输入在死区外，原样输出 */
}

/**
  * @name   Time_Trigger_inloop
  * @brief  循环内定时触发器
  * @note   ①需要外部定义变量存时间、第一次不直接触发标志位,防止多处调用共用时间或标志位导致错误
  *         ②要么自己外部清标志位，要么执行的内容要紧跟这个函数后面，不然容易错过触发时间
  *         ③*ignore_first_trigger_flag需要初始为0
  *         ④*ignore_first_trigger_flag 退出长按后要清零
  * @param  Time_trigger_struct: 定时触发器结构体指针，包含以下成员：
  *         - private_flag:              触发标志位指针（1=触发，0=未触发）
  *         - last_trigger_tick:         上次触发时间的指针（外部变量，需初始化为0）
  *         - ignore_first_trigger_flag: 首次触发忽略标志位的指针（需初始化为0）
  *         - delay_tick:                触发间隔时间（单位：毫秒）
  *         - if_ignore_first:           是否忽略第一次触发（1=忽略，0=不忽略）
  *         - flag_before_trigger:       触发前的标志位状态（用于未触发时保持原状态）
  * @author HERMIT_PURPLE
  *
  * 使用场景：
  *   在RTOS或裸机的主循环中，需要每隔固定时间执行一次某段代码，
  *   但又不想使用定时器中断时，可以使用此函数。
  *
  * 使用示例：
  *   // 定义触发器结构体
  *   Time_trigger_t myTrigger = {
  *       .private_flag = &flag,
  *       .last_trigger_tick = &lastTick,
  *       .ignore_first_trigger_flag = &ignoreFirst,
  *       .delay_tick = 10,  // 10ms触发一次
  *       .if_ignore_first = 1,
  *       .flag_before_trigger = 0
  *   };
  *   // 在循环中调用
  *   while(1) {
  *       Time_Trigger_inloop(&myTrigger);
  *       if (flag) {
  *           // 执行定时任务
  *           flag = 0;
  *       }
  *   }
  */
void Time_Trigger_inloop(Time_trigger_t *Time_trigger_struct)
{
	/* 参数合法性检查：所有指针不能为空 */
	if (Time_trigger_struct->private_flag == NULL ||
		Time_trigger_struct->last_trigger_tick == NULL ||
		Time_trigger_struct->ignore_first_trigger_flag == NULL)
	{
		return;  /* 指针为空，直接返回，避免空指针解引用 */
	}

	/*
	 * 首次触发忽略机制：
	 * 如果设置了忽略第一次触发（if_ignore_first == 1），
	 * 且当前是首次进入（ignore_first_trigger_flag == 0），
	 * 则记录当前时间作为"上次触发时间"，跳过本次触发。
	 * 这样可以避免在初始化阶段意外触发一次任务。
	 */
	if (Time_trigger_struct->if_ignore_first == 1 &&
		Time_trigger_struct->ignore_first_trigger_flag == 0)
	{
		*Time_trigger_struct->ignore_first_trigger_flag = 1;  /* 标记已忽略过首次 */
		*Time_trigger_struct->last_trigger_tick = HAL_GetTick();  /* 记录当前时间 */
	}

	/*
	 * 定时触发判断：
	 * 条件1：当前时间 - 上次触发时间 > 设定的延迟间隔
	 * 条件2：已经过了首次忽略阶段（或不需要忽略首次）
	 * 两个条件同时满足时触发
	 */
	if ((HAL_GetTick() - *Time_trigger_struct->last_trigger_tick > Time_trigger_struct->delay_tick) &&
		(Time_trigger_struct->ignore_first_trigger_flag != 0 || Time_trigger_struct->if_ignore_first == 0))
	{
		*Time_trigger_struct->private_flag = 1;                /* 设置触发标志 */
		*Time_trigger_struct->last_trigger_tick = HAL_GetTick(); /* 更新上次触发时间 */
	}
	/* 未达到触发条件时，保持标志位为触发前的状态 */
	else
	{
		*Time_trigger_struct->private_flag = Time_trigger_struct->flag_before_trigger;
	}
}
