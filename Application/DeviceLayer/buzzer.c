/**
  ******************************************************************************
  * @file           : buzzer.c
  * @brief          : 蜂鸣器驱动模块
  * @details        : 通过PWM控制蜂鸣器的频率和音量。
  *                   频率决定音调（pitch），占空比决定音量（volume）。
  *                   使用TIM12通道2输出PWM信号。
  *                   支持动态计算分频系数和自动重装载值以适配不同频率。
  ******************************************************************************
  */

#include "buzzer.h"

/**
  * @brief  限幅宏定义，将x限制在[min, max]范围内
  * @param  x: 要限制的值
  * @param  min: 最小值
  * @param  max: 最大值
  * @retval 限制后的值
  */
#define constrain(x, min, max) ((x > max) ? max : (x < min ? min : x))

/* 函数前向声明 */
void Buzzer_init(buzzer_t* buzzer);   /* 蜂鸣器初始化 */
void Buzzer_Work(buzzer_t* buzzer);   /* 蜂鸣器工作（更新PWM参数） */

/**
  * @brief  蜂鸣器全局对象定义
  * @details 绑定初始化函数和工作函数
  */
buzzer_t buzzer={
	.work = Buzzer_Work,    /* 绑定工作函数 */
	.init = Buzzer_init,    /* 绑定初始化函数 */
};

/* Private macro -------------------------------------------------------------*/

/**
  * @brief  蜂鸣器初始化函数
  * @param  buzzer: 蜂鸣器对象指针
  * @retval None
  * @details 配置TIM12通道2的PWM参数并启动输出：
  *          - 定时器: TIM12
  *          - 通道: TIM_CHANNEL_2
  *          - 最大ARR: 65535（16位定时器）
  *          - 定时器时钟频率: 240MHz
  *          - 占空比范围: 0~0.8（80%最大占空比）
  *          - 音量范围: 0~100%
  *          初始占空比设为0，蜂鸣器不发声
  */
void Buzzer_init(buzzer_t* buzzer)
{
	/* 配置定时器和通道 */
	buzzer->config.tim = &htim12;
	buzzer->config.channel = TIM_CHANNEL_2;

	/* 配置定时器参数 */
	buzzer->config.max_tim_arr = 65535;          /* 16位定时器最大值 */
	buzzer->config.tim_freq = 240000000;         /* 定时器时钟频率240MHz */
	buzzer->config.min_pwm_duty = 0;             /* 最小占空比0% */
	buzzer->config.max_pwm_duty = 0.8f;          /* 最大占空比80%（防止蜂鸣器过载） */
	buzzer->config.max_volume = 100.f;            /* 最大音量100 */
	buzzer->config.min_volume = 0.f;              /* 最小音量0 */

	/* 启动PWM输出，初始占空比为0（不发声） */
	HAL_TIM_PWM_Start(buzzer->config.tim, buzzer->config.channel);
    __HAL_TIM_SET_COMPARE(buzzer->config.tim, buzzer->config.channel, 0);
}

/**
  * @brief  动态计算最优预分频系数
  * @param  buzzer: 蜂鸣器对象指针
  * @param  buzzer_freq: 目标频率（Hz）
  * @retval 预分频系数PSC（0~65535）
  * @details 计算公式：PSC = (定时器时钟频率 / (目标频率 * (ARR+1))) - 1
  *          目的是在给定频率下，让ARR尽可能大（接近65535）以提高频率精度
  * @note   当输入频率<=0时返回0
  */
static uint16_t Buzzer_Calc_Optimal_Presc(buzzer_t* buzzer, float buzzer_freq)
{
    if (buzzer_freq <= 0) {
        return 0;  /* 频率非法，返回默认分频系数 */
    }

    uint32_t tim_freq = buzzer->config.tim_freq;   /* 定时器时钟频率 */
    uint16_t max_arr = buzzer->config.max_tim_arr;  /* 最大ARR值 */

    /* 计算预分频系数，使ARR接近最大值以获得最佳频率分辨率 */
    float presc_float = (tim_freq / (buzzer_freq * (max_arr + 1)));

    /* 限制分频系数范围（PSC是16位寄存器，0~65535） */
    uint16_t presc = (uint16_t)constrain(presc_float, 0.0f, 65535.0f);

    return presc;
}

/**
  * @brief  根据频率和预分频系数计算ARR（自动重装载值）
  * @param  buzzer: 蜂鸣器对象指针
  * @retval ARR值（0~65535）
  * @details 计算公式：ARR = 定时器时钟 / (目标频率 * (PSC+1)) - 1
  *          ARR决定了PWM波的周期
  */
static uint16_t Buzzer_Calc_ARR(buzzer_t* buzzer)
{
    if (buzzer->base_info.input_info.freq <= 0) {
        return 0;  /* 频率非法，返回0 */
    }

    /* 计算ARR原始浮点值 */
	float arr_float = buzzer->config.tim_freq /
	                  buzzer->base_info.input_info.freq /
	                  (buzzer->base_info.tim_presc + 1.f) - 1.f;

	buzzer->base_info.ARR_raw = arr_float;  /* 保存原始ARR值用于调试 */

	/* 限制ARR范围，避免异常值导致定时器溢出 */
    return (uint16_t)constrain(arr_float, 0.0f, (float)buzzer->config.max_tim_arr);
}

/**
  * @brief  根据占空比和ARR计算CCR（比较值）
  * @param  buzzer: 蜂鸣器对象指针
  * @retval CCR值
  * @details 计算公式：CCR = 占空比 * ARR
  *          CCR决定了PWM波的高电平持续时间，即音量大小
  */
static uint16_t Buzzer_Calc_CCR(buzzer_t* buzzer)
{
	return (uint16_t)constrain(
        buzzer->base_info.duty * buzzer->base_info.ARR,  /* 占空比乘以ARR */
        0.0f, (float)buzzer->base_info.ARR               /* 限制在0~ARR之间 */
    );
}

/**
  * @brief  将音量值转换为PWM占空比
  * @param  buzzer: 蜂鸣器对象指针
  * @param  volume: 音量值（范围：min_volume ~ max_volume，即0~100）
  * @retval 对应的占空比（范围：min_pwm_duty ~ max_pwm_duty，即0~0.8）
  * @details 使用线性映射将音量范围映射到占空比范围
  *          公式：duty = (max_duty - min_duty) / (max_vol - min_vol) * volume + min_duty
  */
static float Buzzer_Volume_to_Duty(buzzer_t* buzzer, float volume)
{
	/* 限幅音量值 */
	volume = constrain(volume, buzzer->config.min_volume, buzzer->config.max_volume);
	/* 线性映射：音量 -> 占空比 */
	return (buzzer->config.max_pwm_duty - buzzer->config.min_pwm_duty) /
	       (buzzer->config.max_volume - buzzer->config.min_volume) * volume +
	       buzzer->config.min_pwm_duty;
}

/**
  * @brief  蜂鸣器工作函数，根据目标频率和音量更新PWM参数
  * @param  buzzer: 蜂鸣器对象指针
  * @retval None
  * @details 工作流程：
  *          1. 检查频率和音量是否有效
  *          2. 计算最优预分频系数PSC
  *          3. 计算自动重装载值ARR（决定频率）
  *          4. 将音量转换为占空比
  *          5. 计算比较值CCR（决定音量）
  *          6. 依次更新PSC、ARR、CCR到定时器寄存器
  * @note   更新顺序很重要：先PSC再ARR（都影响频率），最后CCR（影响音量）
  */
void Buzzer_Work(buzzer_t* buzzer)
{
    /* 获取目标频率和音量 */
    float target_freq = buzzer->base_info.input_info.freq;
    float target_volume = buzzer->base_info.input_info.volume;

    /* 频率或音量无效时关闭蜂鸣器输出 */
    if (target_freq <= 0 || target_volume <= buzzer->config.min_volume) {
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);  /* CCR=0，占空比为0 */
        return;
    }

    /* 计算PWM参数 */
    buzzer->base_info.tim_presc = Buzzer_Calc_Optimal_Presc(buzzer, target_freq);  /* 预分频 */
    buzzer->base_info.ARR = Buzzer_Calc_ARR(buzzer);                                  /* 自动重装载值 */
    buzzer->base_info.duty = Buzzer_Volume_to_Duty(buzzer, target_volume);           /* 占空比（音量） */
    buzzer->base_info.CCR = Buzzer_Calc_CCR(buzzer);                                  /* 比较值 */

    /* 按顺序更新定时器寄存器 */
    __HAL_TIM_PRESCALER(buzzer->config.tim, buzzer->base_info.tim_presc);           /* 先设分频（影响频率） */
    __HAL_TIM_SET_AUTORELOAD(buzzer->config.tim, buzzer->base_info.ARR);            /* 再设ARR（影响频率） */
    __HAL_TIM_SET_COMPARE(buzzer->config.tim, buzzer->config.channel, buzzer->base_info.CCR);  /* 最后设CCR（影响音量） */
}



