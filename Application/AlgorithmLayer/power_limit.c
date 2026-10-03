/**
  ******************************************************************************
  * @file           : power_limit.c
  * @brief          : 底盘功率限制算法实现
  ******************************************************************************
  * @details
  * 本文件实现了底盘电机的功率限制算法，用于机器人裁判系统的功率管理。
  * 在RoboMaster比赛中，裁判系统会实时监测机器人底盘的功率消耗，
  * 当功率超过限制时会进行扣罚，因此需要对底盘电机的输出进行功率限制。
  *
  * 功率限制原理：
  *   底盘功率缓冲区（power_buffer）反映了当前功率的余量：
  *   - 缓冲区越大，说明功率余量充足，允许更大的电机输出
  *   - 缓冲区越小，说明功率接近上限，需要限制电机输出
  *
  *   本算法通过动态调整功率限制系数 Limit_k 来实现：
  *   - 缓冲区 >= 60J：不限制（Limit_k = 1.0）
  *   - 缓冲区 25~60J：线性限制（Limit_k = buffer/60）
  *   - 缓冲区 < 25J：二次方限制（Limit_k = (buffer/60)^2），更激进地降功率
  *
  *   当四个电机总输出超过限制值时，按比例缩放所有电机输出。
  *
  * 注意事项：
  *   - 缓冲区上限限制为60J，防止飞坡等特殊情况导致缓冲区异常增大
  *   - 缓冲区 < 25J 时采用二次方限制，比线性限制更快速地降低功率
  *   - 本算法等比例缩放四个电机的输出，保持运动方向不变
  ******************************************************************************
  */

#include "Power_Limit.h"

/**
 * @brief  底盘电机功率限制函数
 * @param  data: 指向四个电机输出值的数组指针（int16_t[4]）
 *               data[0]~data[3] 分别对应四个底盘电机的输出值
 * @retval 无
 * @note   本函数根据裁判系统提供的功率缓冲区数据，动态限制四个电机的总输出。
 *         执行流程：
 *         1. 读取当前功率缓冲区值，并限制上限为60J
 *         2. 根据缓冲区大小计算功率限制系数 Limit_k
 *         3. 计算限制后的最大允许总输出 CHAS_LimitOutput
 *         4. 计算四个电机的绝对值总输出 CHAS_TotalOutput
 *         5. 若总输出超过限制，按比例缩放所有电机输出
 *
 *         调用时机：应在电机控制输出发送前调用此函数。
 */
void Chassis_Motor_Power_Limit(int16_t *data)
{
	/* 读取裁判系统提供的底盘功率缓冲区值（单位：焦耳） */
	float buffer = judge.info->power_heat_data.chassis_power_buffer;

	/* 中间变量声明 */
	float heat_rate, Limit_k, CHAS_LimitOutput, CHAS_TotalOutput;

	/* 电机最大输出值：单电机最大输出 * 4个电机 */
	float OUT_MAX = 0.f;
	OUT_MAX = CHAS_SP_MAX_OUT * 4.f;

	/*
	 * 限制缓冲区上限为60J：
	 * 防止飞坡等特殊情况导致缓冲区突然变大（如从负值跳变为正值），
	 * 使得限制系数过大，导致电机输出突然增加造成危险。
	 */
	if (buffer > 60.f) buffer = 60.f;

	/*
	 * 计算基础限制系数 Limit_k：
	 * Limit_k = buffer / 60，范围为 [0, 1]
	 * - 缓冲区满（60J）时 Limit_k = 1.0，不限制
	 * - 缓冲区空（0J）时 Limit_k = 0.0，完全限制
	 */
	Limit_k = buffer / 60.f;

	/*
	 * 根据缓冲区大小选择限制策略：
	 *
	 * 缓冲区 < 25J（危险区域）：
	 *   采用二次方限制 Limit_k = (buffer/60)^2
	 *   二次方限制比线性限制更激进，能更快地降低功率，避免超功率
	 *
	 * 缓冲区 >= 25J（安全区域）：
	 *   采用线性限制 Limit_k = buffer/60
	 *   线性限制更温和，电机输出变化更平滑
	 */
	if (buffer < 25.f)
		Limit_k = Limit_k * Limit_k;   /* 二次方限制 */
	else
		Limit_k = Limit_k;             /* 线性限制 */

	/*
	 * 计算限制后的最大允许总输出：
	 * - 缓冲区未满（< 60J）：CHAS_LimitOutput = Limit_k * OUT_MAX
	 * - 缓冲区已满（>= 60J）：CHAS_LimitOutput = OUT_MAX（不限制）
	 */
	if (buffer < 60.f)
		CHAS_LimitOutput = Limit_k * OUT_MAX;
	else
		CHAS_LimitOutput = OUT_MAX;

	/*
	 * 计算四个电机输出的绝对值总和：
	 * 使用绝对值是因为电机可能正转或反转，功率消耗与方向无关
	 */
	CHAS_TotalOutput = abs(data[0]) + abs(data[1]) + abs(data[2]) + abs(data[3]);

	/*
	 * 计算缩放比例：
	 * heat_rate = 限制值 / 实际总输出
	 * - 若 heat_rate >= 1.0，说明未超限，不需要缩放
	 * - 若 heat_rate < 1.0，说明超限，需要等比例缩小
	 */
	heat_rate = CHAS_LimitOutput / CHAS_TotalOutput;

	/*
	 * 当总输出超过限制时，等比例缩放所有电机输出：
	 * 每个电机的新输出 = 原输出 * heat_rate
	 * 等比例缩放的优点是保持运动方向和各电机输出的比例关系不变
	 */
	if (CHAS_TotalOutput >= CHAS_LimitOutput)
	{
		for (char i = 0; i < 4; i++)
		{
			data[i] = (int16_t)(data[i] * heat_rate);  /* 等比例缩放 */
		}
	}
}
