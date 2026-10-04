/**
  ******************************************************************************
  * @file           : power_model.h
  * @brief          : 底盘功率模型与功率预算
  * @details        : 两件事，都围绕"瓦特"这一个量纲：
  *
  *                   1. 二次功率模型：把电机的"电流+转速"换算成瓦特（正向），
  *                      或反过来在给定瓦特预算下解出该用多大电流（反向）。
  *                      这是裁判系统/功率预算的"瓦"和电机 PID 的"raw count"
  *                      之间唯一的换算桥。
  *
  *                      P(i,w) = k1*w^2 + k2*i^2 + kiw*i*w + c
  *                        k1*w^2   铁损/风阻（随转速平方）
  *                        k2*i^2   铜损（随电流平方）
  *                        kiw*i*w  反电动势做功（主要项）
  *                        c        空载常数
  *
  *                   2. 功率预算：给出"现在底盘最多能用多少瓦"。
  *                      裁判系统在线就用裁判给的功率限制，离线就用训练基准，
  *                      训练和比赛共用一套底盘代码，不需要改源码。
  *
  * @note           系数为台架拟合值，随车重标，见 power_model.c 顶部说明。
  ******************************************************************************
  */
#ifndef POWER_MODEL_H
#define POWER_MODEL_H

/**
  * @brief  正向：由电流和转速预测功率
  * @param  current_raw : 电机电流指令（CAN raw count，M3508 满量程 ±16384）
  * @param  speed_rpm   : 电机转速（rpm）
  * @retval 预测功率（W）
  */
float Power_Predict(float current_raw, float speed_rpm);

/**
  * @brief  反向：在功率预算下解算电流
  * @details 解一元二次方程 k2*i^2 + (kiw*w)*i + (k1*w^2 + c - target_power) = 0，
  *          按原始电流的符号挑根：正电流取正根（驱动），负电流取负根（制动）。
  * @param  target_power : 分摊到这一轮的目标功率（W），须 > 0
  * @param  speed_rpm    : 电机当前转速（rpm）
  * @param  current_raw  : PID 刚算出的原始电流，用于判定驱动/制动取哪个根
  * @retval 重新解算的电流（raw count）；目标功率非法或无实根时原样返回
  */
float Power_Solve_Current(float target_power, float speed_rpm, float current_raw);

/**
  * @brief  取当前底盘功率预算
  * @details 全工程唯一决定"底盘能用多少瓦"的地方：
  *          裁判系统在线且给出有效上限 -> 裁判值 × 1.25（留裕量）
  *          否则（训练 / 裁判离线）   -> CHASSIS_POWER_BUDGET_W
  * @retval 功率上限（W）
  */
float Power_Budget_Get(void);

/**
  * @brief  取发给超电板的底盘缓冲能量
  * @details 裁判在线用裁判实时缓冲能量；离线时给名义满值，
  *          免得电容板以为底盘能量见底而提前收手。
  * @retval 缓冲能量（J）
  */
float Power_Budget_Buffer(void);

#endif /* POWER_MODEL_H */
