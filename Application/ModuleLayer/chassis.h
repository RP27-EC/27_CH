#ifndef CHASSIS_H
#define CHASSIS_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ===========================================================================
 * 1. 物理参数与系统宏定义
 * =========================================================================== */
#define CHASSIS_MAX_WHEEL_RPM        6000.0f   /* 3508 转子最大允许目标转速 (RPM) */
#define CHASSIS_MAX_CURRENT          10000.0f  /* 单电机输出最大电流 (C620 满量程 ±16384) */
#define CHASSIS_RC_MAX_VALUE         660.0f    /* 遥控器单通道最大有效行程 */

/* 3508 硬件参数 */
#define M3508_GEAR_RATIO             (3591.0f / 187.0f) /* 官方精确减速比 19.2032:1 */
#define MECANUM_WHEEL_PERIMETER      0.4775f            /* 152mm 麦轮周长 (米) = pi * 0.152 */

/* 速度环 PID 参数 */
#define CHASSIS_PID_KP               4.5f      
#define CHASSIS_PID_KI               0.1f      
#define CHASSIS_PID_KD               0.0f      
#define CHASSIS_PID_MAX_IOUT         3000.0f   /* 积分限幅 (防过载) */

/* ===========================================================================
 * 2. 数据结构定义
 * =========================================================================== */

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float max_out;
    float max_iout;
} Chassis_PID_t;

typedef struct {
    int16_t feedback_rpm;     /* 转子实际转速 (RPM) */
    int16_t feedback_current; /* 实际转矩电流 */
    uint16_t feedback_ecd;    /* 机械编码器位置 (0 ~ 8191) */

    float target_rpm;         /* 目标转速 (RPM) */
    int16_t give_current;     /* 最终发给 C620 的电流指令 */

    Chassis_PID_t pid;
} Motor3508_t;

typedef struct {
    float vx;                 /* 前后速度比例 (前进为正, -1.0 ~ +1.0) */
    float vy;                 /* 左右横移比例 (向左为正, -1.0 ~ +1.0) */
    float wz;                 /* 原地自旋比例 (逆时针为正, -1.0 ~ +1.0) */

    /* 0:左前(FL), 1:右前(FR), 2:左后(BL), 3:右后(BR) */
    Motor3508_t wheel[4];

    bool enabled;             /* 使能标志位 */
} Chassis_t;

/* ===========================================================================
 * 3. 外部接口函数声明
 * =========================================================================== */
void Chassis_Init(Chassis_t *chassis);
void Chassis_Control_Loop(Chassis_t *chassis, int16_t ch_vx, int16_t ch_vy, int16_t ch_wz, bool failsafe, CAN_HandleTypeDef *hcan);
void Chassis_Parse_Feedback(Chassis_t *chassis, uint32_t std_id, const uint8_t data[8]);

#endif /* CHASSIS_H */
