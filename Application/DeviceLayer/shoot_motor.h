/**
  ******************************************************************************
  * @file           : shoot_motor.h
  * @brief          : 拨弹电机驱动模块头文件
  * @details        : 定义拨弹RM3508的CAN ID、单发角度、卡弹判据等参数和外部接口
  ******************************************************************************
  */

#ifndef SHOOT_MOTOR_H
#define SHOOT_MOTOR_H

#include "RM_motor.h"  

/* 拨弹电机反馈ID（电调ID=5，挂在CAN2） */
#define SHOOT_MOTOR_RX_ID         (0x205)

/* 0x1FF：电机5~8电流帧；电机5占tx_buff[0..1]，故born_info.rxId=0 */
#define SHOOT_MOTOR_TX_ID         (0x1FF)

/* 单发拨一格弹：拨弹盘是“5 槽拨盘”，编码器计数（根据8192，减速比约19.2） */
#define SHOOT_ONESHOT_ANGLE       (31481.0f)

/* 单发到位判据：|angle_sum err| < 该值视为到位（编码器计数） */
#define SHOOT_ARRIVE_ERR          (600.0f)

/* 连发定频步进周期：150×2ms = 300ms 一发，首发立即 */
#define SHOOT_REPEAT_PERIOD_TICKS (150U)

/* ========================================================================== */
/* 卡弹判据：起步宽限后，角度未到位 + 高电流 + 低转速连续满足才判卡。 */
/* ========================================================================== */
#define SHOOT_STUCK_ANGLE_ERR     (2000.0f)  /* 卡弹角度误差阈值(编码器计数) */
#define SHOOT_STUCK_CURRENT       (8000.0f)  /* 卡弹电流阈值(raw current) */
#define SHOOT_STUCK_SPEED_RPM     (80.0f)    /* 转子转速绝对值低于此值才累计 */
#define SHOOT_STUCK_GRACE_TICKS   (100U)     /* 每次推弹/补位起步200ms不累计判卡 */
#define SHOOT_STUCK_TICKS         (250U)     /* 宽限后连续约500ms满足才判卡 */

/* 卡弹重试：同一发弹最多补位重试次数，超过进卡弹停机 */
#define SHOOT_JAM_RETRY_MAX       (3U)

/* 反转松弹：250×2ms = 500ms 未到位也视为完成 */
#define SHOOT_RECOIL_TIMEOUT_TICKS (250U)
#define SHOOT_RECOIL_ARRIVE_ERR   (3000.0f)  /* 反转到位判据(编码器计数) */


extern Motor_RM_t Shoot_Motor;

void Shoot_Motor_Init(void);

void Shoot_Motor_Stop(void);

#endif /* SHOOT_MOTOR_H */
