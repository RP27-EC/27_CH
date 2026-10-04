/**
  ******************************************************************************
  * @file           : shoot_motor.h
  * @brief          : 拨弹电机驱动模块头文件
  * @details        : 定义拨弹RM3508的CAN ID、单发角度、卡弹判据等参数和外部接口
  ******************************************************************************
  */

#ifndef SHOOT_MOTOR_H
#define SHOOT_MOTOR_H

#include "RM_Motor.h"  /* RoboMaster电机驱动库 */

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
/*                          卡弹判据（三取二：角度没走完 + 电流大 + 持续）      */
/* ========================================================================== */
#define SHOOT_STUCK_ANGLE_ERR     (2000.0f)  /* 卡弹角度误差阈值(编码器计数) */
#define SHOOT_STUCK_CURRENT       (6000.0f)  /* 卡弹电流阈值(raw current) */
#define SHOOT_STUCK_TICKS         (100U)     /* 持续100×2ms = 200ms 判卡 */

/* 卡弹重试：同一发弹最多补位重试次数，超过进卡弹停机 */
#define SHOOT_JAM_RETRY_MAX       (1U)

/* 补弹超时兜底：单发/补位 500×2ms = 1s 未到位 */
#define SHOOT_FIRE_TIMEOUT_TICKS  (500U)

/* 反转松弹：150×2ms = 300ms 未到位也视为完成 */
#define SHOOT_RECOIL_TIMEOUT_TICKS (150U)
#define SHOOT_RECOIL_ARRIVE_ERR   (3000.0f)  /* 反转到位判据(编码器计数) */

/**
  * @brief  拨弹电机对象
  * @details RM3508，type=_3508_Single，控制约定与底盘轮子一致：
  *          速度反馈=encoder_speed(rpm)，角度反馈=encoder_sum(编码器计数)
  */
extern Motor_RM_t Shoot_Motor;

/* 拨弹电机初始化：绑定函数指针、上电先按在线处理 */
void Shoot_Motor_Init(void);

/* 拨弹电机卸力：清PID、力矩清零并下发一帧零电流 */
void Shoot_Motor_Stop(void);

#endif /* SHOOT_MOTOR_H */
