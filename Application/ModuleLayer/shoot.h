/**
  ******************************************************************************
  * @file           : shoot.h
  * @brief          : 拨弹控制模块头文件
  * @details        : 消费整车状态机标志，驱动拨弹RM3508：
  *                   - is_shoot_system_on：总开关（关=卸力并停止卡弹重试）
  *                   - is_single_shoot_cmd：单发脉冲，忙时最多缓存一发，空闲后角度环加一格
  *                   - is_continuous_shoot：按当前电平连发，定频步进（每格一发）
  *                   摩擦轮在上层板，经Board_Tx_D1的is_fric_on通知；
  *                   上层板经0x023回传达速/在线状态，未收到或丢失0x023
  *                   一律视为未就绪，不设延时兜底（防上层板未起转时误推弹）。
  ******************************************************************************
  */

#ifndef SHOOT_H
#define SHOOT_H

#include <stdbool.h>
#include "car_control.h"

/* 拨弹运行状态 */
typedef enum {
    SHOOT_STATE_SLEEP = 0,   /* 卸力：总开关关 / 失联 / 拨盘离线 */
    SHOOT_STATE_SPINUP,      /* 摩擦轮起转中：拨盘锁位，指令丢弃 */
    SHOOT_STATE_READY,       /* 就绪：锁位待发 */
    SHOOT_STATE_FIRE,        /* 补弹中：角度环跟踪一格 */
    SHOOT_STATE_RECOIL,      /* 卡弹反转：退一格松弹，到位或超时后补位，无次数上限 */
} ShootState_e;




void Shoot_Init(void);
void Shoot_Update(const Car_t *car, bool failsafe);
bool Shoot_Is_Fric_On(void);

#endif /* SHOOT_H */
