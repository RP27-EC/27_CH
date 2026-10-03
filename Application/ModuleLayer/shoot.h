/**
  ******************************************************************************
  * @file           : shoot.h
  * @brief          : 拨弹控制模块头文件
  * @details        : 消费整车状态机标志，驱动拨弹RM3508：
  *                   - is_shoot_system_on：总开关（关=卸力，也是卡弹停机的解锁动作）
  *                   - is_single_shoot_cmd：单发脉冲，角度环加一格
  *                   - is_continuous_shoot：连发，定频步进（每格一发，定周期推进）
  *                   摩擦轮在上层板，经Board_Tx_D1的is_fric_on通知；
  *                   上层板经0x023回传达速/在线状态，未就绪拨盘不许动；
  *                   首次收到0x023后锁存严格门禁，之后丢失0x023视为
  *                   未就绪，不再退回延时兜底（防上层板失联时误推弹）。
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
    SHOOT_STATE_RECOIL,      /* 卡弹反转：退一格松弹，成功后补位重试 */
    SHOOT_STATE_STOP_JAM,    /* 卡弹停机：停射停摩擦轮，重拨总开关解锁 */
} ShootState_e;

/* 拨弹模块初始化：进入休眠并卸力（电机初始化在DEVICE_Init） */
void Shoot_Init(void);

/**
  * @brief  拨弹控制更新（2ms周期，由control_task调用）
  * @param  car      : 整车状态机（含发射标志）
  * @param  failsafe : 失联保护（与底盘同一条件）
  * @retval None
  */
void Shoot_Update(const Car_t *car, bool failsafe);

/* 当前是否通知上层板开摩擦轮（= 总开关且未卡弹停机） */
bool Shoot_Is_Fric_On(void);

#endif /* SHOOT_H */
