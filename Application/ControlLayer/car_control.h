/**
  ******************************************************************************
  * @file           : car_control.h
  * @brief          : 整车状态机管理模块（纯遥控器版）
  ******************************************************************************
  */

#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/* 整车运动模式枚举 */
typedef enum {
    CAR_MODE_SLEEP = 0,     /* 休眠/关控保护模式 */
    CAR_MODE_INIT,          /* 上电复位对齐模式 */
    CAR_MODE_GYRO,          /* 陀螺仪跟随模式 (默认主力模式) */
    CAR_MODE_MECH,          /* 机械模式 (底盘与云台对齐车头) */
    CAR_MODE_SPIN,          /* 小陀螺自旋模式 */
} CarMode_e;

/* 复位状态枚举 */
typedef enum {
    RESET_PENDING = 0,      /* 正在复位 */
    RESET_DONE,             /* 复位完成 */
} ResetState_e;

/* 整车控制标志位集合 */
typedef struct {
    bool is_shoot_system_on;    /* 摩擦轮开启标志 */
    bool is_single_shoot_cmd;   /* 单发打弹脉冲指令 */
    bool is_continuous_shoot;   /* 连发标志（拨弹持续转动） */
} CarFlags_t;

/* 整车总控对象 */
typedef struct {
    CarMode_e mode;             /* 当前整车模式 */
    CarMode_e last_mode;        /* 上一周期模式 */
    ResetState_e reset_state;   /* 复位状态 */
    uint16_t init_timer;        /* 复位超时计时器 */
    
    CarFlags_t flags;           /* 控制标志位 */
} Car_t;

extern Car_t hero_car;

/* 外部调用接口 */
void Car_Control_Init(Car_t *car);
void Car_Control_Update(Car_t *car);

#endif /* CAR_CONTROL_H */
