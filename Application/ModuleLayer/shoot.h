/**
  ******************************************************************************
  * @file           : shoot.h
  * @brief          : 上层板摩擦轮控制头文件
  * @details        : 上层板只负责发射机构的摩擦轮部分：
  *                   - 按下层板 0xD1 的 is_fric_on 起转 / 停转
  *                   - 回传 0x023 状态帧（已达速 + 全部在线），供下层板做
  *                     摩擦轮就绪安全锁，未就绪时下层板拨盘不许动作
  *                   拨盘的单发 / 连发时序全部在下层板角度环里完成，
  *                   上层板不参与，因此这里没有单发连发之分。
  ******************************************************************************
  */
#ifndef SHOOT_H
#define SHOOT_H

#include <stdint.h>
#include <stdbool.h>
#include "motor.h"

/* 摩擦轮目标转速 (rpm)，台架实测后标定 */
#define FRIC_TARGET_F_RPM        (2200.0f)   /* 前层目标转速 */
#define FRIC_TARGET_B_RPM        (2100.0f)   /* 后层目标转速 */

/* 达速判据：转速越过该阈值（按各自转向符号）才算就绪 */
#define FRIC_READY_RPM           (1300.0f)

/*
 * 六轮转向约定：左正、右负、上负（两轮夹弹，左右对滚、上轮压弹）。
 * 台架上若某颗反转，只翻对应符号，不要改 PID 符号。
 */
#define FRIC_SIGN_U              (-1.0f)     /* 上摩擦轮转向 */
#define FRIC_SIGN_L              (+1.0f)     /* 左摩擦轮转向 */
#define FRIC_SIGN_R              (-1.0f)     /* 右摩擦轮转向 */

/* 摩擦轮控制数据结构体 */
typedef struct {
    bool is_on;          /* 0xD1 下发的摩擦轮开关 */
    float target_f_rpm;  /* 前层目标转速 (rpm) */
    float target_b_rpm;  /* 后层目标转速 (rpm) */

    bool is_fric_speed;  /* 六轮实测均已达速 */
    bool is_fric_work;   /* 六轮电机全部在线 */
} Shoot_Fric_t;

/* 发射机构整机管理对象 */
typedef struct shoot_struct {
    Shoot_Fric_t fric;

    void (*init)(void);      /* 初始化函数指针 */
    void (*work)(void);      /* 2ms 周期工作函数指针 (对应 shoot.work()) */
} Shoot_t;

extern Shoot_t shoot;

/* 外部调用函数声明 */
void Shoot_Init(void);
void Shoot_Work(void);
void Shoot_Safe_Stop(void);

#endif /* SHOOT_H */
