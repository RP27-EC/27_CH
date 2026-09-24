/**
  ******************************************************************************
  * @file           : car_control.c
  * @brief          : 整车状态机实现
  ******************************************************************************
  */

#include "carctrl.h"
#include "rc_sensor.h"
#include "gimbal.h"        // 用于读取云台复位标志 gimbal.gimbal_reset_state
#include "main.h"          // 获取 HAL_GetTick()

Car_t hero_car;

/* 常量宏定义（消除魔数） */
#define INIT_MAX_TIMEOUT_TICKS   750     // 初始化超时时间 750 * 2ms = 1.5s
#define SHOOT_COOLDOWN_TICKS     300     // 打弹冷却计数阈值

/* ========================================================================= */
/*                           内部工具函数：边沿检测                           */
/* ========================================================================= */

/**
 * @brief 拨轮向上拨动边沿检测
 */
static bool RC_Thumbwheel_Up_Edge(void) {
    static uint8_t last_step = 0;
    uint8_t curr_step = rc_sensor.info->thumbwheel.step[RC_TB_UP];
    bool edge = (curr_step != last_step);
    last_step = curr_step;
    return edge;
}

/**
 * @brief 拨轮向下拨动边沿检测
 */
static bool RC_Thumbwheel_Dn_Edge(void) {
    static uint8_t last_step = 0;
    uint8_t curr_step = rc_sensor.info->thumbwheel.step[RC_TB_DN];
    bool edge = (curr_step != last_step);
    last_step = curr_step;
    return edge;
}

/* ========================================================================= */
/*                          1. 上电复位与超时判断                            */
/* ========================================================================= */
static void Car_Init_State_Machine(Car_t *car) {
    if (car->mode == CAR_MODE_SLEEP) {
        car->reset_state = RESET_PENDING;
        car->init_timer = 0;
        return;
    }

    if (car->reset_state == RESET_PENDING) {
        car->init_timer++;
        
        /* 条件 1：云台自身完成机械寻零归位；条件 2：超时强行放行 */
        if (hero_gimbal.reset_state == GIMBAL_RESET_DONE || car->init_timer >= INIT_MAX_TIMEOUT_TICKS) {
            car->reset_state = RESET_DONE;
            car->init_timer = 0;
            car->mode = CAR_MODE_MECH;
        }
    }
}

/* ========================================================================= */
/*                          2. 遥控器模式切换逻辑                            */
/* ========================================================================= */
static void Car_Mode_Switch_Update(Car_t *car, bool tb_up, bool tb_dn) {
    uint8_t s1 = rc_sensor.info->s1;
    uint8_t s2 = rc_sensor.info->s2;

    /* 模式切换矩阵（完全对齐原战队操作逻辑） */
    switch (car->mode) {
        case CAR_MODE_GYRO:
            if (s1 == 1 && s2 == 3 && tb_up) {
                car->mode = CAR_MODE_SPIN;        // s1上, s2下, 拨轮上 -> 切小陀螺
            } else if (s1 == 1 && s2 == 2 && tb_up) {
                car->mode = CAR_MODE_LOB;         // s1上, s2中, 拨轮上 -> 切吊射
            } else if (s1 == 1 && s2 == 2 && tb_dn) {
                car->mode = CAR_MODE_MECH;        // s1上, s2中, 拨轮下 -> 切机械模式
            }
            break;

        case CAR_MODE_SPIN:
            if (s1 == 1 && s2 == 3 && tb_up) {
                car->mode = CAR_MODE_GYRO;        // 再次拨轮上 -> 退出小陀螺回陀螺仪
            } else if (s1 == 1 && s2 == 2 && tb_dn) {
                car->mode = CAR_MODE_MECH;
            }
            break;

        case CAR_MODE_LOB:
            if (s1 == 1 && s2 == 2 && tb_up) {
                car->mode = CAR_MODE_GYRO;        // 退出吊射模式
            }
            break;

        case CAR_MODE_MECH:
            if (s1 == 1 && s2 == 2 && tb_dn) {
                car->mode = CAR_MODE_GYRO;        // 退出机械模式回陀螺仪
            } else if (s1 == 1 && s2 == 3 && tb_up) {
                car->mode = CAR_MODE_SPIN;
            }
            break;

        default:
            break;
    }

    /* 自动前哨站打弹模式快捷组合键：S1上 + S2上 且摩擦轮开启 */
    if (s1 == 1 && s2 == 1 && car->flags.is_shoot_system_on) {
        car->flags.is_outpost_mode = true;
        car->mode = CAR_MODE_LOB;
    } else {
        car->flags.is_outpost_mode = false;
    }
}

/* ========================================================================= */
/*                     3. 摩擦轮、自瞄与发射动作控制                         */
/* ========================================================================= */
static void Car_Action_Cmd_Update(Car_t *car, bool tb_up, bool tb_dn) {
    uint8_t s1 = rc_sensor.info->s1;
    uint8_t s2 = rc_sensor.info->s2;
    static uint16_t shoot_cnt = SHOOT_COOLDOWN_TICKS;

    /* --- 1. 摩擦轮开关：S1下 + S2下，拨轮向上 --- */
    if (s1 == 3 && s2 == 3 && tb_up) {
        car->flags.is_shoot_system_on = !car->flags.is_shoot_system_on;
    }

    /* --- 2. 自瞄开关：S1下 + S2下，拨轮向下 --- */
    if (s1 == 3 && s2 == 3 && tb_dn) {
        if (car->flags.vision_mode == VISION_OFF) {
            car->flags.vision_mode = VISION_AUTO_AIM;
        } else {
            car->flags.vision_mode = VISION_OFF;
        }
    }

    /* --- 3. 发射逻辑（单发 / 自瞄打 / 手动打）--- */
    car->flags.is_single_shoot_cmd = false;

    if (!car->flags.is_shoot_system_on) {
        shoot_cnt = SHOOT_COOLDOWN_TICKS;
        return; // 摩擦轮没开，不响应任何打弹指令
    }

    // 冷却计时递增
    if (shoot_cnt < SHOOT_COOLDOWN_TICKS) {
        shoot_cnt++;
    }

    /* 手动单发发射：右拨杆 S2 置于顶档 (s2 == 1) */
    if (s2 == 1 && shoot_cnt >= SHOOT_COOLDOWN_TICKS) {
        car->flags.is_single_shoot_cmd = true;
        shoot_cnt = 0; // 重置冷却
    }
}

/* ========================================================================= */
/*                          4. 状态机总入口函数                              */
/* ========================================================================= */
void Car_Control_Init(Car_t *car) {
    car->mode = CAR_MODE_SLEEP;
    car->last_mode = CAR_MODE_SLEEP;
    car->reset_state = RESET_PENDING;
    car->init_timer = 0;
    
    car->flags.is_shoot_system_on = false;
    car->flags.is_single_shoot_cmd = false;
    car->flags.is_outpost_mode = false;
    car->flags.vision_mode = VISION_OFF;
}

void Car_Control_Update(Car_t *car) {
    car->last_mode = car->mode;

    /* 1. 失联保护（Failsafe）：掉线直接进 SLEEP */
    if (rc_sensor.work_state != DEV_ONLINE || rc_sensor.errno != NONE_ERR) {
        car->mode = CAR_MODE_SLEEP;
        car->reset_state = RESET_PENDING;
        car->flags.is_shoot_system_on = false;
        car->flags.is_single_shoot_cmd = false;
        car->flags.vision_mode = VISION_OFF;
        return;
    }

    /* 2. 上电复位流程 */
    if (car->reset_state == RESET_PENDING) {
        car->mode = CAR_MODE_INIT;
        Car_Init_State_Machine(car);
        return;
    }

    /* 3. 捕获拨轮边沿触发 */
    bool tb_up = RC_Thumbwheel_Up_Edge();
    bool tb_dn = RC_Thumbwheel_Dn_Edge();

    /* 4. 执行状态转移与动作指令 */
    Car_Mode_Switch_Update(car, tb_up, tb_dn);
    Car_Action_Cmd_Update(car, tb_up, tb_dn);
}
