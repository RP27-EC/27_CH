/**
  ******************************************************************************
  * @file           : carctrl.c
  * @brief          : 整车状态机实现
  ******************************************************************************
  */

#include "car_control.h"
#include "rc_sensor.h"
#include "gimbal.h"    
#include "main.h"         


Car_t hero_car;



/*初始化超时时间：750 * 2ms = 1.5秒，超过此时间，强制完成初始化 ，防止云台寻零失败导致机器人永远无法工作*/
#define INIT_MAX_TIMEOUT_TICKS   750

/** @brief 打弹冷却计数阈值：300 * 2ms = 0.6秒，防止连续打弹过快 */
#define SHOOT_COOLDOWN_TICKS     300

/* ========================================================================= */
/*                           内部工具函数：边沿检测                           */
/* ========================================================================= */

/**
  * @brief  拨轮向上拨动边沿检测
  * @details 通过比较当前帧和上一帧的拨轮step值，检测拨轮是否刚被向上拨动。
  *          边沿检测确保每次拨动只触发一次模式切换，避免持续触发。
  *
  * @retval true: 拨轮刚被向上拨动；false: 无变化或已处理
  */
static bool RC_Thumbwheel_Up_Edge(void) {
    static uint8_t last_step = 0;  /* 上一帧的拨轮step值（静态保持） */
    uint8_t curr_step = rc_sensor.info->thumbwheel.step[RC_TB_UP];  /*  获取当前帧的 step 状态（0 或 1） */
    bool edge = (curr_step != last_step);  /* 边沿判定：当前值 != 上次值，防止抖动 */
    last_step = curr_step;  /* 更新历史值 */
    return edge;
}

/**
  * @brief  拨轮向下拨动边沿检测
  * @details 与向上检测类似，检测拨轮是否刚被向下拨动。
  *
  * @retval true: 拨轮刚被向下拨动；false: 无变化或已处理
  */
static bool RC_Thumbwheel_Dn_Edge(void) {
    static uint8_t last_step = 0;  /* 上一帧的拨轮step值 */
    uint8_t curr_step = rc_sensor.info->thumbwheel.step[RC_TB_DN];  /* 当前帧的向下step */
    bool edge = (curr_step != last_step);  /* 边沿判定 */
    last_step = curr_step;  /* 更新历史值 */
    return edge;
}

/* ========================================================================= */
/*                          1. 上电复位，复位后设置初始模式                           */
/* ========================================================================= */

static void Car_Init_State_Machine(Car_t *car) {
    /* SLEEP模式下：重置初始化计时器，等待模式切换 */
    if (car->mode == CAR_MODE_SLEEP) {
        car->reset_state = RESET_PENDING;
        car->init_timer = 0;
        return;
    }

    /* 初始化等待阶段 */
    if (car->reset_state == RESET_PENDING) {
        car->init_timer++; 

        /* 云台自身完成机械寻零归位（GIMBAL_RESET_DONE）||超时1.5秒（750*2ms），强制放行，进入下一个模式 */
        if (hero_gimbal.reset_state == GIMBAL_RESET_DONE ||
            car->init_timer >= INIT_MAX_TIMEOUT_TICKS) {
            car->reset_state = RESET_DONE;       /* 标记初始化完成 */
            car->init_timer = 0;                 /* 清零计时器 */
            car->mode = CAR_MODE_MECH;           /* 复位完成后默认进入陀螺仪模式CAR_MODE_GYRO，进入机械模式 CAR_MODE_MECH*/
        }
    }
}

/* ========================================================================= */
/*                          2. 遥控器模式切换逻辑                            */
/* ========================================================================= */

/*遥控器拨杆与拨轮的模式切换处理*/
static void Car_Mode_Switch_Update(Car_t *car, bool tb_up, bool tb_dn) {
    uint8_t s1 = rc_sensor.info->s1;  
    uint8_t s2 = rc_sensor.info->s2;  

    /* 模式切换矩阵：根据当前模式和输入组合决定目标模式 */
    switch (car->mode) {

        case CAR_MODE_GYRO:
            /* 陀螺仪模式下可切换 */
            if (s1 == 1 && s2 == 3 && tb_up) { //陀螺仪到小陀螺
                car->mode = CAR_MODE_SPIN;
            } else if (s1 == 1 && s2 == 2 && tb_dn) {
                car->mode = CAR_MODE_MECH;        /*陀螺仪到 机械 */
            }
            break;

        case CAR_MODE_SPIN:
            /* 小陀螺模式下可退出到：陀螺仪、机械 */
            if (s1 == 1 && s2 == 3 && tb_up) {
                car->mode = CAR_MODE_GYRO;        /* 再次拨轮上 -> 退出小陀螺 */
            } else if (s1 == 1 && s2 == 2 && tb_dn) {
                car->mode = CAR_MODE_MECH;        /* 切到机械模式 */
            }
            break;

        case CAR_MODE_MECH:
            /* 机械模式下可退出到：陀螺仪、小陀螺 */
            if (s1 == 1 && s2 == 2 && tb_dn) {
                car->mode = CAR_MODE_GYRO;        /* 机械模式到陀螺仪 */
            } else if (s1 == 1 && s2 == 3 && tb_up) {
                car->mode = CAR_MODE_SPIN;        /* 切到小陀螺 */
            }
            break;

        default:
            break;
    }
}

/* ========================================================================= */
/*                     3. 摩擦轮、自瞄与发射动作控制                         */
/* ========================================================================= */

/**
  * @brief  摩擦轮、自瞄与发射动作指令处理
  * @details 根据遥控器拨杆和拨轮输入，处理发射系统的开关控制：
  *
  *          1. 摩擦轮开关：S1中 + S2中 + 拨轮向上 -> 切换摩擦轮开/关
  *          2. 自瞄开关：S1中 + S2中 + 拨轮向下 -> 切换自瞄开/关
  *          3. 单发发射：S2上（需要摩擦轮已开启 + 冷却时间已过）
  *
  *          冷却机制：每次发射后需要等待SHOOT_COOLDOWN_TICKS（0.6秒）才能再次发射，
  *          防止连续打弹过快导致机构损坏或弹丸堵塞。
  *
  * @param  car   : 整车状态机对象指针
  * @param  tb_up : 拨轮向上边沿触发标志
  * @param  tb_dn : 拨轮向下边沿触发标志
  * @retval None
  */
static void Car_Action_Cmd_Update(Car_t *car, bool tb_up, bool tb_dn) {
    uint8_t s1 = rc_sensor.info->s1;
    uint8_t s2 = rc_sensor.info->s2;
    static uint16_t shoot_cnt = SHOOT_COOLDOWN_TICKS;  /* 打弹冷却计数器 */

    /* --- 1. 摩擦轮开关控制：S1中 + S2中 + 拨轮向上（RM遥控 1=上 2=下 3=中） --- */
    if (s1 == 3 && s2 == 3 && tb_up) {
        car->flags.is_shoot_system_on = !car->flags.is_shoot_system_on;  /* 切换开/关 */
    }

    /* --- 2. 自瞄开关控制：S1中 + S2中 + 拨轮向下 --- */
    if (s1 == 3 && s2 == 3 && tb_dn) {
        if (car->flags.vision_mode == VISION_OFF) {
            car->flags.vision_mode = VISION_AUTO_AIM;   /* 开启自瞄 */
        } else {
            car->flags.vision_mode = VISION_OFF;        /* 关闭自瞄 */
        }
    }

    /* --- 3. 发射逻辑（单发 / 手动打）--- */
    /* 每次循环先清除单发指令，只在满足条件时置位 */
    car->flags.is_single_shoot_cmd = false;

    /* 摩擦轮未开启时，不响应任何打弹指令 */
    if (!car->flags.is_shoot_system_on) {
        shoot_cnt = SHOOT_COOLDOWN_TICKS;  /* 重置冷却计数器，确保下次可以立即发射 */
        car->flags.is_continuous_shoot = false;
        return;
    }

    /* 冷却计时递增（每2ms加1，到阈值后允许再次发射） */
    if (shoot_cnt < SHOOT_COOLDOWN_TICKS) {
        shoot_cnt++;
    }

    /*
     * 发射绑定（避免与模式切换冲突：模式切换需要 S1上=1）：
     * - 连发：S2下(2) 且 S1非上(≠1)，按住持续出弹
     * - 单发：S2上(1)，冷却内只出一帧脉冲
     */
    if (s2 == 2 && s1 != 1) {
        car->flags.is_continuous_shoot = true;
    } else {
        car->flags.is_continuous_shoot = false;
        if (s2 == 1 && shoot_cnt >= SHOOT_COOLDOWN_TICKS) {
            car->flags.is_single_shoot_cmd = true;   /* 发出单发指令 */
            shoot_cnt = 0;                           /* 重置冷却计数器 */
        }
    }
}

/* ========================================================================= */
/*                          4. 状态机总入口函数                              */
/* ========================================================================= */


void Car_Control_Init(Car_t *car) {
    car->mode = CAR_MODE_SLEEP;           /* 初始模式：休眠 */
    car->last_mode = CAR_MODE_SLEEP;      /* 上一次模式：休眠 */
    car->reset_state = RESET_PENDING;     /* 复位状态：等待中 */
    car->init_timer = 0;                  /* 初始化计时器清零 */

    /* 所有动作标志位清零 */
    car->flags.is_shoot_system_on = false;    /* 摩擦轮关闭 */
    car->flags.is_single_shoot_cmd = false;   /* 无单发指令 */
    car->flags.is_continuous_shoot = false;   /* 无连发 */
    car->flags.vision_mode = VISION_OFF;      /* 自瞄关闭 */
}



/*整车状态机主更新函数（根据拨杆+拨轮组合切换工作模式） */
void Car_Control_Update(Car_t *car) {
    car->last_mode = car->mode;  /* 用于边沿检测 */

    //失联保护
    if (rc_sensor.work_state != DEV_ONLINE || rc_sensor.errno != NONE_ERR) {
        car->mode = CAR_MODE_SLEEP;
        car->reset_state = RESET_PENDING;
        car->flags.is_shoot_system_on = false;    /* 关闭摩擦轮 */
        car->flags.is_single_shoot_cmd = false;   /* 取消单发指令 */
        car->flags.is_continuous_shoot = false;   /* 取消连发 */
        car->flags.vision_mode = VISION_OFF;      /* 关闭自瞄 */
        return;  /* 失联状态下不执行后续逻辑 */
    }

    /*上电复位流程*/
    if (car->reset_state == RESET_PENDING) {
        car->mode = CAR_MODE_INIT;  /* 设为初始化模式 */
        Car_Init_State_Machine(car);  /* 执行初始化状态机 */
        return;  /* 初始化期间不执行模式切换和动作控制 */
    }

    /*捕获拨轮边沿触发*/
    bool tb_up = RC_Thumbwheel_Up_Edge();  /* 拨轮向上边沿 */
    bool tb_dn = RC_Thumbwheel_Dn_Edge();  /* 拨轮向下边沿 */

    /*执行模式切换（根据拨杆S1/S2和拨轮组合）*/
    Car_Mode_Switch_Update(car, tb_up, tb_dn);

    /*执行动作指令处理（摩擦轮、自瞄、发射控制）*/
    Car_Action_Cmd_Update(car, tb_up, tb_dn);
}
