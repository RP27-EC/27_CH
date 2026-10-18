/**
  ******************************************************************************
  * @file           : carctrl.c
  * @brief          : 整车状态机实现
  ******************************************************************************
  */

#include "car_control.h"
#include "rc_sensor.h"
#include "gimbal.h"
#include "chassis.h"
#include "main.h"


Car_t hero_car;
CarMechMouseDebug_t car_mech_mouse_debug;



/*初始化超时时间：750 * 2ms = 1.5秒，超过此时间，强制完成初始化 ，防止云台寻零失败导致机器人永远无法工作*/
#define INIT_MAX_TIMEOUT_TICKS   750

/** @brief 打弹冷却计数阈值：300 * 2ms = 0.6秒，防止连续打弹过快 */
#define SHOOT_COOLDOWN_TICKS     300

/** @brief 拨杆 s1 停稳判定阈值：50 * 2ms = 100ms*/
#define SW_STABLE_TICKS          50

/** @brief 拨杆 s2 停稳判定阈值：15 * 2ms = 30ms */
#define SW2_STABLE_TICKS         15

/* 打弹冷却计数器（遥控器 / 键鼠共用一把闸，保证换输入源不绕过 0.6s 冷却） */
static uint16_t shoot_cnt = SHOOT_COOLDOWN_TICKS;


/* Ctrl 电平快照与下降沿：Car_Control_Update 每拍在关中断块里取一次，
   Car_Chassis_Channel_Get 用同一份读数。底盘通道函数在 2ms 环末尾才跑，
   若直接读 rc_sensor.info，中间落一帧 DBUS 就会读到「value 已归零、
   status 还停在 long_press」的撕裂状态——清零不触发、平移分支照走，
   预充电的 WASD 计数当拍就折算成满速。 */
static uint8_t ctrl_val_snap = 0;
static bool    ctrl_fall_edge = false;



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

/**
  * @brief  拨杆 s1 是否已停稳
  * @details 三挡拨杆从「上」拨到「下」必经「中」，过渡帧会采到 (S1中, S2下)，
  *          正好命中遥控器连发条件，一进/出键鼠就走火。这里要求 s1 连续
  *          SW_STABLE_TICKS 拍没动过才放行发射：
  *
  *            - 单发靠拨 s2、连发靠 s1 停在中位，正常操作时 s1 本来就是停稳的，
  *              所以这道闸对正常手感零延迟；
  *            - 单发仍只看 s1：单发是拨一下就过的短促动作，把 s2 也算进
  *              单发的停稳会把它闸死。连发额外要求 s2 停稳（见 Car_S2_Stable），
  *              堵住「进键鼠时 s2 先动」产生的真实 (中, 下) 错位帧。
  *
  *          【调用契约】必须每个 2ms 拍都被调一次，不能挂在某个输入源分支里。
  *          本函数靠静态计数记「s1 多久没动」，一旦漏拍，计数就停在旧值上：
  *          键鼠期间漏调的话，出键鼠那一瞬拿到的是进键鼠之前攒的数，
  *          「s1 进键鼠前停在中位」的场景回拨第一帧就被判已停稳、零停留走火。
  *
  * @param  s1 : 本拍拨杆 s1 快照
  * @retval true : s1 已停稳，可以发射
  * @retval false: s1 刚动过，禁止发射
  */
static bool Car_S1_Stable(uint8_t s1) {
    static uint8_t last_s1 = 0xFF;   /* 上一拍的 s1，0xFF 保证首拍必算「刚动过」 */
    static uint8_t hold_ticks = 0;   /* s1 连续未变的拍数 */

    if (s1 != last_s1) {
        last_s1 = s1;
        hold_ticks = 0;
        return false;
    }
    if (hold_ticks < SW_STABLE_TICKS) {
        hold_ticks++;
        return false;
    }
    return true;
}

/**
  * @brief  拨杆 s2 是否已停稳
  * @details 与 Car_S1_Stable 同构，但只服务于「连发装配」这一个判定点。
  *          进键鼠时两根拨杆先后拨到下位，s2 先动会先来一帧 (中, 下)——
  *          这帧是线上真实值（两根拨杆本来就先后到位），不是 ISR 撕裂，
  *          Car_S1_Stable 拦不住：s1 还停在中位没动，hold_ticks 一直在涨。
  *          这里要求 s2 也停稳，那帧错位就被 SW2_STABLE_TICKS 的停稳窗挡掉。
  *
  *          【调用契约】与 Car_S1_Stable 相同：必须每个 2ms 拍都被调一次，
  *          不能挂在某个输入源分支里，否则计数会冻结。
  *
  * @param  s2 : 本拍拨杆 s2 快照
  * @retval true : s2 已停稳，可以装配连发
  * @retval false: s2 刚动过，禁止装配连发
  */
static bool Car_S2_Stable(uint8_t s2) {
    static uint8_t last_s2 = 0xFF;   /* 上一拍的 s2，0xFF 保证首拍必算「刚动过」 */
    static uint8_t hold_ticks = 0;   /* s2 连续未变的拍数 */

    if (s2 != last_s2) {
        last_s2 = s2;
        hold_ticks = 0;
        return false;
    }
    if (hold_ticks < SW2_STABLE_TICKS) {
        hold_ticks++;
        return false;
    }
    return true;
}

/**
  * @brief  Ctrl 松开下降沿检测（电平法）
  * @details 不用键盘状态机的 press_to_release：那个边沿由 keyboard_update 在
  *          2ms 环开头产出，而底盘通道函数在环末尾才读，中间落一帧 DBUS 就会
  *          读到「value 已归零、status 还停在 long_press」的撕裂状态——
  *          清零条件不触发、平移分支照走，预充电的 WASD 计数当拍就满速冲车。
  *          这里直接比对相邻两拍的电平快照，两个读数都来自关中断块，天然一致。
  *
  *          【调用契约】必须每个 2ms 拍都被调一次（同 Car_S1_Stable）。
  *
  * @param  ctrl_val : 本拍 Ctrl 电平快照
  * @retval true : Ctrl 刚松开（上一拍还按着、这一拍已松）
  */
static bool Ctrl_Fall_Edge(uint8_t ctrl_val) {
    static uint8_t last_val = 0xFF;  /* 上一拍电平，0xFF 保证首拍不算下降沿 */
    bool edge = (last_val != 0U && ctrl_val == 0U);
    last_val = ctrl_val;
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
            car->mode = CAR_MODE_MECH;           /* 复位完成后默认进入机械模式 CAR_MODE_MECH */
        }
    }
}

/* ========================================================================= */
/*                     2. 模式切换（遥控器拨轮矩阵 / 键鼠 F/G/V）              */
/* ========================================================================= */

/*遥控器拨杆与拨轮的模式切换处理（s1/s2 用本拍快照，见 Car_Control_Update）*/
static void Car_Mode_Switch_Update(Car_t *car, bool tb_up, bool tb_dn, uint8_t s1, uint8_t s2) {
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

/**
  * @brief  键鼠模式下的整车模式切换（F/G/V 三键直达）
  *          绑定（边沿 release_to_press，一拍只触发一次）：
  *          - F : 进入陀螺仪模式 CAR_MODE_GYRO
  *          - G : 进入小陀螺模式 CAR_MODE_SPIN
  *          - V : 进入机械模式   CAR_MODE_MECH
  *          SLEEP / INIT 一律不响应
  */
static void Car_Key_Mode_Switch_Update(Car_t *car) {
    if (car->mode == CAR_MODE_SLEEP || car->mode == CAR_MODE_INIT) {
        return;
    }

    if (rc_sensor.info->F.status == release_to_press) {
        car->mode = CAR_MODE_GYRO;
    } else if (rc_sensor.info->G.status == release_to_press) {
        car->mode = CAR_MODE_SPIN;
    } else if (rc_sensor.info->V.status == release_to_press) {
        car->mode = CAR_MODE_MECH;
    }
}


/*输入源判定（摇杆 / 键鼠） S1下 + S2下（RC_SW_DOWN, RC_SW_DOWN）*/
static InputSrc_e Car_Input_Src_Detect(uint8_t s1, uint8_t s2) {
    if (s1 == RC_SW_DOWN && s2 == RC_SW_DOWN) {
        return INPUT_SRC_KEY;
    }
    return INPUT_SRC_RC;
}

/**
  * @brief  键鼠 R 换头：状态机一拍一变，这里只认「刚进入 release_to_press」
  * @details 键盘状态在 ~14ms 的 DBUS 帧里保持，而本环是 2ms——
  *          直接判 status==release_to_press 会连置多拍脉冲。
  *          换头结束后若 status 仍高，就会再翻一次锁轴，
  *          表现为「翻过去又翻回来、没有换头」。这里用相邻拍边沿掐掉。
  * @retval true : 本拍 R 刚按下（单帧）
  */
static bool Car_R_Turn_Edge(void) {
    static bool prev = false;
    const bool now = (rc_sensor.info->R.status == release_to_press);
    const bool edge = now && !prev;
    prev = now;
    return edge;
}

/* ========================================================================= */
/*                     4. 摩擦轮与发射动作控制                             */
/* ========================================================================= */

/**
  * @brief  摩擦轮与发射动作指令处理
  * @details 根据遥控器拨杆和拨轮输入，处理发射系统的开关控制：
  *
  *          1. 摩擦轮开关：S1中 + S2中 + 拨轮向上 -> 切换摩擦轮开/关
  *          2. 单发：S2上（上升沿）。s2 从非上位拨到上位出一发，
  *             必须回中（s2=3）再拨上来才有下一发——真单发，不是 0.6s 一发的伪单发。
  *             旧的冷却逻辑注释掉保留：它让「拨一下按住」看起来像慢速连发。
  *
  *          发射前有几道闸（都过了才出弹）：
  *            1. S1 不能停在下位：下位是键鼠域，遥控器发射通路在这一位一律不出弹。
  *               否则操作手「先把 S1 拨到下、S2 还在上」这个准备动作会每 0.6s 走一发单发。
  *            2. S1 必须停稳：三挡拨杆从上到下必经中位，过渡帧采到的 (中, 下)
  *               正好是连发条件，不闸住就一进/出键鼠走一发。
  *            3. 连发还要求 S2 停稳：进键鼠时两根拨杆先后拨到下位，S2 先动会
  *               先来一帧真实的 (中, 下)。S1 还停在中位没动，第 2 道闸拦不住
  *               这一帧——那是线上值不是撕裂。S2 停稳窗把这类错位帧挡掉。
  *
  */
static void Car_Action_Cmd_Update(Car_t *car, bool tb_up, uint8_t s1, uint8_t s2, bool s1_stable, bool s2_stable) {
    /* --- 摩擦轮开关控制：S1中 + S2中 + 拨轮向上--- */
    if (s1 == 3 && s2 == 3 && tb_up) {
        car->flags.is_shoot_system_on = !car->flags.is_shoot_system_on;  //翻转切换开/关
    }

    /* --- 发射逻辑（单发 / 手动打）--- */

    car->flags.is_single_shoot_cmd = false; //！！每次循环先清除单发指令，只在满足条件时置位

    //摩擦轮未开启时，不响应任何打弹指令
    if (!car->flags.is_shoot_system_on) {
        shoot_cnt = SHOOT_COOLDOWN_TICKS;  /* 重置冷却计数器，确保下次可以立即发射 */
        car->flags.is_continuous_shoot = false;
        return;
    }

    //冷却计时递增（每2ms加1，到阈值后允许再次发射）
    if (shoot_cnt < SHOOT_COOLDOWN_TICKS) {
        shoot_cnt++;
    }

    /* 两道闸：S1 停在下位 = 键鼠域，S1 刚动过 = 拨杆过渡帧，都不许出弹。
       冷却计时照走，切换输入源不会人为拉长 0.6s。 */
    if (s1 == RC_SW_DOWN || !s1_stable) {
        car->flags.is_continuous_shoot = false;
        return;
    }

    /*---------- 连发：S1中 + S2下，按住持续出弹。额外要求 S2 停稳--------*/

    // if (s2 == RC_SW_UP && shoot_cnt >= SHOOT_COOLDOWN_TICKS) {
    //     car->flags.is_single_shoot_cmd = true;   /* 发出单发指令 */
    //     shoot_cnt = 0;                           /* 重置冷却计数器 */
    // }
    static uint8_t s2_last = 0xFF;  /* 上一拍的 s2，0xFF 保证首拍不算上升沿 */
    if (s2 == RC_SW_DOWN && s1 == RC_SW_MID && s2_stable) {
        car->flags.is_continuous_shoot = true;
    } else {
        car->flags.is_continuous_shoot = false;
        if (s2 == RC_SW_UP && s2_last != RC_SW_UP) {
            car->flags.is_single_shoot_cmd = true;   /* 上升沿：拨到上位的那一瞬出一发 */
        }
    }
    s2_last = s2;  /* 每拍都更新，不管走没走单发分支 */
}



/*==========键鼠模式下的发射动作指令处理======*/

static void Car_Key_Action_Cmd_Update(Car_t *car) {
    /* 每次循环先清单发指令，只在满足条件时置位 */
    car->flags.is_single_shoot_cmd = false;

    /* 键鼠不给连发，彻底断掉走火通路 */
    car->flags.is_continuous_shoot = false;

    /* B：关闭发射系统 */
    if (rc_sensor.info->B.status == release_to_press) {
        car->flags.is_shoot_system_on = false;
        shoot_cnt = SHOOT_COOLDOWN_TICKS;
        return;
    }

    /* 摩擦轮未开启：左键先开摩擦轮，本次不打弹 */
    if (!car->flags.is_shoot_system_on) {
        shoot_cnt = SHOOT_COOLDOWN_TICKS;   /* 重置冷却，确保下次能立即发射 */
        if (rc_sensor.info->mouse_btn_l.status == release_to_press) {
            car->flags.is_shoot_system_on = true;
        }
        return;
    }

    /* 冷却计时递增（每 2ms 加 1，到阈值后允许再次发射） */
    if (shoot_cnt < SHOOT_COOLDOWN_TICKS) {
        shoot_cnt++;
    }

    /* 左键单发：只认「刚按下」这一帧，冷却内忽略 */
    if (rc_sensor.info->mouse_btn_l.status == release_to_press &&
        shoot_cnt >= SHOOT_COOLDOWN_TICKS) {
        car->flags.is_single_shoot_cmd = true;
        shoot_cnt = 0;
    }
}

/* ========================================================================= */
/*                     5. 底盘通道取值（遥控器 / 键鼠）                       */
/* ========================================================================= */

/* 鼠标每帧增量/真实帧间隔 -> 角速度 -> 等效ch0。
 * 这里保持速度指令，不累加角度；同一DBUS帧在多个2ms控制拍之间保持，不能重复积分。
 * 切入MECH/键鼠或松开Ctrl后先等新帧，避免复用上一模式/微调时的移动。 */
static int16_t Car_Mech_Mouse_Rotate(Car_t *car, bool context_changed) {
    static bool was_active = false;
    static bool waiting_new_frame = false;
    static uint16_t entry_seq;
    uint16_t seq;
    int16_t mouse_dx;
    uint32_t frame_ms, interval_ms;
    bool received;
    uint8_t ctrl;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    seq = rc_sensor.info->frame_seq;
    mouse_dx = rc_sensor.info->mouse_vx;
    frame_ms = rc_sensor.info->frame_update_ms;
    interval_ms = rc_sensor.info->frame_interval_ms;
    received = rc_sensor.info->frame_received;
    ctrl = rc_sensor.info->Ctrl.value;
    __set_PRIMASK(primask);

    /* 只观察现有判据，不改变原来的输入放行/屏蔽条件。 */
    car_mech_mouse_debug.mode = car->mode;
    car_mech_mouse_debug.input_src = car->input_src;
    car_mech_mouse_debug.mouse_dx = mouse_dx;
    car_mech_mouse_debug.frame_seq = seq;
    car_mech_mouse_debug.frame_age_ms = HAL_GetTick() - frame_ms;
    car_mech_mouse_debug.frame_interval_ms = interval_ms;
    car_mech_mouse_debug.frame_received = received;
    car_mech_mouse_debug.rc_online = rc_sensor.work_state == DEV_ONLINE;
    car_mech_mouse_debug.rc_data_ok = rc_sensor.errno == NONE_ERR;
    car_mech_mouse_debug.ctrl_value = ctrl;
    car_mech_mouse_debug.ctrl_snapshot = ctrl_val_snap;
    car_mech_mouse_debug.ctrl_release_edge = ctrl_fall_edge;
    car_mech_mouse_debug.channel = 0;
    car_mech_mouse_debug.requested_motor_rpm = 0.0f;

    bool active = car->input_src == INPUT_SRC_KEY && car->mode == CAR_MODE_MECH &&
        ctrl_val_snap == 0U && ctrl == 0U && !ctrl_fall_edge &&
        rc_sensor.work_state == DEV_ONLINE && rc_sensor.errno == NONE_ERR;
    if (!active) {
        was_active = false;
        waiting_new_frame = false;
        if (car->input_src != INPUT_SRC_KEY || car->mode != CAR_MODE_MECH)
            car_mech_mouse_debug.reason = MECH_MOUSE_INACTIVE_MODE;
        else if (ctrl_val_snap != 0U || ctrl != 0U || ctrl_fall_edge)
            car_mech_mouse_debug.reason = MECH_MOUSE_CTRL_BLOCKED;
        else if (rc_sensor.work_state != DEV_ONLINE)
            car_mech_mouse_debug.reason = MECH_MOUSE_RC_OFFLINE;
        else
            car_mech_mouse_debug.reason = MECH_MOUSE_RC_ERROR;
        return 0;
    }
    if (!was_active || context_changed) {
        entry_seq = seq;
        waiting_new_frame = true;
    }
    was_active = true;
    if (waiting_new_frame) {
        if (seq == entry_seq) {
            car_mech_mouse_debug.reason = MECH_MOUSE_WAIT_NEW_FRAME;
            return 0;
        }
        waiting_new_frame = false;
    }
    if (!received) {
        car_mech_mouse_debug.reason = MECH_MOUSE_NO_VALID_FRAME;
        return 0;
    }
    if (car_mech_mouse_debug.frame_age_ms >= MECH_MOUSE_TIMEOUT_MS) {
        car_mech_mouse_debug.reason = MECH_MOUSE_FRAME_STALE;
        return 0;
    }
    if (interval_ms == 0U || interval_ms > MECH_MOUSE_TIMEOUT_MS) {
        car_mech_mouse_debug.reason = MECH_MOUSE_BAD_INTERVAL;
        return 0;
    }
    if (mouse_dx == 0) {
        car_mech_mouse_debug.reason = MECH_MOUSE_ZERO_MOVEMENT;
        return 0;
    }

    /* 鼠标右移为正，与ch0同号；chassis.c统一取反后得到顺时针转向。
     * 用原始增量避免鼠标停止后均值滤波的尾值继续驱动车辆。 */
    float speed_rads = (float)mouse_dx * MECH_MOUSE_YAW_SENS_RAD_PER_COUNT *
                       1000.0f / (float)interval_ms;
    float norm = speed_rads * CHASSIS_YAW_RPM_PER_RADS / CHASSIS_MAX_MOTOR_RPM;
    if (norm > 1.0f) norm = 1.0f;
    if (norm < -1.0f) norm = -1.0f;
    /* 下游还会扣除旋转通道死区，这里反算通道，避免低速鼠标被30计数死区吞掉。 */
    float channel = norm * (CHASSIS_RC_MAX_VALUE - CHASSIS_ROTATE_DEADZONE);
    if (norm > 0.0f) channel += CHASSIS_ROTATE_DEADZONE;
    if (norm < 0.0f) channel -= CHASSIS_ROTATE_DEADZONE;
    car_mech_mouse_debug.reason = MECH_MOUSE_OUTPUT_OK;
    car_mech_mouse_debug.requested_motor_rpm = norm * CHASSIS_MAX_MOTOR_RPM;
    car_mech_mouse_debug.channel = (int16_t)channel;
    return car_mech_mouse_debug.channel;
}

/**
  * @brief  按当前输入源取底盘三轴通道
  * @details 输出统一是 SBUS 计数域 [-660, 660]：遥控器直接回传摇杆通道，
  *          键鼠把「按住时间比例」折算到同一量纲，底盘模块因此不用改一行。
  *
  *          渐进加速取「每拍赋值」而不是积分：目标 = cnt / cnt_max，
  *          松手 cnt 清零 -> 当拍速度就归零（不用再做斜坡回落），
  *          按满 cnt_max 自动饱和到 1.0（cnt 本身在 keyboard_status_update 里被钳位），
  *          W+S / A+D 同按自然对消。没有斜坡状态要维护。
  *
  *          键鼠旋转映射（仅机械模式）：鼠标左移左旋、右移右旋，与摇杆左右同向
  *          （chassis.c 对 rotate 通道取反后，左打 = 逆时针）。
  *          GYRO/SPIN 的键鼠 rotate 保持0，GYRO 使用云台 IMU 前馈，SPIN 由模式给 wz。
  *
  * @param  car     : 整车状态机对象指针
  * @param  forward : 前后通道（正 = 前）
  * @param  lateral : 左右通道（正 = 左）
  * @param  rotate  : ch0同号通道，右转为正；键鼠仅机械模式映射鼠标，其余为0
  * @retval None
  */
void Car_Chassis_Channel_Get(Car_t *car, int16_t *forward, int16_t *lateral, int16_t *rotate) {
    static InputSrc_e last_input_src = INPUT_SRC_RC;
    static CarMode_e last_mode = CAR_MODE_SLEEP;
    const bool context_changed = car->input_src != last_input_src || car->mode != last_mode;
    if (context_changed) {
        Chassis_Reset_Yaw_Feedforward(); /* 模式或输入源切换时重置 IMU 前馈滤波。 */
        last_input_src = car->input_src;
        last_mode = car->mode;
    }

    const bool ramp_live = (car->input_src == INPUT_SRC_KEY) &&
                           (car->mode != CAR_MODE_SLEEP) &&
                           (car->mode != CAR_MODE_INIT);
    if (!ramp_live || ctrl_fall_edge) {
        rc_sensor.info->W.cnt = 0;
        rc_sensor.info->A.cnt = 0;
        rc_sensor.info->S.cnt = 0;
        rc_sensor.info->D.cnt = 0;
    }

    /* 每拍维护激活状态，保证模式切换/Ctrl释放后不消费旧鼠标样本。 */
    const int16_t mouse_rotate = Car_Mech_Mouse_Rotate(car, context_changed);

    if (car->input_src != INPUT_SRC_KEY) {
        *forward = rc_sensor.info->ch3;
        *lateral = rc_sensor.info->ch2;
        *rotate  = rc_sensor.info->ch0;
        return;
    }

    /* Ctrl = 云台微调模式：底盘平移与旋转都置零，微调时车不能跟着跑。
       WASD 边沿此时归云台做 0.05°/0.2° 步进微调用。 */
    if (ctrl_val_snap != 0U) {
        *forward = 0;
        *lateral = 0;
        *rotate  = 0;
    } else {
        *forward = (int16_t)((float)(rc_sensor.info->W.cnt - rc_sensor.info->S.cnt)
                             * CHASSIS_RC_MAX_VALUE / (float)KEY_W_CNT_MAX);
        /* 与实车横移方向对齐：底盘对 lateral 取反，A 给负值后向左，D 向右。 */
        *lateral = (int16_t)((float)(rc_sensor.info->D.cnt - rc_sensor.info->A.cnt)
                             * CHASSIS_RC_MAX_VALUE / (float)KEY_A_CNT_MAX);

        /* 机械模式用鼠标横移控制底盘旋转，Q/E不再参与转向。
           GYRO 由 chassis.c 使用云台 IMU 实测角速度前馈；SPIN 不接收旋转键。 */
        if (car->mode == CAR_MODE_MECH) {
            *rotate = mouse_rotate;
        } else {
            *rotate = 0;
        }
    }
}

/* ========================================================================= */
/*                          6. 状态机总入口函数                              */
/* ========================================================================= */


void Car_Control_Init(Car_t *car) {
    car->mode = CAR_MODE_SLEEP;           /* 初始模式：休眠 */
    car->last_mode = CAR_MODE_SLEEP;      /* 上一次模式：休眠 */
    car->reset_state = RESET_PENDING;     /* 复位状态：等待中 */
    car->init_timer = 0;                  /* 初始化计时器清零 */
    car->input_src = INPUT_SRC_RC;        /* 默认遥控器摇杆 */

    /* 所有动作标志位清零 */
    car->flags.is_shoot_system_on = false;    /* 摩擦轮关闭 */
    car->flags.is_single_shoot_cmd = false;   /* 无单发指令 */
    car->flags.is_continuous_shoot = false;   /* 无连发 */
    car->flags.is_turn_cmd = false;           /* 无换头 */
}



/*整车状态机主更新函数（根据拨杆+拨轮组合切换工作模式） */
void Car_Control_Update(Car_t *car) {
    car->last_mode = car->mode;  /* 用于边沿检测 */

    /* 本拍输入快照：s1/s2/Ctrl 三者必须取自同一帧 DBUS，所以整体放在关中断临界区里 */
    uint8_t s1, s2;
    {
        /* 备份当前中断总开关状态，取完快照后按原样恢复 */
        const uint32_t primask = __get_PRIMASK();
        //关全局中断
        __disable_irq();

        /* 取拨杆快照：本拍内全函数只认这一份局部值，不再直接碰共享变量 */
        s1 = rc_sensor.info->s1;
        s2 = rc_sensor.info->s2;
        /* 取 Ctrl 键电平快照：写入文件级变量，后面 Ctrl_Fall_Edge 和底盘通道函数都要消费它 */
        ctrl_val_snap = rc_sensor.info->Ctrl.value;
        
        //恢复中断
        __set_PRIMASK(primask);
    }

    /* 停稳判定 + Ctrl 松开边沿：放在分流之前、每 2ms 无条件执行（键鼠期间照算），
       函数内部的计数器才不会冻结成陈旧值、出键鼠第一拍误报「已停稳」走火 */

    const bool s1_stable = Car_S1_Stable(s1);

    const bool s2_stable = Car_S2_Stable(s2);

    ctrl_fall_edge = Ctrl_Fall_Edge(ctrl_val_snap);

    //失联保护
    if (rc_sensor.work_state != DEV_ONLINE || rc_sensor.errno != NONE_ERR) {
        car->mode = CAR_MODE_SLEEP;
        car->reset_state = RESET_PENDING;
        car->flags.is_shoot_system_on = false;    /* 关闭摩擦轮 */
        car->flags.is_single_shoot_cmd = false;   /* 取消单发指令 */
        car->flags.is_continuous_shoot = false;   /* 取消连发 */
        car->flags.is_turn_cmd = false;           /* 取消换头 */
        return;  /* 失联状态下不执行后续逻辑 */
    }

    /* 输入源标记 */
    car->input_src = Car_Input_Src_Detect(s1, s2);

    /*上电复位流程*/
    if (car->reset_state == RESET_PENDING) {
        car->mode = CAR_MODE_INIT;  /* 设为初始化模式 */
        Car_Init_State_Machine(car);  /* 执行初始化状态机 */
        return;  /* 初始化期间不执行模式切换和动作控制 */
    }

    /*捕获拨轮边沿触发*/
    bool tb_up = RC_Thumbwheel_Up_Edge();  /* 拨轮向上边沿 */
    bool tb_dn = RC_Thumbwheel_Dn_Edge();  /* 拨轮向下边沿 */

    /* 换头脉冲：每拍先清，只在满足条件时置一拍（与单发同一套习惯）。
       仅 MECH 可换头（粘性锁轴 FRONT↔BACK 翻转）；
       GYRO/SPIN 的世界系 +180° 掉头已删除，触发条件直接不认。 */
    car->flags.is_turn_cmd = false;

    /*模式切换：按输入源分流，两套并列互不串扰 */
    if (car->input_src == INPUT_SRC_KEY) {
        Car_Key_Mode_Switch_Update(car);
        Car_Key_Action_Cmd_Update(car);

        /* 键鼠换头：R 刚按下 */
        if (Car_R_Turn_Edge() && car->mode == CAR_MODE_MECH) {
            car->flags.is_turn_cmd = true;
        }
    } else {
        Car_Mode_Switch_Update(car, tb_up, tb_dn, s1, s2);
        Car_Action_Cmd_Update(car, tb_up, s1, s2, s1_stable, s2_stable);

        /* 遥控换头 */
        if (s1 == RC_SW_UP && s2 == RC_SW_UP && tb_up &&
            car->mode == CAR_MODE_MECH) {
            car->flags.is_turn_cmd = true;
        }
    }
}
