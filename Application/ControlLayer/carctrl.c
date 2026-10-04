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
#include "cap.h"
#include "main.h"


Car_t hero_car;



/*初始化超时时间：750 * 2ms = 1.5秒，超过此时间，强制完成初始化 ，防止云台寻零失败导致机器人永远无法工作*/
#define INIT_MAX_TIMEOUT_TICKS   750

/** @brief 打弹冷却计数阈值：300 * 2ms = 0.6秒，防止连续打弹过快 */
#define SHOOT_COOLDOWN_TICKS     300

/** @brief 拨杆 s1 停稳判定阈值：50 * 2ms = 100ms ≈ 7 个 DBUS 帧。
  *        拨杆电平只在 ~14ms 一帧的 DBUS 帧里变，慢拨过中位可能采到 3~5 帧，
  *        100ms 从容盖住；真正发射时 s1 本来就停着不动，对正常手感零延迟。 */
#define SW_STABLE_TICKS          50

/* 打弹冷却计数器（遥控器 / 键鼠共用一把闸，保证换输入源不绕过 0.6s 冷却） */
static uint16_t shoot_cnt = SHOOT_COOLDOWN_TICKS;

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
  *            - 只看 s1 不看 s2：单发是拨一下就过的短促动作，
  *              若把 s2 也算进「停稳」，单发会被闸死。
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

/* ========================================================================= */
/*                     3. 输入源标记与超电放电开关                           */
/* ========================================================================= */

/**
  * @brief  输入源判定（摇杆 / 键鼠）
  * @details 键鼠数据与摇杆共用 DR16 同一帧上来（key_v 在 rxBuf[14..15]），
  *          二者天然不冲突。判定只看拨杆位置，是电平语义、每个 2ms 重评一次，
  *          不做边沿锁存：
  *
  *            S1下 + S2下（RC_SW_DOWN, RC_SW_DOWN）-> 键鼠
  *            其余组合                                   -> 遥控器摇杆
  *
  *          用拨杆位而不是「键盘/鼠标有动静」来判，是因为后者会被误触打断：
  *          摇杆开车时手滑敲一下键盘，整套输入源就跳走了。
  *
  * @param  s1 : 本拍拨杆 s1 快照
  * @param  s2 : 本拍拨杆 s2 快照
  * @retval 当前输入源
  */
static InputSrc_e Car_Input_Src_Detect(uint8_t s1, uint8_t s2) {
    if (s1 == RC_SW_DOWN && s2 == RC_SW_DOWN) {
        return INPUT_SRC_KEY;
    }
    return INPUT_SRC_RC;
}

/**
  * @brief  超级电容放电开关（Shift）
  * @details 电平语义：按住 Shift = 电容放电（底盘功率随之放宽），
  *          松开 = 待机充电。不做点按切换，松手即断，防误触后一直放电。
  */
static void Car_Cap_Cmd_Update(void) {
    cap.Y_O_N = (rc_sensor.info->Shift.value != 0U) ? 1U : 0U;
}

/* ========================================================================= */
/*                     4. 摩擦轮与发射动作控制                             */
/* ========================================================================= */

/**
  * @brief  摩擦轮与发射动作指令处理
  * @details 根据遥控器拨杆和拨轮输入，处理发射系统的开关控制：
  *
  *          1. 摩擦轮开关：S1中 + S2中 + 拨轮向上 -> 切换摩擦轮开/关
  *          2. 单发发射：S2上（需要摩擦轮已开启 + 冷却时间已过）
  *
  *          冷却机制：每次发射后需要等待SHOOT_COOLDOWN_TICKS（0.6秒）才能再次发射，
  *          防止连续打弹过快导致机构损坏或弹丸堵塞。
  *
  *          发射前有两道闸（都过了才出弹）：
  *            1. S1 不能停在下位：下位是键鼠域，遥控器发射通路在这一位一律不出弹。
  *               否则操作手「先把 S1 拨到下、S2 还在上」这个准备动作会每 0.6s 走一发单发。
  *            2. S1 必须停稳：三挡拨杆从上到下必经中位，过渡帧采到的 (中, 下)
  *               正好是连发条件，不闸住就一进/出键鼠走一发。
  *
  * @param  car       : 整车状态机对象指针
  * @param  tb_up     : 拨轮向上边沿触发标志
  * @param  s1        : 本拍拨杆 s1 快照
  * @param  s2        : 本拍拨杆 s2 快照
  * @param  s1_stable : s1 已停稳（由 Car_Control_Update 每拍算好传进来，
  *                     计数不归本函数管，否则键鼠期间会冻结）
  * @retval None
  */
static void Car_Action_Cmd_Update(Car_t *car, bool tb_up, uint8_t s1, uint8_t s2, bool s1_stable) {
    /* --- 1. 摩擦轮开关控制：S1中 + S2中 + 拨轮向上--- */
    if (s1 == 3 && s2 == 3 && tb_up) {
        car->flags.is_shoot_system_on = !car->flags.is_shoot_system_on;  //翻转切换开/关
    }

    /* --- 2. 发射逻辑（单发 / 手动打）--- */

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

    /*
     * - 连发：S1中 + S2下，按住持续出弹（必须停在中位，排除拨杆过渡帧）
     * - 单发：S2上，冷却内只出一帧脉冲，0.6s冷却一次，回中再向上实现单发
     */
    if (s2 == RC_SW_DOWN && s1 == RC_SW_MID) {
        car->flags.is_continuous_shoot = true;
    } else {
        car->flags.is_continuous_shoot = false;
        if (s2 == RC_SW_UP && shoot_cnt >= SHOOT_COOLDOWN_TICKS) {
            car->flags.is_single_shoot_cmd = true;   /* 发出单发指令 */
            shoot_cnt = 0;                           /* 重置冷却计数器 */
        }
    }
}

/**
  * @brief  键鼠模式下的发射动作指令处理
  * @details 与遥控器那套并列，只在 input_src == INPUT_SRC_KEY 时被调用，
  *          遥控器分支一个字节都不用动，键鼠分支也永远不会碰到
  *          「S2下 且 S1非上」那条连发条件（否则切进键鼠模式的瞬间就会走火）。
  *
  *          绑定：
  *          - 鼠标左键 : 摩擦轮关着 -> 先开摩擦轮（不打弹，防止冷轮打弹卡弹）
  *                       摩擦轮开着 -> 单发（0.6s 冷却，冷却内忽略）
  *          - B        : 关闭发射系统（边沿触发，关了才安全）
  *          - 连发     : 键鼠一律不给，防误触
  *
  * @param  car : 整车状态机对象指针
  * @retval None
  */
static void Car_Key_Action_Cmd_Update(Car_t *car) {
    /* 每次循环先清单发指令，只在满足条件时置位（与遥控器分支同一套习惯） */
    car->flags.is_single_shoot_cmd = false;

    /* 键鼠不给连发，彻底断掉走火通路 */
    car->flags.is_continuous_shoot = false;

    /* B：关闭发射系统（上升沿/下降沿都行，这里用「刚按下」）。
       关掉当拍直接返回，不给左键复活的机会 —— 否则同一拍里
       左键「刚按下」又把摩擦轮打开，紧急关闭被静默撤销。 */
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
  * @param  car     : 整车状态机对象指针
  * @param  forward : 前后通道（正 = 前）
  * @param  lateral : 左右通道（正 = 左）
  * @param  rotate  : 自旋通道（键鼠暂不映射，恒为 0）
  * @retval None
  */
void Car_Chassis_Channel_Get(Car_t *car, int16_t *forward, int16_t *lateral, int16_t *rotate) {
    /* WASD 计数是按键状态机在 2ms 环里无条件充的，而它兼作「按住渐进加速」的斜坡。
       只有斜坡真被当底盘速度消费时才放它充电，其余一律清零，
       否则斜坡会被后台充满，一进入可用状态当拍就满速冲车：
         - 不在键鼠模式       ：遥控器在用摇杆，计数无意义
         - 整车 SLEEP / INIT  ：既不跑底盘也不跑微调，充了就是纯预充电
                                （INIT 可长达 1.5s，比 0.8s 的斜坡满程还长；
                                 失联窗口 input_src 会停在旧值，也落在这条里）
         - Ctrl 刚松开那一拍  ：微调期间充的值不能带进平移
       Ctrl 按住期间不能清 —— 清了会让按键重新触发下降沿，微调变成连跳。 */
    const bool ramp_live = (car->input_src == INPUT_SRC_KEY) &&
                           (car->mode != CAR_MODE_SLEEP) &&
                           (car->mode != CAR_MODE_INIT);
    if (!ramp_live || rc_sensor.info->Ctrl.status == press_to_release) {
        rc_sensor.info->W.cnt = 0;
        rc_sensor.info->A.cnt = 0;
        rc_sensor.info->S.cnt = 0;
        rc_sensor.info->D.cnt = 0;
    }

    if (car->input_src != INPUT_SRC_KEY) {
        *forward = rc_sensor.info->ch3;
        *lateral = rc_sensor.info->ch2;
        *rotate  = rc_sensor.info->ch0;
        return;
    }

    /* Ctrl = 云台微调模式：底盘平移置零，微调时车不能跟着跑。
       WASD 边沿此时归云台做 0.05°/0.2° 步进微调用。 */
    if (rc_sensor.info->Ctrl.value != 0U) {
        *forward = 0;
        *lateral = 0;
    } else {
        *forward = (int16_t)((float)(rc_sensor.info->W.cnt - rc_sensor.info->S.cnt)
                             * CHASSIS_RC_MAX_VALUE / (float)KEY_W_CNT_MAX);
        *lateral = (int16_t)((float)(rc_sensor.info->A.cnt - rc_sensor.info->D.cnt)
                             * CHASSIS_RC_MAX_VALUE / (float)KEY_A_CNT_MAX);
    }

    /* 键鼠不映射自旋：GYRO/SPIN 下底盘跟随或恒定自旋由模式自己给 wz，
       摇杆那条前馈通道在键鼠模式下留给 0。 */
    *rotate = 0;
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

    cap.Y_O_N = 0;                            /* 超电默认待机充电 */
}



/*整车状态机主更新函数（根据拨杆+拨轮组合切换工作模式） */
void Car_Control_Update(Car_t *car) {
    car->last_mode = car->mode;  /* 用于边沿检测 */

    /* 拨杆快照：一拍只取一次，而且必须「一对」原子取。
       s1/s2 由 UART5 中断（优先级 5，抢占本 2ms 任务）在同一处理里背靠背写入
       （都从 rxBuf[5] 解出来），两次普通加载之间被抢占就会拼出「旧 s1 + 新 s2」
       的假组合 —— (3,2) 假组合正好是连发条件。关中断几拍取一对，每 2ms 一次。
       放在失联判断之前，是为了让停稳计数在任何路径下都跟着 s1 走。 */
    uint8_t s1, s2;
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        s1 = rc_sensor.info->s1;
        s2 = rc_sensor.info->s2;
        __set_PRIMASK(primask);
    }

    /* 停稳计数每拍都要走，不能只挂在遥控器分支里 ——
       键鼠期间计数会冻结，出键鼠那一瞬拿到的是进键鼠之前攒的陈旧值。
       「s1 进键鼠前停在中位 ≥100ms」的场景，回拨第一帧就被判已停稳、零停留走火。 */
    const bool s1_stable = Car_S1_Stable(s1);

    //失联保护
    if (rc_sensor.work_state != DEV_ONLINE || rc_sensor.errno != NONE_ERR) {
        car->mode = CAR_MODE_SLEEP;
        car->reset_state = RESET_PENDING;
        car->flags.is_shoot_system_on = false;    /* 关闭摩擦轮 */
        car->flags.is_single_shoot_cmd = false;   /* 取消单发指令 */
        car->flags.is_continuous_shoot = false;   /* 取消连发 */
        cap.Y_O_N = 0;                            /* 失联即断电容放电 */
        return;  /* 失联状态下不执行后续逻辑 */
    }

    /* 输入源标记 + 超电放电开关（Shift 按住 = 放电） */
    car->input_src = Car_Input_Src_Detect(s1, s2);
    Car_Cap_Cmd_Update();

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
    Car_Mode_Switch_Update(car, tb_up, tb_dn, s1, s2);

    /*执行动作指令处理（摩擦轮与发射控制）：按输入源分流。
       键鼠分支永不进入遥控器那套拨杆判定，
       否则 (s1=2, s2=2) 会同时满足「S2下 且 S1非上」的连发条件，切进键鼠就走火 */
    if (car->input_src == INPUT_SRC_KEY) {
        Car_Key_Action_Cmd_Update(car);
    } else {
        Car_Action_Cmd_Update(car, tb_up, s1, s2, s1_stable);
    }
}
