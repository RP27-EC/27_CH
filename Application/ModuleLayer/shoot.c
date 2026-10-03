/**
  ******************************************************************************
  * @file           : shoot.c
  * @brief          : 拨弹控制模块
  * @details        : 本文件实现了下层板拨弹电机的安全控制逻辑，包括：
  *
  *                   1. 摩擦轮就绪安全锁：未达速/电机离线时拨盘锁位不动，指令丢弃
  *                   2. 单发：角度环跟踪encoder_sum，每次加一格弹
  *                   3. 连发：定频步进（每300ms推进一格），与单发共用角度环
  *                   4. 松手回吸：停射时目标对齐到最近整格，防半格卡弹
  *                   5. 防卡弹反转：堵转先退一格松弹，再补回原目标重试
  *                   6. 卡弹停机：重试用尽则停射停摩擦轮，重拨总开关才解锁
  *
  *                   拨弹状态机：
  *                   SLEEP -> SPINUP -> READY -> FIRE <-> RECOIL -> STOP_JAM
  *                   总开关关闭、失联或电机离线时回退到SLEEP
  ******************************************************************************
  */

#include "shoot.h"
#include "shoot_motor.h"
#include "communicate.h"
#include <math.h>

static ShootState_e shoot_state = SHOOT_STATE_SLEEP;
static float    angle_sum_target = 0.0f;   /* 角度环目标(encoder_sum) */
static float    jam_saved_target = 0.0f;   /* 卡弹瞬间锁存的原目标 */
static bool     fric_on = false;
static bool     jam_latched = false;       /* 卡弹锁存：重拨总开关才清 */
static bool     state_gate_latched = false; /* 0x023门禁锁存：收到过即永久走严格判据 */
static bool     continuous_armed = false;  /* 连发边沿锁存：松开后才能重新武装 */
static bool     continuous_last = false;   /* 连发上一次电平，用于求上升沿 */
static uint16_t spinup_cnt = 0;            /* 摩擦轮就绪等待计数 */
static uint16_t fire_cnt = 0;              /* 补弹超时计数 */
static uint16_t recoil_cnt = 0;            /* 反转超时计数 */
static uint16_t stuck_cnt = 0;             /* 卡弹判据持续计数 */
static uint16_t repeat_cnt = 0;            /* 连发定频周期计数 */
static uint8_t  jam_retry = 0;             /* 本发弹已补位重试次数 */

/**
  * @brief  拨弹模块初始化
  * @details 电机硬件初始化已在DEVICE_Init完成，这里只复位模块状态并卸力。
  * @retval None
  */
void Shoot_Init(void)
{
    shoot_state = SHOOT_STATE_SLEEP;
    angle_sum_target = 0.0f;
    jam_saved_target = 0.0f;
    fric_on = false;
    jam_latched = false;
    state_gate_latched = false;
    continuous_armed = false;
    continuous_last = false;
    spinup_cnt = 0;
    fire_cnt = 0;
    recoil_cnt = 0;
    stuck_cnt = 0;
    repeat_cnt = 0;
    jam_retry = 0;
    Shoot_Motor_Stop();
}

/**
  * @brief  查询当前摩擦轮标志
  * @retval true=通知上层板开摩擦轮
  */
bool Shoot_Is_Fric_On(void)
{
    return fric_on;
}

/**
  * @brief  回吸最近整格
  * @details 把角度环目标对齐到 SHOOT_ONESHOT_ANGLE 的整数倍，
  *          使拨盘停在槽位正对出弹口的位置，防止半格卡弹。
  *          停射、起转等待、卡弹停机时调用。
  * @retval None
  */
static void Shoot_Round_To_Slot(void)
{
    const float slot = SHOOT_ONESHOT_ANGLE;
    float pos = (float)Shoot_Motor.rx_info->encoder_sum;

    angle_sum_target = roundf(pos / slot) * slot;
}

/**
  * @brief  摩擦轮就绪判据
  * @details 首次收到0x023状态帧后锁存严格门禁（state_gate_latched），
  *          此后0x023丢失视为未就绪，绝不再退回延时兜底——
  *          否则上层板复位/失联时拨盘会把弹推进已停转的摩擦轮。
  *          延时兜底仅用于上层板尚未升级、从未回传过0x023的兼容期。
  * @retval true=摩擦轮就绪，允许拨盘动作
  */
static bool Shoot_Is_Fric_Ready(void)
{
    if (board_rx_info.is_state_online) {
        state_gate_latched = true;
        return board_rx_info.is_fric_speed && board_rx_info.is_fric_work;
    }
    if (state_gate_latched) {
        return false;   /* 曾收到0x023，现丢失：锁死拨盘，禁止退化 */
    }
    return spinup_cnt >= SHOOT_SPINUP_TIMEOUT_TICKS;  /* 兼容期延时兜底 */
}

/**
  * @brief  卡弹判据：角度没走完 + 电流大 + 持续 三取二
  * @param  err : 目标与反馈的偏差(encoder_sum)
  * @retval true=判为卡弹
  */
static bool Shoot_Is_Stuck(float err)
{
    float current = fabsf((float)Shoot_Motor.rx_info->torque_current_raw);

    if (fabsf(err) > SHOOT_STUCK_ANGLE_ERR && current > SHOOT_STUCK_CURRENT) {
        if (stuck_cnt < 0xFFFFU) {
            stuck_cnt++;
        }
        return stuck_cnt > SHOOT_STUCK_TICKS;
    }

    stuck_cnt = 0;
    return false;
}

/**
  * @brief  拨弹控制更新
  * @details 每2ms执行一次。
  *
  *          安全规则：
  *          1. 未达速不进READY，进了READY若掉速则退回SPINUP
  *          2. 卡弹目标在FIRE->RECOIL跳转瞬间锁存，RECOIL内只读不写
  *          3. 连发上升沿武装一次，RECOIL/STOP_JAM/SLEEP里解除，
  *             防止卡弹时按住连发杆被反复触发
  *
  * @param  car      : 整车状态机
  * @param  failsafe : 失联保护标志，true时立即卸力
  * @retval None
  */
void Shoot_Update(const Car_t *car, bool failsafe)
{
    /* 心跳：无反馈时offline_cnt累加，超时置DEV_OFFLINE */
    Shoot_Motor.single_heart_beat(&Shoot_Motor);

    bool motor_online = (Shoot_Motor.state->status == DEV_ONLINE);
    bool system_on = car->flags.is_shoot_system_on &&
                     (!failsafe) && motor_online;

    /* 连发上升沿检测：只在按下的那一瞬武装一次 */
    bool continuous_rise = car->flags.is_continuous_shoot && (!continuous_last);
    continuous_last = car->flags.is_continuous_shoot;
    if (continuous_rise) {
        continuous_armed = true;
        repeat_cnt = SHOOT_REPEAT_PERIOD_TICKS;  /* 首发立即出弹 */
    }

    /* 重拨总开关 = 卡弹解锁（人工确认排障完成） */
    if (!car->flags.is_shoot_system_on) {
        jam_latched = false;
    }

    /* ---------- 总开关关 / 失联 / 拨盘离线：卸力并回吸整格 ---------- */
    if (!system_on) {
        Shoot_Motor_Stop();
        Shoot_Round_To_Slot();
        shoot_state = SHOOT_STATE_SLEEP;
        fric_on = false;
        continuous_armed = false;
        jam_retry = 0;
        stuck_cnt = 0;
        board_tx_info.is_fric_on = 0;
        return;
    }

    /* ---------- 卡弹锁存：停射停摩擦轮，等人工排障 ---------- */
    if (jam_latched) {
        Shoot_Motor_Stop();
        Shoot_Round_To_Slot();
        shoot_state = SHOOT_STATE_STOP_JAM;
        fric_on = false;
        continuous_armed = false;
        board_tx_info.is_fric_on = 0;
        return;
    }

    /* 开发射：通知上层板起摩擦轮 */
    fric_on = true;
    board_tx_info.is_fric_on = 1;

    /* 连发定频周期计数（跨状态累计，保证射速恒定） */
    if (continuous_armed && car->flags.is_continuous_shoot) {
        if (repeat_cnt < SHOOT_REPEAT_PERIOD_TICKS) {
            repeat_cnt++;
        }
    }

    switch (shoot_state) {

    /* ---------- 卸力（正常流程不会停在这里） ---------- */
    case SHOOT_STATE_SLEEP:
        shoot_state = SHOOT_STATE_SPINUP;
        spinup_cnt = 0;
        /* fall through */

    /* ---------- 摩擦轮起转中：锁位，指令全丢弃不排队 ---------- */
    case SHOOT_STATE_SPINUP:
        Shoot_Round_To_Slot();
        Shoot_Motor.ctrl->Angle_Input_Flag = true;
        Shoot_Motor.ctrl->angle_ctrl_outer->target = angle_sum_target;
        Shoot_Motor.ctrl->angle_ctrl_outer->measure =
            (float)Shoot_Motor.rx_info->encoder_sum;
        Shoot_Motor.single_set_angle(&Shoot_Motor);

        if (spinup_cnt < 0xFFFFU) {
            spinup_cnt++;
        }
        if (Shoot_Is_Fric_Ready()) {
            shoot_state = SHOOT_STATE_READY;
        }
        break;

    /* ---------- 就绪：锁位待发，触发前再次确认摩擦轮 ---------- */
    case SHOOT_STATE_READY:
        if (!Shoot_Is_Fric_Ready()) {
            /* 摩擦轮掉速/离线：退回起转，新指令一律不响应 */
            shoot_state = SHOOT_STATE_SPINUP;
            spinup_cnt = 0;
            break;
        }

        if (continuous_armed && car->flags.is_continuous_shoot) {
            /* 连发定频步进：到点推进一格，与单发共用角度环 */
            if (repeat_cnt >= SHOOT_REPEAT_PERIOD_TICKS) {
                repeat_cnt = 0;
                angle_sum_target += SHOOT_ONESHOT_ANGLE;
                fire_cnt = 0;
                stuck_cnt = 0;
                shoot_state = SHOOT_STATE_FIRE;
                break;
            }
        } else if (car->flags.is_single_shoot_cmd) {
            /* 单发脉冲：carctrl内为2ms脉冲，每周期读一次即可 */
            angle_sum_target += SHOOT_ONESHOT_ANGLE;
            jam_retry = 0;
            fire_cnt = 0;
            stuck_cnt = 0;
            shoot_state = SHOOT_STATE_FIRE;
            break;
        } else {
            /* 空闲：回吸最近整格，防松手停在半格 */
            Shoot_Round_To_Slot();
        }

        Shoot_Motor.ctrl->Angle_Input_Flag = true;
        Shoot_Motor.ctrl->angle_ctrl_outer->target = angle_sum_target;
        Shoot_Motor.ctrl->angle_ctrl_outer->measure =
            (float)Shoot_Motor.rx_info->encoder_sum;
        Shoot_Motor.single_set_angle(&Shoot_Motor);
        break;

    /* ---------- 补弹中：角度环跟踪一格 ---------- */
    case SHOOT_STATE_FIRE: {
        float err = angle_sum_target - (float)Shoot_Motor.rx_info->encoder_sum;

        if (fabsf(err) < SHOOT_ARRIVE_ERR) {
            /* 到位：回READY，本轮卡弹计数清零 */
            shoot_state = SHOOT_STATE_READY;
            fire_cnt = 0;
            stuck_cnt = 0;
            break;
        }

        if (Shoot_Is_Stuck(err) || (fire_cnt > SHOOT_FIRE_TIMEOUT_TICKS)) {
            /*
             * 卡弹/超时：跳转瞬间锁存原目标与反转目标。
             * 绝不能在RECOIL分支里按encoder_sum重算反转目标，
             * 否则每2ms再减一格，电机会无限倒车。
             */
            jam_saved_target = angle_sum_target;
            angle_sum_target =
                (float)Shoot_Motor.rx_info->encoder_sum - SHOOT_ONESHOT_ANGLE;
            recoil_cnt = 0;
            shoot_state = SHOOT_STATE_RECOIL;
            break;
        }

        if (fire_cnt < 0xFFFFU) {
            fire_cnt++;
        }

        Shoot_Motor.ctrl->Angle_Input_Flag = true;
        Shoot_Motor.ctrl->angle_ctrl_outer->target = angle_sum_target;
        Shoot_Motor.ctrl->angle_ctrl_outer->measure =
            (float)Shoot_Motor.rx_info->encoder_sum;
        Shoot_Motor.single_set_angle(&Shoot_Motor);
        break;
    }

    /* ---------- 卡弹反转：退一格松弹，成功后补回原目标 ---------- */
    case SHOOT_STATE_RECOIL: {
        float err = angle_sum_target - (float)Shoot_Motor.rx_info->encoder_sum;

        /* 反转目标已在FIRE跳转瞬间锁存，此处只跟踪，绝不重算 */

        if (fabsf(err) < SHOOT_RECOIL_ARRIVE_ERR ||
            recoil_cnt > SHOOT_RECOIL_TIMEOUT_TICKS) {
            if (jam_retry < SHOOT_JAM_RETRY_MAX) {
                /* 补位重试：目标补回卡弹瞬间的原目标，继续这一发 */
                jam_retry++;
                angle_sum_target = jam_saved_target;
                fire_cnt = 0;
                stuck_cnt = 0;
                shoot_state = SHOOT_STATE_FIRE;
            } else {
                /* 重试用尽：卡弹停机，停射停摩擦轮 */
                jam_latched = true;
                continuous_armed = false;
            }
            break;
        }

        if (recoil_cnt < 0xFFFFU) {
            recoil_cnt++;
        }

        Shoot_Motor.ctrl->Angle_Input_Flag = true;
        Shoot_Motor.ctrl->angle_ctrl_outer->target = angle_sum_target;
        Shoot_Motor.ctrl->angle_ctrl_outer->measure =
            (float)Shoot_Motor.rx_info->encoder_sum;
        Shoot_Motor.single_set_angle(&Shoot_Motor);
        break;
    }

    /* ---------- 卡弹停机（由jam_latched分支处理，此处兜底） ---------- */
    case SHOOT_STATE_STOP_JAM:
    default:
        Shoot_Motor_Stop();
        break;
    }

    /* PID算完后发送0x1FF电流帧 */
    Shoot_Motor.single_set_torque(&Shoot_Motor);
}
