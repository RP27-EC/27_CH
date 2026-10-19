/**
  ******************************************************************************
  * @file           : shoot.c
  * @brief          : 拨弹控制模块
  * @details        : 本文件实现了下层板拨弹电机的安全控制逻辑，包括：
  *
  *                   1. 摩擦轮就绪安全锁：未达速/电机离线时拨盘锁位不动，指令丢弃
  *                   2. 单发：角度环跟踪encoder_sum，忙时最多保留一发待执行请求
  *                   3. 连发：定频步进（每300ms推进一格），与单发共用角度环
  *                   4. 松手回吸：停射时目标对齐到最近整格，防半格卡弹
  *                   5. 防卡弹反转：堵转先退一格松弹，再补回原目标重试
  *                   6. 卡弹持续重试：反转到位或超时后补回原目标，不因重试次数关闭摩擦轮
  *
  *                   拨弹状态机：
  *                   SLEEP -> SPINUP -> READY -> FIRE <-> RECOIL
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
static bool     single_pending = false;   /* 忙时最多缓存一发，回中不清除 */
static bool     continuous_last = false;  /* 只用于连发首发节拍，不作为发射授权 */
static InputSrc_e last_input_src = INPUT_SRC_RC;
static uint16_t fire_cnt = 0;              /* 推弹经过周期数，仅用于起步判卡宽限 */
static uint16_t recoil_cnt = 0;            /* 反转超时计数 */
static uint16_t stuck_cnt = 0;             /* 卡弹判据持续计数 */
static uint16_t repeat_cnt = 0;            /* 连发定频周期计数 */

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
    single_pending = false;
    continuous_last = false;
    last_input_src = INPUT_SRC_RC;
    fire_cnt = 0;
    recoil_cnt = 0;
    stuck_cnt = 0;
    repeat_cnt = 0;
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
  *          停射、起转等待时调用。
  */
static void Shoot_Round_To_Slot(void)
{
    const float slot = SHOOT_ONESHOT_ANGLE;
    float pos = (float)Shoot_Motor.rx_info->encoder_sum;

    angle_sum_target = roundf(pos / slot) * slot;//四舍五入取整得到比例乘以步进长度
}


/*摩擦轮就绪判据-----true才允许拨盘动作
------------------只认0x023状态帧：未收到或已丢失一律不就绪，
  不设延时兜底，防止上层板未起转/未接通时盲推弹
  */
static bool Shoot_Is_Fric_Ready(void)
{
    return board_rx_info.is_state_online &&
           board_rx_info.is_fric_speed &&
           board_rx_info.is_fric_work;//摩擦轮速度和工作状态都满足条件
}

/* 起步宽限后：角度没走完 + 电流大 + 转速低，连续满足才判卡。 */
static bool Shoot_Is_Stuck(float err, uint16_t elapsed_ticks)
{
    /* 每次FIRE/补位重试重新计算起步宽限，不按推弹总耗时判卡。 */
    if (elapsed_ticks < SHOOT_STUCK_GRACE_TICKS) {
        stuck_cnt = 0;
        return false;
    }

    float current = fabsf((float)Shoot_Motor.rx_info->torque_current_raw);
    float speed_rpm = fabsf((float)Shoot_Motor.rx_info->encoder_speed);

    if (fabsf(err) > SHOOT_STUCK_ANGLE_ERR && current > SHOOT_STUCK_CURRENT &&
        speed_rpm < SHOOT_STUCK_SPEED_RPM) {
        if (stuck_cnt < 0xFFFFU) {//0xFFFFU是uint16_t的最大值，防止溢出+1重新变为0误判不超时
            stuck_cnt++;
        }
        return stuck_cnt > SHOOT_STUCK_TICKS; /* 沿用原有计数边界，约500ms。 */
    }
 
    //不满足卡弹就走这里
    stuck_cnt = 0;
    return false;
}


//摩擦轮没转好之前，拨盘永远锁在整格位置不动
void Shoot_Update(const Car_t *car, bool failsafe)
{
    /* 心跳：无反馈时offline_cnt累加，超时置DEV_OFFLINE */
    Shoot_Motor.single_heart_beat(&Shoot_Motor);

    bool motor_online = (Shoot_Motor.state->status == DEV_ONLINE);
    bool system_on = car->flags.is_shoot_system_on &&(!failsafe) && motor_online;//遥控器上的发射总开关开了&整车没有失联&拨弹电机在线

    const bool source_changed = car->input_src != last_input_src;
    last_input_src = car->input_src;

    /* ---------- 总开关关 / 失联 / 拨盘离线：卸力并回吸整格 ---------- */
    if (!system_on) {
        Shoot_Motor_Stop();
        Shoot_Round_To_Slot();
        shoot_state = SHOOT_STATE_SLEEP;
        fric_on = false;
        single_pending = false;
        continuous_last = false;
        repeat_cnt = 0;
        stuck_cnt = 0;
        board_tx_info.is_fric_on = 0;
        return;
    }

    /* 开发射：通信通知上层板起摩擦轮 */
    fric_on = true;
    board_tx_info.is_fric_on = 1;

    const bool continuous = car->flags.is_continuous_shoot;
    const bool fric_ready = Shoot_Is_Fric_Ready();

    /* 取消旧单发请求；输入源切换当拍也不接收新的单发脉冲。 */
    if (source_changed || continuous || !fric_ready) {
        single_pending = false;
    } else if (car->flags.is_single_shoot_cmd &&
               (shoot_state == SHOOT_STATE_READY || shoot_state == SHOOT_STATE_FIRE ||
                shoot_state == SHOOT_STATE_RECOIL)) {
        single_pending = true;
    }

    /* 连发只跟随当前电平，边沿仅使首发立即可执行。 */
    if (continuous && !continuous_last) {
        repeat_cnt = SHOOT_REPEAT_PERIOD_TICKS;
    } else if (continuous && repeat_cnt < SHOOT_REPEAT_PERIOD_TICKS) {
        repeat_cnt++;
    }
    continuous_last = continuous;

    /*  SPINUP（等摩擦轮） ──────── 锁在最近的格子上（①）
        READY(摩擦轮已好，空闲) ── 锁在最近的格子上（①）
        FIRE（推弹） ───────── 冲向下一格（②）
        RECOIL（卡弹回退） ─────── 退回身后一格（③）
        */
    switch (shoot_state) {

    /* ---------- 卸力（正常流程不会停在这里） ---------- */
    case SHOOT_STATE_SLEEP:
        shoot_state = SHOOT_STATE_SPINUP;
        /* fall through */

    /* ----------摩擦轮还没转好，拨盘暂时不能打弹 ---------- */
    case SHOOT_STATE_SPINUP:
        Shoot_Round_To_Slot();//就近锁住拨盘

        Shoot_Motor.ctrl->Angle_Input_Flag = true;//接下来喂的是角度指令
        Shoot_Motor.ctrl->angle_ctrl_outer->target = angle_sum_target;
        Shoot_Motor.ctrl->angle_ctrl_outer->measure =(float)Shoot_Motor.rx_info->encoder_sum;

        Shoot_Motor.single_set_angle(&Shoot_Motor);//执行这次角度控制（算 PID、下发电机指令）

        if (fric_ready) {
            shoot_state = SHOOT_STATE_READY;
        }
        break;

    /* ---------- 就绪：锁位待发，触发前再次确认摩擦轮 ---------- */
    case SHOOT_STATE_READY:

     /* ready状态掉线保护--------------摩擦轮掉速或离线：退回起转，break不再继续*/
        if (!fric_ready) {
            shoot_state = SHOOT_STATE_SPINUP;
            break;
        }

        /* 拨杆保持连发位置就持续请求，不需要额外授权锁存。 */
        if (continuous) {
            /* 连发定频步进：到点推进一格，与单发共用角度环 */
            if (repeat_cnt >= SHOOT_REPEAT_PERIOD_TICKS) { //节拍数满300ms
                repeat_cnt = 0;
                angle_sum_target += SHOOT_ONESHOT_ANGLE;  //目标在此累加先
                fire_cnt = 0;    // 新一发重新计算起步判卡宽限
                stuck_cnt = 0;
                shoot_state = SHOOT_STATE_FIRE;  //切到下一个状态---------推弹状态
                break;
            }

        } else if (single_pending) {
            single_pending = false; /* 消费这一发请求，后续忙时可再缓存一发。 */
            angle_sum_target += SHOOT_ONESHOT_ANGLE;
            fire_cnt = 0;
            stuck_cnt = 0;
            shoot_state = SHOOT_STATE_FIRE;
            break;
        } else {
            //空闲：回吸最近整格
            Shoot_Round_To_Slot();
        }

        Shoot_Motor.ctrl->Angle_Input_Flag = true;
        Shoot_Motor.ctrl->angle_ctrl_outer->target = angle_sum_target;
        Shoot_Motor.ctrl->angle_ctrl_outer->measure =(float)Shoot_Motor.rx_info->encoder_sum;
        Shoot_Motor.single_set_angle(&Shoot_Motor);
        break;

    /* ---------- 进弹：角度环跟踪一格 ---------- */
    case SHOOT_STATE_FIRE: {
        float err = angle_sum_target - (float)Shoot_Motor.rx_info->encoder_sum;

          /* 到位：回READY，本轮卡弹计数清零 */
        if (fabsf(err) < SHOOT_ARRIVE_ERR) {
            shoot_state = SHOOT_STATE_READY;
            fire_cnt = 0;
            stuck_cnt = 0;
            break;
        }

        /* 仅持续满足堵转判据时反转；未到位本身不触发反转。 */
        if (Shoot_Is_Stuck(err, fire_cnt)) {

             //跳转瞬间锁存原目标给RECOIL状态使用，因为下面一行会覆盖掉目标 
            jam_saved_target = angle_sum_target;
            angle_sum_target =(float)Shoot_Motor.rx_info->encoder_sum - SHOOT_ONESHOT_ANGLE;//目标退回一个卡槽
            recoil_cnt = 0;
            shoot_state = SHOOT_STATE_RECOIL;
            break;
        }

        if (fire_cnt < 0xFFFFU) {
            fire_cnt++;
        }

        Shoot_Motor.ctrl->Angle_Input_Flag = true;
        Shoot_Motor.ctrl->angle_ctrl_outer->target = angle_sum_target;
        Shoot_Motor.ctrl->angle_ctrl_outer->measure =(float)Shoot_Motor.rx_info->encoder_sum;
        Shoot_Motor.single_set_angle(&Shoot_Motor);
        break;
    }

    /* ---------- 卡弹反转：退一格松弹，成功后补回原目标 ---------- */
    case SHOOT_STATE_RECOIL: {

        /* 继承Fire的回退一格目标，角度环跟踪退回一格，这里直接赋值后进入pid执行的卡弹pid而不是进弹pid
         现在是赋值还未操作，所以不会进入到下面的if反转到位目标更新*/
        float err = angle_sum_target - (float)Shoot_Motor.rx_info->encoder_sum;

        // 反转到位或超时：补回Fire缓存的原目标，持续重试这一发
        if (fabsf(err) < SHOOT_RECOIL_ARRIVE_ERR ||
            recoil_cnt > SHOOT_RECOIL_TIMEOUT_TICKS) {

            angle_sum_target = jam_saved_target;
            fire_cnt = 0;
            stuck_cnt = 0;
            shoot_state = SHOOT_STATE_FIRE;
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

    /* ---------- 非法状态兜底 ---------- */
    default:
        Shoot_Motor_Stop();
        break;
    }

    /* PID算完后发送0x1FF电流帧 */
    Shoot_Motor.single_set_torque(&Shoot_Motor);
}
