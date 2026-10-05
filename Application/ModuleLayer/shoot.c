/**
  ******************************************************************************
  * @file           : shoot.c
  * @brief          : 拨弹控制模块
  * @details        : 本文件实现了下层板拨弹电机的安全控制逻辑，包括：
  *
  *                   1. 摩擦轮就绪安全锁：未达速/电机离线时拨盘锁位不动，指令丢弃
  *                   2. 单发：角度环跟踪encoder_sum，每次加一格弹
  *                   3. 连发：定频步进（每300ms推进一格），与单发共用角度环
  *                   4. 退弹：反向退一格，到位回READY，不补位（S1中+S2中+拨轮向下）
  *                   5. 松手回吸：停射时目标对齐到最近整格，防半格卡弹
  *                   6. 防卡弹反转：堵转先退一格松弹，再补回原目标重试
  *                   7. 卡弹停机：重试用尽则停射停摩擦轮，重拨总开关才解锁
  *
  *                   拨弹状态机：
  *                   SLEEP -> SPINUP -> READY -> FIRE <-> RECOIL -> STOP_JAM
  *                                              READY -> EJECT -> READY（手动退弹）
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
static bool     continuous_armed = false;  /* 连发边沿锁存：松开后才能重新武装 */
static bool     continuous_last = false;   /* 连发上一次电平，用于求上升沿 */
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
    continuous_armed = false;
    continuous_last = false;
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
  *          停射、起转等待、卡弹停机时调用。
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

//卡弹判据：角度没走完 + 电流大 + 持续
static bool Shoot_Is_Stuck(float err)
{
    float current = fabsf((float)Shoot_Motor.rx_info->torque_current_raw);

    if (fabsf(err) > SHOOT_STUCK_ANGLE_ERR && current > SHOOT_STUCK_CURRENT) {
        if (stuck_cnt < 0xFFFFU) {//0xFFFFU是uint16_t的最大值，防止溢出+1重新变为0误判不超时
            stuck_cnt++;
        }
        return stuck_cnt > SHOOT_STUCK_TICKS;//100*2ms 检测到卡弹但是未超时/超时  返回0/1
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

    //car->flags.is_continuous_shoot状态机检测遥控是否要连发
   
    /*检测到触发“连发”遥控，锁存状态位*/
    bool continuous_rise = car->flags.is_continuous_shoot && (!continuous_last);//拨杆刚刚拨过去要连发的那一瞬为1，也就是下面if首弹的用处
    continuous_last = car->flags.is_continuous_shoot;

    if (continuous_rise) {
        continuous_armed = true;//连发状态锁存，想再触发必须先拨回去再拨过来

        repeat_cnt = SHOOT_REPEAT_PERIOD_TICKS;  /* 首发立即出弹 */
    }

    
    /* 重新开机，卡弹状态清零，一般是卡弹这个标志位为1卸力，人为检查拨弹 */
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

    /* ---------- 卡弹锁存到限额了：停射停摩擦轮，等人工排障 ---------- */
    if (jam_latched) {
        Shoot_Motor_Stop();
        Shoot_Round_To_Slot();
        shoot_state = SHOOT_STATE_STOP_JAM;
        fric_on = false;
        continuous_armed = false;
        board_tx_info.is_fric_on = 0;
        return;
    }

    /* 开发射：通信通知上层板起摩擦轮 */
    fric_on = true;
    board_tx_info.is_fric_on = 1;

    /* 连发射速的节拍器 */
    if (continuous_armed && car->flags.is_continuous_shoot) {
        if (repeat_cnt < SHOOT_REPEAT_PERIOD_TICKS) {
            repeat_cnt++;//150×2ms = 300ms 一发，在下面if里判断是否超时然后触发连发发射
        }
    }

    /*  SPINUP（等摩擦轮） ──────── 锁在最近的格子上（①）
        READY(摩擦轮已好，空闲) ── 锁在最近的格子上（①）
        FIRE（推弹） ───────── 冲向下一格（②）
        RECOIL（卡弹回退） ─────── 退回身后一格（③）
        STOP_JAM（卡弹停机） ──── 停机，需要人工排查后关总开关才能解锁
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

        if (Shoot_Is_Fric_Ready()) {
            shoot_state = SHOOT_STATE_READY;
        }
        break;

    /* ---------- 就绪：锁位待发，触发前再次确认摩擦轮 ---------- */
    case SHOOT_STATE_READY:

     /* ready状态掉线保护--------------摩擦轮掉速或离线：退回起转，break不再继续*/
        if (!Shoot_Is_Fric_Ready()) {
            shoot_state = SHOOT_STATE_SPINUP;
            break;
        }

        /* 摇杆位于连发档  &&  连发锁存continuous_armed授权 */
        if (continuous_armed && car->flags.is_continuous_shoot) {
            /* 连发定频步进：到点推进一格，与单发共用角度环 */
            if (repeat_cnt >= SHOOT_REPEAT_PERIOD_TICKS) { //节拍数满300ms
                repeat_cnt = 0;
                angle_sum_target += SHOOT_ONESHOT_ANGLE;  //目标在此累加先
                fire_cnt = 0;    // 推弹超时计数清零（重新计这一发的耗时）
                stuck_cnt = 0;
                shoot_state = SHOOT_STATE_FIRE;  //切到下一个状态---------推弹状态
                break;
            }

          /* 收到单发指令
          ----------------carctrl内为2ms脉冲，每周期会清零并检测
          */
        } else if (car->flags.is_single_shoot_cmd) {
            angle_sum_target += SHOOT_ONESHOT_ANGLE;
            jam_retry = 0;  //卡弹重试次数，只有单发清零，连发不清，因为连发只要卡弹就停机
            fire_cnt = 0;
            stuck_cnt = 0;
            shoot_state = SHOOT_STATE_FIRE;
            break;
        } else if (car->flags.is_eject_cmd) {
            /* 手动退弹：目标退一格，角度环反向跟踪，到位回READY。
               与卡弹RECOIL的区别：不补位、不重试、不停机，就是把已上膛的弹退回来。 */
            angle_sum_target -= SHOOT_ONESHOT_ANGLE;
            fire_cnt = 0;
            stuck_cnt = 0;
            shoot_state = SHOOT_STATE_EJECT;
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

        //！！卡弹/超时，状态位设置跳到RECOIL状态，break不再进行后续的Fire状态的角度环控制！！！
        if (Shoot_Is_Stuck(err) || (fire_cnt > SHOOT_FIRE_TIMEOUT_TICKS)) {

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

        // 反转到位或超时且有余额：补回Fire缓存的原目标，继续进弹
        if (fabsf(err) < SHOOT_RECOIL_ARRIVE_ERR ||
            recoil_cnt > SHOOT_RECOIL_TIMEOUT_TICKS) {

                 /* 还没到限制次数，补位重试：目标补回卡弹瞬间的原目标，继续这一发 */
            if (jam_retry < SHOOT_JAM_RETRY_MAX) {
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

    /* ---------- 手动退弹：反向退一格，到位回READY ---------- */
    case SHOOT_STATE_EJECT: {
        float err = angle_sum_target - (float)Shoot_Motor.rx_info->encoder_sum;

        /* 到位：回READY，本轮退弹完成 */
        if (fabsf(err) < SHOOT_ARRIVE_ERR) {
            shoot_state = SHOOT_STATE_READY;
            fire_cnt = 0;
            stuck_cnt = 0;
            break;
        }

        /* 超时保护：退不动就回READY，别卡死在退弹状态 */
        if (fire_cnt > SHOOT_FIRE_TIMEOUT_TICKS) {
            shoot_state = SHOOT_STATE_READY;
            break;
        }

        if (fire_cnt < 0xFFFFU) {
            fire_cnt++;
        }

        Shoot_Motor.ctrl->Angle_Input_Flag = true;
        Shoot_Motor.ctrl->angle_ctrl_outer->target = angle_sum_target;
        Shoot_Motor.ctrl->angle_ctrl_outer->measure = (float)Shoot_Motor.rx_info->encoder_sum;
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
