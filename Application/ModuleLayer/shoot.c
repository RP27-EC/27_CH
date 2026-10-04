/**
  ******************************************************************************
  * @file           : shoot.c
  * @brief          : 上层板摩擦轮控制实现（六轮 RM3508 速度环闭环）
  * @details        : 每 2ms 执行一次 Shoot_Work()：
  *                   1. 读六轮转速与在线状态
  *                   2. 按下层板 0xD1 的 is_fric_on 写速度环目标
  *                   3. 判已达速 / 全部在线，写进 0x023 状态帧
  *                   4. 速度环闭环，两组各打包一帧电流发出
  *
  *                   安全规则：
  *                   1. 下层板失联 → 整组卸力并清 PID，防止 0xD1 卡在
  *                      is_fric_on=1 让摩擦轮一直转
  *                   2. 单颗离线只卸这一颗，其余继续闭环，
  *                      is_fric_work 会报 0 给下层板拦住拨盘
  *                   3. 达速标志必须与「正在驱动」相与：关轮刹车或
  *                      卸力溜车途中轮速穿过阈值，不得误报就绪，
  *                      否则下层板会把弹打进降速中的摩擦轮
  ******************************************************************************
  */
#include "shoot.h"
#include "board_comm.h"

/* 六颗摩擦轮数量 */
#define FRIC_MOTOR_CNT  (6U)

/* 发射机构全局管理实体 */
Shoot_t shoot = {
    .init = Shoot_Init,
    .work = Shoot_Work,
};

/*
 * 六颗摩擦轮遍历表，顺序：前层上/右/左，后层上/右/左。
 * 与发送组的槽位无关，仅用于统一做状态检测和闭环。
 */
static Motor_RM_t * const fric_motor[FRIC_MOTOR_CNT] = {
    &F_U_Fric, &F_R_Fric, &F_L_Fric,
    &B_U_Fric, &B_R_Fric, &B_L_Fric,
};

/* 同序的转向符号，见 shoot.h 的 FRIC_SIGN_* */
static const float fric_sign[FRIC_MOTOR_CNT] = {
    FRIC_SIGN_U, FRIC_SIGN_R, FRIC_SIGN_L,
    FRIC_SIGN_U, FRIC_SIGN_R, FRIC_SIGN_L,
};


/* 内部私有函数声明 */
static void Fric_Target_Update(void);
static void Fric_Status_Detect(void);
static void Fric_Speed_Pid_Cal(void);

void Shoot_Safe_Stop(void)
{
    uint8_t i;

    for (i = 0U; i < FRIC_MOTOR_CNT; i++) {
        fric_motor[i]->tx_info->torque = 0.0f;
        pid_clear(fric_motor[i]->ctrl->speed_ctrl);
    }

    /* 两组各打包一帧零电流发出 */
    if (Fric_F_Group.group_set_torque != NULL) {
        Fric_F_Group.group_set_torque(&Fric_F_Group);
    }
    if (Fric_B_Group.group_set_torque != NULL) {
        Fric_B_Group.group_set_torque(&Fric_B_Group);
    }
}

void Shoot_Init(void)
{
    uint8_t i;

    /* 初始化六颗摩擦轮单电机与两个发送组 */
    rm_motor_list_init();

    shoot.fric.is_on = false;
    shoot.fric.target_f_rpm = FRIC_TARGET_F_RPM;
    shoot.fric.target_b_rpm = FRIC_TARGET_B_RPM;
    shoot.fric.is_fric_speed = false;
    shoot.fric.is_fric_work = false;

    for (i = 0U; i < FRIC_MOTOR_CNT; i++) {
        fric_motor[i]->ctrl->speed_ctrl->target = 0.0f;
        pid_clear(fric_motor[i]->ctrl->speed_ctrl);
        fric_motor[i]->tx_info->torque = 0.0f;
    }
}

//写六轮速度环目标
static void Fric_Target_Update(void)
{
    float f_rpm;
    float b_rpm;

    shoot.fric.is_on = (board_rx_info.is_fric_on != 0U);//接收到摩擦轮开启信号

    f_rpm = shoot.fric.is_on ? shoot.fric.target_f_rpm : 0.0f;//关摩擦轮时目标清零，由速度环把轮子刹下来
    b_rpm = shoot.fric.is_on ? shoot.fric.target_b_rpm : 0.0f;

    F_U_Fric.ctrl->speed_ctrl->target = FRIC_SIGN_U * f_rpm;
    F_R_Fric.ctrl->speed_ctrl->target = FRIC_SIGN_R * f_rpm;
    F_L_Fric.ctrl->speed_ctrl->target = FRIC_SIGN_L * f_rpm;
    B_U_Fric.ctrl->speed_ctrl->target = FRIC_SIGN_U * b_rpm;
    B_R_Fric.ctrl->speed_ctrl->target = FRIC_SIGN_R * b_rpm;
    B_L_Fric.ctrl->speed_ctrl->target = FRIC_SIGN_L * b_rpm;
}


/*判摩擦轮已达速与是否全部在线，写进 0x023 状态帧*/
static void Fric_Status_Detect(void)
{
    uint8_t i;
    bool speed_ok = true;
    bool work_ok = true;
    bool driving = shoot.fric.is_on && board_rx_info.is_online;

    for (i = 0U; i < FRIC_MOTOR_CNT; i++) {
        if (!driving ||
            fric_sign[i] * (float)fric_motor[i]->rx_info->encoder_speed < FRIC_READY_RPM) {
            speed_ok = false;//处理带符号的转速达标判断
        }
        if (fric_motor[i]->state->status != DEV_ONLINE) {
            work_ok = false;
        }
    }

    shoot.fric.is_fric_speed = speed_ok;
    shoot.fric.is_fric_work = work_ok;

    /* 组装 0x023 的 flags 字节，由 Board_Comm_Send_State 发出 */
    board_tx_info.fric_state = 0U;
    if (shoot.fric.is_fric_speed) {
        board_tx_info.fric_state |= FRIC_STATE_SPEED;
    }
    if (shoot.fric.is_fric_work) {
        board_tx_info.fric_state |= FRIC_STATE_WORK;
    }
}

/*六轮速度环闭环并下发
is_fric_work 报 0 后由下层板拦住拨盘，不必在这里停全部*/
static void Fric_Speed_Pid_Cal(void)
{
    uint8_t i;

    /* ====== 板间通信断联安全保护 ====== */
    /* 下层板 0xD1 超时后 is_fric_on 已失效，必须立刻卸力 */
    if (!board_rx_info.is_online ||
        Fric_F_Group.group_set_torque == NULL ||
        Fric_B_Group.group_set_torque == NULL) {
        Shoot_Safe_Stop();
        return;
    }

    for (i = 0U; i < FRIC_MOTOR_CNT; i++) {
        if (fric_motor[i]->state->status == DEV_ONLINE) {
            fric_motor[i]->single_set_speed(fric_motor[i]);
        } else {
            fric_motor[i]->tx_info->torque = 0.0f;//单颗离线只卸这一颗，其余继续闭环；
        }
    }

    /* 两组各打包一帧电流发出 */
    Fric_F_Group.group_set_torque(&Fric_F_Group);
    Fric_B_Group.group_set_torque(&Fric_B_Group);
}

/*---------运用上面函数，摩擦轮控制顶层调度函数（对应 shoot.work()）*/
void Shoot_Work(void)
{
    Fric_Target_Update();     /* 1. 读下层板 is_fric_on，写六轮目标 */
    Fric_Status_Detect();     /* 2. 判已达速 / 全部在线，写 0x023 flags */
    Fric_Speed_Pid_Cal();     /* 3. 速度环闭环并下发 */
}
