/**
  ******************************************************************************
  * @file           : gimbal.c
  * @brief          : 云台全模式实现（含 Yaw 轴陀螺仪双环 PID 闭环）
  * @details        : 本文件实现了RoboMaster机器人云台的完整控制逻辑，包括：
  *
  *                   1. KT9025编码器解算：将16位原始编码器值转换为相对车头的标准弧度
  *                   2. 云台初始化与自动寻零：上电后自动对齐机械零位
  *                   3. 机械模式（MECH）：Yaw就近锁轴——|Δθ|<88°锁车头0°，>91°锁车尾180°
  *                   4. 陀螺仪自稳模式（GYRO）：基于大地坐标系的空间自稳跟随
  *                   5. 小陀螺模式（SPIN）：底盘自旋时云台保持空间朝向不变；
  *                      对齐方向一票否决，禁止自旋扫圈触发 FRONT/BACK 翻转
  *                   6. 换头/掉头（仅 MECH）：粘性锁轴 FRONT↔BACK 翻转；
  *                      到角5°或超时1.5s结束；结束后立刻按真实轴向对齐，
  *                      底盘倒车不空转半圈。GYRO/SPIN 世界系掉头已删除

  
  *                   云台模式状态机：
  *                   SLEEP -> INIT(寻零) -> MECH/GYRO(正常工作)
  *                   任何异常情况回退到SLEEP模式
  ******************************************************************************
  */

#include "gimbal.h"
#include "rc_sensor.h"
#include "kt_yaw_motor.h"
#include "communicate.h"
#include "imu_sensor.h"
#include <math.h>

/** @brief 圆周率常量，用于角度归一化计算 */
#define PI 3.14159265358979323846f


Gimbal_t hero_gimbal;


/*将弧度值规范化到 [-PI, PI] 范围内*/
static float Angle_Normalize_PI(float rad) {
    while (rad > PI)  rad -= 2.0f * PI;  
    while (rad < -PI) rad += 2.0f * PI; 
    return rad;
}

/*调用上面函数----------------计算两个角度之间的最短路径误差*/
static float Calc_Shortest_Angle_Err(float target, float feedback) {
    return Angle_Normalize_PI(target - feedback);
}

//模式切换时选 PID
static pid_ctrl_t *YawAnglePid(Gimbal_t *gimbal) {
    return gimbal->yaw.pid_mode == GIMBAL_PID_GYRO ?
           &gimbal->yaw.gyro_angle_pid : &gimbal->yaw.mec_angle_pid;
}
static pid_ctrl_t *YawSpeedPid(Gimbal_t *gimbal) {
    return gimbal->yaw.pid_mode == GIMBAL_PID_GYRO ?
           &gimbal->yaw.gyro_speed_pid : &gimbal->yaw.mec_speed_pid;
}
static void ClearYawPids(Gimbal_t *gimbal) {
    pid_clear(&gimbal->yaw.mec_angle_pid);
    pid_clear(&gimbal->yaw.mec_speed_pid);
    pid_clear(&gimbal->yaw.gyro_angle_pid);
    pid_clear(&gimbal->yaw.gyro_speed_pid);
    gimbal->yaw.speed_d_last_feedback = 0.0f;
    gimbal->yaw.speed_d_filtered_delta = 0.0f;
    gimbal->yaw.speed_d_ready = false;
    gimbal->yaw.speed_d_pid_mode = gimbal->yaw.pid_mode;
    gimbal->yaw.chassis_speed_rads = 0.0f;
    gimbal->yaw.chassis_fow_speed = 0.0f;
    gimbal->yaw.yaw_target_speed = 0.0f;
    gimbal->yaw.chassis_fow_valid = false;
}


void Gimbal_Init(Gimbal_t *gimbal) {
    gimbal->yaw.chassis_fow_rads = 0.85f;//0.65往上调
    gimbal->yaw.key_yaw_ff_gain = 5.0f;
    gimbal->yaw.key_yaw_speed_rads = 0.0f;
    gimbal->yaw.key_yaw_ff_channel = 0;
    gimbal->yaw.key_yaw_ff_last_seq = 0U;
    gimbal->yaw.key_yaw_ff_active = false;
    gimbal->yaw.key_yaw_ff_valid = false;
    /* 初始化为休眠模式，等待整车状态机切换到工作模式 */
    gimbal->mode = GIMBAL_MODE_SLEEP;
    gimbal->last_mode = GIMBAL_MODE_SLEEP;
    gimbal->reset_state = GIMBAL_RESET_PENDING;  /* 复位状态：等待复位 */
    gimbal->init_timer = 0;

    /* Pitch轴软限位设置（单位：弧度） */
    gimbal->pitch.min_limit = PITCH_MIN_MEC_ANGLE;
    gimbal->pitch.max_limit = PITCH_MAX_MEC_ANGLE;
    gimbal->pitch.target_mec_angle = 0.0f;
    gimbal->pitch.target_imu_angle = 0.0f;   /* GYRO 域 pitch 目标，切入陀螺仪时与 IMU 对齐 */
    gimbal->pitch.imu_angle = 0.0f;
    gimbal->pitch.imu_speed = 0.0f;
    gimbal->pitch.target_rebase_pending = false;


  /* 机械模式参数，+不加测速头 */
    gimbal->yaw.mec_angle_pid.kp = 12.0f;
    gimbal->yaw.mec_angle_pid.ki = 0.0f;
    gimbal->yaw.mec_angle_pid.kd = 0.0f;
    gimbal->yaw.mec_angle_pid.out_max = 12.0f;
    gimbal->yaw.mec_angle_pid.integral_max = 0.0f;
    gimbal->yaw.mec_angle_pid.a = 1.0f;
    gimbal->yaw.mec_speed_pid.kp = 900.0f;
    gimbal->yaw.mec_speed_pid.ki = 0.0f;
    gimbal->yaw.mec_speed_pid.kd = 20.0f;//here
    gimbal->yaw.mec_speed_pid.out_max = 2000.0f;//限幅已经修改成正常落地，调试用200
    gimbal->yaw.mec_speed_pid.integral_max = 400.0f;
    gimbal->yaw.mec_speed_pid.a = 0.2f;


    /* GYRO模式参数：当前未加测速头 */
    gimbal->yaw.gyro_angle_pid.kp = 12.0f; //12
    gimbal->yaw.gyro_angle_pid.ki = 0.0f;
    gimbal->yaw.gyro_angle_pid.kd = 0.0f;
    /* 陀螺仪角度环输出目标角速度限制,attention防止反馈异常时高速冲转。 */
    gimbal->yaw.gyro_angle_pid.out_max = 10.0f;    //最上面为12
    gimbal->yaw.gyro_angle_pid.integral_max = 0.0f;
    gimbal->yaw.gyro_angle_pid.a = 1.0f;
    gimbal->yaw.gyro_speed_pid.kp = 900.0f; //900
    gimbal->yaw.gyro_speed_pid.ki = 0.0f;
    gimbal->yaw.gyro_speed_pid.kd = 0.0f;
    /* 陀螺仪速度环输出是最终电流指令，先限制为安全调试值，避免
     * IMU反馈异常或方向不一致时直接满电流疯转。 */
    gimbal->yaw.gyro_speed_pid.out_max = 1800.0f;  //最上面为2000
    gimbal->yaw.gyro_speed_pid.integral_max = 400.0f;
    gimbal->yaw.gyro_speed_pid.a = 0.2f;

    /* 数据清零 */
    gimbal->yaw.target_mec_angle = 0.0f;      /* 机械目标角 */
    gimbal->yaw.feedback_mec_angle = 0.0f;    /* 机械反馈角 */
    gimbal->yaw.feedback_mec_speed = 0.0f;    /* 机械反馈速度 */

    gimbal->yaw.target_imu_angle = 0.0f;      /* IMU目标角 */
    gimbal->yaw.imu_angle = 0.0f;             /* IMU反馈角（来自上层C板） */
    gimbal->yaw.imu_speed = 0.0f;             /* IMU反馈速度（来自上层C板） */
    gimbal->yaw.gyro_sensitivity = 0.000015f; /* 灵敏度对半砍了太慢了现在改到这个参数*/

    gimbal->yaw.align_dir = ALIGN_FRONT;      /* 就近对齐默认锁车头轴 */
    gimbal->yaw.align_veto_cnt = 0U;          /* 无否决 */

    gimbal->yaw.turn_active = false;          /* 无换头 */
    gimbal->yaw.turn_finish = true;           /* 上电视为已就绪（无未完成换头） */
    gimbal->yaw.turn_time = 0U;

    gimbal->yaw.give_current = 0;             /* 输出电流清零 */
    gimbal->yaw.pid_mode = GIMBAL_PID_MEC;    /* 默认PID模式为机械模式 */

    /* 清零PID控制器 */
    ClearYawPids(gimbal);
}



/*云台上电自动寻零对齐车头*/
static void Gimbal_Init_Step(Gimbal_t *gimbal) {
    gimbal->init_timer++;

    /* 机械零位对齐目标为 0.0 rad（正前方） */
    gimbal->yaw.target_mec_angle = 0.0f;
    gimbal->pitch.target_mec_angle = 0.0f;

    /* 复位完成判定条件：
     * 条件1：角度误差 < 0.05弧度（约3度） 且 角速度 < 0.2rad/s
     * 条件2：超时1.5秒（750个2ms周期）*/
    float angle_err = fabsf(gimbal->yaw.feedback_mec_angle);
    if ((angle_err < 0.05f && fabsf(gimbal->yaw.feedback_mec_speed) < 0.2f) ||
        (gimbal->init_timer >= 750)) {
        gimbal->reset_state = GIMBAL_RESET_DONE;  /* 标记复位完成 */
        gimbal->init_timer = 0;
        /* 关键：复位完成瞬间同步空间目标角 = 当前IMU角，
         * 防止从INIT切到GYRO模式时因目标角突变导致云台猛甩 */
        gimbal->yaw.target_imu_angle = gimbal->yaw.imu_angle;
    }
}

/* ========================================================================= */
/*                          2. 目标解算模块                                  */
/* ========================================================================= */


static void Gimbal_Mech_Update(Gimbal_t *gimbal) {
    gimbal->yaw.pid_mode = GIMBAL_PID_MEC;   /* 切换到机械模式PID */
    /* 就近锁轴：FRONT=0°（车头），BACK=180°（车尾）。
       与 88°/91° 迟滞配合后，云台相对当前轴的行程永远 ≤90°，
       不会从车头大幅甩到车尾扯线丢视野。 */
    gimbal->yaw.target_mec_angle =
        (gimbal->yaw.align_dir == ALIGN_BACK) ? PI : 0.0f;
}


/* ========================================================================= */
/*                          2.1 键鼠瞄准（鼠标 + Ctrl 微调）                  */
/* ========================================================================= */

/**
  * @brief  当前 Ctrl 微调步距
  * @details Ctrl+Shift 同按走大步距（0.2°），只按 Ctrl 走小步距（0.05°）。
  *          Shift 的超电放电电平语义不受影响，这里只是把同一个键当和弦用。
  */
static float Gimbal_Trim_Step(void) {
    return (rc_sensor.info->Shift.value != 0U) ? KEY_TRIM_STEP_LARGE : KEY_TRIM_STEP_SMALL;
}

/* 只清前馈观察量，不改目标角与 Watch 中设置的独立倍率。 */
static void Gimbal_Key_Yaw_Feedforward_Clear(Gimbal_t *gimbal) {
    gimbal->yaw.key_yaw_speed_rads = 0.0f;
    gimbal->yaw.key_yaw_ff_channel = 0;
    gimbal->yaw.key_yaw_ff_valid = false;
}

static void Gimbal_Key_Yaw_Feedforward_Enable(Gimbal_t *gimbal, CarMode_e car_mode) {
    const bool active = car_mode == CAR_MODE_GYRO &&
        gimbal->mode == GIMBAL_MODE_GYRO && hero_car.input_src == INPUT_SRC_KEY &&
        rc_sensor.work_state == DEV_ONLINE && rc_sensor.errno == NONE_ERR &&
        board_rx_info.is_online && rc_sensor.info->Ctrl.value == 0U;
    if (!active || !gimbal->yaw.key_yaw_ff_active) {
        uint32_t irq_mask = __get_PRIMASK();
        __disable_irq();
        gimbal->yaw.key_yaw_ff_last_seq = rc_sensor.info->frame_seq;
        __set_PRIMASK(irq_mask);
        Gimbal_Key_Yaw_Feedforward_Clear(gimbal);
    }
    gimbal->yaw.key_yaw_ff_active = active;
}

/**
  * @brief  键鼠模式下的 Yaw 目标角更新
  * @details 两件事：
  *          1. 鼠标横向位移 -> 空间目标角增量。鼠标是「一帧一个采样」，
  *             而本函数跑 2ms，必须用 frame_seq 做「一帧只累加一次」的闸门，
  *             否则同一个采样会被算 ~7 遍（DBUS 帧周期约 14ms）。
  *          2. Ctrl 按住时 WASD 边沿改为步进微调，不再给底盘平移
  *             （底盘那侧由 Car_Chassis_Channel_Get 把平移置零，叫「替换」语义）。
  *
  *          注意：本函数只在 GYRO 模式下被调用。MECH 模式 Yaw 锁死在底盘正前方
  *          （Gimbal_Mech_Update），键鼠瞄准在 MECH 下不生效，这是跟随安全行为，不是漏做。
  *
  * @param  gimbal : 云台对象指针
  * @retval None
  */
static void Gimbal_Key_Yaw_Update(Gimbal_t *gimbal) {
    static uint16_t last_seq = 0;   /* 上次消费过的 DBUS 帧序号 */
    uint16_t seq;
    int16_t mouse_raw;
    float mouse_x;
    uint32_t frame_ms, interval_ms, now_ms;
    bool received;
    uint8_t ctrl;
    uint32_t irq_mask = __get_PRIMASK();

    /* 中断更新数据与时间：短临界区复制同一帧，恢复中断后再换算。 */
    __disable_irq();
    seq = rc_sensor.info->frame_seq;
    mouse_raw = rc_sensor.info->mouse_vx;
    mouse_x = rc_sensor.info->mouse_x;
    frame_ms = rc_sensor.info->frame_update_ms;
    interval_ms = rc_sensor.info->frame_interval_ms;
    received = rc_sensor.info->frame_received;
    ctrl = rc_sensor.info->Ctrl.value;
    now_ms = HAL_GetTick();
    __set_PRIMASK(irq_mask);

    /* 鼠标横向：鼠标右移 = 瞄准向右 = 目标角减小 */
    if (seq != last_seq) {
        last_seq = seq;
        /* 均值来源为 int16_t 鼠标值；异常浮点值不能污染目标或让角度归一化卡死。 */
        if (isfinite(mouse_x) && fabsf(mouse_x) <= 32768.0f) {
            gimbal->yaw.target_imu_angle -= mouse_x * MOUSE_YAW_SENS;
        }
    }

    if (!gimbal->yaw.key_yaw_ff_active || !received || ctrl != 0U ||
        now_ms - frame_ms >= MOUSE_YAW_FF_TIMEOUT_MS || mouse_raw == 0 ||
        interval_ms == 0U || interval_ms > MOUSE_YAW_FF_TIMEOUT_MS ||
        !isfinite(mouse_x) || fabsf(mouse_x) > 32768.0f ||
        !isfinite(gimbal->yaw.gyro_sensitivity) ||
        gimbal->yaw.gyro_sensitivity <= 0.0f ||
        !isfinite(gimbal->yaw.key_yaw_ff_gain) || gimbal->yaw.key_yaw_ff_gain <= 0.0f) {
        Gimbal_Key_Yaw_Feedforward_Clear(gimbal);
        gimbal->yaw.key_yaw_ff_last_seq = seq;
    } else if (seq != gimbal->yaw.key_yaw_ff_last_seq) {
        float speed = -mouse_x * MOUSE_YAW_SENS * 1000.0f / (float)interval_ms;
        /* 遥控每 2 ms 增量 = -ch0 * gyro_sensitivity，因此反解等效 ch0。 */
        float channel = -speed * GIMBAL_CONTROL_PERIOD_S /
                        gimbal->yaw.gyro_sensitivity * gimbal->yaw.key_yaw_ff_gain;
        gimbal->yaw.key_yaw_ff_last_seq = seq;
        if (isfinite(speed) && isfinite(channel)) {
            gimbal->yaw.key_yaw_speed_rads = speed;
            gimbal->yaw.key_yaw_ff_channel = (int16_t)constrain(channel, -660.0f, 660.0f);
            gimbal->yaw.key_yaw_ff_valid = true;
        } else {
            Gimbal_Key_Yaw_Feedforward_Clear(gimbal);
        }
    }

    /* Ctrl + A/D：A = 向左瞄 = 目标角增大（与鼠标右移相反） */
    if (ctrl != 0U) {
        const float step = Gimbal_Trim_Step();
        if (rc_sensor.info->A.status == release_to_press) {
            gimbal->yaw.target_imu_angle += step;
        }
        if (rc_sensor.info->D.status == release_to_press) {
            gimbal->yaw.target_imu_angle -= step;
        }
    }
}

/**
  * @brief  键鼠模式下的 Pitch 目标角更新
  * @details 鼠标纵向位移累加（同样吃 frame_seq 闸门）+ Ctrl+W/S 步进微调。
  *          软限位钳位由调用方 Gimbal_Pitch_Update 统一做，这里只管累加。
  *          target 由调用方传入：MECH 写机械域，GYRO 写 IMU 域。
  *
  * @param  target : 目标角累加指针 (rad)
  * @retval None
  */
static void Gimbal_Key_Pitch_Update(float *target) {
    static uint16_t last_seq = 0;   /* 上次消费过的 DBUS 帧序号 */

    /* 鼠标纵向：鼠标上移 = 抬头 = 目标角增大 */
    if (rc_sensor.info->frame_seq != last_seq) {
        last_seq = rc_sensor.info->frame_seq;
        *target += rc_sensor.info->mouse_y * MOUSE_PITCH_SENS;
    }

    /* Ctrl + W/S：W = 抬头 = 目标角增大 */
    if (rc_sensor.info->Ctrl.value != 0U) {
        const float step = Gimbal_Trim_Step();
        if (rc_sensor.info->W.status == release_to_press) {
            *target += step;
        }
        if (rc_sensor.info->S.status == release_to_press) {
            *target -= step;
        }
    }
}


/**
  * @brief  换头结束后按真实轴向立刻对齐
  * @details 换头完成时头多半已对车尾（|delta|≈180°）。
  *          若继续吃 FRONT + 冷却，底盘会按 follow_err≈180° 掉头半圈，
  *          世界系转过 180° 会被底盘「转回去」，操作手看到的是没换头。
  *          这里直接按当前 delta 轴向写 align_dir，并清零否决计数，
  *          底盘当拍起 follow_err≈0，W = 倒车暴退（有滑环，不担心绕线）。
  */
static void Gimbal_Turn_Apply_Align_After(Gimbal_t *gimbal) {
    const float abs_deg =
        fabsf(gimbal->yaw.feedback_mec_angle) * (180.0f / PI);
    if (abs_deg > YAW_ALIGN_BACK_MIN_DEG) {
        gimbal->yaw.align_dir = ALIGN_BACK;
    } else if (abs_deg < YAW_ALIGN_FRONT_MAX_DEG) {
        gimbal->yaw.align_dir = ALIGN_FRONT;
    }
    /* 88°~91°：保持刚写入的方向，不在此处引入迟滞横跳 */
    gimbal->yaw.align_veto_cnt = 0U;
}

/**
  * @brief  换头/掉头状态机（仅 MECH）
  * @details 消费 Car_t.flags.is_turn_cmd 脉冲：
  *          MECH：粘性锁轴 FRONT↔BACK 翻转（0° ↔ 180°），
  *            不能靠 88°/91° 迟滞——头锁死在 0° 时 |delta| 永远很小，
  *            迟滞翻不到 BACK，机械模式必须手动锁车尾。
  *          GYRO / SPIN 的世界系 +180° 掉头已删除：非 MECH 一票否决，
  *          未完成换头直接中止，is_turn_cmd 脉冲直接丢弃。
  *          必须在双向对齐迟滞之前调用：turn_active 期间冻结 align_dir。
  *          到角或超时后立刻按真实轴向对齐，避免底盘空转半圈。
  */
static void Gimbal_Turn_Update(Gimbal_t *gimbal, CarMode_e car_mode) {
    if (car_mode != CAR_MODE_MECH) {
        /* GYRO/SPIN/SLEEP/INIT：换头不存在。换头进行中被打断时按真实轴向
           落定 align，底盘不留假换头误差；空闲时不动 align（每拍清 veto
           会毁掉 SPIN 一票否决）。 */
        if (gimbal->yaw.turn_active) {
            gimbal->yaw.turn_active = false;
            gimbal->yaw.turn_time = 0U;
            Gimbal_Turn_Apply_Align_After(gimbal);
        }
        return;
    }

    if (gimbal->yaw.turn_active) {
        gimbal->yaw.turn_time++;
        /* 到位误差：编码器相对锁轴目标 */
        const float err = fabsf(Calc_Shortest_Angle_Err(
            gimbal->yaw.target_mec_angle, gimbal->yaw.feedback_mec_angle));
        const float arrive_rad = YAW_TURN_ARRIVE_ERR_DEG * DEG_TO_RAD;
        if (err <= arrive_rad ||
            gimbal->yaw.turn_time >= YAW_TURN_TIMEOUT_TICKS) {
            gimbal->yaw.turn_active = false;
            gimbal->yaw.turn_finish = true;   /* 到角或超时都算本轮结束 */
            gimbal->yaw.turn_time = 0U;
            Gimbal_Turn_Apply_Align_After(gimbal);
        }
        return;
    }

    /* 空闲：边沿脉冲启动换头（进行中不重复触发） */
    if (hero_car.flags.is_turn_cmd) {
        /* 粘性锁轴翻转：车头 ↔ 车尾。目标由 Gimbal_Mech_Update 按 align_dir 写 */
        gimbal->yaw.align_dir =
            (gimbal->yaw.align_dir == ALIGN_BACK) ? ALIGN_FRONT : ALIGN_BACK;
        gimbal->yaw.turn_active = true;
        gimbal->yaw.turn_finish = false;
        gimbal->yaw.turn_time = 0U;
    }
}


static void Gimbal_Gyro_Update(Gimbal_t *gimbal) {
    gimbal->yaw.pid_mode = GIMBAL_PID_GYRO;  /* 切换到陀螺仪模式PID */

    if (hero_car.input_src == INPUT_SRC_KEY) {
        /* 键鼠：鼠标瞄准 + Ctrl 微调 */
        Gimbal_Key_Yaw_Update(gimbal);
    } else {
        //yaw摇杆映射
        float rc_yaw_input = -(float)rc_sensor.info->ch0;

        //死区
        if (fabsf(rc_yaw_input) < RC_YAW_GYRO_DEADZONE) {
            rc_yaw_input = 0.0f;
        }

        //rad
        gimbal->yaw.target_imu_angle += rc_yaw_input * gimbal->yaw.gyro_sensitivity;
    }

    /* 角度规范化，防止目标角溢出 */
    gimbal->yaw.target_imu_angle = Angle_Normalize_PI(gimbal->yaw.target_imu_angle);
}



static void Gimbal_Pitch_Update(Gimbal_t *gimbal)
{
    /* 休眠模式或寻零阶段：Pitch 目标强制水平对齐 (0.0 rad) */
    if (gimbal->mode == GIMBAL_MODE_SLEEP || gimbal->mode == GIMBAL_MODE_INIT) {
        gimbal->pitch.target_mec_angle = 0.0f;
        gimbal->pitch.target_imu_angle = gimbal->pitch.imu_angle;
        return;
    }

    /* GYRO（含 SPIN，云台模式映射到 GIMBAL_MODE_GYRO）：目标在 IMU 域累加。
       MECH：目标在编码器机械域累加。灵敏度与死区两边共用。 */
    const bool imu_domain = (gimbal->mode == GIMBAL_MODE_GYRO);
    float *target = imu_domain ? &gimbal->pitch.target_imu_angle
                               : &gimbal->pitch.target_mec_angle;

    if (imu_domain &&
        (!board_rx_info.is_pitch_online || !board_rx_info.is_pitch_mech_online)) {
        gimbal->pitch.target_rebase_pending = true;
        gimbal->pitch.target_imu_angle = gimbal->pitch.imu_angle;
        return;
    }

    if (gimbal->pitch.target_rebase_pending && board_rx_info.is_pitch_mech_online) {
        gimbal->pitch.target_mec_angle = board_rx_info.pitch_mech_angle;
        gimbal->pitch.target_imu_angle = gimbal->pitch.imu_angle;
        gimbal->pitch.target_rebase_pending = false;
    }

    /* GYRO->MECH 时必须使用当前机械角，不能把 IMU 世界角当机械角。 */
    if (!imu_domain && gimbal->last_mode == GIMBAL_MODE_GYRO) {
        if (board_rx_info.is_pitch_mech_online) {
            gimbal->pitch.target_mec_angle = board_rx_info.pitch_mech_angle;
        } else {
            gimbal->pitch.target_rebase_pending = true;
            return;
        }
    }

    if (gimbal->pitch.target_rebase_pending) return;

    if (hero_car.input_src == INPUT_SRC_KEY) {
        /* 键鼠：鼠标瞄准 + Ctrl 微调（写入当前域的目标） */
        Gimbal_Key_Pitch_Update(target);
    } else {
        /* 遥控器与编码器坐标统一：摇杆向上产生正目标，向下产生负目标。 */
        float rc_pitch_input = (float)rc_sensor.info->ch1;

        /*摇杆中位死区 */
        /* 右摇杆上下(ch1)在死区内不改变目标，避免中位漂移持续累加。 */
        if (fabsf(rc_pitch_input) <= RC_PITCH_MECH_DEADZONE) {
            rc_pitch_input = 0.0f;
        }

        /* 积分累加，灵敏度为 RC_PITCH_MECH_SENSITIVITY */
        *target += rc_pitch_input * RC_PITCH_MECH_SENSITIVITY;
    }

    /* 限幅：GYRO 用标定后的 IMU 世界域静态限位，MECH 用机械行程常量。
       两边都是常数边界，打满时每拍截到同一个数，目标必然是水平直线。
       常数与上板 gimbal.h 同名同值（PITCH_MIN/MAX_IMU_ANGLE）。 */
    const float min_limit = imu_domain ? PITCH_MIN_IMU_ANGLE : gimbal->pitch.min_limit;
    const float max_limit = imu_domain ? PITCH_MAX_IMU_ANGLE : gimbal->pitch.max_limit;
    if (*target > max_limit) {
        *target = max_limit;
    } else if (*target < min_limit) {
        *target = min_limit;
    }
}

/* ========================================================================= 
                    3. Yaw 轴陀螺仪 / 机械 双环 PID 闭环计算              
========================================================================= */

static void Gimbal_Yaw_Pid_Cal(Gimbal_t *gimbal) {
    if (gimbal->yaw.speed_d_pid_mode != gimbal->yaw.pid_mode) {
        ClearYawPids(gimbal);
    }
    pid_ctrl_t *angle_pid = YawAnglePid(gimbal);
    pid_ctrl_t *speed_pid = YawSpeedPid(gimbal);
    /* ====== 休眠模式：立即清空PID并卸力 ====== */
    if (gimbal->mode == GIMBAL_MODE_SLEEP) {
        ClearYawPids(gimbal);
        gimbal->yaw.give_current = 0;            /* 电流输出清零 */
        kt_motor[0].base_info.motor_out = 0;     /* 电机输出清零 */
        KT_Yaw_SendIq(0);                        /* 发送零电流指令 */
        return;
    }

    /* MECH/GYRO 速度内环都依赖上层 IMU；失联时清零，INIT 寻零仍用电机速度。 */
    if ((gimbal->mode == GIMBAL_MODE_GYRO || gimbal->mode == GIMBAL_MODE_MECH) &&
        !board_rx_info.is_online) {
        ClearYawPids(gimbal);
        gimbal->yaw.give_current = 0;
        kt_motor[0].base_info.motor_out = 0;
        KT_Yaw_SendIq(0);
        return;
    }

    float target_angle = 0.0f;
    float feedback_angle = 0.0f;
    float feedback_speed = 0.0f;

    /* ====== 步骤1：根据PID模式选择反馈源 ====== */
    if (gimbal->yaw.pid_mode == GIMBAL_PID_GYRO) {
        /* 陀螺仪模式：使用大地坐标系的IMU数据进行闭环 */
        target_angle   = gimbal->yaw.target_imu_angle;   /* 空间目标角 */
        feedback_angle = gimbal->yaw.imu_angle;           /* IMU反馈角 */
        feedback_speed = gimbal->yaw.imu_speed;           /* IMU角速度反馈 */
    } else {
        /* 机械/初始化：角度外环仍用编码器；MECH 速度内环改用 IMU，INIT 保留电机速度。 */
        target_angle   = gimbal->yaw.target_mec_angle;    /* 机械目标角 */
        feedback_angle = gimbal->yaw.feedback_mec_angle;   /* 编码器反馈角 */
        feedback_speed = (gimbal->mode == GIMBAL_MODE_MECH) ?
                         gimbal->yaw.imu_speed : gimbal->yaw.feedback_mec_speed;
    }

    /* ====== 步骤2：外环角度PID计算 ====== */
   
    float angle_err = Calc_Shortest_Angle_Err(target_angle, feedback_angle); //计算最短路径角度误差
    angle_pid->err = angle_err;
    single_pid_ctrl(angle_pid);

    float target_speed = angle_pid->out;

    /* MECH 外环是相对底盘角度，内环是空间角速度：补上底盘实际转速。
     * 下层 ave_rate_yaw 单位 deg/s，左转为正；上层 imu_speed 单位 rad/s。
     * gain 保存在结构体中，清 PID 时保留，便于 Watch 实时调参。 */
    gimbal->yaw.chassis_speed_rads = 0.0f;
    gimbal->yaw.chassis_fow_speed = 0.0f;
    gimbal->yaw.chassis_fow_valid = false;

    if (gimbal->mode == GIMBAL_MODE_MECH &&
        gimbal->yaw.pid_mode == GIMBAL_PID_MEC &&
        imu_sensor.info != NULL &&
        imu_sensor.work_state.dev_state == DEV_ONLINE &&
        imu_sensor.work_state.err_code == IMU_NONE_ERR &&
        imu_sensor.work_state.cali_end != 0U) {
        float chassis_speed = imu_sensor.info->base_info.ave_rate_yaw * DEG_TO_RAD;
        float fow_speed = chassis_speed * gimbal->yaw.chassis_fow_rads;
        if (isfinite(chassis_speed) && isfinite(gimbal->yaw.chassis_fow_rads) &&
            isfinite(fow_speed)) {
            gimbal->yaw.chassis_speed_rads = chassis_speed;
            gimbal->yaw.chassis_fow_speed =
                constrain(fow_speed, -angle_pid->out_max, angle_pid->out_max);
            gimbal->yaw.chassis_fow_valid = true;
            target_speed = constrain(target_speed + gimbal->yaw.chassis_fow_speed,
                                     -angle_pid->out_max, angle_pid->out_max);
        }
    }
    gimbal->yaw.yaw_target_speed = target_speed;

    /* ====== 步骤3：内环速度PID计算 ====== */
    speed_pid->target = target_speed;
    speed_pid->measure = feedback_speed;
    pid_err_cal(speed_pid);
    /* Differentiate feedback only: target changes must not kick the D term.
       Keep the existing discrete PID gain convention (no division by dt). */
    if (!gimbal->yaw.speed_d_ready) {
        gimbal->yaw.speed_d_filtered_delta = 0.0f;//切换清除累计
        gimbal->yaw.speed_d_ready = true;
    } else {
        float a = fminf(1.0f, fmaxf(0.0f, speed_pid->a));//保上1下0限制，夹紧滤波系数0-1安全措施
        float delta = gimbal->yaw.speed_d_last_feedback - feedback_speed;
        gimbal->yaw.speed_d_filtered_delta =
            a * delta + (1.0f - a) * gimbal->yaw.speed_d_filtered_delta;
    }
    gimbal->yaw.speed_d_last_feedback = feedback_speed;
    single_pid_ctrl_with_dout(speed_pid,
                             speed_pid->kd * gimbal->yaw.speed_d_filtered_delta);

    /* ====== 步骤4：下发控制电流到KT9025电机 ====== */
     gimbal->yaw.give_current = (int16_t)speed_pid->out;  /* 速度环输出转为电流 */
     kt_motor[0].base_info.motor_out = gimbal->yaw.give_current;      /* 记录到电机对象 */
     KT_Yaw_SendIq(gimbal->yaw.give_current);   
                          /* CAN发送电流指令 */

    /* ====== 步骤4：调试——GYRO 下 yaw 输出先给 0，只调上层 pitch ====== */
//    gimbal->yaw.give_current = 0;
//    kt_motor[0].base_info.motor_out = 0;
//    KT_Yaw_SendIq(0);

}


/* ========================================================================= */
/*           云台主工作循环函数-----入口，调用上面所有功能函数                            */
/* ========================================================================= */

void Gimbal_Update(Gimbal_t *gimbal, CarMode_e car_mode) {

    /*==========================================================================
     * 第1步：模式映射与安全保护
     * 根据整车模式和各模块状态，确定云台应该处于什么工作模式
     *==========================================================================*/
    gimbal->last_mode = gimbal->mode;  /* 保存上一次模式（用于边沿检测） */

    if (car_mode == CAR_MODE_SLEEP ||                           /* 整车休眠 */
        rc_sensor.work_state != DEV_ONLINE ||                   /* 遥控器离线 */
        rc_sensor.errno != NONE_ERR ||                          /* 遥控器错误 */
        kt_motor[0].KT_motor_info.state_info.work_state != M_ONLINE) {  /* KT电机离线 */
        gimbal->mode = GIMBAL_MODE_SLEEP;          /* 异常 -> 休眠模式 */
        gimbal->reset_state = GIMBAL_RESET_PENDING; /* 标记需要重新寻零 */
    }
    else if (car_mode == CAR_MODE_INIT || gimbal->reset_state == GIMBAL_RESET_PENDING) {
        gimbal->mode = GIMBAL_MODE_INIT;   /* 初始化模式：自动寻零对齐 */
    }
    else if (car_mode == CAR_MODE_GYRO || car_mode == CAR_MODE_SPIN) {
        gimbal->mode = GIMBAL_MODE_GYRO;   /* 陀螺仪自稳跟随模式 */
    }
    else {
        gimbal->mode = GIMBAL_MODE_MECH;  
    }

    /* 非 GYRO 键鼠或设备失效时关闭前馈；重新切入等待新鼠标帧。 */
    Gimbal_Key_Yaw_Feedforward_Enable(gimbal, car_mode);

    /* 重新进入寻零时必须使用编码器闭环；清掉上一个模式的积分和输出。 */
    if (gimbal->mode == GIMBAL_MODE_INIT) {
        if (gimbal->last_mode != GIMBAL_MODE_INIT) {
            ClearYawPids(gimbal);
        }
        gimbal->yaw.pid_mode = GIMBAL_PID_MEC;
    }

    /* 模式切换平滑过渡，从非GYRO模式切入GYRO模式的瞬间，将空间目标角同步为当前IMU角*/
    if (gimbal->last_mode != GIMBAL_MODE_GYRO && gimbal->mode == GIMBAL_MODE_GYRO) {
        gimbal->yaw.target_imu_angle = gimbal->yaw.imu_angle;
        /* Pitch 同理：IMU 域目标对齐当前 pitch，防止切模式瞬间抬头/低头猛甩 */
        gimbal->pitch.target_imu_angle = gimbal->pitch.imu_angle;
        ClearYawPids(gimbal);
    }

    /*==========================================================================
     * 第2步：读取下层板KT9025电机编码器，解算相对底盘夹角
     * 无论在什么模式，feedback_mec_angle都要实时更新，
     * 因为底盘跟随算法需要这个角度值。
     *==========================================================================*/
    /* 读取KT9025的16位原始编码器值 */
    float raw_encoder = (float)kt_motor[0].KT_motor_info.rx_info.encoder;

    /* 计算相对于编码器中间位置的偏差，作为反馈度数 */
    float encoder_diff = raw_encoder - YAW_KT_ENCODER_MIDDLE;

    /* 编码器最大65536，将偏差约束到[-32768, 32768]范围内 */
    while (encoder_diff > KT_ENCODER_HALF_RANGE)  encoder_diff -= KT_ENCODER_FULL_RANGE;
    while (encoder_diff < -KT_ENCODER_HALF_RANGE) encoder_diff += KT_ENCODER_FULL_RANGE;

    /* 将上述约束后的编码转为弧度 */
    gimbal->yaw.feedback_mec_angle = encoder_diff * (PI / KT_ENCODER_HALF_RANGE);

    /* 编码器速度转换：KT电机返回的速度单位为度/秒，转换为弧度/秒 */
    gimbal->yaw.feedback_mec_speed = (float)kt_motor[0].KT_motor_info.rx_info.speed * (PI / 180.0f);

    /*==========================================================================
     * 第2.4步：换头/掉头状态机（必须在对齐迟滞之前）
     * 仅 MECH 可换头；GYRO/SPIN/SLEEP/INIT 一律否决并中止。
     *==========================================================================*/
    Gimbal_Turn_Update(gimbal, car_mode);

    /*==========================================================================
     * 第2.5步：仅 MECH 使用双向就近对齐迟滞（88°/91°）；GYRO 不切换锁轴方向
     * 保险：小陀螺（SPIN）与换头（turn_active）一票否决。
     * 自旋/换头时相对夹角快速扫过，若允许更新 align_dir，
     * 底盘解算会在正/反向之间高频震荡。
     * 否决期间每拍刷新计数；退出后再冷却 YAW_ALIGN_POST_SPIN_TICKS。
     * 换头结束时 Gimbal_Turn_Apply_Align_After 已按真实轴向写 align_dir
     * 并清零 veto，这里不会再把底盘拖进 FRONT 半圈空转。
     *==========================================================================*/
    if ((car_mode == CAR_MODE_SPIN) || gimbal->yaw.turn_active) {
        gimbal->yaw.align_veto_cnt = YAW_ALIGN_POST_SPIN_TICKS;  /* 在旋/换头：持续否决 */
    } else if (gimbal->yaw.align_veto_cnt > 0U) {
        gimbal->yaw.align_veto_cnt--;  /* 退出后的冷却倒计时 */
    }

    if ((car_mode == CAR_MODE_MECH) &&
        (gimbal->yaw.align_veto_cnt == 0U)) {
        const float abs_deg =
            fabsf(gimbal->yaw.feedback_mec_angle) * (180.0f / PI);
        if (abs_deg > YAW_ALIGN_BACK_MIN_DEG) {
            gimbal->yaw.align_dir = ALIGN_BACK;   /* 离车尾更近：锁 180° */
        } else if (abs_deg < YAW_ALIGN_FRONT_MAX_DEG) {
            gimbal->yaw.align_dir = ALIGN_FRONT;  /* 离车头更近：锁 0° */
        }
        /* 88°~91° 缓冲区：保持上一拍方向，防临界横跳 */
    }

    /*==========================================================================
     * 第3步：根据当前模式更新目标值
     *==========================================================================*/
    switch (gimbal->mode) {
        case GIMBAL_MODE_INIT:
            Gimbal_Init_Step(gimbal);     /* 执行寻零步进 */
            break;

        case GIMBAL_MODE_MECH:
            Gimbal_Mech_Update(gimbal);   /* 机械模式目标更新（按 align_dir 锁轴） */
            break;

        case GIMBAL_MODE_GYRO:
            Gimbal_Gyro_Update(gimbal);   /* 陀螺仪模式目标更新 */
            break;

        case GIMBAL_MODE_SLEEP:
        default:
            break;  /* 休眠模式不更新目标 */
    }
  
    Gimbal_Pitch_Update(gimbal);

    Gimbal_Yaw_Pid_Cal(gimbal);
}
