/**
  ******************************************************************************
  * @file           : gimbal.c
  * @brief          : 云台全模式实现（含 Yaw 轴陀螺仪双环 PID 闭环）
  * @details        : 本文件实现了RoboMaster机器人云台的完整控制逻辑，包括：
  *
  *                   1. KT9025编码器解算：将16位原始编码器值转换为相对车头的标准弧度
  *                   2. 云台初始化与自动寻零：上电后自动对齐机械零位
  *                   3. 机械模式（MECH）：Yaw轴锁死在底盘正前方，随底盘旋转
  *                   4. 陀螺仪自稳模式（GYRO）：基于大地坐标系的空间自稳跟随
  *                   5. 小陀螺模式（SPIN）：底盘自旋时云台保持空间朝向不变

  
  *                   云台模式状态机：
  *                   SLEEP -> INIT(寻零) -> MECH/GYRO(正常工作)
  *                   任何异常情况回退到SLEEP模式
  ******************************************************************************
  */

#include "gimbal.h"
#include "rc_sensor.h"
#include "kt_yaw_motor.h"
#include "communicate.h"
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
}


void Gimbal_Init(Gimbal_t *gimbal) {
    /* 初始化为休眠模式，等待整车状态机切换到工作模式 */
    gimbal->mode = GIMBAL_MODE_SLEEP;
    gimbal->last_mode = GIMBAL_MODE_SLEEP;
    gimbal->reset_state = GIMBAL_RESET_PENDING;  /* 复位状态：等待复位 */
    gimbal->init_timer = 0;

    /* Pitch轴软限位设置（单位：弧度） */
    gimbal->pitch.min_limit = PITCH_MIN_MEC_ANGLE;   /* 下限约-12度 */
    gimbal->pitch.max_limit = PITCH_MAX_MEC_ANGLE;   /* 遥控目标上限 +0.35rad */
    gimbal->pitch.target_mec_angle = 0.0f;


  /* 机械模式参数，+不加测速头 */
    gimbal->yaw.mec_angle_pid.kp = 8.0f;
    gimbal->yaw.mec_angle_pid.ki = 0.0f;
    gimbal->yaw.mec_angle_pid.kd = 0.0f;
    gimbal->yaw.mec_angle_pid.out_max = 12.0f;
    gimbal->yaw.mec_angle_pid.integral_max = 0.0f;
    gimbal->yaw.mec_angle_pid.a = 1.0f;
    gimbal->yaw.mec_speed_pid.kp = 240.0f;
    gimbal->yaw.mec_speed_pid.ki = 0.5f;
    gimbal->yaw.mec_speed_pid.kd = 0.0f;
    gimbal->yaw.mec_speed_pid.out_max = 2000.0f;//限幅已经修改成正常落地，调试用200
    gimbal->yaw.mec_speed_pid.integral_max = 400.0f;
    gimbal->yaw.mec_speed_pid.a = 1.0f;


    /* GYRO模式参数：当前未加测速头 */
    gimbal->yaw.gyro_angle_pid.kp = 10.0f;
    gimbal->yaw.gyro_angle_pid.ki = 0.0f;
    gimbal->yaw.gyro_angle_pid.kd = 0.0f;
    /* 陀螺仪角度环输出目标角速度限制,attention防止反馈异常时高速冲转。 */
    gimbal->yaw.gyro_angle_pid.out_max = 10.0f;    //最上面为12
    gimbal->yaw.gyro_angle_pid.integral_max = 0.0f;
    gimbal->yaw.gyro_angle_pid.a = 1.0f;
    gimbal->yaw.gyro_speed_pid.kp = 410.0f;
    gimbal->yaw.gyro_speed_pid.ki = 0.03f;
    gimbal->yaw.gyro_speed_pid.kd = 0.0f;
    /* 陀螺仪速度环输出是最终电流指令，先限制为安全调试值，避免
     * IMU反馈异常或方向不一致时直接满电流疯转。 */
    gimbal->yaw.gyro_speed_pid.out_max = 1800.0f;  //最上面为2000
    gimbal->yaw.gyro_speed_pid.integral_max = 400.0f;
    gimbal->yaw.gyro_speed_pid.a = 1.0f;

    /* 数据清零 */
    gimbal->yaw.target_mec_angle = 0.0f;      /* 机械目标角 */
    gimbal->yaw.feedback_mec_angle = 0.0f;    /* 机械反馈角 */
    gimbal->yaw.feedback_mec_speed = 0.0f;    /* 机械反馈速度 */

    gimbal->yaw.target_imu_angle = 0.0f;      /* IMU目标角 */
    gimbal->yaw.imu_angle = 0.0f;             /* IMU反馈角（来自上层C板） */
    gimbal->yaw.imu_speed = 0.0f;             /* IMU反馈速度（来自上层C板） */
    gimbal->yaw.gyro_sensitivity = 0.00001f; /* 灵敏度*/

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
    gimbal->yaw.target_mec_angle = 0.0f;     /* 锁死在底盘正前方 */
}


static void Gimbal_Gyro_Update(Gimbal_t *gimbal) {
    gimbal->yaw.pid_mode = GIMBAL_PID_GYRO;  /* 切换到陀螺仪模式PID */

    //yaw摇杆映射
    float rc_yaw_input = -(float)rc_sensor.info->ch0;

    //死区 
    if (fabsf(rc_yaw_input) < RC_YAW_GYRO_DEADZONE) {
        rc_yaw_input = 0.0f;
    }

    //rad
    gimbal->yaw.target_imu_angle += rc_yaw_input * gimbal->yaw.gyro_sensitivity;

    /* 角度规范化，防止目标角溢出 */
    gimbal->yaw.target_imu_angle = Angle_Normalize_PI(gimbal->yaw.target_imu_angle);
}



static void Gimbal_Pitch_Update(Gimbal_t *gimbal)
{
    /* 休眠模式或寻零阶段：Pitch 目标强制水平对齐 (0.0 rad) */
    if (gimbal->mode == GIMBAL_MODE_SLEEP || gimbal->mode == GIMBAL_MODE_INIT) {
        gimbal->pitch.target_mec_angle = 0.0f;
        return;
    }


    /* 遥控器与编码器坐标统一：摇杆向上产生正目标，向下产生负目标。 */
    float rc_pitch_input = (float)rc_sensor.info->ch1;

    /*摇杆中位死区 */
    /* 右摇杆上下(ch1)在死区内不改变目标，避免中位漂移持续累加。 */
    if (fabsf(rc_pitch_input) <= RC_PITCH_MECH_DEADZONE) {
        rc_pitch_input = 0.0f;
    }

    /* 积分累加，灵敏度为 RC_PITCH_MECH_SENSITIVITY */
    gimbal->pitch.target_mec_angle += rc_pitch_input * RC_PITCH_MECH_SENSITIVITY;


    /* 这里只限制遥控器累加出来的目标角，不用反馈角改写目标。 */
    if (gimbal->pitch.target_mec_angle > gimbal->pitch.max_limit) {
        gimbal->pitch.target_mec_angle = gimbal->pitch.max_limit;
    } else if (gimbal->pitch.target_mec_angle < gimbal->pitch.min_limit) {
        gimbal->pitch.target_mec_angle = gimbal->pitch.min_limit;
    }
}

/* ========================================================================= 
                    3. Yaw 轴陀螺仪 / 机械 双环 PID 闭环计算              
========================================================================= */

static void Gimbal_Yaw_Pid_Cal(Gimbal_t *gimbal) {
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

    /* ====== 陀螺仪模式下通信失联清零停止保护 ====== */
    if (gimbal->mode == GIMBAL_MODE_GYRO && !board_rx_info.is_online) {
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
        /* 机械/初始化模式：使用底盘相对坐标系的编码器数据进行闭环 */
        target_angle   = gimbal->yaw.target_mec_angle;    /* 机械目标角 */
        feedback_angle = gimbal->yaw.feedback_mec_angle;   /* 编码器反馈角 */
        feedback_speed = gimbal->yaw.feedback_mec_speed;   /* 编码器速度反馈 */
    }

    /* ====== 步骤2：外环角度PID计算 ====== */
   
    float angle_err = Calc_Shortest_Angle_Err(target_angle, feedback_angle); //计算最短路径角度误差
    angle_pid->err = angle_err;
    single_pid_ctrl(angle_pid);

    float target_speed = angle_pid->out;

    /* ====== 步骤3：内环速度PID计算 ====== */
    speed_pid->target = target_speed;
    speed_pid->measure = feedback_speed;
    pid_err_cal(speed_pid);
    single_pid_ctrl(speed_pid);

    /* ====== 步骤4：下发控制电流到KT9025电机 ====== */
    gimbal->yaw.give_current = (int16_t)speed_pid->out;  /* 速度环输出转为电流 */
    kt_motor[0].base_info.motor_out = gimbal->yaw.give_current;      /* 记录到电机对象 */
    KT_Yaw_SendIq(gimbal->yaw.give_current);                         /* CAN发送电流指令 */
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
     * 第3步：根据当前模式更新目标值
     *==========================================================================*/
    switch (gimbal->mode) {
        case GIMBAL_MODE_INIT:
            Gimbal_Init_Step(gimbal);     /* 执行寻零步进 */
            break;

        case GIMBAL_MODE_MECH:
            Gimbal_Mech_Update(gimbal);   /* 机械模式目标更新 */
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
