/**
  ******************************************************************************
  * @file           : gimbal.h
  * @brief          : 云台控制模块头文件（支持机械模式与陀螺仪自稳模式）
  * @details        : 声明云台数据结构、PID模式枚举、常量宏定义与控制接口
  ******************************************************************************
  */

#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>
#include <stdbool.h>
#include "PID.h"
#include "car_control.h"

#define DEG_TO_RAD                  (0.01745329251994329577f)

/* ========================================================================= */
/*                          KT9025 硬件与几何参数                            */
/* ========================================================================= */
#define KT_ENCODER_FULL_RANGE       (65536.0f)   /* 16位编码器满量程 (2^16) */
#define KT_ENCODER_HALF_RANGE       (32768.0f)   /* 半圈值 */

/* 战队机械零点偏置 (对应移相后的基准值) */
#define YAW_KT_ENCODER_MIDDLE       (25823.0f + 32768.0f)

/* Pitch 轴软限位 (rad) */
#define PITCH_MIN_DEG               (-16.0f)//从-18降低
#define PITCH_MAX_RAD               (0.55f)
#define PITCH_MIN_MEC_ANGLE         (PITCH_MIN_DEG * DEG_TO_RAD)
#define PITCH_MAX_MEC_ANGLE         (PITCH_MAX_RAD)

/* GYRO 域 Pitch 目标静态限位 (rad)：IMU 世界域，标定后定死不再调。
 * 上板 gimbal.h 同名同值，改一处必须改两处。 */
#define PITCH_MIN_IMU_ANGLE         (-0.10f)
#define PITCH_MAX_IMU_ANGLE         (0.70f)

/* 摇杆控制灵敏度 (2ms 周期下的积分步长) */
#define RC_YAW_MECH_SENSITIVITY     (0.0001f)    /* 机械模式 Yaw 灵敏度 */
#define RC_YAW_GYRO_DEADZONE        (40.0f)      /* GYRO 模式摇杆死区，过滤中位偏差，此处的数值是摇杆通道值 */

#define RC_PITCH_DEG_PER_TICK       (0.00015f)
#define RC_PITCH_MECH_SENSITIVITY   (RC_PITCH_DEG_PER_TICK * DEG_TO_RAD)
#define RC_PITCH_MECH_DEADZONE      (30.0f)  /* 右摇杆上下(ch1)中位死区，单位SBUS计数 */

/* ----------------------- 键鼠瞄准灵敏度 -------------------------------- */
/* 鼠标位移是「一帧一个采样」，由 frame_seq 闸门保证一帧只累加一次，
   所以下面两个量纲是「rad / 每帧滤波后鼠标位移」，不是每 2ms。
   符号约定：鼠标右移 = 瞄准向右 = 目标角减小；鼠标上移 = 抬头 = 目标角增大。
   上机手感不对就调这两个数，别去动闸门。 */
#define MOUSE_YAW_SENS              (0.0007f)    /* 鼠标横向 -> Yaw 目标角增量 (rad) */
#define MOUSE_PITCH_SENS            (0.0008f)   /* 鼠标纵向 -> Pitch 目标角增量 (rad) */

/* Ctrl + WASD 单击 = 云台目标角步进微调（度 -> 弧度）。
   只认「刚按下」边沿，不连发，操作手可以数着步数校准。
   Ctrl+Shift 同按走大步距。 */
#define KEY_TRIM_STEP_SMALL         (0.05f * DEG_TO_RAD)
#define KEY_TRIM_STEP_LARGE         (0.2f  * DEG_TO_RAD)

/* ---------------- 仅机械模式双向就近对齐（88°/91° 迟滞，上机调参） ----------------
   夹角 < 88°：锁车头轴（目标 0°）
   夹角 > 91°：锁车尾轴（目标 180°）
   88°~91°：保持上一拍方向，防 90° 临界横跳。
   小陀螺期间对齐方向一票否决（不更新），退出后再冷却，
   禁止自旋时相对夹角扫圈导致 FRONT/BACK 高频翻转。 */
#define YAW_ALIGN_FRONT_MAX_DEG     (88.0f)
#define YAW_ALIGN_BACK_MIN_DEG      (91.0f)
#define YAW_ALIGN_POST_SPIN_TICKS   (250U)  /* 退出小陀螺后的否决冷却：250×2ms=0.5s */

/* ---------------- 换头/掉头（仅 MECH） ----------------
   触发：遥控 S1上+S2上+拨轮上，或键鼠 R（只在机械模式）。
   MECH：粘性锁轴 FRONT↔BACK（0° ↔ 180°）。头锁死时 |delta| 很小，
   88°/91° 迟滞翻不到车尾，机械模式必须手动锁轴。
   GYRO/SPIN 的世界系目标角 +180° 掉头已删除，这两个模式不响应触发。
   误差≤5° 或超时 1.5s 结束。
   换头期间冻结对齐迟滞；结束后按真实轴向立刻写 align_dir，
   底盘 follow_err≈0，W = 倒车，不再空转半圈「假换头」。 */
#define YAW_TURN_ARRIVE_ERR_DEG     (5.0f)
#define YAW_TURN_TIMEOUT_TICKS      (750U)  /* 750×2ms=1.5s */

/* 云台运行模式 */
typedef enum {
    GIMBAL_MODE_SLEEP = 0,     
    GIMBAL_MODE_INIT,           /* 上电自动寻中归位 */
    GIMBAL_MODE_MECH,         
    GIMBAL_MODE_GYRO,     
} GimbalMode_e;

/* 云台复位状态 */
typedef enum {
    GIMBAL_RESET_PENDING = 0,   /* 复位中 */
    GIMBAL_RESET_DONE,          /* 复位完成 */
} GimbalReset_e;

/* PID 控制器类型枚举 */
typedef enum {
    GIMBAL_PID_MEC = 0,         /* 机械角度闭环 (使用编码器反馈) */
    GIMBAL_PID_GYRO,            /* 陀螺仪空间闭环 (使用上层板 IMU 反馈) */
} GimbalPidMode_e;


/* Yaw 轴执行结构体 */
typedef struct {
    /* 机械模式数据 (底盘相对坐标系) */
    float target_mec_angle;     /* 目标机械角度 (rad)：按 align_dir 锁 0° 或 180° */
    float feedback_mec_angle;   /* 当前机械角度 (rad, 给底盘跟随使用) */
    float feedback_mec_speed;   /* 当前电机转速 (rad/s) */

    /* 陀螺仪模式数据 (上层 C 板大地惯性坐标系) */
    float target_imu_angle;     /* 目标空间绝对角度 (rad) */
    float imu_angle;            /* 上层 C 板发来的绝对角度 yaw_imu (rad) */
    float imu_speed;            /* 上层 C 板发来的绝对角速度 yaw_v (rad/s) */
    float gyro_sensitivity;      /* GYRO 模式每周期、每单位摇杆输入的目标角增量 (rad) */

    /* 仅 MECH 使用双向就近对齐；GYRO 保持空间目标，不使用车头/车尾锁轴。 */
    AlignDir_e align_dir;       /* FRONT=锁0° / BACK=锁180°，88°/91° 迟滞维护 */
    uint16_t align_veto_cnt;    /* >0 时禁止更新 align_dir（小陀螺/换头及退出冷却） */

    /* 换头/掉头（仅 MECH）：粘性锁轴 FRONT↔BACK 翻转 */
    bool turn_active;           /* 换头进行中：冻结对齐迟滞 */
    bool turn_finish;           /* 最近一次换头已结束（到角或超时） */
    uint16_t turn_time;         /* 换头已进行拍数 */

    /* 控制模式与双环 PID */
    GimbalPidMode_e pid_mode;   /* 当前激活的 PID 模式 */

    pid_ctrl_t mec_angle_pid;//机械模式pid
    pid_ctrl_t mec_speed_pid;

    /* MECH：底盘实测角速度补偿，Watch 可实时调系数；其余字段用于观察。 */
    float chassis_fow_rads;     /* 无量纲补偿系数 */
    float chassis_speed_rads;   /* 有效的下层 IMU 角速度 (rad/s) */
    float chassis_fow_speed;    /* 限幅后的底盘角速度补偿量 (rad/s) */
    float yaw_target_speed;     /* 外环输出 + 补偿，最终限幅目标 (rad/s) */
    bool chassis_fow_valid;     /* 本周期底盘补偿数据是否有效 */

    pid_ctrl_t gyro_angle_pid;//陀螺仪模式pid
    pid_ctrl_t gyro_speed_pid;

    float speed_d_last_feedback; /* Speed D uses feedback difference per tick. */
    float speed_d_filtered_delta;
    bool speed_d_ready;
    GimbalPidMode_e speed_d_pid_mode;

    int16_t give_current;       /* 最终下发力矩电流 */
} Gimbal_Yaw_t;


/* Pitch 轴目标管理
   MECH：目标在编码器机械域（下层只生成目标，上层 C 板闭环）
   GYRO/SPIN：目标在 IMU 世界域累加，随 0xD1 的 pitch_use_imu=1 发给上层，
              上层用自带 IMU 双环 + 重力补偿闭环，目标受机械行程约束 */
typedef struct {
    float target_mec_angle;     /* MECH：编码器机械目标角 (rad) */
    float target_imu_angle;     /* GYRO/SPIN：IMU 域 pitch 目标角 (rad) */
    float imu_angle;            /* 上层 C 板 0x022 回传 pitch 角 (rad) */
    float imu_speed;            /* 上层 C 板 0x022 回传 pitch 角速度 (rad/s) */
    bool target_rebase_pending; /* 反馈恢复后按当前角度重建目标 */
    float min_limit;            /* 机械下限位（与上层 mec_min_rad 一致） */
    float max_limit;            /* 机械上限位 */
} Gimbal_Pitch_Target_t;


/* 云台整机管理对象 */
typedef struct {
    GimbalMode_e mode;
    GimbalMode_e last_mode;
    GimbalReset_e reset_state;
    uint16_t init_timer;

    Gimbal_Yaw_t yaw;
    Gimbal_Pitch_Target_t pitch;
} Gimbal_t;

extern Gimbal_t hero_gimbal;

/* 外部调用接口 */
void Gimbal_Init(Gimbal_t *gimbal);
void Gimbal_Update(Gimbal_t *gimbal, CarMode_e car_mode);

#endif /* GIMBAL_H */
