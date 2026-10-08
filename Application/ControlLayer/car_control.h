/**
  ******************************************************************************
  * @file           : car_control.h
  * @brief          : 整车状态机管理模块（纯遥控器版）
  ******************************************************************************
  */

#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/* 整车运动模式枚举 */
typedef enum {
    CAR_MODE_SLEEP = 0,     /* 休眠/关控保护模式 */
    CAR_MODE_INIT,          /* 上电复位对齐模式 */
    CAR_MODE_GYRO,          /* 陀螺仪跟随模式 (默认主力模式) */
    CAR_MODE_MECH,          /* 机械模式 (底盘与云台就近锁轴：0°或180°) */
    CAR_MODE_SPIN,          /* 小陀螺自旋模式 */
} CarMode_e;

/* 双向就近对齐方向：云台/底盘锁车头轴还是车尾轴。
   由 gimbal按 88°/91° 迟滞从 feedback_mec_angle 解出；
   小陀螺（SPIN）期间一票否决，禁止翻转。 */
typedef enum {
    ALIGN_FRONT = 0,        /* 近车头：对齐 0° */
    ALIGN_BACK  = 1,        /* 近车尾：对齐 180° */
} AlignDir_e;

/* 输入源枚举：底盘/云台/发射读哪一套指令
   键盘鼠标经 DR16 同一帧送上来（key_v 在 rxBuf[14..15]），与摇杆不冲突，
   这里只做「操作手现在用哪套」的标记，供后续键鼠控制分流用 */
typedef enum {
    INPUT_SRC_RC = 0,       /* 遥控器摇杆 */
    INPUT_SRC_KEY,          /* 键盘鼠标 */
} InputSrc_e;

/* 复位状态枚举 */
typedef enum {
    RESET_PENDING = 0,      /* 正在复位 */
    RESET_DONE,             /* 复位完成 */
} ResetState_e;

/* 整车控制标志位集合 */
typedef struct {
    bool is_shoot_system_on;    /* 摩擦轮开启标志 */
    bool is_single_shoot_cmd;   /* 单发打弹脉冲指令 */
    bool is_continuous_shoot;   /* 连发标志（拨弹持续转动） */
    bool is_turn_cmd;           /* 换头/掉头脉冲：目标角+180°，仅 GYRO/SPIN 置位 */
} CarFlags_t;

/* 整车总控对象 */
typedef struct {
    CarMode_e mode;             /* 当前整车模式 */
    CarMode_e last_mode;        /* 上一周期模式 */
    ResetState_e reset_state;   /* 复位状态 */
    uint16_t init_timer;        /* 复位超时计时器 */
    InputSrc_e input_src;       /* 当前输入源（键鼠模式用） */

    CarFlags_t flags;           /* 控制标志位 */
} Car_t;

extern Car_t hero_car;

/* 外部调用接口 */
void Car_Control_Init(Car_t *car);
void Car_Control_Update(Car_t *car);

/**
  * @brief  按当前输入源取底盘三轴通道
  * @details 输出统一是 SBUS 计数域 [-660, 660]，与遥控器摇杆同量纲：
  *          - INPUT_SRC_RC  : 直接回传 ch3 / ch2 / ch0（与逐通道传参完全一致）
  *          - INPUT_SRC_KEY : W/S、A/D 的「按住时间比例」折算到 ±660，Ctrl 按住时平移置零
  *                            机械模式下 Q/E 同样折算到旋转通道（Q=左旋、E=右旋）；
  *                            GYRO/SPIN 不映射旋转，*rotate 恒为 0
  *
  *          底盘模块因此不必知道输入源，Chassis_Update 的归一化/死区/麦轮解算一条不用改。
  *
  * @param  car     : 整车状态机对象指针
  * @param  forward : 前后通道（正 = 前）
  * @param  lateral : 左右通道（正 = 左）
  * @param  rotate  : 自旋通道（正 = 逆时针；键鼠仅机械模式映射 Q/E，其余恒为 0）
  */
void Car_Chassis_Channel_Get(Car_t *car, int16_t *forward, int16_t *lateral, int16_t *rotate);

#endif /* CAR_CONTROL_H */
