/**
  ******************************************************************************
  * @file           : power_model.c
  * @brief          : 底盘功率模型与功率预算
  * @details        : 二次功率模型的正/反解，以及功率预算取值。接口见 power_model.h。
  *
  * @note           ====== 系数标定说明（上车必读） ======
  *                  下列系数是台架拟合值，形如：
  *                      P(i,w) = k1*w^2 + k2*i^2 + kiw*i*w + c
  *                  其中 i 为 CAN raw current（M3508 满量程 ±16384），
  *                  w 为电机转速 rpm —— 与本工程底盘 PID 的输出同量纲
  *                  （chassis 电机是 _3508_Single，力矩转电流是直通的）。
  *
  *                  数值随车（电机批次 / 轮径 / 电池内阻 / 车重）变化，
  *                  换车必须重标：台架上只开预测不开限流，打印 Power_Predict
  *                  的四轮累加值，对照功率计，再回来改下面三个系数。
  ******************************************************************************
  */
#include "power_model.h"

#include <math.h>

#include "judge.h"
#include "rp_config.h"

/* ==================== 二次功率模型系数（台架拟合，换车重标） ==================== */
#define POWER_K_W2    (3.0e-7f)        /* 铁损/风阻项：w^2 */
#define POWER_K_I2    (1.23e-7f)       /* 铜损项：i^2 */
#define POWER_K_IW    (1.99688994e-6f) /* 反电动势项：i*w */
#define POWER_K_CONST (4.081f)         /* 空载常数项 */

/**
  * @brief  正向：由电流和转速预测功率
  */
float Power_Predict(float current_raw, float speed_rpm)
{
    return POWER_K_W2 * speed_rpm * speed_rpm
         + POWER_K_I2 * current_raw * current_raw
         + POWER_K_IW * current_raw * speed_rpm
         + POWER_K_CONST;
}

/**
  * @brief  反向：在功率预算下解算电流
  * @details 把 P(i,w) 看成关于 i 的一元二次方程：
  *              POWER_K_I2 * i^2 + (POWER_K_IW * w) * i
  *                                + (POWER_K_W2 * w^2 + POWER_K_CONST - target_power) = 0
  *          两个实根分别对应"加速"和"减速"工况，按原始电流符号选取，
  *          保证限流后出力方向与 PID 意图一致。
  */
float Power_Solve_Current(float target_power, float speed_rpm, float current_raw)
{
    /* 目标功率非法：不改动 PID 结果 */
    if (target_power <= 0.0f) {
        return current_raw;
    }

    const float a = POWER_K_I2;
    const float b = POWER_K_IW * speed_rpm;
    const float c_term = POWER_K_W2 * speed_rpm * speed_rpm
                       + POWER_K_CONST - target_power;

    const float disc = b * b - 4.0f * a * c_term;

    /* 无实根：这个转速下打不到目标功率，退回原值交给下一拍再试 */
    if (disc < 0.0f) {
        return current_raw;
    }

    const float sqrt_disc = sqrtf(disc);
    const float inv_2a = 1.0f / (2.0f * a);

    /* 正电流走正根（驱动），负电流走负根（制动），零电流不动 */
    if (current_raw > 0.0f) {
        return (-b + sqrt_disc) * inv_2a;
    }
    if (current_raw < 0.0f) {
        return (-b - sqrt_disc) * inv_2a;
    }
    return 0.0f;
}

/**
  * @brief  取当前底盘功率预算
  */
float Power_Budget_Get(void)
{
    /* 裁判系统在线且给出有效上限：用裁判值，留 POWER_BUDGET_MARGIN 裕量 */
    if (My_Judge.status->status == DEV_ONLINE &&
        My_Judge.org_info->game_robot_status.chassis_power_limit > 0U) {
        return (float)My_Judge.org_info->game_robot_status.chassis_power_limit
               * CHASSIS_POWER_BUDGET_MARGIN;
    }

    /* 裁判离线 / 上限为 0：退回训练基准 */
    return CHASSIS_POWER_BUDGET_W;
}

/**
  * @brief  取发给超电板的底盘缓冲能量
  */
float Power_Budget_Buffer(void)
{
    if (My_Judge.status->status == DEV_ONLINE) {
        return (float)My_Judge.org_info->power_heat_data.chassis_power_buffer;
    }

    /* 无裁判时给名义满值，免得电容板误判底盘能量见底 */
    return (float)CAP_TX_BUFFER_NOMINAL;
}
