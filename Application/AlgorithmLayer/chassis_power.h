#ifndef CHASSIS_POWER_H
#define CHASSIS_POWER_H

#include <stdbool.h>
#include <stdint.h>

#define CHASSIS_POWER_WHEEL_COUNT 4U
#define CHASSIS_POWER_RAW_CURRENT_MAX 16384.0f

/* 接收层快照：序号只在收到有效新帧时变化，时间单位 ms。 */
typedef struct {
    bool limit_received;
    uint32_t limit_seq, limit_ms;
    float limit_w;
    bool buffer_received;
    uint32_t buffer_seq, buffer_ms;
    float buffer_j;
    bool cap_received;
    uint32_t cap_seq, cap_ms;
    float cap_w;
} ChassisPowerInput_t;

typedef struct {
    /* Watch 可调参数；功率单位 W，能量 J，转速 RPM，电流 CAN raw。 */
    float training_limit_w;
    float margin_w;
    float buffer_target_j;
    float buffer_kp;                   /* W/J，只产生降额 */
    float buffer_recovery_w_per_s;
    float measured_down_gain;          /* 每个新帧的超额功率修正系数 */
    float measured_recovery_w_per_s;
    float measured_hysteresis_w;
    float budget_recovery_w_per_s;
    float buffer_drop_threshold_j;
    uint32_t limit_timeout_ms, buffer_timeout_ms, cap_timeout_ms;
    float k_i2, k_iw, k_w2, k_const;    /* 参考模型起点，需要本车实测标定 */

    /* 以下为自动更新的观察量，不是控制开关。 */
    float judge_limit_w, last_valid_limit_w;
    float base_budget_w, target_power_w;
    float buffer_energy_j, buffer_derate_w;
    float buffer_drop_j, buffer_drop_rate_w;
    float measured_power_w, measured_derate_w;
    float predicted_before_w, predicted_after_w;
    float wheel_power_before_w[4], wheel_power_after_w[4];
    float current_before[4], current_after[4];
    float current_scale;
    uint32_t limit_age_ms, buffer_age_ms, cap_age_ms;
    bool judge_limit_valid, buffer_valid, cap_valid;
    bool fallback_active, buffer_drop_detected;
    bool limiting_active, model_fault, budget_unreachable, config_fault;
} ChassisPower_t;

extern ChassisPower_t chassis_power;

void Chassis_Power_Init(void);
void Chassis_Power_Update(const ChassisPowerInput_t *input, uint32_t now_ms);
/* 就地限制电流：共同缩放，不增大、不反向；结果为实际 CAN 整数电流。 */
void Chassis_Power_Limit(float current[4], const float rpm[4]);
/* 停车时只清输出观察量，保留调参值、最近有效上限和既有降额。 */
void Chassis_Power_Clear_Output(void);
/* STM32 适配层：快照裁判/超电/电机数据，限制后写回 tx_info->torque。 */
void Chassis_Power_Apply(void);

#endif
