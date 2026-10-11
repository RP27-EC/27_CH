#ifndef CHASSIS_POWER_H
#define CHASSIS_POWER_H

#include <stdbool.h>
#include <stdint.h>

#define CHASSIS_POWER_WHEEL_COUNT 4U
#define CHASSIS_POWER_RAW_CURRENT_MAX 16384.0f /* CAN 原始电流指令上限，非安培。 */

typedef enum {
    CHASSIS_POWER_LIMIT_NONE = 0, /* 无可用上限，预算为零。 */
    CHASSIS_POWER_LIMIT_JUDGE,
    CHASSIS_POWER_LIMIT_HISTORY, /* 裁判上限过期，沿用最近有效值。 */
    CHASSIS_POWER_LIMIT_TRAINING /* 无历史上限时使用显式训练额度。 */
} ChassisPowerLimitSource_t;

typedef struct {
    /* 遥测快照：seq 仅随有效新帧变化，ms 为接收时间。 */
    bool limit_received;
    uint32_t limit_seq, limit_ms;
    float limit_w; /* 上限必须大于零；0 J / 0 W 是有效缓冲与超电反馈。 */
    bool buffer_received;
    uint32_t buffer_seq, buffer_ms;
    float buffer_j;
    bool cap_received;
    uint32_t cap_seq, cap_ms;
    float cap_w;
} ChassisPowerInput_t;

typedef struct {
    /* 可调参数：功率 W，能量 J，时间 ms，转速 RPM，电流 CAN raw。 */
    float training_limit_w; /* 默认 0 禁用；正值启用训练回退，不覆盖裁判上限。 */
    float margin_w;
    float buffer_target_j;
    float buffer_kp; /* W/J；缓冲充足时也不增加基础预算。 */
    float buffer_recovery_w_per_s;
    float measured_warning_margin_w;
    float measured_soft_kp; /* W/W，警戒区的非累积修正。 */
    float measured_soft_max_w;
    float measured_integral_gain_per_s; /* 1/s，按新帧实际间隔积累超额偏差。 */
    float measured_recovery_w_per_s;
    float measured_hysteresis_w;
    float budget_recovery_w_per_s; /* 限制预算回升速度；下降立即生效。 */
    float buffer_drop_threshold_j; /* 仅用于观察提示，不直接限流。 */
    uint32_t limit_timeout_ms, buffer_timeout_ms, cap_timeout_ms;
    uint32_t recovery_hold_ms; /* 三路反馈有效、功率有余量且缓冲满或上升的持续确认时间。 */
    float k_i2, k_iw, k_w2, k_const; /* 每轮 I²、I×RPM、RPM²、常数项系数；I 为 CAN raw，需实车标定。 */

    /* 自动更新的状态与观察量。 */
    float judge_limit_w, last_valid_limit_w; /* 历史有效上限在反馈过期后仍保留。 */
    float base_budget_w, target_power_w; /* 基础预算、反馈修正与回升限速后的最终预算。 */
    float buffer_energy_j, buffer_derate_w;
    float buffer_drop_j, buffer_drop_rate_w; /* 相邻新帧的能量下降量及下降率，仅观察。 */
    float measured_power_w, measured_derate_w;
    float measured_warning_w;
    float measured_soft_derate_w, measured_bias_derate_w; /* 提前修正、累积偏差修正。 */
    float feedback_derate_w; /* max(缓冲降额, 实测降额)，避免重复扣减。 */
    float predicted_before_w, predicted_after_w; /* 限流前/后的模型预测总功率，非实测值。 */
    float wheel_power_before_w[4], wheel_power_after_w[4]; /* 顺序与电机数组一致。 */
    float current_before[4], current_after[4]; /* CAN raw；after 已按实际发送值整数化。 */
    float current_scale; /* 共同缩放系数，范围 0~1。 */
    uint32_t limit_age_ms, buffer_age_ms, cap_age_ms; /* 未收到时为 UINT32_MAX。 */
    bool judge_limit_valid, buffer_valid, cap_valid;
    bool fallback_active, buffer_drop_detected;
    bool buffer_recovering, recovery_ready;
    uint32_t recovery_stable_ms;
    ChassisPowerLimitSource_t limit_source;
    bool limiting_active, model_fault, budget_unreachable, config_fault; /* 限流、模型异常、零电流仍超预算、配置异常。 */
} ChassisPower_t;

extern ChassisPower_t chassis_power;

void Chassis_Power_Init(void);
void Chassis_Power_Update(const ChassisPowerInput_t *input, uint32_t now_ms);

void Chassis_Power_Limit(float current[4], const float rpm[4]);

void Chassis_Power_Clear_Output(void);

void Chassis_Power_Apply(void);

#endif
