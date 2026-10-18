#ifndef CHASSIS_POWER_H
#define CHASSIS_POWER_H

#include <stdbool.h>
#include <stdint.h>

#define CHASSIS_POWER_WHEEL_COUNT 4U
#define CHASSIS_POWER_RAW_CURRENT_MAX 16384.0f /* CAN raw，非安培。 */

typedef enum {
    CHASSIS_POWER_LIMIT_NONE = 0,
    CHASSIS_POWER_LIMIT_JUDGE,
    CHASSIS_POWER_LIMIT_HISTORY,
    CHASSIS_POWER_LIMIT_TRAINING
} ChassisPowerLimitSource_t;

typedef struct {
    /* 接收元数据来自设备，不在功率算法里伪造。 */
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
    /* 可调参数：预算=max(0,选定上限-margin_w)。 */
    float training_limit_w; /* 默认0；仅在从未收到有效裁判上限时使用。 */
    float margin_w;         /* 默认5 W。 */
    uint32_t limit_timeout_ms, buffer_timeout_ms, cap_timeout_ms;
    float k_i2, k_iw, k_w2, k_const;

    /* 预算和原始反馈。缓冲/超电只观察，不参与降额或恢复判定。 */
    float judge_limit_w, last_valid_limit_w;
    float base_budget_w, target_power_w;
    float buffer_energy_j, measured_power_w;
    uint32_t limit_age_ms, buffer_age_ms, cap_age_ms;
    bool judge_limit_valid, buffer_valid, cap_valid, fallback_active;
    ChassisPowerLimitSource_t limit_source;

    /* 四轮顺序：前左、前右、后左、后右。 */
    float predicted_before_w, predicted_after_w;
    float wheel_power_before_w[4], wheel_power_after_w[4];
    float current_before[4], current_after[4]; /* CAN raw，after为最终整数指令。 */
    float target_rpm[4], actual_rpm[4];        /* 模型使用实际RPM；目标仅供观察。 */
    float current_scale;
    bool limiting_active, model_fault, budget_unreachable, config_fault;
} ChassisPower_t;

extern ChassisPower_t chassis_power;

void Chassis_Power_Init(void);
void Chassis_Power_Update(const ChassisPowerInput_t *input, uint32_t now_ms);
void Chassis_Power_Limit(float current[4], const float rpm[4]);
void Chassis_Power_Clear_Output(void);
void Chassis_Power_Apply(void);

#endif
