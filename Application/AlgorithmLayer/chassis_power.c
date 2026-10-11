#include "chassis_power.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

ChassisPower_t chassis_power;

static struct {
    bool has_limit, seen_buffer, seen_cap, seen_update;
    uint32_t limit_seq, buffer_seq, cap_seq;
    uint32_t buffer_ms, cap_ms, update_ms;
    float buffer_j;
} history;

static float Clamp(float x, float lo, float hi) {
    return fminf(hi, fmaxf(lo, x));
}

/* 长时间失联不能按失联时长一次性释放降额，恢复新帧最多计 100ms。 */
static float Sample_Dt(uint32_t now, uint32_t before, bool seen) {
    return seen ? fminf((float)(uint32_t)(now - before) * .001f, .1f) : .1f;
}

static bool Nonnegative(float x) {
    return isfinite(x) && x >= 0.0f;
}

static bool Budget_Config_Valid(void) {
    return Nonnegative(chassis_power.training_limit_w) &&
           chassis_power.training_limit_w > 0.0f &&
           Nonnegative(chassis_power.margin_w) &&
           Nonnegative(chassis_power.buffer_target_j) &&
           Nonnegative(chassis_power.buffer_kp) &&
           Nonnegative(chassis_power.buffer_recovery_w_per_s) &&
           Nonnegative(chassis_power.measured_down_gain) &&
           Nonnegative(chassis_power.measured_recovery_w_per_s) &&
           Nonnegative(chassis_power.measured_hysteresis_w) &&
           Nonnegative(chassis_power.budget_recovery_w_per_s) &&
           Nonnegative(chassis_power.buffer_drop_threshold_j);
}

void Chassis_Power_Clear_Output(void) {
    chassis_power.current_scale = 0.0f;
    chassis_power.predicted_before_w = chassis_power.predicted_after_w = 0.0f;
    chassis_power.limiting_active = false;
    chassis_power.model_fault = chassis_power.budget_unreachable = false;
    memset(chassis_power.current_before, 0, sizeof(chassis_power.current_before));
    memset(chassis_power.current_after, 0, sizeof(chassis_power.current_after));
    memset(chassis_power.wheel_power_before_w, 0, sizeof(chassis_power.wheel_power_before_w));
    memset(chassis_power.wheel_power_after_w, 0, sizeof(chassis_power.wheel_power_after_w));
}

void Chassis_Power_Init(void) {
    memset(&chassis_power, 0, sizeof(chassis_power));
    memset(&history, 0, sizeof(history));
    chassis_power.training_limit_w = 80.0f;
    chassis_power.margin_w = 5.0f;
    chassis_power.buffer_target_j = 50.0f;
    chassis_power.buffer_kp = 1.0f;
    chassis_power.buffer_recovery_w_per_s = 10.0f;
    chassis_power.measured_down_gain = 1.0f;
    chassis_power.measured_recovery_w_per_s = 1.0f;
    chassis_power.measured_hysteresis_w = 2.0f;
    chassis_power.budget_recovery_w_per_s = 20.0f;
    chassis_power.buffer_drop_threshold_j = .5f;
    chassis_power.limit_timeout_ms = 500U;
    chassis_power.buffer_timeout_ms = 500U;
    chassis_power.cap_timeout_ms = 300U;
    chassis_power.k_w2 = 3.0e-7f;
    chassis_power.k_i2 = 1.23e-7f;
    chassis_power.k_iw = 1.99688994e-6f;
    chassis_power.k_const = 4.081f;
    chassis_power.base_budget_w = chassis_power.target_power_w = 75.0f;
    chassis_power.fallback_active = true;
}

void Chassis_Power_Update(const ChassisPowerInput_t *input, uint32_t now_ms) {
    if (input == NULL) return;
    float dt = Sample_Dt(now_ms, history.update_ms, history.seen_update);
    history.update_ms = now_ms;
    history.seen_update = true;
    chassis_power.limit_age_ms = input->limit_received ? now_ms - input->limit_ms : UINT32_MAX;
    chassis_power.buffer_age_ms = input->buffer_received ? now_ms - input->buffer_ms : UINT32_MAX;
    chassis_power.cap_age_ms = input->cap_received ? now_ms - input->cap_ms : UINT32_MAX;

    bool usable_limit = input->limit_received && isfinite(input->limit_w) && input->limit_w > 0.0f;
    /* 接收层已校验的历史上限，即使控制任务首次看到时已过期也保留。 */
    if (usable_limit && (!history.has_limit || history.limit_seq != input->limit_seq)) {
        history.has_limit = true;
        history.limit_seq = input->limit_seq;
        chassis_power.last_valid_limit_w = chassis_power.judge_limit_w = input->limit_w;
    }
    chassis_power.judge_limit_valid = usable_limit &&
        chassis_power.limit_age_ms <= chassis_power.limit_timeout_ms;
    chassis_power.buffer_valid = input->buffer_received && Nonnegative(input->buffer_j) &&
        chassis_power.buffer_age_ms <= chassis_power.buffer_timeout_ms;
    chassis_power.cap_valid = input->cap_received && isfinite(input->cap_w) &&
        chassis_power.cap_age_ms <= chassis_power.cap_timeout_ms;
    chassis_power.fallback_active = !chassis_power.judge_limit_valid;
    if (!chassis_power.buffer_valid) chassis_power.buffer_drop_detected = false;
    chassis_power.config_fault = !Budget_Config_Valid();
    if (chassis_power.config_fault) {
        chassis_power.target_power_w = 0.0f;
        return;
    }
    float limit = history.has_limit ? chassis_power.last_valid_limit_w : chassis_power.training_limit_w;
    chassis_power.base_budget_w = fmaxf(0.0f, limit - chassis_power.margin_w);

    if (chassis_power.buffer_valid &&
        (!history.seen_buffer || history.buffer_seq != input->buffer_seq)) {
        float sample_dt = Sample_Dt(input->buffer_ms, history.buffer_ms, history.seen_buffer);
        uint32_t gap = input->buffer_ms - history.buffer_ms;
        /* 保留最近一次新帧的下降量，避免 2ms 清零导致 Watch 看不到。 */
        chassis_power.buffer_drop_detected = false;
        chassis_power.buffer_drop_j = chassis_power.buffer_drop_rate_w = 0.0f;
        if (history.seen_buffer && gap > 0U && gap <= chassis_power.buffer_timeout_ms) {
            chassis_power.buffer_drop_j = fmaxf(0.0f, history.buffer_j - input->buffer_j);
            chassis_power.buffer_drop_rate_w = chassis_power.buffer_drop_j * 1000.0f / (float)gap;
            chassis_power.buffer_drop_detected =
                chassis_power.buffer_drop_j > chassis_power.buffer_drop_threshold_j;
        }
        chassis_power.buffer_energy_j = input->buffer_j;
        float desired = chassis_power.buffer_kp *
            fmaxf(0.0f, chassis_power.buffer_target_j - input->buffer_j);
        desired = fminf(desired, chassis_power.base_budget_w);
        if (desired >= chassis_power.buffer_derate_w) chassis_power.buffer_derate_w = desired;
        else chassis_power.buffer_derate_w = fmaxf(desired, chassis_power.buffer_derate_w -
            chassis_power.buffer_recovery_w_per_s * sample_dt);
        history.seen_buffer = true;
        history.buffer_seq = input->buffer_seq;
        history.buffer_ms = input->buffer_ms;
        history.buffer_j = input->buffer_j;
    }

    float nominal = fmaxf(0.0f, chassis_power.base_budget_w - chassis_power.buffer_derate_w);
    if (chassis_power.cap_valid && (!history.seen_cap || history.cap_seq != input->cap_seq)) {
        float sample_dt = Sample_Dt(input->cap_ms, history.cap_ms, history.seen_cap);
        chassis_power.measured_power_w = input->cap_w;
        /* 只在新实测帧上修正；下降快，恢复慢。参考点不随自身降额下降。 */
        if (input->cap_w > nominal) chassis_power.measured_derate_w +=
            chassis_power.measured_down_gain * (input->cap_w - nominal);
        else if (input->cap_w < nominal - chassis_power.measured_hysteresis_w)
            chassis_power.measured_derate_w = fmaxf(0.0f, chassis_power.measured_derate_w -
                chassis_power.measured_recovery_w_per_s * sample_dt);
        chassis_power.measured_derate_w = Clamp(chassis_power.measured_derate_w, 0.0f,
                                               chassis_power.base_budget_w);
        history.seen_cap = true;
        history.cap_seq = input->cap_seq;
        history.cap_ms = input->cap_ms;
    }
    float desired_budget = fmaxf(0.0f, nominal - chassis_power.measured_derate_w);
    if (desired_budget <= chassis_power.target_power_w) chassis_power.target_power_w = desired_budget;
    else chassis_power.target_power_w = fminf(desired_budget, chassis_power.target_power_w +
                                             chassis_power.budget_recovery_w_per_s * dt);
}

static bool Model_Valid(void) {
    return Nonnegative(chassis_power.k_i2) && Nonnegative(chassis_power.k_iw) &&
           Nonnegative(chassis_power.k_w2) && Nonnegative(chassis_power.k_const) &&
           Nonnegative(chassis_power.target_power_w);
}

static float Can_Current(float current) {
    return (float)(int16_t)Clamp(current, -CHASSIS_POWER_RAW_CURRENT_MAX,
                                         CHASSIS_POWER_RAW_CURRENT_MAX);
}

static float Predict(const float original[4], const float rpm[4], float scale,
                     float wheel_power[4]) {
    float sum = 0.0f;
    for (unsigned i = 0; i < CHASSIS_POWER_WHEEL_COUNT; ++i) {
        float current = Can_Current(original[i] * scale);
        float power = chassis_power.k_i2 * current * current +
                      chassis_power.k_iw * current * rpm[i] +
                      chassis_power.k_w2 * rpm[i] * rpm[i] + chassis_power.k_const;
        if (!isfinite(power)) return NAN;
        power = fmaxf(0.0f, power); /* 制动负功率不抵扣其他轮耗电 */
        if (wheel_power != NULL) wheel_power[i] = power;
        sum += power;
    }
    return sum;
}

void Chassis_Power_Limit(float current[4], const float rpm[4]) {
    if (current == NULL || rpm == NULL) return;
    Chassis_Power_Clear_Output();
    memcpy(chassis_power.current_before, current, sizeof(chassis_power.current_before));
    bool valid = Model_Valid() && !chassis_power.config_fault;
    for (unsigned i = 0; i < CHASSIS_POWER_WHEEL_COUNT; ++i)
        valid = valid && isfinite(current[i]) && isfinite(rpm[i]);
    if (!valid) {
        chassis_power.model_fault = true;
        chassis_power.limiting_active = true;
        memset(current, 0, sizeof(chassis_power.current_after));
        return;
    }
    chassis_power.predicted_before_w = Predict(current, rpm, 1.0f, chassis_power.wheel_power_before_w);
    float baseline = Predict(current, rpm, 0.0f, NULL);
    if (!isfinite(chassis_power.predicted_before_w) || !isfinite(baseline)) {
        chassis_power.model_fault = true;
        chassis_power.limiting_active = true;
        memset(current, 0, sizeof(chassis_power.current_after));
        return;
    }
    float scale = 1.0f;
    if (chassis_power.predicted_before_w > chassis_power.target_power_w) {
        if (baseline > chassis_power.target_power_w) {
            /* 转速损耗已超预算：不存在从零开始的可行驱动额度，撤去驱动。 */
            scale = 0.0f;
            chassis_power.budget_unreachable = true;
        } else {
            float lower = 0.0f, upper = 1.0f;
            for (unsigned step = 0; step < 16U; ++step) {
                float middle = .5f * (lower + upper);
                float predicted = Predict(current, rpm, middle, NULL);
                if (isfinite(predicted) && predicted <= chassis_power.target_power_w) lower = middle;
                else upper = middle;
            }
            scale = lower; /* 始终保存已经实际校验过的可行端点 */
        }
    }
    chassis_power.current_scale = scale;
    chassis_power.limiting_active = scale < 1.0f;
    for (unsigned i = 0; i < CHASSIS_POWER_WHEEL_COUNT; ++i) current[i] = Can_Current(current[i] * scale);
    chassis_power.predicted_after_w = Predict(current, rpm, 1.0f, chassis_power.wheel_power_after_w);
    /* 量化后的最终复核；异常时不能放行原 PID 大电流。 */
    if (!isfinite(chassis_power.predicted_after_w) ||
        (!chassis_power.budget_unreachable && chassis_power.predicted_after_w > chassis_power.target_power_w)) {
        memset(current, 0, sizeof(chassis_power.current_after));
        chassis_power.current_scale = 0.0f;
        chassis_power.limiting_active = true;
        chassis_power.model_fault = true;
        chassis_power.predicted_after_w = Predict(current, rpm, 1.0f, chassis_power.wheel_power_after_w);
    }
    memcpy(chassis_power.current_after, current, sizeof(chassis_power.current_after));
}
