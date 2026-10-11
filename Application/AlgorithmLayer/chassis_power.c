/* 功率限制算法阅读入口：
 * 1. Apply（adapter.c）取设备快照，调用 Update，再调用 Limit，最后写回电流。
 * 2. Update 只算允许多少 W，结果存在 chassis_power.target_power_w 中。
 * 3. Limit 读取这个预算，预测 PID 电流的功率，必要时共同缩小四轮电流。
 * 模型约束不等于实测每个瞬间都不超限；系数需要本车标定。
 */
/* 引入接口和数据类型；本文件只计算预算和限制电流，不发送 CAN。 */
#include "chassis_power.h"

/* 提供浮点限幅、有限数检查和 NAN。 */
#include <math.h>
/* 提供空指针常量 NULL。 */
#include <stddef.h>
/* 提供清零 memset 和复制 memcpy。 */
#include <string.h>

/* 全局对象：Update 写 target_power_w，Limit 从同一对象读它，两函数这样传递预算。 */
ChassisPower_t chassis_power;

/* 私有历史数据，用于识别新帧、保存上限、计算时间差，其他文件不能直接访问。 */
static struct {
    /* 依次记录是否保存过上限、处理过缓冲、处理过超电功率、执行过预算更新。 */
    bool has_limit, seen_buffer, seen_cap, seen_update;
    bool budget_initialized, recovery_timing; /* 已建立首个预算；联合恢复条件是否正在计时。 */
    /* 已处理的三种反馈序号；序号不变表示还是同一帧。 */
    uint32_t limit_seq, buffer_seq, cap_seq;
    /* 上次缓冲帧、功率帧和预算更新的时间，ms。 */
    uint32_t buffer_ms, cap_ms, update_ms;
    uint32_t recovery_ms; /* 本轮恢复确认的起点，ms，条件不满足时重新计时。 */
    /* 上次缓冲能量，J，用于求相邻帧下降量。 */
    float buffer_j;
/* 定义私有历史对象，初始化时清零。 */
} history;

/* 小工具：将 x 限制在 lo 到 hi 范围内。 */
static float Clamp(float x, float lo, float hi) {
    /* 先不低于下界，再不高于上界，返回限幅结果。 */
    return fminf(hi, fmaxf(lo, x));
}

/* 长时间失联不能按失联时长一次性释放降额，恢复新帧最多计 100ms。 */
/* 小工具：计算秒单位时间差，单次恢复最多按 0.1 s 计算。 */
static float Sample_Dt(uint32_t now, uint32_t before, bool seen) {
    /* 有历史则毫秒差乘 0.001 并封顶 0.1；首次取 0.1 s；无符号差值兼顾计时回绕。 */
    return seen ? fminf((float)(uint32_t)(now - before) * .001f, .1f) : .1f;
}

/* 小工具：检查一个数有限且非负。 */
static bool Nonnegative(float x) {
    /* 排除 NaN、无穷大和负数；零对多数参数是允许值。 */
    return isfinite(x) && x >= 0.0f;
}

/* 新反馈帧的真实间隔，s；首次、重复时间戳、跨失联的未知时间不积分。 */
static float Frame_Dt(uint32_t now, uint32_t before, bool seen, uint32_t timeout_ms) {
    uint32_t gap = now - before; /* 无符号相减兼顾 HAL 毫秒计时回绕。 */
    if (!seen || gap == 0U || gap > timeout_ms) return 0.0f; /* 无连续有效间隔就只重新建立基准。 */
    return (float)gap * .001f; /* 在有效期内按实际间隔计时，不能封顶导致低帧率少积分。 */
}

/* Watch 参数必须有限非负；训练额度允许为零，表示不凭空假设裁判上限。 */
static bool Budget_Config_Valid(void) {
    return Nonnegative(chassis_power.training_limit_w) && /* 默认零合法，正值是显式训练回退。 */
           Nonnegative(chassis_power.margin_w) && /* 裁判上限下方预留余量，W。 */
           Nonnegative(chassis_power.buffer_target_j) && /* 满缓冲目标，J，不是当前测量值。 */
           Nonnegative(chassis_power.buffer_kp) && /* 缓冲缺额换成降额的系数，W/J。 */
           Nonnegative(chassis_power.buffer_recovery_w_per_s) && /* 缓冲降额最大释放速度，W/s。 */
           Nonnegative(chassis_power.measured_warning_margin_w) && /* 提前干预区宽度，W。 */
           Nonnegative(chassis_power.measured_soft_kp) && /* 提前修正比例，W/W。 */
           Nonnegative(chassis_power.measured_soft_max_w) && /* 提前修正上限，W，不能无限累加。 */
           Nonnegative(chassis_power.measured_integral_gain_per_s) && /* 超额偏差积分增益，1/s。 */
           Nonnegative(chassis_power.measured_recovery_w_per_s) && /* 两项实测修正合计释放速度，W/s。 */
           Nonnegative(chassis_power.measured_hysteresis_w) && /* 低于警戒值多少才允许恢复，W。 */
           Nonnegative(chassis_power.budget_recovery_w_per_s) && /* 最终预算回升速度，W/s。 */
           Nonnegative(chassis_power.buffer_drop_threshold_j); /* 下降提示阈值，J，仅观察。 */
}

/* 只清本次限流观察量；保留参数、预算、历史降额，本函数不操作电机。 */
void Chassis_Power_Clear_Output(void) {
    /* 先清共同缩放系数记录，正常计算后会重新赋值。 */
    chassis_power.current_scale = 0.0f;
    /* 清上次限流前后总功率预测，W。 */
    chassis_power.predicted_before_w = chassis_power.predicted_after_w = 0.0f;
    /* 先清本次限流标志，后面重新计算。 */
    chassis_power.limiting_active = false;
    /* 清本次模型异常和不可达标志，不清预算配置异常标志。 */
    chassis_power.model_fault = chassis_power.budget_unreachable = false;
    /* 清四轮限流前电流的观察副本。 */
    memset(chassis_power.current_before, 0, sizeof(chassis_power.current_before));
    /* 清四轮限流后电流的观察副本。 */
    memset(chassis_power.current_after, 0, sizeof(chassis_power.current_after));
    /* 清四轮限流前预测功率记录。 */
    memset(chassis_power.wheel_power_before_w, 0, sizeof(chassis_power.wheel_power_before_w));
    /* 清四轮限流后预测功率记录。 */
    memset(chassis_power.wheel_power_after_w, 0, sizeof(chassis_power.wheel_power_after_w));
}

/* 上电初始化：尚未收到裁判上限时预算为零；参数与原始接收数据分开存储。 */
void Chassis_Power_Init(void) {
    memset(&chassis_power, 0, sizeof(chassis_power)); /* 清算法参数/观察量，不修改裁判或超电结构体。 */
    memset(&history, 0, sizeof(history)); /* 清序号、时间和恢复计时，表示尚无有效历史。 */
    chassis_power.training_limit_w = 0.0f; /* 默认禁用训练额度；离线调试需用户显式改为正值。 */
    chassis_power.margin_w = 5.0f; /* 例如裁判实收 80 W，基础预算为 75 W。 */
    chassis_power.buffer_target_j = 60.0f; /* 希望保持满缓冲；实际能量始终从裁判读取。 */
    chassis_power.buffer_kp = 1.0f; /* 缓冲从 60 掉到 59 J，就要求降额 1 W。 */
    chassis_power.buffer_recovery_w_per_s = 10.0f; /* 联合条件允许后，缓冲降额每秒最多释放 10 W。 */
    chassis_power.measured_warning_margin_w = 2.0f; /* 基础预算 75 W 时从警戒值 73 W 起干预。 */
    chassis_power.measured_soft_kp = 1.0f; /* 实测高于警戒值 1 W，提前修正目标增加 1 W。 */
    chassis_power.measured_soft_max_w = 5.0f; /* 提前修正目标最多 5 W，不随帧数无限累加。 */
    chassis_power.measured_integral_gain_per_s = 1.0f; /* 持续超基础预算时，按超额 W 乘实际秒数增长偏差。 */
    chassis_power.measured_recovery_w_per_s = 1.0f; /* 提前项和偏差项共用每秒 1 W 的释放额度。 */
    chassis_power.measured_hysteresis_w = 2.0f; /* 实测需低于警戒值减 2 W 才开始恢复确认。 */
    chassis_power.budget_recovery_w_per_s = 20.0f; /* 后续最终预算上升最多 20 W/s；下降立即生效。 */
    chassis_power.buffer_drop_threshold_j = .5f; /* 缓冲下降超过 0.5 J 时提示，不用单帧下降率直接限流。 */
    chassis_power.limit_timeout_ms = 500U; /* 裁判上限有效期，过期保存最近真实接收值。 */
    chassis_power.buffer_timeout_ms = 500U; /* 缓冲有效期，过期冻结已有降额。 */
    chassis_power.cap_timeout_ms = 300U; /* 超电功率有效期，过期冻结已有实测修正。 */
    chassis_power.recovery_hold_ms = 500U; /* 三路反馈满足条件持续 500 ms 才放开。 */
    chassis_power.k_w2 = 3.0e-7f; /* 每轮 RPM 平方项系数，需本车实测标定。 */
    chassis_power.k_i2 = 1.23e-7f; /* 每轮 CAN 原始电流平方项系数，原始值不是 A。 */
    chassis_power.k_iw = 1.99688994e-6f; /* 每轮原始电流乘实际 RPM 项系数。 */
    chassis_power.k_const = 4.081f; /* 每轮基础损耗项，W；四轮分别计入。 */
    chassis_power.limit_source = CHASSIS_POWER_LIMIT_NONE; /* Watch 明示尚无真实上限或显式训练额度。 */
    chassis_power.fallback_active = true; /* 尚无新鲜裁判上限，不把初始化零当作真实反馈。 */
}

/* 步骤一：只计算允许多少 W，写全局 target_power_w；Apply/Limit 负责电流与发送。 */
void Chassis_Power_Update(const ChassisPowerInput_t *input, uint32_t now_ms) {
    if (input == NULL) return; /* 没有快照时不能访问指针；正常 Apply 始终提供快照。 */
    float dt = Sample_Dt(now_ms, history.update_ms, history.seen_update); /* 控制更新间隔，s，回升最多按 0.1 s。 */
    history.update_ms = now_ms; /* 保存本次时间给下一控制拍使用。 */
    history.seen_update = true; /* 此后存在有效的更新时间基准。 */
    chassis_power.limit_age_ms = input->limit_received ? now_ms - input->limit_ms : UINT32_MAX; /* 上限独立年龄。 */
    chassis_power.buffer_age_ms = input->buffer_received ? now_ms - input->buffer_ms : UINT32_MAX; /* 缓冲独立年龄。 */
    chassis_power.cap_age_ms = input->cap_received ? now_ms - input->cap_ms : UINT32_MAX; /* 功率独立年龄，均为 ms。 */

    bool usable_limit = input->limit_received && isfinite(input->limit_w) && input->limit_w > 0.0f; /* 零不是可用上限。 */
    if (usable_limit && (!history.has_limit || history.limit_seq != input->limit_seq)) { /* 只接纳新的可用上限帧。 */
        history.has_limit = true; /* 一旦有真实上限，后续不能用训练额度覆盖它。 */
        history.limit_seq = input->limit_seq; /* 保存已处理接收序号。 */
        chassis_power.last_valid_limit_w = chassis_power.judge_limit_w = input->limit_w; /* 只复制实收值，不伪造反馈。 */
    }
    chassis_power.judge_limit_valid = usable_limit && chassis_power.limit_age_ms <= chassis_power.limit_timeout_ms; /* 当前上限新鲜。 */
    chassis_power.buffer_valid = input->buffer_received && Nonnegative(input->buffer_j) &&
        chassis_power.buffer_age_ms <= chassis_power.buffer_timeout_ms; /* 当前缓冲有限、非负且新鲜；0 J 有效。 */
    chassis_power.cap_valid = input->cap_received && isfinite(input->cap_w) &&
        chassis_power.cap_age_ms <= chassis_power.cap_timeout_ms; /* 当前实测有限且新鲜；0 W 有效。 */
    chassis_power.fallback_active = !chassis_power.judge_limit_valid; /* 历史/训练/无额度都不是当前新鲜裁判上限。 */
    if (!chassis_power.buffer_valid) { /* 缓冲失联时不拿旧趋势允许恢复。 */
        chassis_power.buffer_drop_detected = false; /* 清过期提示，不清已有缓冲降额。 */
        chassis_power.buffer_recovering = false; /* 上升趋势必须由有效连续新帧证明。 */
    }
    chassis_power.config_fault = !Budget_Config_Valid(); /* Watch 参数非法时由 Limit 撤去驱动。 */
    if (chassis_power.config_fault) { /* 不让非法参数放行 PID 大电流。 */
        chassis_power.target_power_w = 0.0f; /* 将允许功率清零。 */
        chassis_power.recovery_ready = false; /* 异常期间禁止恢复。 */
        chassis_power.recovery_stable_ms = 0U; /* 清本轮确认时长。 */
        history.recovery_timing = false; /* 参数恢复后重新确认，不能沿用异常前计时。 */
        return; /* 结束本次预算计算。 */
    }

    float limit = 0.0f; /* 默认没有允许上限，不能凭空设成 80 W。 */
    if (history.has_limit) { /* 已收到过真实可用上限。 */
        limit = chassis_power.last_valid_limit_w; /* 过期仍用最近实收值，避免回退到更大的训练上限。 */
        chassis_power.limit_source = chassis_power.judge_limit_valid ? CHASSIS_POWER_LIMIT_JUDGE : CHASSIS_POWER_LIMIT_HISTORY; /* 标明来源。 */
    } else if (chassis_power.training_limit_w > 0.0f) { /* 只在用户显式设置正训练额度时启用。 */
        limit = chassis_power.training_limit_w; /* 训练参数不写入 judge_limit_w 或裁判接收结构体。 */
        chassis_power.limit_source = CHASSIS_POWER_LIMIT_TRAINING; /* Watch 明示当前是离线训练。 */
    } else { /* 无裁判值也无训练授权额度。 */
        chassis_power.limit_source = CHASSIS_POWER_LIMIT_NONE; /* 后续预算为零。 */
    }
    chassis_power.base_budget_w = fmaxf(0.0f, limit - chassis_power.margin_w); /* 基础预算永远不超过依据的上限。 */
    chassis_power.measured_warning_w = fmaxf(0.0f, chassis_power.base_budget_w - chassis_power.measured_warning_margin_w); /* 提前警戒值。 */

    bool new_buffer = chassis_power.buffer_valid && (!history.seen_buffer || history.buffer_seq != input->buffer_seq); /* 缓冲新帧。 */
    bool new_cap = chassis_power.cap_valid && (!history.seen_cap || history.cap_seq != input->cap_seq); /* 实测新帧。 */
    float buffer_dt = new_buffer ? Frame_Dt(input->buffer_ms, history.buffer_ms, history.seen_buffer, chassis_power.buffer_timeout_ms) : 0.0f; /* 缓冲连续间隔。 */
    float cap_dt = new_cap ? Frame_Dt(input->cap_ms, history.cap_ms, history.seen_cap, chassis_power.cap_timeout_ms) : 0.0f; /* 功率连续间隔。 */
    if ((new_buffer && buffer_dt == 0.0f) || (new_cap && cap_dt == 0.0f)) history.recovery_timing = false; /* 首帧/重连重新计时。 */

    if (new_buffer) { /* 下降率、趋势与历史只在有效新缓冲帧更新。 */
        chassis_power.buffer_drop_j = chassis_power.buffer_drop_rate_w = 0.0f; /* 新帧先清旧下降量/率，仅用于观察。 */
        chassis_power.buffer_drop_detected = false; /* 随本帧下降量重新计算提示。 */
        chassis_power.buffer_recovering = false; /* 未满且保持不变，不算正在恢复。 */
        if (buffer_dt > 0.0f) { /* 不对首次或跨失联帧推断连续变化。 */
            chassis_power.buffer_drop_j = fmaxf(0.0f, history.buffer_j - input->buffer_j); /* 相邻帧下降量，J。 */
            chassis_power.buffer_drop_rate_w = chassis_power.buffer_drop_j / buffer_dt; /* J/s=W，仅表示缓冲净下降率。 */
            chassis_power.buffer_drop_detected = chassis_power.buffer_drop_j > chassis_power.buffer_drop_threshold_j; /* 提示阈值。 */
            chassis_power.buffer_recovering = input->buffer_j > history.buffer_j; /* 相邻有效新帧确实上升。 */
        }
        chassis_power.buffer_energy_j = input->buffer_j; /* Watch 复制真实当前能量，不把目标 60 J 写进反馈。 */
        history.seen_buffer = true; /* 标记已有缓冲历史。 */
        history.buffer_seq = input->buffer_seq; /* 保存本帧序号，防止重复消费。 */
        history.buffer_ms = input->buffer_ms; /* 保存本帧接收时间。 */
        history.buffer_j = input->buffer_j; /* 保存本帧能量，供下一帧比较。 */
    }
    if (new_cap) { /* 只复制新的有效实测，不把预测功率当传感器反馈。 */
        chassis_power.measured_power_w = input->cap_w; /* 超电实收底盘功率，W。 */
        history.seen_cap = true; /* 标记已有实测历史。 */
        history.cap_seq = input->cap_seq; /* 保存本帧序号。 */
        history.cap_ms = input->cap_ms; /* 保存本帧接收时间。 */
    }

    bool buffer_safe = chassis_power.buffer_valid && (input->buffer_j >= chassis_power.buffer_target_j || chassis_power.buffer_recovering); /* 满或上升才支持恢复。 */
    bool recovery_conditions = chassis_power.judge_limit_valid && buffer_safe && chassis_power.cap_valid &&
        input->cap_w < chassis_power.measured_warning_w - chassis_power.measured_hysteresis_w; /* 三路新鲜且实测明显低于警戒。 */
    if (!recovery_conditions) { /* 缺反馈、缓冲掉落/停在未满值或实测偏高，都重新开始确认。 */
        history.recovery_timing = false; /* 不累计不满足条件的时长。 */
        chassis_power.recovery_stable_ms = 0U; /* Watch 显示确认尚未建立。 */
    } else { /* 三路条件同时满足。 */
        if (!history.recovery_timing) { /* 本轮首次满足条件。 */
            history.recovery_timing = true; /* 开始持续确认。 */
            history.recovery_ms = now_ms; /* 保存本轮起点，兼容无符号计时回绕。 */
        }
        chassis_power.recovery_stable_ms = now_ms - history.recovery_ms; /* 仅连续维持的时间才计入。 */
    }
    chassis_power.recovery_ready = recovery_conditions && chassis_power.recovery_stable_ms >= chassis_power.recovery_hold_ms; /* 默认持续 500 ms。 */

    if (chassis_power.buffer_valid) { /* 缺额目标直接采用真实反馈；无新帧不重复累加。 */
        float desired_buffer = fminf(chassis_power.base_budget_w, chassis_power.buffer_kp *
            fmaxf(0.0f, chassis_power.buffer_target_j - input->buffer_j)); /* 默认 max(0,60-实收 J)，每缺 1 J 降 1 W。 */
        if (desired_buffer >= chassis_power.buffer_derate_w) chassis_power.buffer_derate_w = desired_buffer; /* 需要收紧立即采用。 */
        else if (new_buffer && chassis_power.recovery_ready && buffer_dt > 0.0f) { /* 放开需联合确认和连续新帧。 */
            chassis_power.buffer_derate_w = fmaxf(desired_buffer, chassis_power.buffer_derate_w -
                chassis_power.buffer_recovery_w_per_s * fminf(buffer_dt, .1f)); /* 每帧最多按 0.1 s 释放，避免长间隔突然放开。 */
        }
    }

    if (chassis_power.cap_valid) { /* 提前目标可随真实反馈/基础预算重算，但不会重复积分。 */
        float desired_soft = Clamp(chassis_power.measured_soft_kp * fmaxf(0.0f, input->cap_w - chassis_power.measured_warning_w),
            0.0f, fminf(chassis_power.measured_soft_max_w, chassis_power.base_budget_w)); /* 比例提前修正，默认封顶 5 W。 */
        if (desired_soft >= chassis_power.measured_soft_derate_w) chassis_power.measured_soft_derate_w = desired_soft; /* 更强修正立即采用。 */
        if (new_cap && cap_dt > 0.0f) { /* 只有相邻连续有效新帧才允许积分或释放。 */
            if (input->cap_w > chassis_power.base_budget_w) { /* 只有实际超基础预算才累计偏差，警戒区不会无限扣减。 */
                chassis_power.measured_bias_derate_w += chassis_power.measured_integral_gain_per_s *
                    (input->cap_w - chassis_power.base_budget_w) * cap_dt; /* 超额 W 乘真实秒数，帧频不改变同时间修正。 */
            } else if (chassis_power.recovery_ready) { /* 功率有余量且缓冲证明恢复，才释放已有实测修正。 */
                float release = chassis_power.measured_recovery_w_per_s * fminf(cap_dt, .1f); /* 两项共同使用本帧恢复额度。 */
                float soft_release = fminf(release, fmaxf(0.0f, chassis_power.measured_soft_derate_w - desired_soft)); /* 先释放提前项多余部分。 */
                chassis_power.measured_soft_derate_w -= soft_release; /* 不低于当前比例修正目标。 */
                release -= soft_release; /* 剩余额度再给偏差项，避免两项各释放一次造成翻倍。 */
                chassis_power.measured_bias_derate_w = fmaxf(0.0f, chassis_power.measured_bias_derate_w - release); /* 偏差项最低为零。 */
            }
        }
    }
    chassis_power.measured_soft_derate_w = Clamp(chassis_power.measured_soft_derate_w, 0.0f, chassis_power.base_budget_w); /* 上限降低时同步夹紧。 */
    chassis_power.measured_bias_derate_w = Clamp(chassis_power.measured_bias_derate_w, 0.0f, chassis_power.base_budget_w); /* 偏差不超基础预算。 */
    chassis_power.measured_derate_w = fminf(chassis_power.base_budget_w,
        chassis_power.measured_soft_derate_w + chassis_power.measured_bias_derate_w); /* 实测两项合计，W。 */
    chassis_power.feedback_derate_w = fminf(chassis_power.base_budget_w,
        fmaxf(chassis_power.buffer_derate_w, chassis_power.measured_derate_w)); /* 取较强反馈，不将同次问题完整扣两遍。 */
    float desired_budget = fmaxf(0.0f, chassis_power.base_budget_w - chassis_power.feedback_derate_w); /* 最终目标不超过基础预算。 */
    if (!history.budget_initialized && chassis_power.base_budget_w > 0.0f) { /* 首次获得真实或显式训练额度时建立初始预算。 */
        chassis_power.target_power_w = desired_budget; /* 按当前反馈修正建立，不凭空采用 75 W。 */
        history.budget_initialized = true; /* 后续上涨均要走确认与限速。 */
    } else if (desired_budget <= chassis_power.target_power_w) { /* 上限降低或反馈要求收紧。 */
        chassis_power.target_power_w = desired_budget; /* 下降立即执行。 */
    } else if (chassis_power.recovery_ready) { /* 上升需联合反馈确认，过期不自动放开。 */
        chassis_power.target_power_w = fminf(desired_budget, chassis_power.target_power_w +
            chassis_power.budget_recovery_w_per_s * dt); /* 最终预算每秒最多回升 20 W，不超过本次目标。 */
    }
}

/* 检查功率模型系数与最终预算，输入电流/RPM 在 Limit 中另查。 */
static bool Model_Valid(void) {
    /* 电流平方和电流乘 RPM 系数需有限非负；I、RPM 本身允许带符号。 */
    return Nonnegative(chassis_power.k_i2) && Nonnegative(chassis_power.k_iw) &&
           /* 转速平方和常数项系数需有限非负。 */
           Nonnegative(chassis_power.k_w2) && Nonnegative(chassis_power.k_const) &&
           /* 最终允许功率需有限非负。 */
           Nonnegative(chassis_power.target_power_w);
}

/* 按实际 CAN 指令格式限幅和整数化，保证模型考虑拟发送整数值。 */
static float Can_Current(float current) {
    /* 限幅后转 int16_t 截去小数，再转 float 方便功率计算。 */
    return (float)(int16_t)Clamp(current, -CHASSIS_POWER_RAW_CURRENT_MAX,
                                         /* 原始电流范围 -16384 到 +16384，不是安培。 */
                                         CHASSIS_POWER_RAW_CURRENT_MAX);
}

/* 用共同系数 scale 预测四轮功率，不修改原电流数组。 */
static float Predict(const float original[4], const float rpm[4], float scale,
                     /* 可选每轮功率输出数组，NULL 表示只要总功率。 */
                     float wheel_power[4]) {
    /* 四轮总功率从 0 W 累加。 */
    float sum = 0.0f;
    /* 遍历四轮，逐轮计算并累加功率。 */
    for (unsigned i = 0; i < CHASSIS_POWER_WHEEL_COUNT; ++i) {
        /* 原电流乘共同系数并整数化，保留正负号。 */
        float current = Can_Current(original[i] * scale);
        /* 第一项：电流平方相关损耗，近似铜耗。 */
        float power = chassis_power.k_i2 * current * current +
                      /* 第二项：电流乘实际 RPM，近似转换功率，带符号区分驱动和制动。 */
                      chassis_power.k_iw * current * rpm[i] +
                      /* 第三项为转速平方损耗，加每轮基础常数项，结果 W。 */
                      chassis_power.k_w2 * rpm[i] * rpm[i] + chassis_power.k_const;
        /* 预测非法或溢出返回 NAN，调用者负责保护清零。 */
        if (!isfinite(power)) return NAN;
        /* 单轮负功率按零算，不假设制动能完全抵消其他轮耗电。 */
        power = fmaxf(0.0f, power); /* 制动负功率不抵扣其他轮耗电 */
        /* 提供输出数组时保存本轮预测，方便观察。 */
        if (wheel_power != NULL) wheel_power[i] = power;
        /* 累加本轮非负预测功率。 */
        sum += power;
    }
    /* 返回四轮总预测功率，W，不是实测。 */
    return sum;
}

/* 步骤二：读全局 target_power_w，直接修改传入 current 数组，rpm 只读。 */
void Chassis_Power_Limit(float current[4], const float rpm[4]) {
    /* 数组指针为空就退出，防止无效访问。 */
    if (current == NULL || rpm == NULL) return;
    /* 清上一拍限流记录，保留 Update 已算的预算和降额。 */
    Chassis_Power_Clear_Output();
    /* 保存原 PID 电流供前后对比。 */
    memcpy(chassis_power.current_before, current, sizeof(chassis_power.current_before));
    /* 模型和预算配置都有效才放行正常计算。 */
    bool valid = Model_Valid() && !chassis_power.config_fault;
    /* 逐个检查四轮输入是否为有限数。 */
    for (unsigned i = 0; i < CHASSIS_POWER_WHEEL_COUNT; ++i)
        /* 检查电流与 RPM 数值；这里不独立检查电机反馈时间。 */
        valid = valid && isfinite(current[i]) && isfinite(rpm[i]);
    /* 非法配置或输入进入清零保护，不继续使用 PID 大电流。 */
    if (!valid) {
        /* 记录本次模型计算或输入异常，供 Watch 排查。 */
        chassis_power.model_fault = true;
        /* 标记本次执行保护性限流。 */
        chassis_power.limiting_active = true;
        /* 将传入四轮电流数组清零；Apply 随后写回，外层再发 CAN。 */
        memset(current, 0, sizeof(chassis_power.current_after));
        /* 结束本次函数，异常分支不继续执行正常计算。 */
        return;
    }
    /* scale=1，预测原电流总功率并保存每轮功率。 */
    chassis_power.predicted_before_w = Predict(current, rpm, 1.0f, chassis_power.wheel_power_before_w);
    /* scale=0，预测零驱动电流损耗；保留实际 RPM，车辆不等于已经停下。 */
    float baseline = Predict(current, rpm, 0.0f, NULL);
    /* 原电流或零电流预测非法时清零保护。 */
    if (!isfinite(chassis_power.predicted_before_w) || !isfinite(baseline)) {
        /* 记录本次模型计算或输入异常，供 Watch 排查。 */
        chassis_power.model_fault = true;
        /* 标记本次执行保护性限流。 */
        chassis_power.limiting_active = true;
        /* 将传入四轮电流数组清零；Apply 随后写回，外层再发 CAN。 */
        memset(current, 0, sizeof(chassis_power.current_after));
        /* 结束本次函数，异常分支不继续执行正常计算。 */
        return;
    }
    /* 默认不缩小，只有预测超预算才降低系数。 */
    float scale = 1.0f;
    /* 原四轮电流的预测总功率超最终预算，开始限流。 */
    if (chassis_power.predicted_before_w > chassis_power.target_power_w) {
        /* 零驱动预测仍超预算，采取撤去驱动策略，不搜索其他系数。 */
        if (baseline > chassis_power.target_power_w) {
            /* 转速损耗已超预算：不存在从零开始的可行驱动额度，撤去驱动。 */
            /* 四轮共同系数为零，不代表车或实际功率瞬间归零。 */
            scale = 0.0f;
            /* 记录零驱动模型仍超预算，区别于非法模型和通信超时。 */
            chassis_power.budget_unreachable = true;
        /* 零驱动预测可行，进入 0~1 的共同系数搜索。 */
        } else {
            /* 初始下界 0 可行，上界 1 超预算，目标尽量保留电流。 */
            float lower = 0.0f, upper = 1.0f;
            /* 固定 16 次二分，每次缩小一半系数区间。 */
            for (unsigned step = 0; step < 16U; ++step) {
                /* 当前区间中点作为候选系数。 */
                float middle = .5f * (lower + upper);
                /* 按候选系数整数化并预测四轮功率，不修改原数组。 */
                float predicted = Predict(current, rpm, middle, NULL);
                /* 候选有限且不超预算就保存为可行下界，尝试更大电流。 */
                if (isfinite(predicted) && predicted <= chassis_power.target_power_w) lower = middle;
                /* 超预算或异常就降低上界，下次尝试更小系数。 */
                else upper = middle;
            }
            /* 最终采用已经校验可行的下界，不用可能超限的上界。 */
            scale = lower; /* 始终保存已经实际校验过的可行端点 */
        }
    }
    /* 保存共同系数，Watch 可看电流保留到几成。 */
    chassis_power.current_scale = scale;
    /* 正常路径只有系数小于 1 才标记缩小了电流。 */
    chassis_power.limiting_active = scale < 1.0f;
    /* 共同缩小四轮电流并整数化，不增大指令幅度、不反转符号。 */
    for (unsigned i = 0; i < CHASSIS_POWER_WHEEL_COUNT; ++i) current[i] = Can_Current(current[i] * scale);
    /* 用当前最终电流计算总功率和每轮功率；scale=1 避免重复缩小，保护清零后损耗也可能非零。 */
    chassis_power.predicted_after_w = Predict(current, rpm, 1.0f, chassis_power.wheel_power_after_w);
    /* 量化后的最终复核；异常时不能放行原 PID 大电流。 */
    /* 最终预测非法需要清零保护，下行还检查可达情况是否超预算。 */
    if (!isfinite(chassis_power.predicted_after_w) ||
        /* 可达路径最终整数功率不能超预算；不可达路径已撤驱动，保留残余损耗预测。 */
        (!chassis_power.budget_unreachable && chassis_power.predicted_after_w > chassis_power.target_power_w)) {
        /* 将传入四轮电流数组清零；Apply 随后写回，外层再发 CAN。 */
        memset(current, 0, sizeof(chassis_power.current_after));
        /* 最终复核失败，缩放系数记录设为零，表示已经撤去驱动电流。 */
        chassis_power.current_scale = 0.0f;
        /* 标记本次执行保护性限流。 */
        chassis_power.limiting_active = true;
        /* 记录本次模型计算或输入异常，供 Watch 排查。 */
        chassis_power.model_fault = true;
        /* 用当前最终电流计算总功率和每轮功率；scale=1 避免重复缩小，保护清零后损耗也可能非零。 */
        chassis_power.predicted_after_w = Predict(current, rpm, 1.0f, chassis_power.wheel_power_after_w);
    }
    /* 记录最终电流；传入数组已修改，Apply 会把它写回电机发送对象。 */
    memcpy(chassis_power.current_after, current, sizeof(chassis_power.current_after));
}
