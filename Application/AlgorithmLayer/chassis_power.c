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
    /* 已处理的三种反馈序号；序号不变表示还是同一帧。 */
    uint32_t limit_seq, buffer_seq, cap_seq;
    /* 上次缓冲帧、功率帧和预算更新的时间，ms。 */
    uint32_t buffer_ms, cap_ms, update_ms;
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

/* 检查 Watch 可调预算参数，防止非法数或负恢复速率造成错误输出。 */
static bool Budget_Config_Valid(void) {
    /* 训练上限需有限非负，下面全部条件也必须成立。 */
    return Nonnegative(chassis_power.training_limit_w) &&
           /* 训练上限还必须严格大于零。 */
           chassis_power.training_limit_w > 0.0f &&
           /* 安全余量需有限非负。 */
           Nonnegative(chassis_power.margin_w) &&
           /* 缓冲目标能量需有限非负。 */
           Nonnegative(chassis_power.buffer_target_j) &&
           /* 缓冲降额系数不能为负，否则缓冲不足反而放宽预算。 */
           Nonnegative(chassis_power.buffer_kp) &&
           /* 缓冲降额恢复速度需有限非负。 */
           Nonnegative(chassis_power.buffer_recovery_w_per_s) &&
           /* 实测超额修正增益需有限非负。 */
           Nonnegative(chassis_power.measured_down_gain) &&
           /* 实测降额恢复速度需有限非负。 */
           Nonnegative(chassis_power.measured_recovery_w_per_s) &&
           /* 实测恢复迟滞需有限非负。 */
           Nonnegative(chassis_power.measured_hysteresis_w) &&
           /* 最终预算回升速度需有限非负。 */
           Nonnegative(chassis_power.budget_recovery_w_per_s) &&
           /* 缓冲下降提示阈值需有限非负，返回所有检查的综合结果。 */
           Nonnegative(chassis_power.buffer_drop_threshold_j);
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

/* 上电初始化参数和历史，默认初始允许功率为 75 W。 */
void Chassis_Power_Init(void) {
    /* 全局参数和观察量先全部清零，再设默认参数。 */
    memset(&chassis_power, 0, sizeof(chassis_power));
    /* 清历史序号、时间和已见标志，表示尚未处理遥测。 */
    memset(&history, 0, sizeof(history));
    /* 从未收到有效裁判上限时使用 80 W 训练上限，不代表比赛固定上限。 */
    chassis_power.training_limit_w = 80.0f;
    /* 从允许上限扣除 5 W 余量，给模型误差和动态变化留空间。 */
    chassis_power.margin_w = 5.0f;
    /* 缓冲低于 50 J 才降额；高于它不允许额外超基础预算。 */
    chassis_power.buffer_target_j = 50.0f;
    /* 默认缓冲每缺少 1 J，降额目标增加 1 W，单位 W/J。 */
    chassis_power.buffer_kp = 1.0f;
    /* 缓冲恢复后，每秒最多释放 10 W 缓冲降额。 */
    chassis_power.buffer_recovery_w_per_s = 10.0f;
    /* 每个新功率帧超参考值 1 W，就增加 1 W 实测降额。 */
    chassis_power.measured_down_gain = 1.0f;
    /* 实测足够低时，每秒最多释放 1 W 实测降额。 */
    chassis_power.measured_recovery_w_per_s = 1.0f;
    /* 实测低于参考值减 2 W 才恢复，避免临界点反复变化。 */
    chassis_power.measured_hysteresis_w = 2.0f;
    /* 最终预算每秒最多回升 20 W；下降立即生效。 */
    chassis_power.budget_recovery_w_per_s = 20.0f;
    /* 新缓冲帧下降超过 0.5 J 才提示；提示不直接参与降额。 */
    chassis_power.buffer_drop_threshold_j = .5f;
    /* 裁判上限新鲜有效期 500 ms；过期仍保存最近可用上限。 */
    chassis_power.limit_timeout_ms = 500U;
    /* 缓冲有效期 500 ms；过期冻结已有缓冲降额。 */
    chassis_power.buffer_timeout_ms = 500U;
    /* 超电功率有效期 300 ms；过期冻结已有实测降额。 */
    chassis_power.cap_timeout_ms = 300U;
    /* 每轮 RPM 平方项系数，近似转速损耗；需本车标定。 */
    chassis_power.k_w2 = 3.0e-7f;
    /* 每轮原始电流平方项系数，近似电流损耗；原始值不是 A。 */
    chassis_power.k_i2 = 1.23e-7f;
    /* 每轮原始电流乘实际 RPM 系数，近似转换功率，保留符号。 */
    chassis_power.k_iw = 1.99688994e-6f;
    /* 每轮常数项 4.081 W；四轮分别计入，不是全车只加一次。 */
    chassis_power.k_const = 4.081f;
    /* 初始基础与最终预算均为 80 - 5 = 75 W，尚无两种降额。 */
    chassis_power.base_budget_w = chassis_power.target_power_w = 75.0f;
    /* 上电无新鲜有效上限，先标记使用回退策略。 */
    chassis_power.fallback_active = true;
}

/* 步骤一：读 Apply 的快照，算允许多少 W，结果存 target_power_w，不修改电流数组。 */
void Chassis_Power_Update(const ChassisPowerInput_t *input, uint32_t now_ms) {
    /* 没有输入快照就退出，防止空指针访问。 */
    if (input == NULL) return;
    /* 计算本次控制更新间隔，s；通常约 0.002，使用实际时间差。 */
    float dt = Sample_Dt(now_ms, history.update_ms, history.seen_update);
    /* 保存当前更新时间，下一次用它计算 dt。 */
    history.update_ms = now_ms;
    /* 标记已有一次更新，下一次有历史时间可用。 */
    history.seen_update = true;
    /* 上限年龄，ms；从未收到则用最大无符号值表示无数据。 */
    chassis_power.limit_age_ms = input->limit_received ? now_ms - input->limit_ms : UINT32_MAX;
    /* 独立计算缓冲年龄，不让其他裁判帧刷新它。 */
    chassis_power.buffer_age_ms = input->buffer_received ? now_ms - input->buffer_ms : UINT32_MAX;
    /* 超电功率年龄，ms；未收到同样记最大值。 */
    chassis_power.cap_age_ms = input->cap_received ? now_ms - input->cap_ms : UINT32_MAX;

    /* 数值可用上限必须收到过、有限且大于零；此处尚未检查年龄。 */
    bool usable_limit = input->limit_received && isfinite(input->limit_w) && input->limit_w > 0.0f;
    /* 接收层已校验的历史上限，即使控制任务首次看到时已过期也保留。 */
    /* 首次或上限序号变化时保存可用上限；同一帧不重复更新历史。 */
    if (usable_limit && (!history.has_limit || history.limit_seq != input->limit_seq)) {
        /* 记录已保存过上限，后续回退使用保存值而非训练值。 */
        history.has_limit = true;
        /* 保存已接纳的上限序号。 */
        history.limit_seq = input->limit_seq;
        /* 保存最近可用上限，W，即使随后通信过期也保留。 */
        chassis_power.last_valid_limit_w = chassis_power.judge_limit_w = input->limit_w;
    }
    /* 当前有效还要满足下行时间条件；有历史值不等于当前新鲜。 */
    chassis_power.judge_limit_valid = usable_limit &&
        /* 年龄不超过默认 500 ms 才标上限有效。 */
        chassis_power.limit_age_ms <= chassis_power.limit_timeout_ms;
    /* 缓冲收到过且有限非负，0 J 有效，下行再检查年龄。 */
    chassis_power.buffer_valid = input->buffer_received && Nonnegative(input->buffer_j) &&
        /* 缓冲年龄不超过默认 500 ms 才能更新降额。 */
        chassis_power.buffer_age_ms <= chassis_power.buffer_timeout_ms;
    /* 功率收到过且有限；这段允许负功率，下行检查年龄。 */
    chassis_power.cap_valid = input->cap_received && isfinite(input->cap_w) &&
        /* 超电功率年龄不超过默认 300 ms 才能修正降额。 */
        chassis_power.cap_age_ms <= chassis_power.cap_timeout_ms;
    /* 无新鲜有效上限就标回退，不是清空历史或立即停车。 */
    chassis_power.fallback_active = !chassis_power.judge_limit_valid;
    /* 缓冲过期清下降提示，但保留已有缓冲降额。 */
    if (!chassis_power.buffer_valid) chassis_power.buffer_drop_detected = false;
    /* 检查预算参数，异常标志会被 Limit 读取。 */
    chassis_power.config_fault = !Budget_Config_Valid();
    /* 预算参数异常时停止正常预算计算。 */
    if (chassis_power.config_fault) {
        /* 异常配置使预算为零；随后 Limit 还依据异常标志清零电流。 */
        chassis_power.target_power_w = 0.0f;
        /* 结束本次函数，异常分支不继续执行正常计算。 */
        return;
    }
    /* 有保存上限就用保存值，否则用训练值；过期不会自动放宽到训练值。 */
    float limit = history.has_limit ? chassis_power.last_valid_limit_w : chassis_power.training_limit_w;
    /* 基础预算 = 上限 - 余量，最低 0 W；默认 80 - 5 = 75 W。 */
    chassis_power.base_budget_w = fmaxf(0.0f, limit - chassis_power.margin_w);

    /* 缓冲必须当前有效，才处理新帧；无效时冻结已有降额。 */
    if (chassis_power.buffer_valid &&
        /* 首次或序号变化才处理，避免同一帧被每 2 ms 重复消费。 */
        (!history.seen_buffer || history.buffer_seq != input->buffer_seq)) {
        /* 缓冲新帧间隔，s，最多按 0.1 s 恢复；不是固定控制周期。 */
        float sample_dt = Sample_Dt(input->buffer_ms, history.buffer_ms, history.seen_buffer);
        /* 相邻缓冲帧实际间隔，ms，用于计算下降率。 */
        uint32_t gap = input->buffer_ms - history.buffer_ms;
        /* 保留最近一次新帧的下降量，避免 2ms 清零导致 Watch 看不到。 */
        /* 新帧先清旧提示，再按本次差值重算。 */
        chassis_power.buffer_drop_detected = false;
        /* 清旧下降量和下降率，两者仅供观察。 */
        chassis_power.buffer_drop_j = chassis_power.buffer_drop_rate_w = 0.0f;
        /* 上一帧存在、间隔非零且未跨长失联，才计算连续下降量。 */
        if (history.seen_buffer && gap > 0U && gap <= chassis_power.buffer_timeout_ms) {
            /* 下降量 = 上次 J - 本次 J；能量上升记零。 */
            chassis_power.buffer_drop_j = fmaxf(0.0f, history.buffer_j - input->buffer_j);
            /* J 除以秒得到 W，乘 1000 将 ms 换成 s。 */
            chassis_power.buffer_drop_rate_w = chassis_power.buffer_drop_j * 1000.0f / (float)gap;
            /* 本次是否明显下降，条件在下行。 */
            chassis_power.buffer_drop_detected =
                /* 下降超过默认 0.5 J 才提示，不用下降率直接限流。 */
                chassis_power.buffer_drop_j > chassis_power.buffer_drop_threshold_j;
        }
        /* 保存最近有效新帧的缓冲能量，供 Watch 看。 */
        chassis_power.buffer_energy_j = input->buffer_j;
        /* 缓冲降额目标 = W/J 系数乘下行的缺少能量。 */
        float desired = chassis_power.buffer_kp *
            /* 能量缺额 = max(0, 50 J - 当前 J)，不会产生负降额。 */
            fmaxf(0.0f, chassis_power.buffer_target_j - input->buffer_j);
        /* 缓冲降额目标最多等于基础预算。 */
        desired = fminf(desired, chassis_power.base_budget_w);
        /* 需要更大降额就立即采用，例如缓冲 40 J 对应默认降额 10 W。 */
        if (desired >= chassis_power.buffer_derate_w) chassis_power.buffer_derate_w = desired;
        /* 需要释放降额就慢慢减，但不低于新目标。 */
        else chassis_power.buffer_derate_w = fmaxf(desired, chassis_power.buffer_derate_w -
            /* 每个新缓冲帧最多释放 10 W/s 乘该帧间隔的降额。 */
            chassis_power.buffer_recovery_w_per_s * sample_dt);
        /* 记录已经处理缓冲。 */
        history.seen_buffer = true;
        /* 保存已处理序号，下拍能识别重复数据。 */
        history.buffer_seq = input->buffer_seq;
        /* 保存本帧时间，下次算间隔。 */
        history.buffer_ms = input->buffer_ms;
        /* 保存本帧能量，下次算下降量。 */
        history.buffer_j = input->buffer_j;
    }

    /* 实测参考值 = 基础预算 - 缓冲降额；不扣实测降额，避免参考点随自身修正一直下降。 */
    float nominal = fmaxf(0.0f, chassis_power.base_budget_w - chassis_power.buffer_derate_w);
    /* 只处理当前有效的新功率帧，不能每 2 ms 重复累加同一超额反馈。 */
    if (chassis_power.cap_valid && (!history.seen_cap || history.cap_seq != input->cap_seq)) {
        /* 相邻功率帧间隔，s，恢复最多按 0.1 s，避免失联恢复时一次放开。 */
        float sample_dt = Sample_Dt(input->cap_ms, history.cap_ms, history.seen_cap);
        /* 保存实测功率，W，与模型预测分开观察。 */
        chassis_power.measured_power_w = input->cap_w;
        /* 只在新实测帧上修正；下降快，恢复慢。参考点不随自身降额下降。 */
        /* 实测超参考值时累加实测降额，增加量见下行。 */
        if (input->cap_w > nominal) chassis_power.measured_derate_w +=
            /* 每个新帧增加默认增益 1 乘超额 W；此项不乘 dt。 */
            chassis_power.measured_down_gain * (input->cap_w - nominal);
        /* 实测低于参考值减 2 W 才恢复；迟滞带内保持降额。 */
        else if (input->cap_w < nominal - chassis_power.measured_hysteresis_w)
            /* 缓慢释放实测降额，但不低于零。 */
            chassis_power.measured_derate_w = fmaxf(0.0f, chassis_power.measured_derate_w -
                /* 本次释放最多 1 W/s 乘新功率帧间隔。 */
                chassis_power.measured_recovery_w_per_s * sample_dt);
        /* 限制累计实测降额，避免持续超额使它无限累加。 */
        chassis_power.measured_derate_w = Clamp(chassis_power.measured_derate_w, 0.0f,
                                               /* 实测降额上限为基础预算，W。 */
                                               chassis_power.base_budget_w);
        /* 记录已处理超电功率。 */
        history.seen_cap = true;
        /* 保存已处理功率序号，防止重复修正。 */
        history.cap_seq = input->cap_seq;
        /* 保存本帧时间，下次算恢复间隔。 */
        history.cap_ms = input->cap_ms;
    }
    /* 预算目标 = 基础预算 - 缓冲降额 - 实测降额，最低 0 W。 */
    float desired_budget = fmaxf(0.0f, nominal - chassis_power.measured_derate_w);
    /* 预算下降立即生效，优先收紧驱动。 */
    if (desired_budget <= chassis_power.target_power_w) chassis_power.target_power_w = desired_budget;
    /* 预算上升慢慢恢复，且不超过新目标。 */
    else chassis_power.target_power_w = fminf(desired_budget, chassis_power.target_power_w +
                                             /* 每次最多升 20 W/s 乘更新间隔；约 2 ms 时最多升 0.04 W。 */
                                             chassis_power.budget_recovery_w_per_s * dt);
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
