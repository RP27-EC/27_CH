/* 数据流：Apply 取快照 -> Update 计算预算 -> Limit 预测并缩放电流 -> CAN 下发。
 * 预算只等于选定上限减裕量；缓冲和超电实测仅供观察，不参与预算。
 */
#include "chassis_power.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

/* AC6的默认std模式不保证NaN/Inf语义；异常保护需要本文件使用full模式。 */
#if defined(__ARMCC_VERSION) && defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__
#error "Compile chassis_power.c with -ffp-mode=full so finite-value guards remain effective."
#endif

ChassisPower_t chassis_power;

static struct {
    bool has_limit;
    uint32_t limit_seq;
} history;

static float Clamp(float x, float lo, float hi) {
    return fminf(hi, fmaxf(lo, x));
}

static bool Nonnegative(float x) {
    return isfinite(x) && x >= 0.0f;
}

static bool Budget_Config_Valid(void) {
    return Nonnegative(chassis_power.training_limit_w) &&
           Nonnegative(chassis_power.margin_w);
}

/* 停车和每次限流前清输出观察量，不清预算、参数或设备反馈。 */
void Chassis_Power_Clear_Output(void) {
    chassis_power.current_scale = 0.0f;
    chassis_power.predicted_before_w = 0.0f;
    chassis_power.predicted_after_w = 0.0f;
    chassis_power.limiting_active = false;
    chassis_power.model_fault = false;
    chassis_power.budget_unreachable = false;
    memset(chassis_power.current_before, 0, sizeof(chassis_power.current_before));
    memset(chassis_power.current_after, 0, sizeof(chassis_power.current_after));
    memset(chassis_power.wheel_power_before_w, 0, sizeof(chassis_power.wheel_power_before_w));
    memset(chassis_power.wheel_power_after_w, 0, sizeof(chassis_power.wheel_power_after_w));
    memset(chassis_power.target_rpm, 0, sizeof(chassis_power.target_rpm));
    memset(chassis_power.actual_rpm, 0, sizeof(chassis_power.actual_rpm));
}

void Chassis_Power_Init(void) {
    memset(&chassis_power, 0, sizeof(chassis_power));
    memset(&history, 0, sizeof(history));
    chassis_power.training_limit_w = 0.0f; /* 无裁判历史时的独立训练额度，默认禁用。 */
    chassis_power.margin_w = 5.0f;  // 裁判上限减裕量，默认5W。实际预算 = max(0, 裁判上限 - 裕量)。
    chassis_power.limit_timeout_ms = 500U;
    chassis_power.buffer_timeout_ms = 500U; /* 后两项只影响观测有效标志。 */
    chassis_power.cap_timeout_ms = 300U;

    /* 沿用原模型，I 为带符号的 CAN raw，n 为电机实际 RPM。 */
    chassis_power.k_w2 = 3.0e-7f;
    chassis_power.k_i2 = 1.23e-7f;
    chassis_power.k_iw = 1.99688994e-6f;
    chassis_power.k_const = 4.081f;
    chassis_power.limit_source = CHASSIS_POWER_LIMIT_NONE;
    chassis_power.fallback_active = true;
}

/* 上限新帧可更新历史；上限过期后保留最近有效值。
 * 缓冲/实测掉线、能量下降或功率偏高，都不会额外扣减/锁住预算。
 */
void Chassis_Power_Update(const ChassisPowerInput_t *input, uint32_t now_ms) {
    if (input == NULL) {
        chassis_power.base_budget_w = 0.0f;
        chassis_power.target_power_w = 0.0f;
        chassis_power.config_fault = true;
        return;
    }

    chassis_power.limit_age_ms = input->limit_received ? now_ms - input->limit_ms : UINT32_MAX;
    chassis_power.buffer_age_ms = input->buffer_received ? now_ms - input->buffer_ms : UINT32_MAX;
    chassis_power.cap_age_ms = input->cap_received ? now_ms - input->cap_ms : UINT32_MAX;

    chassis_power.judge_limit_valid = input->limit_received &&
        isfinite(input->limit_w) && input->limit_w > 0.0f &&
        chassis_power.limit_age_ms <= chassis_power.limit_timeout_ms;
    chassis_power.buffer_valid = input->buffer_received && Nonnegative(input->buffer_j) &&
        chassis_power.buffer_age_ms <= chassis_power.buffer_timeout_ms;
    chassis_power.cap_valid = input->cap_received && isfinite(input->cap_w) &&
        chassis_power.cap_age_ms <= chassis_power.cap_timeout_ms;
    chassis_power.fallback_active = !chassis_power.judge_limit_valid;

    if (chassis_power.judge_limit_valid &&
        (!history.has_limit || history.limit_seq != input->limit_seq)) {
        history.has_limit = true;
        history.limit_seq = input->limit_seq;
        chassis_power.last_valid_limit_w = input->limit_w;
        chassis_power.judge_limit_w = input->limit_w;
    }

    /* 保存实收值供 Watch 比较；数据过期时用 age/valid 区分旧值。 */
    if (input->buffer_received && Nonnegative(input->buffer_j))
        chassis_power.buffer_energy_j = input->buffer_j;
    if (input->cap_received && isfinite(input->cap_w))
        chassis_power.measured_power_w = input->cap_w;

    float limit = 0.0f;
    if (history.has_limit) {
        limit = chassis_power.last_valid_limit_w;
        chassis_power.limit_source = chassis_power.judge_limit_valid ?
            CHASSIS_POWER_LIMIT_JUDGE : CHASSIS_POWER_LIMIT_HISTORY;
    } else if (chassis_power.training_limit_w > 0.0f) {
        limit = chassis_power.training_limit_w;
        chassis_power.limit_source = CHASSIS_POWER_LIMIT_TRAINING;
    } else {
        chassis_power.limit_source = CHASSIS_POWER_LIMIT_NONE;
    }

    chassis_power.config_fault = !Budget_Config_Valid();
    chassis_power.base_budget_w = chassis_power.config_fault ? 0.0f :
        fmaxf(0.0f, limit - chassis_power.margin_w);
    /* 每次更新直接跟随上限/裕量变化，没有恢复计时、积分降额或回升限速。 */
    chassis_power.target_power_w = chassis_power.base_budget_w;
}

static bool Model_Valid(void) {
    
    return Nonnegative(chassis_power.k_i2) && Nonnegative(chassis_power.k_iw) &&/* 电流平方和电流乘 RPM 系数需有限非负；I、RPM 本身允许带符号。 */
           Nonnegative(chassis_power.k_w2) && Nonnegative(chassis_power.k_const) &&  /* 转速平方和常数项系数需有限非负。 */
           Nonnegative(chassis_power.target_power_w); /* 最终允许功率需有限非负。 */
}



static float Can_Current(float current) {
    /* 限幅后转 int16_t 截去小数，再转 float 方便功率计算。 */
    return (float)(int16_t)Clamp(current, -CHASSIS_POWER_RAW_CURRENT_MAX,
                                         /* 原始电流范围 -16384 到 +16384，不是安培。 */
                                         CHASSIS_POWER_RAW_CURRENT_MAX);
}


static float Predict(const float original[4], const float rpm[4], float scale,
                     /* 可选每轮功率输出数组，NULL 表示只要总功率。 */
                     float wheel_power[4]) {

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
      
        if (!isfinite(power)) return NAN;
        /* 单轮负功率按零算，不假设制动能完全抵消其他轮耗电。 */
        power = fmaxf(0.0f, power); 
    
        if (wheel_power != NULL) wheel_power[i] = power;
    
        sum += power;
    }
    /* 返回四轮总预测功率，W */
    return sum;
}

/* 步骤二：读全局 target_power_w，直接修改传入 current 数组，rpm 只读。 */
void Chassis_Power_Limit(float current[4], const float rpm[4]) {
  
    if (current == NULL) return;

    /* 清上一拍输出记录，保留 Update 已算的预算和原始反馈。 */
    Chassis_Power_Clear_Output();

    /* 保存原 PID 电流供前后对比。 */
    memcpy(chassis_power.current_before, current, sizeof(chassis_power.current_before));

  
    if (rpm == NULL) {
        chassis_power.model_fault = true;
        chassis_power.limiting_active = true;
        memset(current, 0, sizeof(chassis_power.current_after));
        return;
    }
    memcpy(chassis_power.actual_rpm, rpm, sizeof(chassis_power.actual_rpm));

    bool valid = Model_Valid() && !chassis_power.config_fault;

    /* 逐个检查四轮电流与 RPM 数值输入是否为有限数。 */
    for (unsigned i = 0; i < CHASSIS_POWER_WHEEL_COUNT; ++i)
        valid = valid && isfinite(current[i]) && isfinite(rpm[i]);

    /* 非法配置或输入进入清零保护，不继续使用 PID 大电流。 */
    if (!valid) {
        chassis_power.model_fault = true;
        chassis_power.limiting_active = true;

        /* 将传入四轮电流数组清零 */
        memset(current, 0, sizeof(chassis_power.current_after));
   
        return;
    }

    /* scale=1，预测原电流总功率并保存每轮功率。 */
    chassis_power.predicted_before_w = Predict(current, rpm, 1.0f, chassis_power.wheel_power_before_w);

    /* scale=0，预测零驱动电流损耗；保留实际 RPM，车辆不等于已经停下。 */
    float baseline = Predict(current, rpm, 0.0f, NULL);

    /* 原电流或零电流预测非法时清零保护。 */
    if (!isfinite(chassis_power.predicted_before_w) || !isfinite(baseline)) {
        chassis_power.model_fault = true;
        chassis_power.limiting_active = true;
        memset(current, 0, sizeof(chassis_power.current_after));
    
        return;
    }

    /* 默认不缩小，只有预测超预算才降低系数。 */
    float scale = 1.0f;

    /* 原四轮电流的预测总功率超最终预算，开始限流。 */
    if (chassis_power.target_power_w <= 0.0f) {
        /* 无有效额度时撤去驱动，不因制动预测为零而放行非零电流。 */
        scale = 0.0f;
        chassis_power.budget_unreachable = baseline > 0.0f;
    } else if (chassis_power.predicted_before_w > chassis_power.target_power_w) {

        /* 零驱动预测仍超预算，采取撤去驱动策略，不搜索其他系数。 */
        if (baseline > chassis_power.target_power_w) {
            scale = 0.0f;
            chassis_power.budget_unreachable = true;    /* 记录零驱动模型仍超预算，区别于非法模型和通信超时。 */
 
          
        } else {  /* 零驱动预测可行，进入 0~1 的共同系数搜索。 */

            /* 初始下界 0 可行，上界 1 超预算，目标尽量保留电流。 */
            float lower = 0.0f, upper = 1.0f;
            /* 固定 16 次二分，每次缩小一半系数区间。 */
            for (unsigned step = 0; step < 16U; ++step) {
             
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

   
    chassis_power.current_scale = scale; /* 保存共同系数，Watch 可看电流保留到几成。 */

    
    chassis_power.limiting_active = scale < 1.0f;/* 正常路径只有系数小于 1 才标记缩小了电流。 */


    /* 共同缩小四轮电流并整数化，不增大指令幅度、不反转符号。 */
    for (unsigned i = 0; i < CHASSIS_POWER_WHEEL_COUNT; ++i) current[i] = Can_Current(current[i] * scale);
    /* 用当前最终电流计算总功率和每轮功率；scale=1 避免重复缩小，保护清零后损耗也可能非零。 */
    chassis_power.predicted_after_w = Predict(current, rpm, 1.0f, chassis_power.wheel_power_after_w);


    /* 最终预测非法需要清零保护，下行还检查可达情况是否超预算。 */
    if (!isfinite(chassis_power.predicted_after_w) ||
        /* 可达路径最终整数功率不能超预算；不可达路径已撤驱动，保留残余损耗预测。 */
        (!chassis_power.budget_unreachable && chassis_power.predicted_after_w > chassis_power.target_power_w)) {

        memset(current, 0, sizeof(chassis_power.current_after));
        
        /* 最终复核失败，缩放系数记录设为零，表示已经撤去驱动电流。 */
        chassis_power.current_scale = 0.0f;
        chassis_power.limiting_active = true;
        chassis_power.model_fault = true;       /* 记录本次模型计算或输入异常，供 Watch 排查。 */


        /* 用当前最终电流计算总功率和每轮功率；scale=1 避免重复缩小，保护清零后损耗也可能非零。 */
        chassis_power.predicted_after_w = Predict(current, rpm, 1.0f, chassis_power.wheel_power_after_w);
    }
    /* 记录最终电流；传入数组已修改，Apply 会把它写回电机发送对象。 */
    memcpy(chassis_power.current_after, current, sizeof(chassis_power.current_after));
}
