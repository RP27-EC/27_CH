/* 阅读顺序：先看 adapter.c 的取数/写回，再看本文件的数据定义，最后看 chassis_power.c 的计算。
 * 本文件只声明类型、参数和函数；真正的函数实现位于 .c 文件。
 */
#ifndef CHASSIS_POWER_H /* 头文件保护：同一次编译重复包含时，不重复定义下面的类型。 */
#define CHASSIS_POWER_H /* 标记这个头文件已经被包含。 */

#include <stdbool.h> /* 提供 bool、true、false。 */
#include <stdint.h>  /* 提供 uint32_t、int16_t 等固定宽度整数类型。 */

#define CHASSIS_POWER_WHEEL_COUNT 4U /* 底盘有四个驱动轮，功率按四轮相加。 */
#define CHASSIS_POWER_RAW_CURRENT_MAX 16384.0f /* CAN 原始电流指令的绝对值上限，不是 16384 A。 */

/* 接收层快照：序号只在收到有效新帧时变化，时间单位 ms。 */
typedef struct {
    bool limit_received; /* 是否收到过有效裁判功率上限。 */
    uint32_t limit_seq, limit_ms; /* limit_seq：上限接收序号；limit_ms：上限接收时间，ms。 */
    float limit_w; /* 裁判上限，W，必须大于零才能作为可用上限。 */
    bool buffer_received; /* 是否收到过裁判缓冲能量；0 J 是有效数据。 */
    uint32_t buffer_seq, buffer_ms; /* buffer_seq：缓冲接收序号；buffer_ms：缓冲接收时间，ms。 */
    float buffer_j; /* 缓冲剩余能量，J，不能与 W 混用。 */
    bool cap_received; /* 是否收到过超电功率反馈；0 W 是有效数据。 */
    uint32_t cap_seq, cap_ms; /* cap_seq：功率接收序号；cap_ms：功率接收时间，ms。 */
    float cap_w; /* 超电协议报告的底盘功率，W。 */
} ChassisPowerInput_t; /* 输入快照类型：Apply 复制真实数据，Update 读取这个副本。 */

typedef struct {
    /* Watch 可调参数；功率单位 W，能量 J，转速 RPM，电流 CAN raw。 */
    float training_limit_w; /* 从未收到有效上限时使用的训练上限，默认 80 W。 */
    float margin_w; /* 从上限扣除的余量，默认 5 W；基础预算 = 上限 - 余量。 */
    float buffer_target_j; /* 希望维持的缓冲能量，默认 50 J；低于它才产生缓冲降额。 */
    float buffer_kp; /* 缓冲少 1 J 要少用多少 W，默认 1 W/J；缓冲充足也不允许额外超预算。 */
    float buffer_recovery_w_per_s; /* 缓冲恢复后，缓冲降额最大释放速度，默认 10 W/s。 */
    float measured_down_gain; /* 每个新实测帧的超额修正系数，默认 1；它不是每 2 ms 都累加。 */
    float measured_recovery_w_per_s; /* 实测偏低后，实测降额最大释放速度，默认 1 W/s。 */
    float measured_hysteresis_w; /* 实测恢复迟滞，默认 2 W；需明显低于参考功率才恢复。 */
    float budget_recovery_w_per_s; /* 最终允许功率的最大上升速度，默认 20 W/s；下降立即生效。 */
    float buffer_drop_threshold_j; /* 缓冲下降提示阈值，默认 0.5 J；只影响观察提示，不直接控制限流。 */
    uint32_t limit_timeout_ms, buffer_timeout_ms, cap_timeout_ms; /* 上限/缓冲/超电有效期，分别默认 500/500/300 ms。 */
    float k_i2, k_iw, k_w2, k_const; /* 每轮模型 I^2、I*RPM、RPM^2、常数项系数；需本车标定，I 是 CAN 原始值。 */

    /* 以下为自动更新的观察量，不是控制开关。 */
    float judge_limit_w, last_valid_limit_w; /* 最近接纳的裁判上限及其保存值，W；过期也不清除保存值。 */
    float base_budget_w, target_power_w; /* base：扣余量后的预算；target：扣两种降额并限制回升后的最终预算，W。 */
    float buffer_energy_j, buffer_derate_w; /* 最近有效缓冲能量，J；缓冲不足引起的预算扣减，W。 */
    float buffer_drop_j, buffer_drop_rate_w; /* 相邻有效新缓冲帧的下降量，J；下降量/时间得到的下降率，W，仅观察。 */
    float measured_power_w, measured_derate_w; /* 最近有效实测功率，W；实测超额累计形成的预算扣减，W。 */
    float predicted_before_w, predicted_after_w; /* 限流前/后四轮模型预测总功率，W，不是传感器实测。 */
    float wheel_power_before_w[4], wheel_power_after_w[4]; /* 四轮分别的限流前/后预测功率，W，数组顺序沿用电机数组。 */
    float current_before[4], current_after[4]; /* 四轮限流前/后 CAN 原始电流指令；after 已按实际发送整数化。 */
    float current_scale; /* 共同电流缩放系数，范围 0~1；1 为未缩小，0 为撤去驱动电流。 */
    uint32_t limit_age_ms, buffer_age_ms, cap_age_ms; /* 三种反馈距现在多久，ms；未收到时记 UINT32_MAX。 */
    bool judge_limit_valid, buffer_valid, cap_valid; /* 对应反馈是否同时满足数值要求和有效期要求。 */
    bool fallback_active, buffer_drop_detected; /* 上限过期/缺失回退标志；最近缓冲新帧下降超过阈值的提示。 */
    bool limiting_active, model_fault, budget_unreachable, config_fault; /* 正在限流；模型/输入异常；零电流预测仍超预算；预算配置异常。 */
} ChassisPower_t; /* 参数与运行观察量类型：在 Watch 中查看同名成员可理解每步结果。 */

extern ChassisPower_t chassis_power; /* 声明全局对象；实际存储在 .c 中，Update 和 Limit 通过它共享最终预算。 */

void Chassis_Power_Init(void); /* 初始化参数和历史状态，上电时调用。 */
void Chassis_Power_Update(const ChassisPowerInput_t *input, uint32_t now_ms); /* 读取遥测快照，更新 chassis_power.target_power_w。 */
/* 就地限制电流：共同缩放，不增大、不反向；结果为实际 CAN 整数电流。 */
void Chassis_Power_Limit(float current[4], const float rpm[4]); /* 使用最终预算，直接修改传入的 current 数组；rpm 只读。 */
/* 停车时只清输出观察量，保留调参值、最近有效上限和既有降额。 */
void Chassis_Power_Clear_Output(void); /* 清除本次输出记录；不清预算历史，不等于把电机实际电流置零。 */
/* STM32 适配层：快照裁判/超电/电机数据，限制后写回 tx_info->torque。 */
void Chassis_Power_Apply(void); /* 主流程入口：取设备快照 -> Update -> Limit -> 写回电流，由外层继续发 CAN。 */

#endif /* 结束 CHASSIS_POWER_H 头文件保护。 */
