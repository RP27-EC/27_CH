/* 本文件负责把真实设备接到功率算法上，不负责推导功率模型。
 * 调用位置：chassis.c 中，四轮速度 PID 算完后、四轮 CAN 发送前。
 * 三步关系：Apply 取数 -> Update 算允许功率 -> Limit 修改电流 -> Apply 写回。
 * 算法修改的是准备发送的电流指令，不是编码器反馈，也不是轮子目标 RPM。
 * Debug 原始量：My_Judge.org_info 的功率上限/当前缓冲，cap.info->chassis_power。
 * 目标 60 J、警戒值、最终预算只存在算法对象里，不写回这三个实收字段。
 */
#include "chassis_power.h" /* 引入功率算法的输入结构体和函数声明。 */
#include "chassis_motor.h" /* 访问四个底盘电机的 PID 电流和编码器转速。 */
#include "judge.h"         /* 访问裁判系统的功率上限、缓冲能量及接收记录。 */
#include "cap.h"           /* 访问超电模块返回的底盘实测功率及接收记录。 */

void Chassis_Power_Apply(void) {
    ChassisPowerInput_t input = {0}; /* 本次计算使用的遥测副本，初始全部清零。 */
    float current[4], rpm[4]; /* current 是 CAN 原始电流指令；rpm 是实际电机转速。 */
    /* 接收中断会同时改数据、时间和序号，短临界区保证拿到同一帧快照。 */
    uint32_t irq_mask = __get_PRIMASK(); /* 保存进入前中断是否关闭，支持外层已关中断的情况。 */
    __disable_irq(); /* 暂停可屏蔽中断处理，防止复制到一半接收回调更新字段；硬件接收仍可进行。 */
    uint32_t now_ms = HAL_GetTick(); /* 取得本次计算的当前时间，单位 ms。 */
    input.limit_received = My_Judge.status->limit_received; /* 是否曾收到有效功率上限；不能只看数值猜是否收到。 */
    input.limit_seq = My_Judge.status->limit_rx_count; /* 上限帧接收序号，算法用它识别新帧。 */
    input.limit_ms = My_Judge.status->limit_update_ms;
    input.limit_w = (float)My_Judge.org_info->game_robot_status.chassis_power_limit; /* 裁判允许的底盘功率，W。 */
    input.buffer_received = My_Judge.status->buffer_received; /* 是否收到过缓冲能量；真实 0 J 也算收到。 */
    input.buffer_seq = My_Judge.status->buffer_rx_count; /* 缓冲帧序号，避免同一帧每 2 ms 被重复处理。 */
    input.buffer_ms = My_Judge.status->buffer_update_ms; /* 缓冲帧自己的时间，不借用其他裁判帧时间。 */
    input.buffer_j = (float)My_Judge.org_info->power_heat_data.chassis_power_buffer; /* 裁判剩余缓冲能量，J，不是 W。 */
    input.cap_received = cap.status->power_received; /* 是否收到过超电功率反馈；真实 0 W 也有效。 */
    input.cap_seq = cap.status->power_rx_count; /* 超电功率帧序号，用于只在新实测帧上修正预算。 */
    input.cap_ms = cap.status->power_update_ms; /* 超电功率反馈的更新时间，ms。 */
    input.cap_w = (float)cap.info->chassis_power; /* 超电协议报告的底盘实测功率，W。 */
    for (unsigned i = 0; i < CHASSIS_POWER_WHEEL_COUNT; ++i) { /* 复制四个轮子的指令与实际转速。 */
        current[i] = Chassis_Motor[i].tx_info->torque; /* 当前 _3508_Single 路径中，torque 实际存 CAN 原始电流值。 */
        rpm[i] = (float)Chassis_Motor[i].rx_info->encoder_speed; /* 用实际 RPM 估计功率，不能用目标 RPM 代替。 */
    }
    __set_PRIMASK(irq_mask); /* 恢复原中断状态；复制结束就恢复，不把下面的功率计算放在临界区内。 */

    Chassis_Power_Update(&input, now_ms); /* 第一步：&input 传真实快照；预算和来源写入算法对象，不伪造裁判/超电字段。 */
    Chassis_Power_Limit(current, rpm); /* 第二步：算法读全局预算；传数组地址，所以直接修改这里的 current 数组，不需要返回新数组。 */
    for (unsigned i = 0; i < CHASSIS_POWER_WHEEL_COUNT; ++i) /* 第三步：把修改后的四轮电流放回电机发送对象。 */
        Chassis_Motor[i].tx_info->torque = current[i]; /* 随后的 group_set_torque() 才负责打包并发送 CAN。 */
}
