# 底盘功率提前干预设计

用户已在对话中批准落地。作用范围为本下板工程的功率限制；保持超电只读、既有电机模型、PID 和运动参数。

## 数据与预算

- 上限来自 `My_Judge.org_info->game_robot_status.chassis_power_limit`，当前缓冲来自 `My_Judge.org_info->power_heat_data.chassis_power_buffer`，实测功率来自 `cap.info->chassis_power`；均不得被算法覆盖。
- 默认从未收到裁判上限时预算为 0 W。`training_limit_w=0` 表示禁用离线训练额度；显式改为正值后才能使用训练回退，并暴露来源。收到真实上限后优先使用真实值；过期保留最后有效上限并标记历史来源。
- 基础预算为 `max(0, limit-margin_w)`，默认余量 5 W。首次有明确预算来源时建立初始预算，后续下降立即生效，上升最多 20 W/s。
- 缓冲目标默认 60 J，是期望值，不是假造当前反馈。缺额降额 `buffer_kp*max(0,buffer_target_j-buffer_j)`，默认 1 W/J；下降立即收紧，恢复最多 10 W/s，仅用有效新帧。

## 实测修正与合并

- 警戒值为 `max(0,base_budget_w-measured_warning_margin_w)`，默认提前 2 W。
- 提前修正为 `clamp(measured_soft_kp*max(0,cap_w-warning),0,min(measured_soft_max_w,base_budget_w))`，默认增益 1、最大 5 W；不累计。同一反馈帧不能重复积分。
- 超过基础预算时，偏差修正按 `measured_integral_gain_per_s*max(0,cap_w-base_budget_w)*frame_dt` 增长，默认增益 1/s。首次无前帧、时间差为零或跨越反馈超时的帧不积分未知时间。
- 实测修正是提前修正与偏差修正之和，限到基础预算。综合修正取 `max(buffer_derate_w,measured_derate_w)`，不完整叠加两类反馈。
- 最终预算为 `max(0,base_budget_w-feedback_derate_w)`。任何分支不得超过基础预算。

## 恢复与离线

- 提前修正和偏差修正的释放需裁判上限、缓冲、超电功率均新鲜，实测低于警戒值减 2 W，缓冲达到目标或连续有效相邻帧正在上升，条件持续 500 ms。
- 偏差恢复最大 1 W/s；首次重连不按失联时长释放；零时间差不积分、不释放。
- 未满缓冲保持不变不作为实测偏差释放依据。反馈过期冻结对应降额；最终预算上升需恢复条件成立。
- 缓冲下降量/下降率保留为观察量，不把短间隔整数差值直接当总功率。失联后首帧不计算跨失联下降率。

## 代码与验收

`Apply` 保持短临界区快照和写回；`Update` 更新预算；`Limit` 保持四轮共同缩放、CAN 整数量化复核和异常清零。参数与关键语句保持中文注释。

验收涵盖真实数据不可改写、60→59 J、警戒区无无限累计、帧频独立积分、重复/过期/重连反馈、联合恢复、源标识、上下限变化、符号/量化/随机电流和组发送前限制。超电 TX 不恢复。ARMCC 构建写临时目录，不覆盖用户产物、不烧录。

当前工作区 `chassis.c` 有缺失注释结束符引发的既有编译错误，只补齐结束符；不改变用户运动参数。保留 `chassis_power.c` 已有阅读排版，新的预算段同步更新注释。
