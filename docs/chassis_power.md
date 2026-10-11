# 底盘功率限制与调试

本功能在主下层板工程实现。遥控、底盘运动解算、云台参数保持原值；四轮速度 PID 算出电流后，统一限制电流再发送 CAN。超级电容只接收功率反馈，`cap_send_2E()` 仍不发送控制帧。

## 控制和数据链路

1. USART1 DMA 的 IDLE 或完整缓冲回调把实际接收长度交给 `judge_recive()`。流解析器保留跨 DMA 块的半帧，校验长度、CRC8、CRC16；错误候选逐字节重新同步。
2. `judge_update()` 分别更新裁判 `0x0201` 的 `game_robot_status.chassis_power_limit` 与 `0x0202` 的 `power_heat_data.chassis_power_buffer`，分别记录接收序号和毫秒时间。支持 13 字节状态帧、14 字节新缓冲帧与16 字节旧缓冲帧。
3. CAN `0x211` 经 `cap_update()` 更新 `cap.info->chassis_power`，同时记录功率帧接收时间和序号。零功率、零缓冲均是有效值，零功率上限不是有效上限。
4. `Chassis_Update()` 完成四个 `single_set_speed()` 后调用 `Chassis_Power_Apply()`：短临界区读取裁判、超电及四轮编码器实际 RPM 的一致快照。
5. `Chassis_Power_Update()` 计算预算；`Chassis_Power_Limit()` 计算共同电流系数，把限流结果写回 `Chassis_Motor[i].tx_info->torque`；之后 `group_set_torque()` 生成实际 CAN 电流并发送。MECH、GYRO、SPIN 均经过该路径。

主要实现文件：

| 文件 | 作用 |
| --- | --- |
| `Application/AlgorithmLayer/chassis_power.h` | 全局 `chassis_power` 的可调参数和观察量 |
| `Application/AlgorithmLayer/chassis_power.c` | 预算、降额、功率模型及四轮电流限幅 |
| `Application/ModuleLayer/chassis_power_adapter.c` | 真实遥测快照与电机电流写回 |
| `Application/ModuleLayer/chassis.c` | 在四轮 PID 和组发送之间接入限制器 |
| `Application/DeviceLayer/judge.c`、`Application/ProtocolLayer/judge_protocol.c` | 裁判解包、有效性和独立时间戳 |
| `Application/DriverLayer/drv_uart.c` | 实际 DMA 长度、避免 IDLE/TC 抢占重入、串口错误恢复 |
| `Application/ProtocolLayer/cap_protocol.c`、`Application/DeviceLayer/cap.c` | 超电只读功率反馈及接收时间戳 |

## 预算规则

基础预算为 `max(0, 最近有效裁判上限 - margin_w)`。未收到有效上限时使用 `training_limit_w`；上限过期使用最近有效值，置 `fallback_active`，不因通信过期主动停车。缓冲和功率实测过期时冻结已有降额。即使两种裁判帧都过期，也继续按保存的上限和降额运行。

缓冲降额目标为 `buffer_kp × max(0, buffer_target_j - buffer_energy_j)`；降额立即增加，回升按设定速率释放，仅在新的有效缓冲帧更新。缓冲高于目标不会放宽到裁判上限以上。缓冲下降由裁判系统实际能量消耗形成，本代码没有主动申请额外缓冲功率的路径。

功率实测大于“基础预算减缓冲降额”时，每个新帧增加 `measured_down_gain × 超额瓦数` 的实测降额；低于该参考点减滞回量时慢速恢复。这个参考点不随实测降额自身继续下降，以免形成正反馈。所有降额只收紧预算。

最终预算为 `max(0, 基础预算 - 缓冲降额 - 实测降额)`。下降立即生效，上升受 `budget_recovery_w_per_s` 限制。

例如裁判上限 80W、余量 5W：缓冲充足时预算 75W；缓冲 40J 时降额 10W，预算 65W。过期保留已有降额，不把缓冲换算成允许额外消耗的功率。

## Watch 调参

全部参数在 `chassis_power` 中，初始化之后可在 Watch 修改；重新启动会恢复默认值。

| 可调成员 | 默认 | 意义 |
| --- | --- | --- |
| `training_limit_w` | 80 W | 从未收到有效上限时的训练回退值 |
| `margin_w` | 5 W | 裁判上限以下预留余量 |
| `buffer_target_j` | 50 J | 低于此值收紧预算 |
| `buffer_kp` | 1 W/J | 缓冲每少 1J 产生的降额 |
| `buffer_recovery_w_per_s` | 10 W/s | 有效缓冲恢复时释放降额的速度 |
| `measured_down_gain` | 1 | 每个新功率帧的超额反馈增益 |
| `measured_recovery_w_per_s` | 1 W/s | 实测降额的释放速度 |
| `measured_hysteresis_w` | 2 W | 实测恢复的滞回量 |
| `budget_recovery_w_per_s` | 20 W/s | 最终预算的最大回升速度 |
| `buffer_drop_threshold_j` | 0.5 J | 缓冲下降提示阈值 |
| `limit_timeout_ms`、`buffer_timeout_ms` | 500 ms | 两种裁判数据独立的有效期 |
| `cap_timeout_ms` | 300 ms | 超电功率反馈有效期 |
| `k_i2` | 1.23e-7 | 原始 CAN 电流平方系数 |
| `k_iw` | 1.99688994e-6 | 原始 CAN 电流 × 电机 RPM 系数 |
| `k_w2` | 3e-7 | 电机 RPM 平方系数 |
| `k_const` | 4.081 | 每轮模型常数，单位 W |

优先观察：

- `judge_limit_w`、`last_valid_limit_w`、`judge_limit_valid`、`fallback_active`、`limit_age_ms`：确认裁判上限链路。
- `buffer_energy_j`、`buffer_valid`、`buffer_age_ms`、`buffer_derate_w`：确认缓冲只在有效新帧降额/恢复。
- `buffer_drop_j`、`buffer_drop_rate_w`、`buffer_drop_detected`：最近新缓冲帧的下降量、下降率和提示。下降值保持到下一新帧，方便 Watch 查看；过期提示清零。下降率用于观察，不加入控制。
- `measured_power_w`、`cap_valid`、`cap_age_ms`、`measured_derate_w`：确认真实功率反馈及其降额。缓冲下降需与实际功率一起观察。
- `base_budget_w`、`target_power_w`、`predicted_before_w`、`predicted_after_w`、`current_scale`：确认预算及限流效果。
- `current_before[4]`、`current_after[4]`、`limiting_active`：确认四轮共同缩放；实际输出已量化为 CAN 整数。
- `config_fault`、`model_fault`、`budget_unreachable`：无效配置/NaN 会撤去驱动电流；零电流的模型转速损耗仍高于预算时置 `budget_unreachable` 并撤去驱动，属于模型不可达情况，不属于通信超时停车。

需要检查接收层时，可看 `My_Judge.status->limit_rx_count/limit_update_ms`、`buffer_rx_count/buffer_update_ms` 和 `cap.status->power_rx_count/power_update_ms`。

## 模型和实车验证

每轮按 `k_i2 × i² + k_iw × i × rpm + k_w2 × rpm² + k_const` 估计功率，`i` 是 M3508 原始 CAN 电流值，`rpm` 是编码器实际电机转速，均保留符号。负功率按零计，不用制动负功率抵扣其它轮耗电。16 次二分搜索共同系数，最终复核量化后发送电流的预测功率不高于预算。

系数取参考工程作为起点，尚未在本车实测标定。软件约束的是模型预测功率；要确认实际功率持续低于裁判上限，需要烧录后用超电测量校验。先查看数据有效性，再逐步测试前后、横移、旋转和加减速，比较 `predicted_after_w` 与 `measured_power_w`，并观察缓冲是否下降。预测偏低时应增大相应模型系数或余量。实测反馈存在通信延迟，不能代替模型标定来保证每个瞬间都不超限。

## 已执行验证

- Python/GCC：10 个功率/超电测试、4 个裁判/串口测试、6 个底盘集成测试、28 个云台回归测试。
- 独立 C：8 个底盘控制测试、7 个运动解算测试；5 个 PowerShell 接口契约。
- ARMCC5：编译与链接主工程，生成临时目录的 AXF/HEX，不覆盖已有 Keil 构建文件、不烧录。执行 `python test/build_firmware.py` 可重复该检查；可用 `--toolchain` 指定 ARMCC5 的 bin 路径。警告来自现有未使用函数和 CMSIS-DSP 源码。
