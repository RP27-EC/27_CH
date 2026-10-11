# 底盘功率限制与 Debug

本说明对应 2026-10-11 落地的提前干预方案，替代旧版“50 J 阈值＋两类降额相加”的策略。只改本下板功率算法；超电保持只读，`cap_send_2E()` 不发送控制帧。没有发送指令仅说明程序未请求放能，实际模块状态仍需实车确认。

## 三个函数如何衔接

```text
四轮速度 PID 算出 CAN 原始电流
  -> Chassis_Power_Apply：取真实遥测、电流和实际 RPM 的快照
  -> Chassis_Power_Update：把允许瓦数写入 chassis_power.target_power_w
  -> Chassis_Power_Limit：读预算，直接修改 current[4]
  -> Apply：把受限电流写回 Chassis_Motor[i].tx_info->torque
  -> group_set_torque：整数化、打包 0x200、FDCAN1 发送
```

`Update` 返回 `void`，预算通过全局对象传给 `Limit`；数组实参传地址，`Limit` 直接修改 `Apply` 的电流数组。模型使用实际编码器 RPM，不能用目标 RPM 代替。当前 `_3508_Single` 路径中的 `torque` 存原始电流指令，不是 N·m 或 A。

## 第一组 Watch：真实收到的三个量

复制下面三个表达式到 Keil Watch：

```c
My_Judge.org_info->game_robot_status.chassis_power_limit
My_Judge.org_info->power_heat_data.chassis_power_buffer
cap.info->chassis_power
```

| 表达式 | 含义与来源 | 单位 |
| --- | --- | --- |
| `My_Judge.org_info->game_robot_status.chassis_power_limit` | 裁判 0x0201 的底盘功率上限 | W |
| `My_Judge.org_info->power_heat_data.chassis_power_buffer` | 裁判 0x0202 的当前缓冲能量 | J |
| `cap.info->chassis_power` | 超电 0x211 报告的实测底盘功率 | W |

算法不写回这些字段。`buffer_target_j=60` 是希望维持的满缓冲目标，不是把实收缓冲伪造为 60 J；裁判反馈 59 J 时就按 59 J 计算。超电测量位置、功率单位及其与裁判底盘输入功率的对应关系需要实车核对。

裁判按规则的功率差结算缓冲：低于上限时可以恢复，上限普通为 60 J；满值截断会隐藏额外余量，因此不能只靠缓冲满值判断每个瞬间是否超功率。不主动把缓冲变成额外允许功率。

## 第二组 Watch：是否真正收到、新不新、预算依据是什么

```c
chassis_power.judge_limit_valid
chassis_power.buffer_valid
chassis_power.cap_valid
chassis_power.limit_age_ms
chassis_power.buffer_age_ms
chassis_power.cap_age_ms
chassis_power.limit_source
```

| `limit_source` | 含义 |
| --- | --- |
| `CHASSIS_POWER_LIMIT_NONE` / 0 | 从未收到可用裁判上限，训练额度未启用；预算为 0 W |
| `CHASSIS_POWER_LIMIT_JUDGE` / 1 | 使用当前新鲜有效的裁判上限 |
| `CHASSIS_POWER_LIMIT_HISTORY` / 2 | 上限过期，保留最近一次真实接收的可用上限 |
| `CHASSIS_POWER_LIMIT_TRAINING` / 3 | 从未收到上限，用户显式设了正的离线训练额度 |

默认 `training_limit_w=0`，不再凭空按 80 W 开车。离线需要调试底盘时可显式设置 `chassis_power.training_limit_w`，例如 80 W；这不改变裁判结构体和 `judge_limit_w`，一旦收到真实上限即优先使用真实值。首次有明确可用预算时按当前修正建立初始预算；以后上升要确认并限速。

上限/缓冲默认有效期 500 ms，超电功率 300 ms。看见旧结构体里还有数值不代表正在接收；计数、时间戳和上述有效标志一起看。需要排查接收可观察：

```c
My_Judge.status->limit_received
My_Judge.status->limit_rx_count
My_Judge.status->limit_update_ms
My_Judge.status->buffer_received
My_Judge.status->buffer_rx_count
My_Judge.status->buffer_update_ms
cap.status->power_received
cap.status->power_rx_count
cap.status->power_update_ms
```

## 0x0201 长度兼容与拒收排查

实车调试发现 `0x0201` 的 `data_length=17`，原来的 `length != 13` 校验会拒收，使 `limit_received=0`，默认功率预算为零。现按同工作区 `Down/RP26_Lob_Hero_Down/Application/ProtocolLayer/judge_protocol.h` 的 17 字节布局兼容解析，并保留 13 字节版本。

下表偏移从载荷第一个字节 `rxBuf[0]` 起算，不包括帧头与命令 ID。

| 载荷长度 | 底盘功率上限 | 射速上限 | 输出标志 |
| --- | --- | --- | --- |
| 13 字节 | 偏移 10，小端 uint16，单位 W | 不携带，解析后置零 | 偏移 12 |
| 17 字节 | 偏移 10，小端 uint16，单位 W | 偏移 12，float | 偏移 16 |

输出标志的 bit 0/1/2 分别表示云台、底盘、射击机构。不能将 17 字节帧直接按 13 字节结构复制，否则会把射速 float 的首字节当输出标志。内部 `ext_game_robot_status_t` 已包含 `bullet_speed_limit`；必须重新构建并下载整份固件。

无需条件断点，添加以下常驻 Watch：

```c
My_Judge.status->robot_state_rx_count
My_Judge.status->robot_state_update_ms
My_Judge.status->robot_state_length
My_Judge.status->robot_state_limit_raw
My_Judge.status->robot_state_result
```

`robot_state_rx_count` 记录所有通过 CRC 并进入状态帧解析的次数，包括拒收帧；`robot_state_update_ms` 是最近一次尝试的时间；`robot_state_limit_raw` 是支持布局中读到的功率上限，不支持的长度记零。

| `robot_state_result` | 含义 |
| --- | --- |
| 0 | 初始化后还未处理状态帧 |
| 1 | 已接纳非零功率上限 |
| 2 | 长度不是已核对的 13/17 字节 |
| 3 | 长度支持，但功率上限为零，未接纳 |

未知长度、零上限与 CRC 错误均不会刷新 `limit_update_ms` 或覆盖最后有效上限；CRC 错误帧不进入 `judge_update()`，也不会增加 `robot_state_rx_count`。13/17 字节并不能代表任意版本的通用兼容：如发送端布局变化，应重新核对字段定义。

下载后，先确认 `robot_state_length=17`、`robot_state_result=1`、`limit_received=1`、`limit_rx_count` 持续增长，以及 `game_robot_status.chassis_power_limit` 为实际收到的非零上限；再检查功率预算。正常接收仍为零上限时保持拒收，不用固定功率掩盖通信问题。

底盘初始化中的 `Chassis_Power_Init()` 和发送前的 `Chassis_Power_Apply()` 已恢复调用；这两处如果被注释，接收正常也不会执行限流。

## 第三组 Watch：允许功率怎么得到

```c
chassis_power.base_budget_w
chassis_power.measured_warning_w
chassis_power.measured_soft_derate_w
chassis_power.measured_bias_derate_w
chassis_power.measured_derate_w
chassis_power.buffer_derate_w
chassis_power.feedback_derate_w
chassis_power.target_power_w
```

### 基础预算

`base = max(0,有效来源的上限-margin_w)`。例如裁判实收 80 W、余量 5 W，基础预算为 75 W；基础预算不会因为缓冲充足或超电有电而放大。

### 实测提前与偏差修正

警戒值 `warning=max(0,base-measured_warning_margin_w)`，默认提前 2 W，75 W 基础预算对应 73 W 警戒。

提前目标 `soft=clamp(measured_soft_kp*max(0,实测-warning),0,min(measured_soft_max_w,base))`，默认比例 1、最多 5 W。实测 74 W 要求 1 W，75 W 要求 2 W；它不是每帧累加。长期稳定在 74 W 不会把预算无限扣到零。

只有实测超过基础预算，偏差才按 `measured_integral_gain_per_s*(实测-base)*frame_dt` 增长。默认增益 1/s；50 ms 和 200 ms 有效反馈对同一持续时间的超额产生相同积分。首次、相同时间戳、跨超时的新帧不积分未知时间，仅建立基准。

`measured_derate_w=min(base,soft+bias)`。提前目标增大立即采用；目标减小并不立即放开，需下述联合恢复确认。

### 缓冲修正

目标降额 `buffer_kp*max(0,buffer_target_j-实收缓冲)`，默认 60 J、1 W/J：59 J 要求 1 W，55 J 要求 5 W，0 J 是有效反馈并要求 60 W 降额（最多基础预算）。高于目标不增加基础预算。

`buffer_drop_j`、`buffer_drop_rate_w`、`buffer_drop_detected` 保留最近有效新帧的下降信息，方便 Watch 观察；它们不直接控制电流。下降率是 J/s，不是底盘总功率。长失联后首帧不算跨失联下降率。

### 合并与最终预算

```text
feedback_derate_w = min(base, max(buffer_derate_w, measured_derate_w))
target_budget = max(0, base - feedback_derate_w)
```

例如基础 75 W、缓冲修正 1 W、实测修正 1 W，最终目标为 74 W，不是 73 W。两类反馈可能是同一次超额的结果，采用较强修正。实际 `target_power_w` 在恢复期间可以低于新目标，但不能超过基础预算。

## 恢复：下降快、回升慢

同时满足以下条件并持续默认 500 ms，才置 `recovery_ready`：裁判上限、缓冲、超电功率都新鲜；实测低于警戒值减默认 2 W；缓冲满或最近连续有效相邻帧正在上升。未满且保持不变不能单独证明足够余量。失联、下降、功率偏高会重新开始确认。

- 缓冲修正仅在有效连续新缓冲帧上释放，最大 10 W/s。
- 两项实测修正共用 1 W/s 的释放额度，先释放提前项多余部分，再释放偏差；不重复使用恢复额度。
- 最终预算回升最多 20 W/s。约 2 ms 的控制更新最多回升 0.04 W。
- 过期冻结对应修正并禁止最终预算上涨；相同反馈帧不积分、不释放。

观察 `buffer_recovering`、`recovery_stable_ms`、`recovery_ready` 可解释为什么暂时没有恢复。

## 第四组 Watch：最终电流与模型约束

```c
chassis_power.predicted_before_w
chassis_power.predicted_after_w
chassis_power.current_scale
chassis_power.current_before
chassis_power.current_after
chassis_power.model_fault
chassis_power.config_fault
chassis_power.budget_unreachable
```

每轮模型 `k_i2*i*i+k_iw*i*rpm+k_w2*rpm*rpm+k_const`，`i` 是 CAN 原始电流值，`rpm` 是编码器实际电机 RPM。制动负功率按零，不抵扣其他轮耗电。16 次二分求共同电流系数，并复核最终 CAN 整数电流。

`current_scale=1` 表示不缩小，小于 1 表示限流，0 表示撤去驱动。零电流下模型常数/转速损耗仍可能非零；如果它们已经超预算，则 `budget_unreachable=true`，电流清零不等于立即停车或实际功率立刻为零。默认等待裁判上限时预算为零也可能出现此标志；先看 `limit_source`。

模型系数为参考起点，需要本车标定。软件约束的是预测功率，实测反馈也有延迟，不能承诺每个瞬间实际功率绝不超限。调试时比较 `predicted_after_w` 与真实 `cap.info->chassis_power`，并观察裁判缓冲是否持续下降。

## 参数默认值

| 参数 | 默认 | 单位/用途 |
| --- | --- | --- |
| `training_limit_w` | 0 | W，禁用无裁判离线额度，设正值才启用 |
| `margin_w` | 5 | W，裁判上限下方余量 |
| `buffer_target_j` | 60 | J，满缓冲目标，不是当前能量 |
| `buffer_kp` | 1 | W/J，缓冲缺额降额 |
| `buffer_recovery_w_per_s` | 10 | W/s，缓冲降额释放速度 |
| `measured_warning_margin_w` | 2 | W，警戒值比基础预算低多少 |
| `measured_soft_kp` | 1 | W/W，提前比例 |
| `measured_soft_max_w` | 5 | W，提前修正上限 |
| `measured_integral_gain_per_s` | 1 | 1/s，持续超基础预算偏差修正，替代旧 measured_down_gain |
| `measured_recovery_w_per_s` | 1 | W/s，两项实测修正共同释放额度 |
| `measured_hysteresis_w` | 2 | W，警戒值下方恢复迟滞 |
| `recovery_hold_ms` | 500 | ms，联合条件连续确认 |
| `budget_recovery_w_per_s` | 20 | W/s，最终预算回升速度 |
| `buffer_drop_threshold_j` | 0.5 | J，仅下降提示 |
| `limit_timeout_ms` / `buffer_timeout_ms` / `cap_timeout_ms` | 500 / 500 / 300 | ms，各自有效期 |
| `k_i2` / `k_iw` / `k_w2` / `k_const` | 1.23e-7 / 1.99688994e-6 / 3e-7 / 4.081 | 每轮模型系数，需实测标定 |

## 验证命令

```powershell
python -X utf8 -m unittest discover -s test/power -p test_*.py -v
python -X utf8 -m unittest discover -s test/chassis -p test_*.py -v
python -X utf8 -m unittest discover -s test/referee -p test_*.py -v
python -X utf8 -m unittest discover -s test/gimbal -p test_*.py -v
python -X utf8 test/build_firmware.py
```

主机测试执行实际 C 算法/快照/组发送链路；超电测试链接时不提供发送函数，验证 TX 仍停用。ARMCC 构建生成临时 AXF/HEX，不覆盖原 Keil 产物，不烧录。实车验收仍需确认供电状态、实测与裁判测量位置、模型系数和动态余量。
