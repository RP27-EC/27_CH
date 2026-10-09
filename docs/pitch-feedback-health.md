# Pitch 使能与机械反馈诊断

下板 `is_pitch_mech_online` 失效会令 `target_rebase_pending` 置位、目标对齐当前 IMU 角，并撤销 `pitch_enable`。上板接到失能后清空 PID、将 `give_torque` 置零。这条链路可以解释目标跟着手动运动以及力矩在零和正常值间变化；仍须根据帧诊断确定实机最初失效的原因。

## 本次修改

- 上板 0x023 的机械有效标记改由达妙反馈在线状态决定，不再借用 IMU 有效位。
- IMU 校准或错误期间继续刷新机械反馈，但仍禁止 Pitch 驱动。
- 达妙状态初始离线，收到反馈立即上线，心跳仅负责超时下线。离线停机期间发送 0xFD 失能报文，避免等待反馈和等待使能互锁；实际电机是否回报仍需实机确认。
- 下板校验完整帧后一次写入机械在线位，不再对每个正常帧先写 false 再写 true。不能仅凭这个瞬态写入认定它就是控制抖动的根因。
- 真正的无效帧、超时仍会失能，没有增加掩盖故障的延时或锁存。

原达妙心跳在超时饱和后会保持离线，并不会逐拍自行恢复在线。本次修复的是首次反馈前的虚假在线和接收后的在线状态发布。

## 上板 Watch

| 变量 | 含义 |
| --- | --- |
| `board_tx_info.pitch_mech_valid` | 本次 0x023 的机械角有效标记来源 |
| `board_tx_info.state_tx_fail_count` | 0x023 CAN 入队失败累计次数；持续增加需检查发送邮箱/总线负载 |
| `Pitch_Motor.state->status` | `DEV_ONLINE` / `DEV_OFFLINE`；不要按数字 0/1 猜语义 |
| `imu_sensor.work_state.err_code`、`cali_end` | 必须无错误且校准完成才允许上板控制 |

## 下板 Watch

以下字段均在 `board_rx_info` 内：

| 字段 | 含义 |
| --- | --- |
| `pitch_mech_invalid_reason` | 0=有效；1=标记不是 0xA5；2=角度非有限值或超出 ±π；3=状态帧超时/尚未收到 |
| `state_last_marker` | 最近一次 0x023 的 byte[5]；机械反馈有效时应为 0xA5 |
| `state_rx_count` | 收到 0x023 的累计次数 |
| `state_invalid_count` | 收到但机械角无效的累计次数 |
| `state_timeout_count` | 状态帧从在线变超时的累计次数，不逐周期重复累加 |

`is_online` 只反映 0x021，不能证明 0x023 或 Pitch 机械反馈正常。观察稳定运行区间内计数是否继续增加；启动期间尚无电机反馈时可能已经记录无效帧。

更新上下板后，校准完成、反馈正常且进入运行模式时，应看到 `is_pitch_mech_online=1`、`target_rebase_pending=0`、`pitch_enable=1` 稳定。若再次抖动，记录发生时的上述原因码与累计计数，不先调整 PID。

## 验证记录

- 上板主机测试：`test/imu` 12 项、`test/gimbal` 9 项通过。
- 下板主机测试：`test/gimbal` 8 项通过。
- 上下板 Keil 增量构建均为 0 errors / 0 warnings；未烧录、未进行实机验证。
- 下板旧 `test_gimbal_imu_contract.ps1` 未通过：仍要求退出 GYRO 时把 IMU 目标直接作为机械目标，与当前按机械反馈重定位的实现不符。本次未修改该旧脚本，也未为其回退控制逻辑。

测试覆盖校准禁止驱动、IMU 错误时机械角仍更新、反馈收发有效性、离线/恢复、无效标记与角度、状态帧超时计数，以及 CAN 入队失败计数。主机测试不能代替真实 CAN 报文与电机响应验证。
