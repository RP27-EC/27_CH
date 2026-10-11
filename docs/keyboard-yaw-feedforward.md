# GYRO 键鼠底盘跟随前馈与调试

鼠标仍按原灵敏度更新云台世界坐标系目标角，每个 DBUS 帧只累加一次。新增的是底盘的输入前馈：即使实际云台与底盘的夹角还为零，鼠标转动也能先给底盘旋转指令，夹角反馈随后修正偏差。

## 数据链路

```text
USART5_rxDataHandler（rc_protocol.c）
  -> 记录有效 DBUS 帧时间和实际帧间隔
Gimbal_Update -> Gimbal_Key_Yaw_Update（gimbal.c）
  -> 短临界区复制序号、鼠标、时间和 Ctrl
  -> 每帧一次更新角目标，并计算鼠标期望转速与等效 ch0
Car_Chassis_Channel_Get（carctrl.c）
  -> GYRO 键鼠把等效 ch0 放入 rotate
Chassis_Update（chassis.c）
  -> 原通道死区、原低通与原前馈增益
  -> 与真实机械夹角的跟随项相加、限幅、麦轮解算、四轮 PID
```

输入源和模式切换、底盘停车通过 `Chassis_Reset_Yaw_Feedforward()` 清低通历史。`Chassis_Update()` 的接口及遥控通道映射保留。功率调用按用户本次开始时的调试版本保持注释；需要恢复功率限制时，必须另外恢复头文件、初始化、停车清观察量及发送前限流调用。

## 为什么不能除以 2 ms

鼠标每个 DBUS 帧携带一次位移；2 ms 是控制任务周期。接收侧 `frame_interval_ms` 记录相邻有效帧的实际间隔：

```c
angle_delta = -mouse_x * MOUSE_YAW_SENS;
speed_rads = angle_delta * 1000 / frame_interval_ms;
equivalent_ch0 = -speed_rads * 0.002 / gyro_sensitivity * key_yaw_ff_gain;
```

负号与原遥控关系 `angle_delta_per_tick = -ch0 * gyro_sensitivity` 对应。鼠标右移时角增量为负、等效 ch0 为正，底盘再取反得到向右旋转。

例如滤波鼠标值 20、灵敏度 0.001、实际间隔 14 ms：角增量 -0.02 rad，期望转速约 -1.429 rad/s。当前摇杆灵敏度 0.000015，键鼠倍率 1，对应等效通道约 +190。28 ms 内位移 40 的期望转速相同，前馈通道也相同。

等效通道限制在 ±660，底盘继续使用 30 通道死区、0.20 低通系数和 0.8 前馈增益。该换算匹配遥控前馈尺度，不代表底盘实际角速度已标定；快速甩鼠标或边移动边转动时，轮速能力、惯性和轮速整体缩放仍会限制跟随。

## 清零与保持

- 新帧到来时计算一次；两个新帧之间保持速度前馈，不重复累计鼠标位移。
- 首帧没有真实间隔、间隔为零或大于 50 ms：不计算前馈，等待连续有效新帧。
- 最近帧龄达到 50 ms：清零；接收序号和 HAL 毫秒计时均支持无符号回绕。
- 新帧的原始 `mouse_vx=0`：立即清零前馈及底盘低通尾值。
- Ctrl 微调、非 GYRO、切换输入源、云台未工作、遥控异常或上板失联：关闭鼠标前馈；重新进入等待新帧。
- 倍率为零/负数/非有限数、摇杆灵敏度无效、鼠标值非有限或超出 int16 均值可能范围：清零。异常鼠标浮点值也不会写入角目标。

原有鼠标十帧滑动均值滤波保留。因此原始鼠标归零之后，滤波后的鼠标值和角目标仍可能有原有的尾部；停止输入前馈不会取消角度闭环，底盘仍会修正已经形成的真实夹角。

## Keil Watch

| 表达式 | 含义 |
| --- | --- |
| `hero_gimbal.yaw.key_yaw_ff_gain` | 键鼠前馈独立倍率，初始化为 1；0 停用鼠标前馈 |
| `hero_gimbal.yaw.key_yaw_speed_rads` | 本次有效鼠标前馈采用的目标转速，rad/s，失效时清零 |
| `hero_gimbal.yaw.key_yaw_ff_channel` | 限幅后的等效 ch0，±660，送入底盘前馈 |
| `hero_gimbal.yaw.key_yaw_ff_active` | 当前模式与设备条件允许使用前馈 |
| `hero_gimbal.yaw.key_yaw_ff_valid` | 当前有新鲜、连续、非零鼠标输入及有效参数 |
| `hero_gimbal.yaw.feedback_mec_angle` | 云台相对底盘的真实夹角，rad |
| `rc_sensor.info->frame_interval_ms` | 两次有效 DBUS 帧的间隔，ms |
| `rc_sensor.info->frame_update_ms` | 最近有效 DBUS 帧的到达时间，ms |

先比较相同鼠标动作下倍率 0 与 1 的跟随夹角。倍率增大会增强前馈；追赶不足可小步增加，底盘抢跑或明显超过云台时降低。倍率改动不修改鼠标瞄准灵敏度、云台 PID 或夹角跟随增益；达到 ±660 后再增加倍率不会继续提高前馈。

## 验证

```powershell
python -X utf8 -m unittest discover -s test/chassis -p test_keyboard*.py -v
python -X utf8 -m unittest discover -s test/chassis -p test_rc_frame_timing.py -v
python -X utf8 -m unittest discover -s test/gimbal -p test_*.py -v
python -X utf8 -m unittest discover -s test/referee -p test_*.py -v
python -X utf8 -m unittest discover -s test/power -p test_*.py -v
python -X utf8 test/build_firmware.py
```

主机验证真实 C 的角目标、通道映射、底盘跟随和轮速目标，以及真实接收回调的时间记录。`test_power_integration.py` 的四项链路测试在用户当前注释功率调用的版本中会失败，本次开始前已复现；功率算法独立测试另行验证。ARMCC 构建输出到临时目录，不覆盖 Keil 原产物、不烧录。实车需重新构建、下载后确认方向与实际跟随效果。
