# GYRO 键鼠底盘跟随前馈 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** 鼠标转动云台时，底盘通过与遥控同一条前馈链路提前跟随。

**Architecture:** DBUS 接收记录真实帧间隔；云台把每帧角增量换算为等效 ch0；底盘通道获取函数将其交给原前馈链路，并清除切换历史。

**Tech Stack:** C99、STM32H7 HAL、FreeRTOS、ARMCC5；Python unittest 编译真实 C 到主机 GCC。

## Global Constraints

- 原控制周期 2 ms、鼠标角增量每帧只消费一次；不改变瞄准灵敏度和 PID 参数。
- 等效通道限幅 ±660，键鼠倍率默认 1；有效帧间隔及持有时限 50 ms。
- 不改用户功率旁路、裁判和超电，不烧录；既有未提交修改不收入本次提交。

### Task 1: 接入鼠标前馈并验证

**Files:**
- Modify: `Application/DeviceLayer/Sensor/rc_sensor.h`, `rc_sensor.c`，接收时间字段及复位。
- Modify: `Application/ProtocolLayer/rc_protocol.c`，记录有效帧间隔。
- Modify: `Application/ModuleLayer/gimbal.h`, `gimbal.c`，前馈状态、计算和失效处理。
- Modify: `Application/ControlLayer/carctrl.c`, `car_control.h`，GYRO 鼠标等效通道及说明。
- Modify: `Application/ModuleLayer/chassis.c`, `chassis.h`，前馈历史复位接口。
- Test: `test/chassis/test_keyboard_gyro.py`, `test_keyboard_feedforward.py`。
- Create: `docs/keyboard-yaw-feedforward.md`。

**Interfaces:**
- 输入：`rc_sensor.info` 的 frame_seq、mouse_x、mouse_vx、frame_update_ms、frame_interval_ms、frame_received。
- 输出：`hero_gimbal.yaw.key_yaw_speed_rads`、`key_yaw_ff_channel`、`key_yaw_ff_valid`；`key_yaw_ff_gain` 默认 1。
- `Chassis_Reset_Yaw_Feedforward(void)` 清底盘滤波；`Chassis_Update` 签名不变。

- [ ] **Step 1: 写失败用例并确认原代码没有前馈。**

主机用例调用真实云台目标更新、通道获取和底盘函数：

```c
hero_car.input_src=INPUT_SRC_KEY;hero_car.mode=CAR_MODE_GYRO;
hero_gimbal.mode=GIMBAL_MODE_GYRO;
rc.mouse_x=20;rc.mouse_vx=20;rc.frame_seq=1;
Gimbal_Key_Yaw_Update(&hero_gimbal);
Car_Chassis_Channel_Get(&hero_car,&f,&l,&r);
Chassis_Update(f,l,r,0,ALIGN_FRONT,CAR_MODE_GYRO,false);
CHECK(wheel_pid[0].target*CHASSIS_FL_DIRECTION>0 && wheel_pid[1].target*CHASSIS_FR_DIRECTION<0);
```

Run: `python -X utf8 -m unittest discover -s test/chassis -p test_keyboard_feedforward.py -v`；Expected: 零夹角旋转断言失败。

- [ ] **Step 2: 实现有界前馈与时序。**

接收分支按有效状态维护时间，消费侧以短临界区复制字段，使用以下换算，输出通道在两个新帧之间保持：

```c
float speed = -mouse_x * MOUSE_YAW_SENS * 1000.0f / (float)frame_interval_ms;
float channel = -speed * 0.002f / gimbal->yaw.gyro_sensitivity * gimbal->yaw.key_yaw_ff_gain;
gimbal->yaw.key_yaw_ff_channel = (int16_t)constrain(channel, -660.0f, 660.0f);
```

计算前检验帧接收标记、间隔非零且不大于 50 ms、帧龄小于 50 ms、原始鼠标不为零、参数与结果有限非负/有效。不满足则通道和有效标志清零；切换时同步序号，等待新帧。底盘映射仅 GYRO 使用等效通道，Ctrl/SPIN 保持零，MECH 保留 Q/E。

- [ ] **Step 3: 补充验收矩阵并跑回归。**

验证 14 ms 输入 20 与 28 ms 输入 40 的速度均为 `-0.02/0.014 rad/s`、通道约 190；同帧调用七次目标仍为 -0.02 rad；零原始输入、50 ms 超时、零间隔、NaN、换源/模式/失联均停止前馈，反向和满输入有界。测试真实接收回调的首帧、正常、错误、重连和计时回绕。回归时记录用户注释功率调用造成的四项已有链路失败，本次不改变这项调试旁路；单独执行已有功率算法测试。

Run: `python -X utf8 -m unittest discover -s test/chassis -p test_keyboard*.py -v`、同命令替换路径为 `test/gimbal`、`test/referee`、`test/power` 并使用 `test_*.py`；Expected: PASS。完整 chassis 套件应只有上述四项修改前已复现的功率旁路失败。

- [ ] **Step 4: 文档、构建、选择性提交。**

文档解释换算、同帧保持、零输入与均值滤波尾部、Watch 字段和调参。Run: `python -X utf8 test/build_firmware.py`；Expected: ARMCC5 编译/链接成功。用本次开始时各文件快照生成仅本次差异的暂存内容，检查暂存路径和 diff，再执行 `git commit -m "feat: 为陀螺仪键鼠增加底盘跟随前馈"`。
