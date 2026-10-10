# Pitch 陀螺仪模式静态限位

## 问题

持续打满摇杆时目标呈锯齿：限位边界曾经由 `imu_offset + 机械限位` 动态推算，
坐标差带采样时差噪声，边界一动被截断的目标就跟着跳，PID 被跳变目标反复激励。
锁存/边界保持都只是压噪声，路径本身还在。

## 现行为（静态限位，和机械模式同一形状）

操作手标定后定死，两板同名同值，改一处必须改两处：

```c
#define PITCH_MIN_IMU_ANGLE  (0.04f)   /* IMU 世界域 (rad) */
#define PITCH_MAX_IMU_ANGLE  (0.70f)
```

- 下板 `Gimbal_Pitch_Update` GYRO 域：积分后直接截断这两个常数；
  MECH 域截机械行程（`min_limit`/`max_limit`，不变）。
- 上板 `Gimbal_Mech_Target_Update` 按 `use_imu` 选同一对常数或机械行程
  做最终限幅（设计文档的 final authority）。
- 0xD1 的目标角域随 `pitch_use_imu` 声明：MECH=编码器机械域，
  GYRO/SPIN=IMU 世界域（见 ProtocolLayer/communicate.h）。

边界是常数，打满时每拍截到同一个数，目标必然是水平直线；
反馈抖动、车身倾斜都改不了它。反向输入当拍离开限位，行程内目标
不跟随反馈平移。原先的坐标差锁存（`imu_limit_offset`）与边界保持
（`held_target`/`target_held`）整套删除。

## 已知取舍

- 静态世界限位不跟随车身姿态：上坡时限位下端、下坡时上端可能顶到
  机械硬限位，打满就顶住。软件限位不代替电机输出保护，也没有做
  硬限位切力矩（评估后暂缓）；需要时在 `Gimbal_Pitch_Pid_Cal`
  输出端按 `feedback_pitch` 方向砍向外力矩即可（注意现有注释版本符号是反的）。
- 标定值跨度 0.66 rad，机械行程约 0.86 rad，比动态映射少约 12° 行程。
- 两板常数是复制粘贴的两份，没有公共头文件；改限位必须两处一起改。

## 验证

```powershell
# 下板目标生成
python -m unittest discover -s test/gimbal -p test_pitch_limits.py
# 上板最终限幅（Up\RP27-Train_Template-C\test\gimbal）
python -m unittest discover -s test/gimbal -p test_pitch_clamp.py
```

覆盖：双向打满在反馈抖动与车身倾斜下目标逐拍恒定（锯齿回归）、
反向当拍离开限位、行程内目标不跟随反馈、键鼠同限位、断联重建、
模式切换、机械模式行程限位。

烧录两板后固定底盘验证：到两端持续打满时看
`hero_gimbal.pitch.target_imu_angle`，必须是 0.70 / 0.04 的水平直线。
反向推杆立即离开边界。上板看 `gimbal.pitch.imu_angle_pid.target`
应同步平稳。
