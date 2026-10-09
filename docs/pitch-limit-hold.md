# Pitch 陀螺仪模式限位目标保持

原先持续打满摇杆时，每周期将目标截到 `imu_angle + (机械限位 - 机械反馈)`。
IMU 和编码器的噪声、报文采样时差会直接改变目标，进而扰动上板角度环。

现在锁存 `imu_angle - pitch_mech_angle` 作为限位坐标参考，只有相对锁存值
累计变化超过 `PITCH_LIMIT_FRAME_HYST_RAD` 才更新。持续向外输入被同一边界
截断，不积压；反向输入当拍生效。键鼠和摇杆共用此规则。
切模式、休眠或反馈丢失会使参考失效，恢复时按当前反馈重建。

`gimbal.h` 中阈值初值为 0.5°；IMU 模式两端各内缩同等余量，覆盖锁存期间
未跟踪的坐标差。机械模式仍使用原机械限位。机身实际倾斜时边界仍会更新，
只修正已不可达的目标，不平移仍在可达范围内的世界系目标。

这是一种有阈值的目标保持，不是同步采样，也不能区分超过阈值的噪声与真实
姿态变化。阈值需根据实机 `imu_angle - pitch_mech_angle` 的波动校验；软件
目标余量不能代替电机输出保护，也不保证机械运动没有超调。

验证命令：

```powershell
python -m unittest discover -s test/gimbal -p test_pitch_limits.py
```

测试执行实际 `Gimbal_Pitch_Update` 函数，覆盖双向持续打满、小幅反馈扰动、
立即反向、反复进出限位、机身快慢倾斜、反馈恢复、模式切换及键鼠输入。
旧 `test_gimbal_imu_contract.ps1` 在本次修改前已失败：仍要求 GYRO 切 MECH
时将 IMU 目标直接复制为机械目标；现有实现使用回传机械角，本次不改该规则。

烧录下板后固定底盘验证：到达两端并持续打满时，查看
`hero_gimbal.pitch.target_imu_angle` 和 `hero_gimbal.pitch.imu_limit_offset`。
阈值内的反馈扰动不应改变限位目标，反向推杆应立即回退。
