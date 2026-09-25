# 上下层板 IMU 通信设计

## 目标与职责边界

上层 C 板仅作为云台 IMU 数据供应商，不实现 Yaw 角度环、角速度环、电机控制或底盘跟随控制。

数据链路固定为：

```text
上层 BMI088 采集与姿态解算
  -> yaw_imu + yaw_v 打包
  -> CAN2 / 标准帧 0x021
  -> 下层解包
  -> Yaw 陀螺仪双环 PID
  -> KT9025 电机输出
  -> 底盘跟随闭环
```

下层板继续独占以下行为：

- 右摇杆生成云台空间 Yaw 目标。
- Yaw 空间角度外环和角速度内环。
- KT9025 电机电流输出。
- 云台相对底盘夹角计算及底盘跟随。

## 上层模块

在 `Application/ProtocolLayer` 新增 `board_comm.h` 和 `board_comm.c`。模块只负责协议打包和发送，不读取遥控器，也不执行控制计算。

公开接口：

```c
void Board_Comm_Init(void);
HAL_StatusTypeDef Board_Comm_Send_IMU(float yaw_angle_rad,
                                      float yaw_rate_rad_s);
```

发送函数调用工程现有 `CAN_SendData(&hcan2, 0x021, tx_buf)`，并返回 HAL 状态，避免引入不存在的 `CAN_Send_Msg()` 占位接口。

## 协议

- CAN 总线：上层 `CAN2`，对应参考工程板间总线及下层 `FDCAN2` 接收分支。
- 帧类型：11 位标准数据帧。
- CAN ID：`0x021`。
- DLC：8 字节。
- Byte 0..3：IEEE-754 单精度 `yaw_imu`，单位 rad。
- Byte 4..7：IEEE-754 单精度 `yaw_v`，单位 rad/s。
- 字节序：STM32 小端内存序，通过 `memcpy` 打包，与下层当前 `memcpy` 解包严格对应。

协议不发送 C 结构体本体，避免填充和对齐影响帧布局。

## IMU 数据与时序

上层现有 `imu_sensor.info->base_info.yaw` 和 `rate_yaw` 均为度制。发送前统一乘以 `PI / 180`：

```text
yaw_imu = base_info.yaw * PI / 180
yaw_v   = base_info.rate_yaw * PI / 180
```

发送调用放在上层 `StartControlTask()` 的 `imu_sensor.update()` 成功执行之后，与现有任务保持 2 ms 周期，即 500 Hz。只有 IMU 状态允许更新时才发送本周期姿态，防止把未初始化或失效数据持续刷新给下层。

不在通信模块内对 Yaw 再次积分、滤波或改变正负号。符号一致性在台架测试阶段通过正向手动转动云台确认；如物理坐标方向相反，只在单一协议边界配置符号，不在上下层重复取反。

## 下层接收与安全

下层现有 `Board_Rx_C1()` 保持两个 `float` 的解包方式，并刷新在线状态。陀螺仪模式必须把 `board_rx_info.is_online` 纳入安全条件：超过 100 ms 未收到 `0x021` 时，退出或禁止陀螺仪闭环输出，清空相关 PID，不能继续使用冻结的 IMU 数据驱动 Yaw 电机。

同时修正下层控制任务中云台机械夹角字段引用，使其与 `Gimbal_Yaw_t.feedback_mec_angle` 定义一致。

## 工程集成

- 将 `board_comm.c` 加入 Keil 工程的 `Protocol` 分组。
- 头文件沿用已有 `Application/ProtocolLayer` include path。
- 不修改底层 CAN 驱动，不新建重复的 CAN 发送函数。
- 不在上层新增云台控制对象或 PID 参数。

## 验证

1. 主机侧协议测试验证 CAN ID、DLC、两个 float 的偏移和度到弧度换算。
2. 静态契约测试验证发送使用 `hcan2`、调用位于 IMU 更新之后、Keil 工程包含新增源文件。
3. 下层测试验证 0x021 解包、在线计数刷新、超时后陀螺仪输出被禁止。
4. 使用 Keil 命令行构建上下层工程；若本机缺少可用编译器，明确记录未执行项。
5. 台架验证时先架空车轮并限制 Yaw 输出，检查角度与角速度符号，再进行底盘跟随测试。
