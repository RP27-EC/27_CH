\# Up 板 IMU 数据的完整代码链路



从 \*\*SPI 寄存器\*\*到 \*\*Pitch 反馈/目标\*\*，一共经过 \*\*6 个文件、7 层变换\*\*。我逐层列清楚。



\---



\## 链路总览



```

① BMI088Middleware.c   SPI 单字节交换 + 片选 + 微秒延时 + 失败计数

&#x20;       ↓  uint8\_t

② BMI088driver.c       int16 拼装 + 量程换算 → rad/s、g、℃

&#x20;       ↓  fp32 gyro\[3] / accel\[3] / temp

③ imu\_sensor.c         ○ 坐标变换（矩阵）

&#x20;                      ○ 陀螺零偏校准（只在前 2000 次）

&#x20;                      ○ 一阶低通

&#x20;                      ○ Mahony 四元数解算 → 欧拉角

&#x20;                      ○ 角速度换算 + 均值滤波

&#x20;       ↓  base\_info.{pitch, rate\_pitch, yaw, rate\_yaw}

④ control\_task.c       把 (pitch, rate\_pitch, yaw, rate\_yaw) 喂给 Pitch

&#x20;       ↓

⑤ gimbal\_pitch.c       选择反馈源 + 生成目标 → PID → 力矩

&#x20;       ↓

⑥ 电机（MIT 力矩帧，CAN2 ID 0x001）

```



\---



\## ① `BMI088Middleware.c` —— 最底层，只管"把字节换出来"



| 函数 | 做什么 |

|---|---|

| `BMI088\_ACCEL\_NS\_L/H()` `BMI088\_GYRO\_NS\_L/H()`（`:57-73`） | 拉低/拉高两个片选（加速度计、陀螺仪各一个 CS） |

| `BMI088\_read\_write\_byte()`（`:76-81`） | `HAL\_SPI\_TransmitReceive(\&hspi1, \&tx, \&rx, 1, 5ms)` 单字节全双工交换；\*\*失败就 `++BMI088\_io\_errors`\*\* |

| `bmi2\_micros()`（`:27-38`） | 用 `HAL\_GetTick()\*1000 + TIM2->CNT` 拼出微秒级时间，给初始化延时用 |



\*\*硬件参数\*\*（`Core/Src/spi.c`）：`SPI1`，Mode 3（`CLKPolarity=HIGH` + `CLKPhase=2EDGE`），8bit，MSB，\*\*预分频 16 → 5.25 Mbit/s\*\*，引脚 `PB3=SCK / PB4=MISO / PA7=MOSI` + 两个软片选。



\*\*这一层唯一的"改动"\*\*：记了一个 SPI 错误计数 `BMI088\_io\_errors`，供上层判断"这一帧能不能信"。



\---



\## ② `BMI088driver.c` —— 原始字节 → 物理量



\### 初始化时定了量程（`BMI088driver.h:44,50` + `BMI088driver.c:73-93`）



| 项 | 值 | 出处 |

|---|---|---|

| 加速度量程 | \*\*±3g\*\*，灵敏度 `0.0008974358974` | `BMI088\_ACCEL\_3G\_SEN` |

| 加速度 ODR | \*\*800 Hz\*\*，Normal 模式 | `BMI088\_ACC\_800\_HZ` |

| 陀螺量程 | \*\*±2000 °/s\*\*，灵敏度 `0.0010652644...`（rad/s per LSB） | `BMI088\_GYRO\_2000\_SEN` |

| 陀螺带宽 | \*\*1000Hz ODR / 116Hz 低通\*\* | `BMI088\_GYRO\_1000\_116\_HZ` |



> 注意单位：\*\*陀螺输出是 rad/s\*\*（`0.0010652644 rad/s per LSB` = 2000°/s ÷ 32768 × π/180），不是 °/s。这一点在 ③ 里会被换算。



\### 采样 `BMI088\_read()`（`:203-241`）



```c

BMI088\_read\_valid = 0U;                                    // 先清有效性

uint32\_t errors = BMI088\_io\_errors;                        // 记下当前 SPI 错误数



BMI088\_accel\_read\_muli\_reg(BMI088\_ACCEL\_XOUT\_L, buf, 6);  // 一次读 6 字节

accel\[0] = (int16\_t)(buf\[1]<<8 | buf\[0]) \* BMI088\_ACCEL\_SEN;   // 小端拼装 × 灵敏度

accel\[1] = (int16\_t)(buf\[3]<<8 | buf\[2]) \* BMI088\_ACCEL\_SEN;

accel\[2] = (int16\_t)(buf\[5]<<8 | buf\[4]) \* BMI088\_ACCEL\_SEN;



BMI088\_gyro\_read\_muli\_reg(BMI088\_GYRO\_CHIP\_ID, buf, 8);    // 一次读 8 字节，首字节是 CHIP\_ID

if (buf\[0] == BMI088\_GYRO\_CHIP\_ID\_VALUE) {                 // 用 ID 校验这次读的合法性

&#x20;   BMI088\_read\_valid = 1U;

&#x20;   gyro\[0] = (int16\_t)(buf\[3]<<8 | buf\[2]) \* BMI088\_GYRO\_SEN;

&#x20;   gyro\[1] = (int16\_t)(buf\[5]<<8 | buf\[4]) \* BMI088\_GYRO\_SEN;

&#x20;   gyro\[2] = (int16\_t)(buf\[7]<<8 | buf\[6]) \* BMI088\_GYRO\_SEN;

}



BMI088\_accel\_read\_muli\_reg(BMI088\_TEMP\_M, buf, 2);

BMI088\_read\_valid = BMI088\_read\_valid \&\& (errors == BMI088\_io\_errors);  // SPI 无新错误才算有效

temp\_raw = (int16\_t)(buf\[0]<<3 | buf\[1]>>5);              // 12 位温度

if (temp\_raw > 1023) temp\_raw -= 2048;                    // 符号扩展

\*temperate = temp\_raw \* 0.125f + 23.0f;                   // ℃

```



\*\*这一层的改动\*\*：

1\. int16 → 物理量（gyro 是 \*\*rad/s\*\*，accel 是 \*\*g\*\*，temp 是 \*\*℃\*\*）

2\. \*\*输出一个有效性标志\*\* `BMI088\_read\_valid` = 陀螺 ID 正常 \*\*且\*\* 本次 SPI 没有新增错误



\*\*所以"数据可信"的第一道门在这里。\*\*



\---



\## ③ `imu\_sensor.c` —— 滤波与姿态解算（改动最多的一层）



入口 `imu\_update()`（`:162-299`），按顺序做 6 件事：



\### 3.1 取原始数据 + 有效性判断（`:168-181`）



```c

BMI088\_read(gyro, accel, \&temp);

if (!BMI088\_read\_valid) {                      // 无效帧直接拒绝

&#x20;   imu\_sen->work\_state.err\_code = IMU\_DATA\_ERR;

&#x20;   imu\_sen->work\_state.dev\_state = DEV\_OFFLINE;

&#x20;   imu\_sen->work\_state.cali\_end = 0U;

&#x20;   return;                                     // ← 这一拍什么姿态都不更新

}

imu\_info->raw\_info.acc\_x/y/z = accel\[0..2];     // 留档，给 J-Scope 看

imu\_info->raw\_info.gyro\_x/y/z = gyro\[0..2];

```



\### 3.2 坐标变换（`:184-185` → `bmi.c:120-146`）



```c

Vector\_Transform(gyro\[0..2], accel\[0..2], \&gyrox,\&gyroy,\&gyroz, \&accx,\&accy,\&accz);

```

内部就是 \*\*3×3 旋转矩阵 × 向量\*\*（`arm\_mat\_mult\_f32`），矩阵由 \*\*Z-Y-X 欧拉角\*\* 构造：



```c

// bmi.c:100-107，用 arz/ary/arx 三个角算旋转矩阵

gim\_trans.arz = PITCH\_IMU\_ARZ\_DEG;   // 180°

gim\_trans.ary = PITCH\_IMU\_ARY\_DEG;   // 180°

gim\_trans.arx = PITCH\_IMU\_ARX\_DEG;   // 0°

// ↑ 在 DeviceLayer/device.c:15-17 赋值

```



\*\*改动\*\*：把"IMU 芯片坐标系"转成"车体/云台坐标系"。\*\*这是安装方向差异被吸收的地方。\*\*



\### 3.3 陀螺零偏校准（`:188-223`，只在前 2000 拍）



```c

if (err\_code == IMU\_DATA\_CALI) {

&#x20;   if (imu\_cnt < 2000) {

&#x20;       gx\_offset -= gyrox \* 0.0005f;      // 慢速累加求均值（0.0005 是积分系数）

&#x20;       gy\_offset -= gyroy \* 0.0005f;

&#x20;       gz\_offset -= gyroz \* 0.0005f;

&#x20;       imu\_cnt++;

&#x20;   } else {

&#x20;       imu\_cnt = 0; err\_code = IMU\_NONE\_ERR;

&#x20;       if (|gx\_offset| > 0.005f) gx\_offset = 0;   // 数值离谱（车在动）就放弃这次校准

&#x20;       ... 同理 gy/gz

&#x20;       cali\_end = 1;                                // ← 校准完成标志

&#x20;   }

} else {

&#x20;   gyrox += gx\_offset; gyroy += gy\_offset; gyroz += gz\_offset;   // 三个轴都去零偏

}

```



\*\*改动\*\*：减掉陀螺三轴零偏。\*\*`cali\_end` 是后面喂数据时的必要许可之一。\*\*



\### 3.4 一阶低通（`:226-232`）



```c

gyrox\_ = Lowpass(gyrox\_, gyrox, 1);    // alpha = 1   → y = x，等于不滤波

gyroy\_ = Lowpass(gyroy\_, gyroy, 1);    // 同上

gyroz\_ = Lowpass(gyroz\_, gyroz, 1);

accx\_  = Lowpass(accx\_,  accx,  0.8);

accy\_  = Lowpass(accy\_,  accy,  0.2);

accz\_  = Lowpass(accz\_,  accz,  0.2);

```

`Lowpass` 实现（`low\_pass\_filter.c:5-8`）：

```c

return alpha \* input + (1.0f - alpha) \* prevOutput;   // 一阶 IIR 指数滑动

```



\*\*改动\*\*：加速度计被压得比较重（0.8/0.2/0.2），陀螺\*\*完全没压\*\*（alpha=1）。



\### 3.5 Mahony 四元数解算（`:235-239` → `bmi.c:154-236`）



```c

BMI\_Get\_EulerAngle(\&base\_info.pitch, \&base\_info.roll, \&base\_info.yaw,

&#x20;                  \&gyrox\_, \&gyroy\_, \&gyroz\_,      // 去偏 + 低通后的角速度

&#x20;                  \&accx\_,  \&accy\_,  \&accz\_);       // 低通后的加速度

```



内部（`bmi.c:168-232`）：

```c

// 1) 加速度归一化

norm = 1/sqrt(ax²+ay²+az²);  ax\*=norm; ay\*=norm; az\*=norm;



// 2) 由四元数算出"重力方向参考向量"

vx = -2\*(q1\*q3 - q0\*q2);

vy = -2\*(q0\*q1 + q2\*q3);

vz = -(q0² - q1² - q2² + q3²);



// 3) 与实测加速度做外积 → 姿态误差

ex = az\*vy - ay\*vz;  ey = ax\*vz - az\*vx;  ez = ay\*vx - ax\*vy;



// 4) 用 Kp 把误差补偿进角速度（互补滤波的核心）

gx += ex\*Kp;  gy += ey\*Kp;  gz += ez\*Kp;



// 5) 一阶欧拉积分更新四元数（步长 halfT）

q0 += (-q1\*gx - q2\*gy - q3\*gz) \* halfT;   // 以及 q1/q2/q3

// 6) 归一化四元数



// 7) 解欧拉角

pitch = atan2(2\*q1\*q3 - 2\*q0\*q2, sqrt(1 - 该值²));

yaw   = atan2(2\*(q1\*q2 + q0\*q3), q0²+q1²-q2²-q3²);

roll  = atan2(2\*q2\*q3 + 2\*q0\*q1, q0²-q1²-q2²+q3²);

roll  \*= 57.2958;   pitch \*= -57.2958;   yaw \*= 57.2958;    // 转度，pitch 取负

```



`Kp` 和 `halfT` 由 \*\*`control\_task.c:31-32`\*\* 在调用前设置：

```c

bmi.halfT = (interval 限幅 1\~20ms) \* 0.0005f;   // 动态算半步长

bmi.Kp    = (now <= 400U) ? 1000.0f : 0.125f;   // 前 400ms 猛收敛，之后正常

```



\*\*改动\*\*：把"角速度 + 加速度"融合成 \*\*四元数 → 欧拉角\*\*，输出 `pitch/roll/yaw`（度）。



\### 3.6 角速度换算 + 均值滤波（`:268-279`）



```c

base\_info.rate\_pitch = gyroy\_ / 0.017453;      // rad/s → °/s   ← ★ Pitch 内环就用这个

base\_info.ave\_rate\_pitch = ave\_fil\_update(\&imu\_pitch\_dif\_speed\_ave\_filter, rate\_pitch, 3);  // 3 次均值（Pitch 没用）

base\_info.rate\_roll  = gyrox\_ / 0.017453;

base\_info.ave\_rate\_roll  = ave\_fil\_update(...);

base\_info.rate\_yaw   = gyroz\_ / 0.017453;      // ← 这个回传给下板

base\_info.ave\_rate\_yaw   = ave\_fil\_update(...);

```



\*\*改动\*\*：rad/s → °/s，并额外算一份 3 点滑动均值版本。



\### 3.7 其余副作用（顺带算的，不参与 Pitch）



\- `base\_info.accx/accy/accz`：用 pitch/roll/yaw 把加速度转到世界系（`BMI\_Get\_Acceleration()`，`bmi.c:241+`）

\- `base\_info.temperature = temp`（给温控 PID 用，目前未启用）

\- `offline\_cnt = 0`（心跳）

\- 全零数据连续 100 次 → 判 `IMU\_DATA\_ERR`



\---



\## ④ `control\_task.c` —— 把结果喂给 Pitch（唯一的"交接点"）



`control\_task.c:26-43`：



```c

if (imu\_sensor.work\_state.err\_code == IMU\_NONE\_ERR || == IMU\_DATA\_CALI) {   // 只在正常/校准中

&#x20;   bmi.halfT = ...;  bmi.Kp = ...;                                         // ★ 改滤波参数

&#x20;   imu\_sensor.update(\&imu\_sensor);                                         // ★ 跑完 ③ 的全部链路

&#x20;   ...

&#x20;   Gimbal\_Pitch\_ImuAttitudeUpdate(

&#x20;       imu\_sensor.info->base\_info.pitch,        // ← 角度

&#x20;       imu\_sensor.info->base\_info.rate\_pitch,   // ← 角速度

&#x20;       imu\_sensor.info->base\_info.yaw,          // ← yaw（回传下板）

&#x20;       imu\_sensor.info->base\_info.rate\_yaw,

&#x20;       HAL\_GetTick(),

&#x20;       BMI088\_read\_valid \&\& init\_flag \&\& cali\_end \&\& (err\_code == IMU\_NONE\_ERR));  // ← 四个许可全部满足

} else {

&#x20;   Gimbal\_Pitch\_ImuAttitudeUpdate(0,0,0,0, now, 0);   // 无效样本，许可置 0

}

```



\*\*这一层是"数据的出口"\*\*：`imu\_sensor.info->base\_info.\*` 被取出，同时把 \*\*4 个许可\*\* 一起传下去。



\---



\## ⑤ `gimbal\_pitch.c` —— 变成反馈和目标



\### 5.1 存起来（`:278-297`）



```c

Gimbal\_Pitch\_ImuUpdate(pitch\_deg, speed\_dps, now, ready) {

&#x20;   gimbal\_pitch.imu\_ready = ready \&\& isfinite(pitch\_deg) \&\& isfinite(speed\_dps);

&#x20;   if (!imu\_ready) return;                                // ← 无效就直接返回，保留上一次值

&#x20;   gimbal\_pitch.imu\_pitch\_deg = PITCH\_IMU\_DIRECTION \* pitch\_deg;    // ★ 最后一次方向修正

&#x20;   gimbal\_pitch.imu\_speed\_dps = PITCH\_IMU\_DIRECTION \* speed\_dps;

&#x20;   gimbal\_pitch.imu\_last\_ms = now;

&#x20;   if (PITCH\_USE\_IMU) gimbal\_pitch.speed\_dps = gimbal\_pitch.imu\_speed\_dps;   // 内环反馈

}

```



\*\*这里是 IMU 数据的最后一个改动：乘 `PITCH\_IMU\_DIRECTION`（`pitch\_config.h:15`，当前 = 1.0）定义"抬头为正"。\*\*



\### 5.2 决定谁当反馈（`:367`）



```c

gimbal\_pitch.control\_actual\_deg = gimbal\_pitch.gyro\_mode

&#x20;                                 ? gimbal\_pitch.imu\_pitch\_deg     // ★ 陀螺模式：IMU 绝对角

&#x20;                                 : gimbal\_pitch.actual\_deg;       //   机械模式：编码器相对角

```



\### 5.3 内环反馈（`:286`，在上面 5.1 已设）



```c

if (PITCH\_USE\_IMU) speed\_dps = imu\_speed\_dps;   // ★ 陀螺仪 Y 轴角速度

// 否则 = encoder 差分速度

```



\### 5.4 目标从哪来（\*\*IMU 不产生目标，只产生反馈\*\*）



```c

// 目标 = 下板 rate\_dps 逐周期累加（gimbal\_pitch.c:126-137）

target\_deg += command.rate\_dps \* dt;

target\_deg = clip(target\_deg, low, high);           // 限位



// 误差

error\_deg = target\_deg - control\_actual\_deg;        // ★ IMU 角在这里进来



// 外环 → 速度目标

angle\_integral\_dps = Ki \* angle\_integral\_sum;

speed\_target\_dps = clip(Kp \* error\_deg + angle\_integral\_dps, max\_speed\_dps);



// 内环 → 力矩

error      = speed\_target\_dps - speed\_dps;          // ★ IMU 角速度在这里进来

integral   = Ki \* speed\_integral\_sum;

speed\_loop\_torque = 方向 \* clip(Kp\*error + integral, PITCH\_SPEED\_OUTPUT\_LIMIT);



// 加重力补偿（IMU 角度第二次被用）

gravity\_torque = gravity\_compensation(PITCH\_USE\_IMU ? imu\_pitch\_deg : actual - FLAT);



// 最终

torque = clip(speed\_loop\_torque + gravity\_torque, PITCH\_TORQUE\_LIMIT);

```



\*\*所以 IMU 数据在控制律里出现 3 次\*\*：

1\. `control\_actual\_deg` → 角度环反馈

2\. `speed\_dps` → 速度环反馈

3\. `imu\_pitch\_deg` → 重力补偿输入



\### 5.5 imu\_ready 的二次校验（`:383-384`）



```c

if (PITCH\_USE\_IMU \&\& (!imu\_ready || now - imu\_last\_ms > PITCH\_IMU\_TIMEOUT\_MS /\*20ms\*/))

&#x20;   blocks |= PITCH\_BLOCK\_IMU;      // → 触发停机

```

\*\*IMU 超 20ms 没更新，Pitch 直接退出，不是"用旧值凑合"。\*\*



\---



\## ⑥ IMU 的第二个用途：Yaw 回传下板



同一次 `Gimbal\_Pitch\_ImuAttitudeUpdate()` 里还存了 yaw：



```c

gimbal\_pitch.imu\_yaw\_deg = remainderf(yaw\_deg, 360.0f);   // :295

gimbal\_pitch.imu\_yaw\_dps = yaw\_dps;

```



然后经 CAN2 帧 \*\*`0x021`（`BOARD\_IMU\_STATUS\_ID`，`board\_pitch\_protocol.h:15`）\*\* 发给下板（`PITCH\_IMU\_STATUS\_MS = 10ms` 一次），下板在：



```c

// 27\_YYZ\_down/Application/DriverLayer/drv\_pitch\_link.c:117-119

} else if (header.Identifier == BOARD\_IMU\_STATUS\_ID \&\& Board\_Imu\_Decode(data, 8U, \&imu)) {

&#x20;   pitch\_link\_diag.last\_imu = imu;

&#x20;   Gyro\_ReceiveImu(\&imu, now);        // → gyro\_control.yaw\_deg / speed\_dps

}

```



再被下板用于 `Gyro\_Transform()`（底盘坐标变换）和 `Gyro\_Follow()`（底盘跟随/小陀螺）。



\---



\## 汇总表：每个文件对数据做了什么



| # | 文件 | 函数 | 对数据做的改动 | 数据形态变化 |

|---|---|---|---|---|

| ① | `BMI088Middleware.c` | `BMI088\_read\_write\_byte` | SPI 全双工交换；失败计数 | — → `uint8\_t` |

| ② | `BMI088driver.c` | `BMI088\_read` | 小端 int16 拼装；× 灵敏度；ID 校验；SPI 错误比对 | `uint8\_t\[6/8]` → \*\*rad/s、g、℃\*\* + `read\_valid` |

| ③a | `imu\_sensor.c` | `imu\_update`（:`184`） | \*\*3×3 旋转矩阵\*\*（arz/ary/arx） | 芯片系 → \*\*车体系\*\* |

| ③b | `imu\_sensor.c` | `imu\_update`（:`188-223`） | \*\*减三轴零偏\*\*（前 2000 拍求均值） | 去偏 |

| ③c | `imu\_sensor.c` | `imu\_update`（:`226-232`） | \*\*一阶 IIR 低通\*\*：陀螺 α=1、加速度 α=0.8/0.2/0.2 | 平滑 |

| ③d | `bmi.c` | `BMI\_Get\_EulerAngle` | \*\*Mahony\*\*：加速度校重力方向 → 误差 → 补偿角速度 → 四元数积分 → atan2 出欧拉角 | 角速度+加速度 → \*\*pitch/roll/yaw（度）\*\* |

| ③e | `imu\_sensor.c` | `imu\_update`（:`270,278`） | rad/s → \*\*°/s\*\*；另算 3 点均值版 | `rate\_pitch/rate\_yaw` |

| ④ | `control\_task.c` | `StartControlTask`（`:31-41`） | 设 `bmi.Kp`（1000→0.125）、`bmi.halfT`；\*\*汇总 4 个许可\*\*并交付 | → `Gimbal\_Pitch\_ImuAttitudeUpdate()` |

| ⑤a | `gimbal\_pitch.c` | `Gimbal\_Pitch\_ImuUpdate`（`:278-288`） | × `PITCH\_IMU\_DIRECTION` 定正负；有限性检查；打时间戳 | → `imu\_pitch\_deg` / `imu\_speed\_dps` |

| ⑤b | `gimbal\_pitch.c` | `Gimbal\_Pitch\_Update`（`:367,286`） | 按模式选反馈源 | → \*\*`control\_actual\_deg`（外环反馈）\*\*、\*\*`speed\_dps`（内环反馈）\*\* |

| ⑤c | `gimbal\_pitch.c` | `gravity\_compensation`（`:446`） | 分段折线 → 力矩前馈 | → `gravity\_torque` |

| ⑤d | `gimbal\_pitch.c` | `Gimbal\_Pitch\_Update`（`:442-451`） | 外环 PI → 速度目标；内环 PI → 力矩；相加限幅 | → \*\*`torque`\*\* |



\---



\## 三句话记住这条链路



1\. \*\*原始数据\*\*：`BMI088driver.c` 从 SPI 拿 int16，换成 \*\*rad/s（陀螺）/ g（加速度）\*\*，并给出"这一帧可不可信"。

2\. \*\*姿态数据\*\*：`imu\_sensor.c` + `bmi.c` 依次做 \*\*坐标变换 → 去零偏 → 低通 → Mahony 四元数 → 欧拉角 → 单位换算\*\*，产出 `base\_info.pitch / rate\_pitch / yaw / rate\_yaw`；其中 `Kp` 和 `halfT` 是 `control\_task.c` 每次调用前动态设的。

3\. \*\*控制数据\*\*：`gimbal\_pitch.c` 给 IMU 角度乘一次 `PITCH\_IMU\_DIRECTION` 定正负，然后 \*\*角度当外环反馈、角速度当内环反馈、角度再当重力补偿输入\*\*，PID 输出 MIT 力矩；同时把 \*\*yaw 经 CAN2 `0x021` 回传下板\*\*做整车稳向。



\*\*最容易出问题的两个点\*\*（我在前面提过，这里再落到具体文件）：

\- `imu\_sensor.c:227-229` 陀螺低通 \*\*α=1 等于没滤\*\*，噪声直通速度环（`PITCH\_GYRO\_SPEED\_KP=0.14`）——这是你那个"尖峰"的首选嫌疑；现成的 `ave\_rate\_pitch`（3 点均值）\*\*没被 Pitch 用\*\*。

\- `imu\_sensor.c:282` 的 `PITCH\_IMU\_DIRECTION` 和 `device.c:15-17` 的 `arz/ary` 是\*\*两个独立的符号/方向来源\*\*，换 IMU 或换安装必须两个一起核对，否则会出现"抬头发散"。

