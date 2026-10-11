# 底盘功率提前干预 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking. 用户已授权当前会话实施，按顺序执行，不再次请求实施确认。

**Goal:** 按裁判实收上限限制四轮电流，超电实测提前干预，满缓冲缺额修正，保持只读超电通信。

**Architecture:** `Apply` 取快照并写回；`Update` 计算预算和两类修正；`Limit` 保留功率模型与共同系数二分。原始遥测不写回，来源与新鲜度在 Watch 中明确展示。

**Tech Stack:** C99、STM32H7、FreeRTOS、ARMCC5、Python unittest + GCC。

## Global Constraints

- 仅改当前下板功率功能和必要的注释结束符修复；保留用户运动参数、上板、裁判协议、超电发送状态、Keil 产物。
- 默认训练回退 0 W，真实上限优先；过期用历史值，明确来源。当前缓冲完全采用接收值，60 J 仅为目标。
- 默认余量 5 W、提前宽度 2 W、软修正增益 1/上限 5 W、积分增益 1/s、迟滞 2 W、联合恢复确认 500 ms。
- 缓冲恢复 10 W/s、偏差恢复 1 W/s、最终预算恢复 20 W/s；无新帧不积分，过期不放开。
- 合并用最大修正，不是两类修正相加；最终预算永不超过基础预算。

## Task 1：先写预算行为测试

Files: `test/power/test_chassis_power.py`；保留原四轮模型测试，显式提供真实上限。

Interfaces: `Chassis_Power_Init()`、`Chassis_Power_Update(const ChassisPowerInput_t*, uint32_t)`。

- [x] 写真实上限、默认无上限零预算、显式训练来源、60→59 J 以及警戒区无累计用例。
- [x] 写帧频独立积分、双反馈取最大值、联合确认后恢复、重连和计时回绕用例。

关键断言：

```c
Chassis_Power_Update(&input,0);
near(chassis_power.target_power_w,0); /* 未收到裁判上限 */
limit_frame(80,10);buffer_frame(60,10);Chassis_Power_Update(&input,10);
near(chassis_power.base_budget_w,75);
buffer_frame(59,110);cap_frame(74,110);Chassis_Power_Update(&input,110);
near(chassis_power.target_power_w,74); /* 两类各 1 W，不扣两次 */
```

- [x] 运行 `python -X utf8 test/power/test_chassis_power.py -v`，确认旧实现因行为不符失败；新字段用例在声明后继续验证。

## Task 2：实现预算与中文说明

Files: `Application/AlgorithmLayer/chassis_power.c`、`.h`、`Application/ModuleLayer/chassis_power_adapter.c`。

Interfaces: 不改变三函数调用签名；新增来源枚举、警戒参数、提前/偏差/综合修正观察量和恢复确认状态。

- [x] 在 `.h` 定义上述参数和状态；保留 `training_limit_w` 作为显式训练入口，默认值改 0。
- [x] 在 `Update` 实现本设计，逐句解释输入、单位、条件和恢复。

核心计算：

```c
float warning = fmaxf(0.0f, chassis_power.base_budget_w - chassis_power.measured_warning_margin_w);
float soft = Clamp(chassis_power.measured_soft_kp * fmaxf(0.0f,input->cap_w-warning),
                   0.0f,fminf(chassis_power.measured_soft_max_w,chassis_power.base_budget_w));
float combined = fmaxf(chassis_power.buffer_derate_w,chassis_power.measured_derate_w);
float desired = fmaxf(0.0f,chassis_power.base_budget_w-combined);
```

- [x] 只在相邻有效新帧实际间隔内积分；首次/跨超时帧 dt=0。软修正下降与偏差释放需联合恢复确认；最终上升限速。
- [x] 运行预算测试，确认功能用例和原模型保护用例通过。

## Task 3：整车链路、调试文档与构建

Files: `Application/ModuleLayer/chassis.c`（仅补齐损坏注释）；`Application/ConfigLayer/rp_config.h`（同步旧功率停用注释，配置值不变）；`test/chassis/test_power_integration.py`；`docs/chassis_power.md`。

- [x] 集成测试提供实际裁判数据；校验新的预算、原始遥测不改写、收到上限前零输出及 failsafe 零发送。
- [x] 调试文档列出三项原始 Watch 变量、来源/新鲜度、参数和预算链路；标明旧 50 J 策略已废止。
- [x] 运行功率/超电、裁判/串口、底盘集成/键鼠和云台回归；PowerShell 契约和 C 主机测试按现有命令运行。
- [x] 执行 `python -X utf8 test/build_firmware.py`，要求编译链接成功，生成临时产物；不烧录。
- [x] 检查差异、中文编码、原始接收与 TX 文件无修改；只提交本次功能文件，中文 Conventional Commit。

最终验收以测试和临时 ARMCC 构建为软件证据；真实功率、测量位置和模型系数仍需实车校验。

## 已执行的验证（2026-10-11）

- 先在旧实现上运行新预算测试，10 项因旧行为不符合新设计失败；四轮模型保护测试保留。
- 功率/超电 18 项、底盘集成/键鼠 8 项、裁判/串口 4 项、云台 28 项，共 58 项 Python 主机回归全部通过。
- C 主机运动学 7 项、底盘控制 8 项通过；云台、底盘模块/接线、UART5、车控共 5 项 PowerShell 契约检查通过。
- ARMCC5 编译并链接 122 个源文件成功，21 条警告均来自未修改的其他模块或 CMSIS 库；本次功率文件无编译警告。
- 程序规模：Code=71000、RO-data=4296、RW-data=5020、ZI-data=26288 字节。
- 构建产物位于临时目录 `C:/Users/feather/AppData/Local/Temp/rp27-armcc-ogv8p2pi`，未覆盖 Keil 工程已有产物，未烧录。
- 提交只包含本次功率功能、回归、文档及必要注释同步；已有用户运动参数、其他注释及工程产物保留在工作区。
