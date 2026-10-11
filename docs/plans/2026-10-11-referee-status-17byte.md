# 0x0201 的 17 字节兼容修复

目标：修复实车 17 字节状态帧被 13 字节校验拒绝、功率预算因此保持零的问题。用户已授权直接修改当前工程；保留已有未提交改动。

依据：实车在 `judge_update()` 的 0x0201 分支停住，帧头长度为 17。同工作区 `Down/RP26_Lob_Hero_Down/Application/ProtocolLayer/judge_protocol.h` 定义了对应布局：前 12 字节与本工程一致，偏移 12 增加 float `bullet_speed_limit`，输出标志位于偏移 16。13 字节版本的标志在偏移 12。两者功率上限都是偏移 10 的小端 uint16。

设计：仅兼容这两个已核对的布局；将它们解析为一个含射速上限的内部结构。13 字节帧将不存在的射速上限清零。其他长度、零功率上限不更新已接受的数据或其时间戳。原 CRC 校验和独立缓冲帧处理不变。增加常驻接收长度、次数、时间、原始功率及处理结果，方便 Keil Watch 排查，不依赖条件断点。功率预算继续使用真实反馈，不增加固定额度，不改超电发送。

执行步骤（按测试先行、逐步验证流程执行）：

1. 在 `test/referee/test_referee.py` 给真实 C 解析器加入 17 字节 CRC 帧测试；先确认旧代码因 `limit_received` 为假而失败。
2. 修改 `judge_protocol.h`、`judge.h`、`judge.c`：规范化两个布局，分别取标志，补充诊断及初始化；新增布局切换、错误长度、零功率、CRC 和分包回归用例。
3. 更新 `docs/chassis_power.md` 的 Watch 排查说明。
4. 运行 referee、power、chassis 测试与 `test/build_firmware.py` 的 ARMCC 全量构建；检查差异，只提交本次文件。实车是否恢复需重新构建、下载后确认。

验证时发现当前未提交版本注释了 `chassis.c` 的功率头文件、`Chassis_Power_Init()`、`Chassis_Power_Apply()` 与停车时的 `Chassis_Power_Clear_Output()`，功率链路测试因此失败。恢复这四处已存在的代码，使真实裁判上限重新进入功率限制链路，并恢复停车观察量清零；其余已有改动保留。

验证结果：17 字节回归测试先在旧解析器上因 `limit_received` 为假而失败；修复后 referee 8 项、power 18 项、chassis 8 项全部通过。ARMCC5 对最终版本编译并链接 122 个源文件，零错误，21 个警告；构建输出位于临时目录，未烧录硬件。未知长度、零上限、CRC 错误及 13/17 布局切换均有回归覆盖。
