$ErrorActionPreference = 'Stop'
$repo = Resolve-Path (Join-Path $PSScriptRoot '..\..')
$header = Get-Content -Raw (Join-Path $repo 'Application\ControlLayer\car_control.h')
$control = Get-Content -Raw (Join-Path $repo 'Application\ControlLayer\carctrl.c')

foreach ($obsolete in @('VisionMode_e', 'VISION_OFF', 'VISION_AUTO_AIM', 'vision_mode')) {
    if ($header.Contains($obsolete) -or $control.Contains($obsolete)) {
        throw "Obsolete auto-aim control state remains: $obsolete"
    }
}

# 锁两件事：拨轮边沿仍接在模式切换 / 动作上，且动作链路必须吃拨杆快照。
# 快照参数是必填项，不是可选 —— 少了它就是「判定用旧值、动作用新值」的撕裂。
if ($control -notmatch 'Car_Mode_Switch_Update\s*\(\s*car\s*,\s*tb_up\s*,\s*tb_dn\s*,\s*s1\s*,\s*s2\s*\)') {
    throw 'Thumbwheel-up/down edges must remain connected to chassis mode switching with the lever snapshot'
}
if ($control -notmatch 'Car_Action_Cmd_Update\s*\(\s*car\s*,\s*tb_up\s*,\s*tb_dn\s*,\s*s1\s*,\s*s2\s*,\s*s1_stable\s*,\s*s2_stable\s*\)') {
    throw 'Action control must receive both thumbwheel edges and both lever-stability flags'
}
foreach ($flag in @('is_shoot_system_on', 'is_single_shoot_cmd', 'is_continuous_shoot', 'is_eject_cmd')) {
    if (-not $header.Contains($flag) -or -not $control.Contains($flag)) {
        throw "Firing control flag missing: $flag"
    }
}

# 退弹绑定：S1中 + S2中 + 拨轮向下
if ($control -notmatch 's1\s*==\s*3\s*&&\s*s2\s*==\s*3\s*&&\s*tb_dn') {
    throw 'Eject binding must be S1-mid + S2-mid + thumbwheel-down'
}

# 单发必须是上升沿触发（s2 从非上位拨到上位），不是冷却计数——
# 冷却逻辑让「拨一下按住」看起来像慢速连发，感受不出单发。
if ($control -notmatch 's2\s*==\s*RC_SW_UP\s*&&\s*s2_last\s*!=\s*RC_SW_UP') {
    throw 'Single-shot must be edge-triggered on s2 entering UP, not cooldown-gated'
}

# 停稳计数必须每拍都走。放进某个输入源分支里的话计数会冻结，
# 出键鼠那一瞬拿到陈旧 hold_ticks，拨杆过渡帧照样走火。
$updateBody = [regex]::Match($control, 'void\s+Car_Control_Update\s*\([^)]*\)\s*\{(?<body>[\s\S]*?)\n\}')
if (-not $updateBody.Success) {
    throw 'Car_Control_Update body not found'
}
if ($updateBody.Groups['body'].Value -notmatch 'Car_S1_Stable\s*\(\s*s1\s*\)') {
    throw 'Car_S1_Stable must be evaluated every tick from Car_Control_Update (a branch-local call freezes the counter)'
}
if ($updateBody.Groups['body'].Value -notmatch 'Car_S2_Stable\s*\(\s*s2\s*\)') {
    throw 'Car_S2_Stable must be evaluated every tick from Car_Control_Update (same freeze hazard as Car_S1_Stable)'
}
if ($updateBody.Groups['body'].Value -notmatch 'Ctrl_Fall_Edge\s*\(\s*ctrl_val_snap\s*\)') {
    throw 'Ctrl fall edge must be recomputed every tick from the atomic level snapshot'
}

# 拨杆/Ctrl 快照必须关中断原子取：s1/s2 由 UART5 中断背靠背写入，
# 两次普通加载之间被抢占会拼出 (旧s1, 新s2) 的假组合——正好是连发位。
if ($updateBody.Groups['body'].Value -notmatch '__disable_irq\s*\(\s*\)') {
    throw 's1/s2/Ctrl snapshot must be taken inside an interrupt-masked block (plain loads can tear)'
}

# 连发装配必须同时吃 s2 停稳：进键鼠时 s2 先动一帧，线上来一帧真实的
# (S1中, S2下)，单靠 s1 停稳拦不住（s1 还停在中位没动，计数一直在涨）。
if ($control -notmatch 's2\s*==\s*RC_SW_DOWN\s*&&\s*s1\s*==\s*RC_SW_MID\s*&&\s*s2_stable') {
    throw 'Continuous fire must require s2 stability (keyboard-entry s2-lead frame would otherwise fire one pellet)'
}

# 底盘通道函数必须用关中断快照读 Ctrl，不能直接读 rc_sensor.info：
# 那个读数在 2ms 环末尾才发生，中间落一帧 DBUS 就是「value 已归零、
# status 还停在 long_press」的撕裂状态，预充电的 WASD 计数当拍满速冲车。
$channelBody = [regex]::Match($control, 'void\s+Car_Chassis_Channel_Get\s*\([^)]*\)\s*\{(?<body>[\s\S]*?)\n\}')
if (-not $channelBody.Success) {
    throw 'Car_Chassis_Channel_Get body not found'
}
if ($channelBody.Groups['body'].Value -notmatch 'ctrl_val_snap') {
    throw 'Car_Chassis_Channel_Get must read Ctrl from the atomic snapshot, not from rc_sensor.info directly'
}
if ($channelBody.Groups['body'].Value -notmatch 'ctrl_fall_edge') {
    throw 'Car_Chassis_Channel_Get must zero WASD cnt on the atomic Ctrl fall edge'
}
if ($channelBody.Groups['body'].Value -notmatch 'CAR_MODE_INIT') {
    throw 'ramp_live must exclude CAR_MODE_INIT (INIT can last 1.5s > 0.8s ramp; precharge slams the chassis on INIT exit)'
}
if ($channelBody.Groups['body'].Value -match 'rc_sensor\.info->Ctrl\.status') {
    throw 'Car_Chassis_Channel_Get must not gate zeroing on keyboard FSM status (torn read across a DBUS frame)'
}

Write-Host 'Car-control contract passed.'
