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
if ($control -notmatch 'Car_Action_Cmd_Update\s*\(\s*car\s*,\s*tb_up\s*,\s*s1\s*,\s*s2\s*,\s*s1_stable\s*\)') {
    throw 'Action control must receive the thumbwheel-up edge and the lever snapshot'
}
foreach ($flag in @('is_shoot_system_on', 'is_single_shoot_cmd', 'is_continuous_shoot')) {
    if (-not $header.Contains($flag) -or -not $control.Contains($flag)) {
        throw "Existing firing control flag was removed: $flag"
    }
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

Write-Host 'Car-control contract passed.'
