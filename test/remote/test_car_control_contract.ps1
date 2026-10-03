$ErrorActionPreference = 'Stop'
$repo = Resolve-Path (Join-Path $PSScriptRoot '..\..')
$header = Get-Content -Raw (Join-Path $repo 'Application\ControlLayer\car_control.h')
$control = Get-Content -Raw (Join-Path $repo 'Application\ControlLayer\carctrl.c')

foreach ($obsolete in @('VisionMode_e', 'VISION_OFF', 'VISION_AUTO_AIM', 'vision_mode')) {
    if ($header.Contains($obsolete) -or $control.Contains($obsolete)) {
        throw "Obsolete auto-aim control state remains: $obsolete"
    }
}

if ($control -notmatch 'Car_Mode_Switch_Update\s*\(\s*car\s*,\s*tb_up\s*,\s*tb_dn\s*\)') {
    throw 'Thumbwheel-up/down edges must remain connected to chassis mode switching'
}
if ($control -notmatch 'Car_Action_Cmd_Update\s*\(\s*car\s*,\s*tb_up\s*\)') {
    throw 'Action control must receive only the thumbwheel-up edge'
}
foreach ($flag in @('is_shoot_system_on', 'is_single_shoot_cmd', 'is_continuous_shoot')) {
    if (-not $header.Contains($flag) -or -not $control.Contains($flag)) {
        throw "Existing firing control flag was removed: $flag"
    }
}

Write-Host 'Car-control contract passed.'
