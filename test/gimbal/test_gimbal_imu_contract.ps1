$ErrorActionPreference = 'Stop'

$repo = Resolve-Path (Join-Path $PSScriptRoot '../..')
$gimbal = Get-Content -Raw (Join-Path $repo 'Application/ModuleLayer/gimbal.c')
$control = Get-Content -Raw (Join-Path $repo 'Application/TaskLayer/control_task.c')

function Assert-Match([string]$text, [string]$pattern, [string]$message) {
    if ($text -notmatch $pattern) {
        throw $message
    }
}

function Assert-NotMatch([string]$text, [string]$pattern, [string]$message) {
    if ($text -match $pattern) {
        throw $message
    }
}

Assert-Match $gimbal '#include\s+"communicate\.h"' 'gimbal.c must consume board communication state'
Assert-Match $gimbal 'gimbal->mode\s*==\s*GIMBAL_MODE_GYRO\s*&&\s*!board_rx_info\.is_online' 'gyro mode must reject stale IMU feedback'

$offlineBranch = [regex]::Match(
    $gimbal,
    'if\s*\(gimbal->mode\s*==\s*GIMBAL_MODE_GYRO\s*&&\s*!board_rx_info\.is_online\)\s*\{(?<body>[\s\S]*?)\n\s*\}')
if (-not $offlineBranch.Success) {
    throw 'gyro IMU offline branch is missing'
}

$body = $offlineBranch.Groups['body'].Value
Assert-Match $body 'pid_clear\(&gimbal->yaw\.angle_pid\)' 'offline branch must clear the angle PID'
Assert-Match $body 'pid_clear\(&gimbal->yaw\.speed_pid\)' 'offline branch must clear the speed PID'
Assert-Match $body 'gimbal->yaw\.give_current\s*=\s*0' 'offline branch must clear commanded current'
Assert-Match $body 'KT_Yaw_SendIq\(0\)' 'offline branch must command zero KT current'
Assert-Match $body 'return\s*;' 'offline branch must stop PID calculation'

Assert-Match $control 'hero_gimbal\.yaw\.feedback_mec_angle' 'chassis following must use feedback_mec_angle'
Assert-NotMatch $control 'hero_gimbal\.yaw\.feedback_angle' 'undefined feedback_angle must not be referenced'

Write-Output 'gimbal IMU safety contract passed'
