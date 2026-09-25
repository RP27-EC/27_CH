$ErrorActionPreference = 'Stop'

$repo = Resolve-Path (Join-Path $PSScriptRoot '../..')
$control = Get-Content -Raw (Join-Path $repo 'Application/TaskLayer/control_task.c')
$project = Get-Content -Raw (Join-Path $repo 'MDK-ARM/My_C.uvprojx')

function Assert-Match([string]$text, [string]$pattern, [string]$message) {
    if ($text -notmatch $pattern) {
        throw $message
    }
}

Assert-Match $control '#include\s+"board_comm\.h"' 'control_task.c must include board_comm.h'
Assert-Match $control '#define\s+DEG_TO_RAD\s+\(0\.01745329251994329577f\)' 'DEG_TO_RAD must use the specified conversion constant'
Assert-Match $control 'base_info\.yaw\s*\*\s*DEG_TO_RAD' 'Yaw angle must be converted from degrees to radians'
Assert-Match $control 'base_info\.rate_yaw\s*\*\s*DEG_TO_RAD' 'Yaw rate must be converted from degrees/s to radians/s'

$updateIndex = $control.IndexOf('imu_sensor.update(&imu_sensor);')
$sendIndex = $control.IndexOf('Board_Comm_Send_IMU(')
if ($updateIndex -lt 0 -or $sendIndex -le $updateIndex) {
    throw 'Board_Comm_Send_IMU must run after imu_sensor.update'
}

Assert-Match $project '<GroupName>Protocol</GroupName>[\s\S]*?<FileName>board_comm\.c</FileName>[\s\S]*?<FilePath>\.\.\\Application\\ProtocolLayer\\board_comm\.c</FilePath>' 'Keil Protocol group must include board_comm.c'

Write-Output 'board_comm integration contract passed'
