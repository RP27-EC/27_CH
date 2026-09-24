$ErrorActionPreference = 'Stop'

function Assert-Match {
    param([string]$Text, [string]$Pattern, [string]$Message)
    if ($Text -notmatch $Pattern) { throw $Message }
}

$source = Get-Content -LiteralPath 'Application/ModuleLayer/chassis.c' -Raw
$header = Get-Content -LiteralPath 'Application/ModuleLayer/chassis.h' -Raw

if ($source -match 'PID_Calculate|Chassis_PID_t') {
    throw 'private chassis PID implementation is still present'
}
Assert-Match $source 'Chassis_Mecanum_Calculate\s*\(' 'kinematics function is not used'
Assert-Match $source 'single_set_speed\s*\(' 'RM motor speed controller is not used'
Assert-Match $source 'pid_clear\s*\(' 'common PID clear function is not used'
foreach ($field in 'target', 'err', 'last_err', 'out') {
    Assert-Match $source "speed_pid->$field\s*=\s*0\.0f" "PID field $field is not cleared"
}
Assert-Match $source 'tx_info->torque\s*=\s*0\.0f' 'motor torque is not cleared'
Assert-Match $source 'Chassis_Motor_Group\.group_set_torque\s*\(' 'group command is not sent'
Assert-Match $header 'void\s+Chassis_Init\s*\(\s*void\s*\)' 'Chassis_Init interface is missing'
Assert-Match $header 'void\s+Chassis_Update\s*\(' 'Chassis_Update interface is missing'
Assert-Match $header 'void\s+Chassis_Stop\s*\(\s*void\s*\)' 'Chassis_Stop interface is missing'

Write-Output 'chassis module contract passed'
