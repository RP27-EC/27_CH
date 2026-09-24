$ErrorActionPreference = 'Stop'

function Assert-Match {
    param([string]$Text, [string]$Pattern, [string]$Message)
    if ($Text -notmatch $Pattern) {
        throw $Message
    }
}

$motorSource = Get-Content -LiteralPath 'Application/DeviceLayer/chassis_motor.c' -Raw
$motorHeader = Get-Content -LiteralPath 'Application/DeviceLayer/chassis_motor.h' -Raw
$deviceSource = Get-Content -LiteralPath 'Application/DeviceLayer/device.c' -Raw
$protocolSource = Get-Content -LiteralPath 'Application/ProtocolLayer/can_protocol.c' -Raw
$rmHeader = Get-Content -LiteralPath 'Application/HardwareLayer/RM_motor.h' -Raw

Assert-Match $motorSource 'Motor_RM_t\s+Chassis_Motor\s*\[\s*4\s*\]' 'missing four-motor chassis array'
Assert-Match $motorSource 'Motor_RM_Group_t\s+Chassis_Motor_Group' 'missing chassis motor group'
foreach ($id in 0..3) {
    Assert-Match $motorSource "\.rxId\s*=\s*$id" "missing rxId $id"
}
if (([regex]::Matches($motorSource, '\.type\s*=\s*_3508_Single')).Count -ne 4) {
    throw 'all four chassis motors must use _3508_Single raw-current semantics'
}
if (([regex]::Matches($motorSource, '\.hcan\s*=\s*&hfdcan1')).Count -lt 5) {
    throw 'four motors and their group must use hfdcan1'
}
Assert-Match $motorSource '\.stdId\s*=\s*0x200' 'missing command frame ID 0x200'
Assert-Match $motorHeader 'extern\s+Motor_RM_t\s+Chassis_Motor\s*\[\s*4\s*\]' 'motor array is not exported'
Assert-Match $deviceSource 'Chassis_Motor_Group\.group_init\s*\(\s*&Chassis_Motor_Group\s*\)' 'device init does not initialize chassis group'
foreach ($id in 201..204) {
    Assert-Match $protocolSource "case\s+0x$id\s*:" "CAN1 does not dispatch 0x$id"
}
Assert-Match $rmHeader 'CAN_HandleTypeDef\s+\*hcan' 'RM motor group handle is missing'

Write-Output 'chassis wiring contract passed'
