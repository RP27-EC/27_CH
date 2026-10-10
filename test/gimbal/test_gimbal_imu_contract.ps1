$ErrorActionPreference = 'Stop'

$repo = Resolve-Path (Join-Path $PSScriptRoot '../..')
$gimbal = Get-Content -Raw (Join-Path $repo 'Application/ModuleLayer/gimbal.c')
$control = Get-Content -Raw (Join-Path $repo 'Application/TaskLayer/control_task.c')
$project = Get-Content -Raw (Join-Path $repo 'MDK-ARM/DM-MC02.uvprojx')
$protocol = Get-Content -Raw (Join-Path $repo 'Application/ProtocolLayer/can_protocol.c')
$communication = Get-Content -Raw (Join-Path $repo 'Application/ProtocolLayer/communicate.c')
$carControlHeader = Join-Path $repo 'Application/ControlLayer/car_control.h'
$legacyCarControlHeader = Join-Path $repo 'Application/ControlLayer/carctrl.h'

function Assert-Match([string]$text, [string]$pattern, [string]$message) {
    if (-not $text -or ($text -notmatch $pattern)) {
        if (-not $message) { $message = "Assert-Match failed for pattern: $pattern" }
        throw $message
    }
}

function Assert-NotMatch([string]$text, [string]$pattern, [string]$message) {
    if ($text -match $pattern) {
        throw $message
    }
}

Assert-Match $gimbal '#include\s+"communicate\.h"' 'gimbal.c must consume board communication state'
Assert-Match $gimbal '\(gimbal->mode\s*==\s*GIMBAL_MODE_GYRO\s*\|\|\s*gimbal->mode\s*==\s*GIMBAL_MODE_MECH\)\s*&&\s*!board_rx_info\.is_online' 'gyro and mechanical modes must reject stale IMU feedback'

$offlineBranch = [regex]::Match(
    $gimbal,
    'if\s*\(\(gimbal->mode\s*==\s*GIMBAL_MODE_GYRO\s*\|\|\s*gimbal->mode\s*==\s*GIMBAL_MODE_MECH\)\s*&&\s*!board_rx_info\.is_online\)\s*\{(?<body>[\s\S]*?)\n\s*\}')
if (-not $offlineBranch.Success) {
    throw 'gyro/mechanical IMU offline branch is missing'
}

$body = $offlineBranch.Groups['body'].Value
# Yaw 双环四份 PID 状态由 ClearYawPids() 统一清（mec_/gyro_ x angle_/speed_），
# 断言走这个封装而不是旧的 pid_clear(&gimbal->yaw.angle_pid) 单环写法。
Assert-Match $body 'ClearYawPids\s*\(\s*gimbal\s*\)' 'offline branch must clear the yaw PID state'
Assert-Match $body 'gimbal->yaw\.give_current\s*=\s*0' 'offline branch must clear commanded current'
Assert-Match $body 'KT_Yaw_SendIq\(0\)' 'offline branch must command zero KT current'
Assert-Match $body 'return\s*;' 'offline branch must stop PID calculation'

$gyroTransition = [regex]::Match(
    $gimbal,
    'if\s*\(gimbal->last_mode\s*!=\s*GIMBAL_MODE_GYRO\s*&&\s*gimbal->mode\s*==\s*GIMBAL_MODE_GYRO\)\s*\{(?<body>[\s\S]*?)\n\s*\}')
if (-not $gyroTransition.Success) {
    throw 'gyro mode transition branch is missing'
}

$transitionBody = $gyroTransition.Groups['body'].Value
Assert-Match $transitionBody 'gimbal->yaw\.target_imu_angle\s*=\s*gimbal->yaw\.imu_angle' 'gyro transition must synchronize the IMU target angle'
Assert-Match $transitionBody 'ClearYawPids\s*\(\s*gimbal\s*\)' 'gyro transition must clear the yaw PID state'

Assert-Match $control 'hero_gimbal\.yaw\.feedback_mec_angle' 'chassis following must use feedback_mec_angle'
Assert-NotMatch $control 'hero_gimbal\.yaw\.feedback_angle' 'undefined feedback_angle must not be referenced'
Assert-Match $project '<FileName>communicate\.c</FileName>[\s\S]*?<FilePath>\.\.\\Application\\ProtocolLayer\\communicate\.c</FilePath>' 'Keil project must compile communicate.c'
Assert-Match $protocol 'case\s+SLAVE_RX_ID\s*:[\s\S]*?Board_Rx_C1\(rxBuf\)' 'CAN2 must dispatch ID 0x021 to Board_Rx_C1'
Assert-Match $communication 'memcpy\(&board_rx_info\.yaw_imu,\s*&rxBuf\[0\],\s*sizeof\(float\)\)' 'yaw_imu must be decoded from bytes 0..3'
Assert-Match $communication 'memcpy\(&board_rx_info\.yaw_v,\s*&rxBuf\[4\],\s*sizeof\(float\)\)' 'yaw_v must be decoded from bytes 4..7'
Assert-Match $communication 'board_rx_info\.is_online\s*=\s*true' 'valid frames must refresh the online state'
if (-not (Test-Path -LiteralPath $carControlHeader)) {
    throw 'car_control.h must exist for the current include directives'
}
if (Test-Path -LiteralPath $legacyCarControlHeader) {
    throw 'legacy carctrl.h filename must be removed'
}
Assert-Match $project '<FileName>car_control\.h</FileName>[\s\S]*?<FilePath>\.\.\\Application\\ControlLayer\\car_control\.h</FilePath>' 'Keil project must reference car_control.h'

# 双向就近对齐：88°/91° 迟滞（上机调参）+ MECH 按 align_dir 锁 0°/180°
$gimbalHeader = Get-Content -Raw (Join-Path $repo 'Application/ModuleLayer/gimbal.h')
Assert-Match $gimbalHeader 'YAW_ALIGN_FRONT_MAX_DEG' 'align front threshold macro missing'
Assert-Match $gimbalHeader 'YAW_ALIGN_BACK_MIN_DEG' 'align back threshold macro missing'
Assert-Match $gimbal 'YAW_ALIGN_BACK_MIN_DEG' 'gimbal.c must use the align-back threshold macro'
Assert-Match $gimbal 'YAW_ALIGN_FRONT_MAX_DEG' 'gimbal.c must use the align-front threshold macro'
Assert-Match $gimbal 'align_dir\s*=\s*ALIGN_BACK' 'hysteresis must latch ALIGN_BACK past the back threshold'
Assert-Match $gimbal 'align_dir\s*=\s*ALIGN_FRONT' 'hysteresis must latch ALIGN_FRONT below the front threshold'
Assert-Match $gimbal 'ALIGN_BACK\)\s*\?\s*PI\s*:\s*0\.0f' 'MECH yaw target must be 0° or 180° by align_dir'

# 保险：小陀螺一票否决——自旋禁止更新 align_dir，退出后再冷却
Assert-Match $gimbal 'car_mode\s*==\s*CAR_MODE_SPIN' 'SPIN must be detected for align veto'
Assert-Match $gimbal 'align_veto_cnt\s*=\s*YAW_ALIGN_POST_SPIN_TICKS' 'SPIN must refresh the align veto counter'
Assert-Match $gimbal 'align_veto_cnt\s*==\s*0U' 'hysteresis updates must be gated on veto==0'
Assert-Match $control 'hero_gimbal\.yaw\.align_dir' 'control task must pass align_dir into Chassis_Update'
$carControl = Get-Content -Raw $carControlHeader
Assert-Match $carControl 'ALIGN_FRONT' 'AlignDir_e must be declared in car_control.h'
Assert-Match $carControl 'ALIGN_BACK' 'AlignDir_e must include ALIGN_BACK'

# 换头/掉头：仅 MECH 粘性锁轴翻转；GYRO/SPIN 世界系 +180° 已删除；结束后立刻按真实轴向对齐
Assert-Match $gimbal 'Gimbal_Turn_Update' 'turn state machine must exist'
Assert-NotMatch $gimbal 'target_imu_angle\s*\+\s*PI' 'GYRO/SPIN world +180° turn must stay deleted'
Assert-Match $gimbal 'ALIGN_BACK.*ALIGN_FRONT|ALIGN_FRONT.*ALIGN_BACK' 'MECH turn must toggle sticky FRONT/BACK lock'
Assert-Match $gimbal 'Gimbal_Turn_Apply_Align_After' 'turn finish must re-apply align from real delta (chassis must not half-turn)'
Assert-Match $gimbal 'align_veto_cnt\s*=\s*0U' 'after turn, align veto must clear so chassis follow_err is immediate'
Assert-Match $gimbal 'turn_active' 'turn_active latch must exist'
Assert-Match $gimbal 'YAW_TURN_ARRIVE_ERR_DEG' 'turn arrive threshold macro missing'
Assert-Match $gimbal 'YAW_TURN_TIMEOUT_TICKS' 'turn timeout macro missing'
# 仅 MECH 进换头状态机：非 MECH 一票否决
Assert-Match $gimbal 'car_mode\s*!=\s*CAR_MODE_MECH' 'turn must veto every non-MECH mode'
$turnUpdate = [regex]::Match($gimbal, 'static\s+void\s+Gimbal_Turn_Update\s*\([^)]*\)\s*\{(?<body>[\s\S]*?)\n\}')
if (-not $turnUpdate.Success) { throw 'Gimbal_Turn_Update not found' }
if ($turnUpdate.Groups['body'].Value -match 'CAR_MODE_GYRO|CAR_MODE_SPIN|target_imu_angle') {
    throw 'turn state machine must stay MECH-only (GYRO/SPIN branch deleted)'
}
# 触发端也只认 MECH（键鼠 R / 遥控 S1上+S2上+拨轮上）
$carCtrl = Get-Content -Raw (Join-Path $repo 'Application/ControlLayer/carctrl.c')
Assert-Match $carCtrl 'Car_R_Turn_Edge\(\)\s*&&\s*car->mode\s*==\s*CAR_MODE_MECH' 'R-key turn trigger must be MECH-only'
Assert-Match $carCtrl 'tb_up\s*&&\s*car->mode\s*==\s*CAR_MODE_MECH' 'RC turn trigger must be MECH-only'
Assert-Match $gimbal 'CAR_MODE_SPIN.*turn_active|turn_active.*CAR_MODE_SPIN|\(car_mode\s*==\s*CAR_MODE_SPIN\)\s*\|\|\s*gimbal->yaw\.turn_active' 'align veto must also freeze during turn'

# ===== GYRO pitch 走 IMU（双板协议 + 上层闭环）=====
# 下层：0x022 收 pitch；0xD1 byte[6] pitch_use_imu；GYRO 累加 target_imu_angle
Assert-Match $gimbalHeader 'target_imu_angle' 'pitch struct must carry IMU-domain target'
Assert-Match $gimbalHeader 'imu_angle' 'pitch struct must carry 0x022 pitch feedback'
Assert-Match $gimbal 'pitch\.target_imu_angle' 'gimbal.c must accumulate GYRO pitch in IMU domain'
Assert-Match $gimbal 'pitch\.target_imu_angle\s*=\s*gimbal->pitch\.imu_angle' 'GYRO entry must sync pitch IMU target to current IMU'
Assert-Match $gimbal 'imu_domain' 'pitch update must branch MECH vs GYRO domain'
# GYRO->MECH 退出时按当前真实机械角重定目标，绝不能拿 IMU 世界角当机械角
Assert-Match $gimbal 'target_mec_angle\s*=\s*board_rx_info\.pitch_mech_angle' 'GYRO->MECH exit must rebase mec pitch target to the real mech angle (IMU world angle is not a mech angle)'
Assert-Match $protocol 'SLAVE_RX_PITCH_IMU_ID' 'CAN2 must dispatch pitch IMU ID'
Assert-Match $protocol 'Board_Rx_Pitch\(rxBuf\)' 'CAN2 must call Board_Rx_Pitch for 0x022'
Assert-Match $communication 'pitch_use_imu' 'Board_Tx_D1 must carry pitch domain flag'
Assert-Match $communication 'tx_data\[6\]' 'pitch_use_imu must ride 0xD1 byte[6]'
Assert-Match $communication 'Board_Rx_Pitch' 'communicate.c must decode pitch IMU'
Assert-Match $control 'pitch_use_imu' 'control task must set pitch domain flag from gimbal mode'
Assert-Match $control 'target_imu_angle' 'control task must TX IMU-domain pitch target in GYRO'

# 上层 C 板：0x022 发送；pitch_use_imu 分支；IMU 闭环 + mec 软限位
# repo = .../Down/RP2026_Templete_DM_MC02/RP2026_Templete-Keil_MDK_DM_MC02
# Up 工程在 RP27-Train_Template-main/Up/RP27-Train_Template-C
$upRoot = Resolve-Path (Join-Path $repo '../../../Up/RP27-Train_Template-C')
$upBoardCommH = Get-Content -Raw (Join-Path $upRoot 'Application/ProtocolLayer/board_comm.h')
$upBoardCommC = Get-Content -Raw (Join-Path $upRoot 'Application/ProtocolLayer/board_comm.c')
$upGimbalH = Get-Content -Raw (Join-Path $upRoot 'Application/ModuleLayer/gimbal.h')
$upGimbalC = Get-Content -Raw (Join-Path $upRoot 'Application/ModuleLayer/gimbal.c')
$upControl = Get-Content -Raw (Join-Path $upRoot 'Application/TaskLayer/control_task.c')

Assert-Match $upBoardCommH 'MASTER_RX_PITCH_IMU_CAN_ID' 'Up board_comm must define pitch IMU CAN ID 0x022'
Assert-Match $upBoardCommH 'pitch_use_imu' 'Up board_comm Rx must parse pitch domain flag'
Assert-Match $upBoardCommH 'Board_Comm_Send_Pitch_IMU' 'Up board_comm must declare pitch IMU TX'
Assert-Match $upBoardCommC 'MASTER_RX_PITCH_IMU_CAN_ID' 'Up board_comm TX must use 0x022'
Assert-Match $upBoardCommC 'pitch_use_imu\s*=\s*rx_buf\[6\]' 'Up must read pitch_use_imu from 0xD1 byte[6]'
Assert-Match $upGimbalH 'use_imu' 'Up pitch struct must carry IMU closed-loop flag'
# 符号约定已定死「抬头为正」：imu_pitch = base_info.pitch * DEG_TO_RAD（无负号、无符号宏）
Assert-Match $upGimbalC 'imu_pitch\s*=\s*imu_sensor\.info->base_info\.pitch\s*\*\s*DEG_TO_RAD' 'Up must store IMU pitch with the fixed positive-up sign convention'
Assert-NotMatch $upGimbalH 'PITCH_IMU_SIGN' 'pitch sign macro must stay deleted (sign fixed positive-up)'
# MECH/GYRO pitch 必须两套独立 PID（结构存在即可，不锁增益数值）
Assert-Match $upGimbalH 'mech_angle_pid' 'Up pitch must keep MECH angle PID separate'
Assert-Match $upGimbalH 'mech_speed_pid' 'Up pitch must keep MECH speed PID separate'
Assert-Match $upGimbalH 'imu_angle_pid' 'Up pitch must keep IMU angle PID separate'
Assert-Match $upGimbalH 'imu_speed_pid' 'Up pitch must keep IMU speed PID separate'
Assert-Match $upGimbalH 'max_speed_limit' 'Up pitch must keep the outer speed limit'
Assert-Match $upGimbalC 'mech_angle_pid' 'Up init/clear must manage MECH angle PID'
Assert-Match $upGimbalC 'mech_speed_pid' 'Up init/clear must manage MECH speed PID'
Assert-Match $upGimbalC 'imu_angle_pid' 'Up init/clear must manage IMU angle PID'
Assert-Match $upGimbalC 'imu_speed_pid' 'Up init/clear must manage IMU speed PID'
Assert-Match $upGimbalC 'Gimbal_Pitch_Clear_Pids' 'domain switch / Safe_Stop must clear all pitch PID sets'
Assert-Match $upGimbalC 'board_rx_info\.pitch_use_imu' 'Up gimbal must consume pitch domain flag'
# 软限位：GYRO 域吃静态 IMU 限位（0.04/0.70 定死），MECH 域吃 mec_min/max_rad
Assert-Match $upGimbalC 'PITCH_MIN_IMU_ANGLE' 'IMU path must soft-limit via the static IMU floor'
Assert-Match $upGimbalC 'PITCH_MAX_IMU_ANGLE' 'IMU path must soft-limit via the static IMU ceiling'
Assert-Match $upGimbalC 'mec_min_rad' 'MECH path must soft-limit via mec min'
Assert-Match $upGimbalC 'mec_max_rad' 'MECH path must soft-limit via mec max'
Assert-Match $upGimbalC 'use_imu\s*\?\s*gimbal\.pitch\.imu_pitch' 'outer angle loop must close on world IMU pitch in GYRO'
Assert-Match $upGimbalC 'imu_speed' 'Up must carry IMU pitch rate for the speed loop'
Assert-Match $upGimbalC 'Pitch_Gravity_Compensation' 'gravity feedforward must remain in the torque path'
# 重力前馈：一条 cos、永远吃编码器机械角（两域共用）；禁切 use_imu 分支、禁吃 IMU 角（自激励）
Assert-Match $upGimbalC 'Pitch_Gravity_Compensation\s*\(\s*gimbal\.pitch\.feedback_pitch\s*\)' 'FF must eat the encoder mech angle in both modes'
Assert-NotMatch $upGimbalC 'Pitch_Gravity_Compensation\s*\(\s*gimbal\.pitch\.imu_pitch' 'FF must never eat the IMU angle (self-excitation)'
Assert-NotMatch $upGimbalC 'use_imu\s*\?\s*gimbal\.pitch\.gravity' 'FF knobs must not fork per domain (single cos model)'
Assert-Match $upGimbalH 'gravity_max_torque' 'FF amplitude knob must exist'
Assert-Match $upGimbalH 'gravity_phase_deg' 'FF phase knob must exist'
Assert-NotMatch $upGimbalH 'gyro_gravity_phase_deg' 'per-domain gravity phase must stay deleted'
Assert-Match $upControl 'Board_Comm_Send_Pitch_IMU' 'Up control task must send 0x022 pitch IMU'

Write-Output 'gimbal IMU safety contract passed'
