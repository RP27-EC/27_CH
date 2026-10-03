# Remove Auto-Aim Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the unused auto-aim control and state while preserving all other robot controls and communication layouts.

**Architecture:** Remove the vision mode enum and flag from the Down car-control interface, and remove its thumbwheel-down action branch from the state machine. Keep thumbwheel-down edge detection for chassis mode switching; only generalize comments that describe reserved communication bytes or UI examples as auto-aim-specific.

**Tech Stack:** STM32 C firmware, Keil MDK project, existing PowerShell contract checks.

## Global Constraints

- Do not change mechanical, gyro, spin, gimbal pitch, shooting, super-capacitor, or keyboard/mouse behavior.
- Preserve the existing CAN frame IDs, lengths, byte positions, and payload behavior.
- Keep thumbwheel-down connected to chassis mode switching.
- Preserve the friction-wheel toggle, single-shot, continuous-shot, and remote-loss safety behavior.
- Do not add dependencies or unrelated refactoring.

---

### Task 1: Remove auto-aim state and remote action

**Files:**
- Modify: `Down/RP2026_Templete_DM_MC02/RP2026_Templete-Keil_MDK_DM_MC02/Application/ControlLayer/car_control.h`
- Modify: `Down/RP2026_Templete_DM_MC02/RP2026_Templete-Keil_MDK_DM_MC02/Application/ControlLayer/carctrl.c`
- Create: `Down/RP2026_Templete_DM_MC02/RP2026_Templete-Keil_MDK_DM_MC02/test/remote/test_car_control_contract.ps1`

**Interfaces:**
- `CarFlags_t` retains only the friction-wheel and firing flags; `VisionMode_e` is removed.
- `Car_Mode_Switch_Update(Car_t *car, bool tb_up, bool tb_dn)` continues consuming both thumbwheel edges.
- `Car_Action_Cmd_Update` accepts only `Car_t *car` and `bool tb_up`; thumbwheel-down remains unavailable to action control.

- [ ] Create `test/remote/test_car_control_contract.ps1` with this contract:

```powershell
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
```

- [ ] Run `powershell -NoProfile -File test\remote\test_car_control_contract.ps1` before changing production code. Expected: FAIL because `VisionMode_e` and `vision_mode` still exist.
- [ ] Remove the following enum and field from `car_control.h`:

```c
/* 自瞄状态枚举 */
typedef enum {
    VISION_OFF = 0,
    VISION_AUTO_AIM,
} VisionMode_e;
```

and remove `VisionMode_e vision_mode;` from `CarFlags_t`, preserving its three firing flags.
- [ ] Change `Car_Action_Cmd_Update(Car_t *car, bool tb_up, bool tb_dn)` to `Car_Action_Cmd_Update(Car_t *car, bool tb_up)`. Delete only the `S1 中位 + S2 中位 + tb_dn` auto-aim toggle; preserve the friction-wheel and firing logic. Keep `Car_Mode_Switch_Update(car, tb_up, tb_dn)` unchanged and call action control as `Car_Action_Cmd_Update(car, tb_up)`.
- [ ] Remove vision-state assignments from initialization and remote-loss handling. Update adjacent comments and function descriptions to describe friction-wheel and firing controls only.
- [ ] Run `powershell -NoProfile -File test\remote\test_car_control_contract.ps1` again. Expected: PASS and `Car-control contract passed.`
- [ ] Review the diff and verify mode selection, friction-wheel, firing, and loss-protection logic is otherwise unchanged.

### Task 2: Generalize auto-aim-only comments and validate

**Files:**
- Modify: `Down/RP2026_Templete_DM_MC02/RP2026_Templete-Keil_MDK_DM_MC02/Application/TaskLayer/UI_Task.c`
- Modify: `Down/RP2026_Templete_DM_MC02/RP2026_Templete-Keil_MDK_DM_MC02/Application/ProtocolLayer/communicate.c`
- Modify: `Up/RP27-Train_Template-C/Application/ProtocolLayer/board_comm.c`

- [ ] Change the UI comment's auto-aim status example to a generic mode/status example.
- [ ] Change reserved-byte comments in both communication files to generic future-extension wording; do not change executable code or payload layout.
- [ ] Search both application trees for `VISION_AUTO_AIM`, `VISION_OFF`, `VisionMode_e`, `vision_mode`, and auto-aim-only comments; expect no matches outside the approved design/spec documents.
- [ ] Run `git diff --check` in each repository and review the diff to confirm no communication or unrelated control code changed.
- [ ] Check `Get-Command UV4.exe -ErrorAction SilentlyContinue`; if Keil MDK is installed, build `MDK-ARM\DM-MC02.uvprojx` with it. If not, report the missing toolchain and retain the passing PowerShell contract and diff checks as validation evidence.
