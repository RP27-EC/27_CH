# Four-Mecanum Chassis Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement four-M3508 mecanum movement on FDCAN1 from the DBUS remote in the FreeRTOS control task.

**Architecture:** A HAL-independent kinematics unit calculates normalized FL/FR/BL/BR rotor-RPM targets. `chassis.c` maps those targets into four existing `RM_Motor` speed controllers; DeviceLayer owns motor objects, ProtocolLayer owns feedback dispatch, and TaskLayer owns the 2 ms control cycle and fail-safe decision.

**Tech Stack:** C11 host tests with GCC, STM32H723 HAL/FDCAN, CMSIS-RTOS2, Keil MDK project, existing `AlgorithmLayer/PID` and `HardwareLayer/RM_Motor`.

## Global Constraints

- Motors are all on FDCAN1: `0x201=FL`, `0x202=FR`, `0x203=BL`, `0x204=BR`; command ID is `0x200`.
- Remote mapping is `ch3=forward`, `ch2=lateral`, `ch0=rotation`; do not add a dead zone or enable switch.
- Remote offline or invalid input must send zero current and clear PID state.
- Reuse `pid_err_cal()`, `single_pid_ctrl()`, and `pid_clear()` through the existing RM motor API; do not create another PID implementation.
- Target values and feedback values use motor-rotor RPM.

---

### Task 1: Pure Mecanum Kinematics

**Files:**
- Create: `Application/ModuleLayer/chassis_kinematics.h`
- Create: `Application/ModuleLayer/chassis_kinematics.c`
- Create: `test/chassis/test_chassis_kinematics.c`

**Interfaces:**
- Produces: `void Chassis_Mecanum_Calculate(float vx, float vy, float wz, float max_rpm, float wheel_rpm[4]);`
- Wheel indices: `0=FL`, `1=FR`, `2=BL`, `3=BR`.

- [ ] **Step 1: Write failing host tests** covering zero, forward, lateral, rotation, and proportional saturation. Assertions use `fabsf(actual - expected) < 0.001f`; saturation case uses `(1, 1, 1, 6000)` and expects `[-2000, 6000, 2000, 2000]`.
- [ ] **Step 2: Verify RED** with `gcc -std=c11 -Wall -Wextra -Werror test/chassis/test_chassis_kinematics.c -IApplication/ModuleLayer -lm -o test/chassis/test_chassis_kinematics.exe`; expected failure is an undefined `Chassis_Mecanum_Calculate` reference.
- [ ] **Step 3: Implement the formula** `FL=vx-vy-wz`, `FR=vx+vy+wz`, `BL=vx+vy-wz`, `BR=vx-vy+wz`, then scale all four by `max_rpm / max_abs` when `max_abs > 1`, otherwise multiply by `max_rpm`; treat a null output or non-positive `max_rpm` as no operation/zero output as applicable.
- [ ] **Step 4: Verify GREEN** by compiling with `Application/ModuleLayer/chassis_kinematics.c` and running the produced executable; expected output is `5 tests passed` and exit code 0.
- [ ] **Step 5: Commit** with `git commit -m "feat: 添加麦轮运动学解算"`.

### Task 2: Four-Motor Device Configuration and Feedback

**Files:**
- Replace: `Application/DeviceLayer/chassis_motor.c`
- Modify: `Application/DeviceLayer/chassis_motor.h`
- Modify: `Application/DeviceLayer/device.c`
- Modify: `Application/ProtocolLayer/can_protocol.c`

**Interfaces:**
- Produces: `Motor_RM_t Chassis_Motor[4]` and `Motor_RM_Group_t Chassis_Motor_Group`.
- Consumes: existing `RM_Motor_Init`, `RM_Group_Motor_Init`, `hfdcan1`, and motor `rx()`.

- [ ] **Step 1: Add a static source-contract test** in `test/chassis/test_chassis_wiring.ps1` that fails unless all four `rxId` values `0..3`, `_3508_Reduction`, `&hfdcan1`, group ID `0x200`, and CAN1 cases `0x201..0x204` are present.
- [ ] **Step 2: Verify RED** with `powershell -ExecutionPolicy Bypass -File test/chassis/test_chassis_wiring.ps1`; expected failure reports the missing four-motor group.
- [ ] **Step 3: Replace the eight-motor swerve configuration** with four complete `Motor_RM_t` objects. Give each motor its own `pid_ctrl_t` using initial parameters `kp=6.0f`, `ki=0.0f`, `kd=0.0f`, `a=1.0f`, `integral_max=6000.0f`, `out_max=10000.0f`; expose wheel-direction constants in `chassis_motor.h` for later bench correction.
- [ ] **Step 4: Update initialization and dispatch** so `DEVICE_Init()` initializes only `Chassis_Motor_Group` for the chassis and `CAN1_rxDataHandler()` routes IDs `0x201..0x204` to indices `0..3` without changing the existing yaw case.
- [ ] **Step 5: Verify GREEN** by rerunning the PowerShell contract test; expected output is `chassis wiring contract passed`.
- [ ] **Step 6: Commit** with `git commit -m "feat: 配置四路3508底盘电机"`.

### Task 3: Chassis Module and Fail-Safe Control

**Files:**
- Replace: `Application/ModuleLayer/chassis.h`
- Replace: `Application/ModuleLayer/chassis.c`
- Create: `test/chassis/test_chassis_contract.ps1`

**Interfaces:**
- Produces: `void Chassis_Init(void)`, `void Chassis_Update(int16_t forward, int16_t lateral, int16_t rotate, bool failsafe)`, and `void Chassis_Stop(void)`.
- Consumes: `Chassis_Mecanum_Calculate`, `Chassis_Motor[4]`, motor `single_set_speed()`, `pid_clear()`, and `Chassis_Motor_Group.group_set_torque()`.

- [ ] **Step 1: Write a failing contract test** requiring no private PID type/function in `chassis.c`, calls to `Chassis_Mecanum_Calculate`, `single_set_speed`, `pid_clear`, and an explicit reset of `target`, `err`, `last_err`, `out`, and motor torque in `Chassis_Stop()`.
- [ ] **Step 2: Verify RED** with `powershell -ExecutionPolicy Bypass -File test/chassis/test_chassis_contract.ps1`; expected failure reports the legacy private PID implementation.
- [ ] **Step 3: Implement the module** by clamping channel values to `[-660,660]`, normalizing without a dead zone, calculating rotor targets with `CHASSIS_MAX_MOTOR_RPM`, applying per-wheel direction constants once, assigning each `speed_ctrl->target`, and calling each motor's `single_set_speed()`.
- [ ] **Step 4: Implement fail-safe** so `Chassis_Stop()` sets all targets and torques to zero, calls `pid_clear()`, resets remaining PID history/output fields, and sends the zero group command. `Chassis_Update(..., true)` must call only `Chassis_Stop()`.
- [ ] **Step 5: Verify GREEN** by rerunning both host kinematics and module contract tests; both must exit 0.
- [ ] **Step 6: Commit** with `git commit -m "feat: 实现四麦轮底盘控制模块"`.

### Task 4: FreeRTOS Integration and Keil Project Registration

**Files:**
- Modify: `Application/TaskLayer/control_task.c`
- Modify: `MDK-ARM/DM-MC02.uvprojx`
- Modify: `test/chassis/test_chassis_wiring.ps1`

**Interfaces:**
- Consumes: `rc_sensor.work_state`, `rc_sensor.errno`, `rc_sensor.info->{ch3,ch2,ch0}`, `Chassis_Init()`, and `Chassis_Update()`.

- [ ] **Step 1: Extend the failing wiring test** to require `chassis.c` and `chassis_kinematics.c` in the Keil project and require `control_task.c` to pass `ch3/ch2/ch0`, offline/data-error fail-safe, and `osDelay(2)`.
- [ ] **Step 2: Verify RED**; expected failure reports missing task integration and project entries.
- [ ] **Step 3: Integrate the task** by calling `Chassis_Init()` once before the loop, deriving fail-safe from `work_state != DEV_ONLINE || errno != NONE_ERR`, calling `Chassis_Update(ch3, ch2, ch0, failsafe)` each cycle, and retaining existing yaw sleep behavior.
- [ ] **Step 4: Register both module sources** in the existing ModuleLayer group of `DM-MC02.uvprojx` without changing generated CubeMX settings.
- [ ] **Step 5: Verify GREEN** with the PowerShell wiring test and GCC kinematics test; expected exit code is 0 for both.
- [ ] **Step 6: Run repository checks**: `git diff --check`, search for obsolete `Front_Group|Back_Group|hero_chassis|HAL_CAN_AddTxMessage`, and inspect `git diff --stat`. Expected: no whitespace errors and no live obsolete chassis symbols.
- [ ] **Step 7: Commit** with `git commit -m "feat: 接入FreeRTOS底盘控制任务"`.

### Task 5: Final Verification and Bench Checklist

**Files:**
- Modify: `docs/superpowers/specs/2026-09-24-mecanum-chassis-design.md` only if implementation reveals an interface correction.

**Interfaces:**
- Consumes all prior deliverables; produces verification evidence only.

- [ ] **Step 1: Run all automated checks**: GCC compile/run of kinematics, both PowerShell contract tests, and `git diff --check`; expected all exit 0.
- [ ] **Step 2: Inspect safety path** from remote heartbeat through `control_task` to the zero `0x200` frame and confirm no second sender controls IDs `0x201..0x204` on FDCAN1.
- [ ] **Step 3: Record toolchain limitation**: this workstation has no `UV4.exe` or `armclang.exe`, so Keil full-build verification must be performed on a machine with MDK installed.
- [ ] **Step 4: Provide the bench sequence**: raise chassis, limit initial RPM, test forward/lateral/rotation separately, change only wheel-direction constants if signs are wrong, then tune the four speed PID parameters.
