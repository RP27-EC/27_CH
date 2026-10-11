# Chassis power implementation plan

> **For agentic workers:** Use subagent-driven-development for the independent referee receive task, then integrate and verify in this workspace. Track each step below.

**Goal:** Limit four-wheel current using a power model, referee limits, buffer feedback and read-only capacitor power measurements.

**Architecture:** A HAL-independent chassis_power module receives a telemetry snapshot, computes the power budget and scales a four-current array. The chassis adapter gathers telemetry and applies this result after all speed PIDs, before the group CAN send. Referee and capacitor frames have independent receive counters and timestamps.

**Tech Stack:** C99, STM32H7/Keil ARMCC, FreeRTOS, Python unittest with host GCC.

## Global constraints

- Modify only the main Down/RP2026_Templete_DM_MC02/RP2026_Templete-Keil_MDK_DM_MC02 project. Reference projects and Up remain unchanged.
- Preserve user build artifacts and current motion/PID parameters; stage only feature files.
- Training limit 80W, margin 5W, buffer target 50J, buffer gain 1W/J; expose all tuning in chassis_power.
- Stale limit: retain last valid limit. Never received: training limit. Stale buffer/measurement: freeze existing derates.
- Buffer and measured-power correction only reduce budget. No capacitor discharge, boost or 1.25 multiplier; capacitor TX stays disabled.
- Compare signed raw current and signed actual motor RPM; scale all four currents by one factor in [0,1]. Never use negative wheel power to offset positive wheel power.
- Protocol supports 13-byte 0x0201, 14-byte modern and 16-byte legacy 0x0202; validate CRC, payload length and actual DMA byte count. Preserve incomplete frames across chunks.

## Task 1: Referee reception

Files: Application/ProtocolLayer/judge_protocol.c/.h, Application/DeviceLayer/judge.c/.h, Application/DriverLayer/drv_uart.c, Application/DeviceLayer/device.c, Application/TaskLayer/monitor_task.c, test/referee/test_referee.py.

Interfaces: `judge_recive(const uint8_t *rxBuf, uint16_t length)`, `judge_update(uint16_t id, const uint8_t *rxBuf, uint16_t length)`; Judge_Status_t exposes limit_received, limit_rx_count, limit_update_ms, buffer_received, buffer_rx_count, buffer_update_ms. USART1 callback includes received length.

- [x] Test actual firmware CRC/parser for fragmentation, multiple frames, noise/CRC failures, malformed lengths, independent freshness, 14/16-byte buffer frames and no 0x0201 overflow.
- [x] Observe failures before implementing. Restore init and heartbeat, but heartbeat must not erase last received frames or freshness history on timeout.
- [x] Implement bounded stream parsing and stop/restart USART1 DMA with actual received-byte count. Run receiver tests.

## Task 2: Power budget and limiter

Files: Application/AlgorithmLayer/chassis_power.c/.h, test/power/test_chassis_power.py.

Interfaces: `Chassis_Power_Init(void)`, `Chassis_Power_Update(const ChassisPowerInput_t *, uint32_t now_ms)`, `Chassis_Power_Limit(float current[4], const float rpm[4])`; global `ChassisPower_t chassis_power`.

Example budget assertions:

```c
Chassis_Power_Init();
ChassisPowerInput_t input = {0};
Chassis_Power_Update(&input, 0);
CHECK(chassis_power.target_power_w == 75);
/* new buffer=40J -> 10W derate; repeat/stale sample freezes derate */
```

- [x] Add/run failing tests for training/last-limit fallback, independent timeouts, recovery, fresh-frame-only correction and signed four-wheel current limits.
- [x] Implement model coefficients from the reference as configurable starting values: k_w2=3e-7, k_i2=1.23e-7, k_iw=1.99688994e-6, k_const=4.081.
- [x] Fast budget reductions, bounded slow increases; measurement derate only on new valid frames and freezes when stale. Clear unsafe model outputs rather than return unrestricted currents.
- [x] Solve a common current factor by bounded bisection; verify final prediction against budget. Cover braking, impossible zero-current baseline, NaN, timer rollover and randomized inputs.

Run: `python -m unittest discover -s test/power -p test_*.py -v`.

## Task 3: Integrate and verify

Files: Application/ModuleLayer/chassis.c, Application/ProtocolLayer/cap_protocol.c/.h, MDK-ARM/DM-MC02.uvprojx, test/chassis host fixtures, docs/chassis_power.md.

```c
/* after four single_set_speed calls */
Chassis_Power_Update(&snapshot, HAL_GetTick());
Chassis_Power_Limit(current, rpm);
for (unsigned i=0; i<4; ++i) Chassis_Motor[i].tx_info->torque=current[i];
Chassis_Motor_Group.group_set_torque(&Chassis_Motor_Group);
```

- [x] Test that limiting occurs before group send, including MECH/GYRO/SPIN and failsafe stop.
- [x] Add capacitor measurement timestamps/counters without enabling TX. Register new source/header in Keil.
- [x] Run all host gimbal, chassis, power and referee tests plus existing PowerShell contracts. Build firmware if compiler available; otherwise explicitly report that gap.
- [x] Review feature diff, run scoped diff-check, write Watch/parameter notes and commit exact feature files with Chinese Conventional Commit.

## Verification results

- Host GCC: power/cap 10, referee/UART 4, chassis integration 6, gimbal 28; C control 8 and kinematics 7. Five PowerShell contracts.
- ARMCC5 full target compiled and linked, including CMSIS-DSP, temporary AXF/HEX output. No flashing.
- Independent review identified DMA TC/IDLE reentry and DME recovery; both reproduced with callback tests before correction.
- Existing yaw default gain 0.65 preserved; old gain tests now set their own 1.0 fixture explicitly.
- Reference projects, Up, user motion comments and existing build outputs are excluded from the commit.
