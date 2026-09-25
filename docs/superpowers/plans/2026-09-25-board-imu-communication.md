# Board IMU Communication Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Send the upper-board BMI088 Yaw angle and rate over CAN2 ID `0x021` every 2 ms while keeping all Yaw and chassis closed-loop control on the lower board.

**Architecture:** A focused upper-board protocol module packs two radian-valued floats and delegates transmission to the existing CAN driver. The upper control task performs degree-to-radian conversion immediately after a valid IMU update. The lower board consumes the existing frame, rejects stale IMU feedback in gyro mode, and uses the declared mechanical-angle field for chassis following.

**Tech Stack:** STM32F407 HAL CAN, STM32H723 HAL FDCAN, CMSIS-RTOS, C99, PowerShell contract tests, host GCC.

## Global Constraints

- The upper board is an IMU data supplier only; it must not contain Yaw PID, KT9025 output, or chassis-follow control.
- Use upper-board `CAN2`, standard ID `0x021`, DLC 8.
- Bytes 0..3 are little-endian IEEE-754 `yaw_imu` in rad; bytes 4..7 are `yaw_v` in rad/s.
- Send at the existing 2 ms control-task cadence only after a valid IMU update.
- The lower board owns the Yaw gyro double loop, KT9025 output, and chassis-follow loop.
- Preserve unrelated user changes in both dirty repositories.

---

### Task 1: Upper-board IMU frame protocol

**Files:**
- Create: `Application/ProtocolLayer/board_comm.h`
- Create: `Application/ProtocolLayer/board_comm.c`
- Create: `test/board_comm/stubs/drv_can.h`
- Create: `test/board_comm/test_board_comm.c`
- Create: `test/board_comm/run_tests.ps1`

**Interfaces:**
- Consumes: `CAN_SendData(CAN_HandleTypeDef *, uint32_t, uint8_t *)` and global `hcan2`.
- Produces: `void Board_Comm_Init(void)` and `HAL_StatusTypeDef Board_Comm_Send_IMU(float yaw_angle_rad, float yaw_rate_rad_s)`.

- [ ] **Step 1: Write the failing host test**

Create a stub CAN driver that records the handle, ID, and eight payload bytes. Test that `Board_Comm_Send_IMU(1.25f, -2.5f)` calls CAN2 with ID `0x021`, preserves both float bit patterns at offsets 0 and 4, and returns the driver's HAL status.

```c
assert(captured_hcan == &hcan2);
assert(captured_id == 0x021U);
memcpy(&yaw, captured_data, sizeof(yaw));
memcpy(&rate, captured_data + 4, sizeof(rate));
assert(yaw == 1.25f);
assert(rate == -2.5f);
assert(Board_Comm_Send_IMU(0.0f, 0.0f) == HAL_ERROR);
```

- [ ] **Step 2: Run the test and verify RED**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File test/board_comm/run_tests.ps1
```

Expected: GCC fails because `board_comm.h` and `board_comm.c` do not exist.

- [ ] **Step 3: Implement the protocol module**

Use a named ID macro, `memcpy` at fixed byte offsets, and the existing driver:

```c
#define MASTER_RX_YAW_IMU_CAN_ID (0x021U)

HAL_StatusTypeDef Board_Comm_Send_IMU(float yaw_angle_rad,
                                      float yaw_rate_rad_s)
{
    uint8_t tx_buf[8];
    memcpy(&tx_buf[0], &yaw_angle_rad, sizeof(float));
    memcpy(&tx_buf[4], &yaw_rate_rad_s, sizeof(float));
    return CAN_SendData(&hcan2, MASTER_RX_YAW_IMU_CAN_ID, tx_buf);
}
```

- [ ] **Step 4: Run the host test and verify GREEN**

Run the same PowerShell command. Expected: `board_comm tests passed` and exit code 0.

- [ ] **Step 5: Commit only Task 1 files**

```powershell
git add Application/ProtocolLayer/board_comm.h Application/ProtocolLayer/board_comm.c test/board_comm
git commit -m "feat: 添加上层IMU板间通信"
```

### Task 2: Upper-board scheduling and Keil integration

**Files:**
- Modify: `Application/TaskLayer/control_task.c`
- Modify: `MDK-ARM/My_C.uvprojx`
- Create: `test/board_comm/test_integration_contract.ps1`

**Interfaces:**
- Consumes: `Board_Comm_Send_IMU(float, float)` from Task 1 and `imu_sensor.info->base_info.{yaw,rate_yaw}` in degrees.
- Produces: one `0x021` CAN2 frame after each valid 2 ms IMU update.

- [ ] **Step 1: Write the failing integration contract**

The PowerShell test must assert that `control_task.c` includes `board_comm.h`, converts both fields with `DEG_TO_RAD`, calls `Board_Comm_Send_IMU` inside the valid-IMU branch after `imu_sensor.update`, and that `My_C.uvprojx` includes `board_comm.c` in the `Protocol` group.

```powershell
Assert-Match $control '#include\s+"board_comm\.h"'
Assert-Match $control 'yaw\s*\*\s*DEG_TO_RAD'
Assert-Match $control 'rate_yaw\s*\*\s*DEG_TO_RAD'
Assert-Match $project '<FileName>board_comm\.c</FileName>'
```

- [ ] **Step 2: Run the contract and verify RED**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File test/board_comm/test_integration_contract.ps1
```

Expected: FAIL because the include, conversions, call, and project entry are absent.

- [ ] **Step 3: Add the minimal task integration**

Inside the existing valid-IMU branch, immediately after update:

```c
#define DEG_TO_RAD (0.01745329251994329577f)

imu_sensor.update(&imu_sensor);
Board_Comm_Send_IMU(imu_sensor.info->base_info.yaw * DEG_TO_RAD,
                    imu_sensor.info->base_info.rate_yaw * DEG_TO_RAD);
```

Add `board_comm.c` to the existing Keil `Protocol` file group without changing unrelated project entries.

- [ ] **Step 4: Run all upper-board tests**

```powershell
powershell -ExecutionPolicy Bypass -File test/board_comm/run_tests.ps1
powershell -ExecutionPolicy Bypass -File test/board_comm/test_integration_contract.ps1
```

Expected: both exit 0.

- [ ] **Step 5: Commit only Task 2 files**

```powershell
git add Application/TaskLayer/control_task.c MDK-ARM/My_C.uvprojx test/board_comm/test_integration_contract.ps1
git commit -m "feat: 周期发送云台IMU姿态"
```

### Task 3: Lower-board stale-data protection and chassis angle contract

**Files:**
- Modify: `Application/ModuleLayer/gimbal.c`
- Modify: `Application/TaskLayer/control_task.c`
- Create: `test/gimbal/test_gimbal_imu_contract.ps1`

**Interfaces:**
- Consumes: `board_rx_info.is_online`, `GIMBAL_MODE_GYRO`, and `hero_gimbal.yaw.feedback_mec_angle`.
- Produces: zero Yaw output with cleared PID state when the gyro-mode IMU link is offline; correct mechanical-angle input to `Chassis_Update`.

- [ ] **Step 1: Write the failing lower-board contract**

The PowerShell test must assert that the gyro safety condition references `board_rx_info.is_online`, that the offline branch clears both Yaw PIDs and sends zero current, and that `Chassis_Update` receives `feedback_mec_angle` rather than the nonexistent `feedback_angle`.

```powershell
Assert-Match $gimbal 'GIMBAL_MODE_GYRO[\s\S]*board_rx_info\.is_online'
Assert-Match $gimbal 'pid_clear\(&gimbal->yaw\.angle_pid\)'
Assert-Match $gimbal 'KT_Yaw_SendIq\(0\)'
Assert-Match $control 'hero_gimbal\.yaw\.feedback_mec_angle'
Assert-NotMatch $control 'hero_gimbal\.yaw\.feedback_angle'
```

- [ ] **Step 2: Run the contract and verify RED**

Run from the lower-board repository:

```powershell
powershell -ExecutionPolicy Bypass -File test/gimbal/test_gimbal_imu_contract.ps1
```

Expected: FAIL because the online flag is not a gyro safety input and the control task names the wrong field.

- [ ] **Step 3: Implement fail-safe behavior and field correction**

Include `communicate.h` in `gimbal.c`. Before gyro PID calculation, handle stale feedback:

```c
if (gimbal->mode == GIMBAL_MODE_GYRO && !board_rx_info.is_online) {
    pid_clear(&gimbal->yaw.angle_pid);
    pid_clear(&gimbal->yaw.speed_pid);
    gimbal->yaw.give_current = 0;
    kt_motor[0].base_info.motor_out = 0;
    KT_Yaw_SendIq(0);
    return;
}
```

Change only the chassis-follow argument to `hero_gimbal.yaw.feedback_mec_angle`.

- [ ] **Step 4: Run the lower contract and existing chassis tests**

```powershell
powershell -ExecutionPolicy Bypass -File test/gimbal/test_gimbal_imu_contract.ps1
powershell -ExecutionPolicy Bypass -File test/chassis/test_chassis_contract.ps1
powershell -ExecutionPolicy Bypass -File test/chassis/test_chassis_wiring.ps1
```

Expected: all commands exit 0.

- [ ] **Step 5: Commit only Task 3 files**

```powershell
git add Application/ModuleLayer/gimbal.c Application/TaskLayer/control_task.c test/gimbal/test_gimbal_imu_contract.ps1
git commit -m "fix: 增加云台IMU失联保护"
```

### Task 4: Final cross-repository verification

**Files:**
- Verify only; no production changes expected.

**Interfaces:**
- Consumes: all previous task outputs.
- Produces: fresh evidence that protocol, scheduling, project integration, lower safety, and existing chassis contracts pass.

- [ ] **Step 1: Run all focused tests from their owning repositories**

```powershell
# Upper repository
powershell -ExecutionPolicy Bypass -File test/board_comm/run_tests.ps1
powershell -ExecutionPolicy Bypass -File test/board_comm/test_integration_contract.ps1

# Lower repository
powershell -ExecutionPolicy Bypass -File test/gimbal/test_gimbal_imu_contract.ps1
powershell -ExecutionPolicy Bypass -File test/chassis/test_chassis_contract.ps1
powershell -ExecutionPolicy Bypass -File test/chassis/test_chassis_wiring.ps1
```

Expected: all commands exit 0 with no failed assertions.

- [ ] **Step 2: Inspect scoped diffs and repository status**

```powershell
git diff HEAD^ --check
git status --short
```

Confirm the task commits contain only named files and all pre-existing unrelated changes remain present.

- [ ] **Step 3: Record target-build limitation**

`UV4`, `armcc`, and `armclang` are not present on PATH. Report that host tests and project-contract checks passed, but do not claim a full STM32 target build was executed.
