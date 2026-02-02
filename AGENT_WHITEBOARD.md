# Agent Whiteboard

**Purpose:** Communication across context windows and between agents.

## Use Cases
1. **Cross-agent review** - Flag concerns about other agents' work (see `CROSS_AGENT_REVIEW.md`)
2. **Cross-context handoff** - Notes for future Claude sessions when context is lost
3. **Work-in-progress tracking** - Track incomplete tasks spanning multiple sessions
4. **Hardware decisions pending** - Flag items needing user input before code changes

**Last reviewed by Nathan**: *[DATE]*

---

## Open Flags

### 2026-02-02 - Claude Code CLI (WIP: FreeRTOS SMP Core 1 Issue)
**Task**: Fix AP_Param crash to enable 6-position accelerometer calibration persistence
**Plan File**: `C:\Users\pow-w\.claude\plans\swift-toasting-diffie.md`
**Status**: BLOCKED - FreeRTOS SMP dual-core issue prevents reliable operation

## Session Findings (2026-02-02)

### Resolved: Static Semaphore Initialization
- **Root cause**: Static `Semaphore` objects in `Scheduler.cpp` and `DeviceBus` called FreeRTOS APIs during static initialization (before `vTaskStartScheduler()`)
- **Fix**: Deferred initialization using placement new in `init()` methods
- **Files**: `Scheduler.cpp`, `DeviceBus.cpp`, `DeviceBus.h`, `I2CDevice.cpp`

### Observed: 6-Position Calibration Works (When Device Runs)
- Status transitions correctly: 1 (WAITING) → 2 (COLLECTING) → 1 (WAITING)
- Step increments properly after sample collection
- Samples fed to AccelCal correctly

### BLOCKING ISSUE: FreeRTOS SMP Core 1 Crashes

**Symptoms:**
- Core 1 consistently ends up at PC=0x000000da (bootrom WFE loop), msp=0xf0000000
- GDB confirmed Core 1 **does launch initially** (prvPassiveIdleTask visible running)
- Core 1 then crashes/dies at some point during operation
- Core 0 continues I2C/sensor operations normally
- When Core 0 tries to acquire FreeRTOS spinlocks, it deadlocks (Core 1 may be holding them)

**Debug Evidence:**
1. `info threads` after fresh boot shows Core 1 in `prvPassiveIdleTask` (correct)
2. `info threads` after ~5 seconds shows Core 1 at `0x000000da` (bootrom WFE)
3. No stack overflow detected (stack fill pattern `0xa5a5a5a5` intact)
4. All application tasks pinned to Core 0 - only FreeRTOS idle task runs on Core 1
5. Deadlock when Core 0's USB stdio ISR tries to acquire scheduler spinlock

**Working Theory:**
The FreeRTOS SMP port for RP2350 has an issue where Core 1's passive idle task crashes or exits. Since only the passive idle task runs on Core 1 (all our tasks are pinned to Core 0), this may be:
- Bug in FreeRTOS-Kernel RP2350 SMP port
- Missing configuration
- Interaction with TinyUSB (which expects Core 1 for some operations)

**Recommended Path Forward:**
1. **Option A**: Switch to single-core mode (`configNUMBER_OF_CORES=1`) - proven stable
2. **Option B**: Pin a real task to Core 1 instead of leaving it idle-only
3. **Option C**: Update FreeRTOS-Kernel to latest and check for fixes

**Key files modified (this session):**
- `lib/ap_compat/AP_HAL_RP2350/Scheduler.cpp` - placement new for Semaphore
- `lib/ap_compat/AP_HAL_RP2350/DeviceBus.cpp` - placement new for Semaphore, pointer instead of member
- `lib/ap_compat/AP_HAL_RP2350/DeviceBus.h` - changed Semaphore to pointer + storage
- `lib/ap_compat/AP_HAL_RP2350/I2CDevice.cpp` - use get_semaphore()
- `FreeRTOSConfig.h` - dual-core enabled (configNUMBER_OF_CORES=2)

---

### 2026-01-31 - Claude Code CLI
**File**: Multiple files (see list below)
**Observation**: ISM330DHCX code and references remain in codebase after switching to ICM-20948 as primary IMU
**Why it seems odd**: ICM-20948 has native ArduPilot (Invensensev2) support; ISM330DHCX code may be dead weight
**Possible explanations**: ISM330DHCX kept as alternative/backup sensor option (FeatherWing #4569 is on-hand)
**Severity**: Medium
**Recommendation**: Review and decide: keep as option, or remove to reduce maintenance burden

**Files with ISM330DHCX code/references (require review):**
- `src/hal/IMU_ISM330DHCX.h` - Full driver header
- `src/hal/IMU_ISM330DHCX.cpp` - Full driver implementation
- `src/hal/IMU_ICM20948.h:16` - Includes ISM330DHCX for Vector3f types (coupling)
- `src/hal/Mag_LIS3MDL.h:19` - Include guard incorrectly references ISM330DHCX
- `tests/smoke_tests/st_sensors_test.cpp` - Test for ISM330DHCX
- `tests/smoke_tests/imu_qwiic_test.c` - Test for ISM330DHCX
- `tests/smoke_tests/hal_validation.cpp` - Probes ISM330DHCX address
- `lib/ap_compat/RocketChip/hwdef.h:61` - Comments about ISM330DHCX
- `lib/ap_compat/AP_HAL_RP2350/SPIDevice.cpp:59,69` - ISM330DHCX SPI mode comments
- `CMakeLists.txt:133,165,868` - ISM330DHCX build targets

**Note**: `lib/st_drivers/` submodule contains ST platform drivers (ISM330DHCX, LIS3MDL) - decision needed on whether to keep.

---

<!-- Template for new flags:

### [DATE] - [AGENT NAME]
**File**: `path/to/file`  
**Lines**: (if applicable)  
**Observation**: [What you noticed]  
**Why it seems odd**: [Your reasoning]  
**Possible explanations**: [What might justify this]  
**Severity**: Low / Medium / High  
**Recommendation**: [Ask Nathan / Defer / Needs discussion]

---

-->

---

## Resolved

*No resolved flags yet.*

<!-- Move resolved flags here with resolution notes:

### [DATE] - [AGENT] (Resolved [DATE])
**File**: `path/to/file`  
**Observation**: [Original observation]  
**Resolution**: [Decision and reasoning]

---

-->
