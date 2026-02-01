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

### 2026-02-01 - Claude Code CLI (WIP: AP_Param Crash Fix)
**Task**: Fix AP_Param crash to enable 6-position accelerometer calibration persistence
**Plan File**: `C:\Users\pow-w\.claude\plans\swift-toasting-diffie.md`
**Status**: IN PROGRESS - Debugging double fault crash

**What's been tried:**
1. **Deferred semaphore initialization** - DONE, working
   - Changed from static `Semaphore` objects to placement-new in `Scheduler::init()`
   - Uses `s_timer_sem_storage` / `s_io_sem_storage` byte arrays with placement new
   - Files: `Scheduler.cpp:41-44, 91-98`

2. **Added null guards** to `register_timer_process()` and `register_io_process()` - DONE
   - Guards against calls before `init()` completes

3. **Added storage task** for AP_FlashStorage flush - ADDED, THEN DISABLED
   - Added `storage_task_entry()` per ESP32 HAL pattern
   - **CURRENTLY DISABLED** (lines 116-126 commented out) to isolate crash

4. **Removed duplicate timer_tick calls** - DONE
   - `HAL_RP2350_Class.cpp` had register_timer_process for storage flush
   - This conflicted with new storage_task
   - Removed the duplicate registration

**Current crash behavior:**
- Device crashes with "double fault" - GDB shows `prvIdleTask` → `prvCheckTasksWaitingTermination`
- Crash happens AFTER compass/baro init complete (user confirmed disabling those didn't help)
- PC shows 0x00000088 with corrupt msp (0xf0000000) - indicates hard fault/lockup
- OpenOCD reports "timed out while waiting for target halted" and "clearing lockup after double fault"

**Next steps to try:**
1. Flash with storage_task disabled and test if crash persists
2. If still crashes, investigate task stack corruption or memory issues
3. If crash resolves, investigate proper storage_task implementation

**Key files modified:**
- `lib/ap_compat/AP_HAL_RP2350/Scheduler.cpp` - semaphore deferred init, storage task (disabled)
- `lib/ap_compat/AP_HAL_RP2350/Scheduler.h` - added m_storage_task, storage_task_entry
- `lib/ap_compat/AP_HAL_RP2350/HAL_RP2350_Class.cpp` - removed duplicate timer_tick registration

**GDB commands for debugging:**
```bash
# Start OpenOCD
taskkill //F //IM openocd.exe 2>/dev/null; sleep 2; /c/Users/pow-w/.pico-sdk/openocd/0.12.0+dev/openocd -s /c/Users/pow-w/.pico-sdk/openocd/0.12.0+dev/scripts -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000" &

# Flash and run
cd /c/Users/pow-w/Documents/Rocket-Chip && arm-none-eabi-gdb build/rocketchip.elf -batch -ex "target extended-remote localhost:3333" -ex "monitor reset halt" -ex "load" -ex "monitor reset run"

# Get backtrace after crash
arm-none-eabi-gdb build/rocketchip.elf -batch -ex "target extended-remote localhost:3333" -ex "monitor halt" -ex "bt"
```

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
