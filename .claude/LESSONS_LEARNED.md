# Lessons Learned - Debugging Journal

**Purpose:** Document significant debugging efforts and their solutions so Claude can learn from past issues and avoid repeating mistakes.

**Format:** Each entry includes the problem, symptoms, root cause, solution, and time spent to emphasize the importance of the lesson.

---

## Entry 1: Stack Overflow from Large Local Variables

**Date:** 2026-01-26
**Time Spent:** ~2 hours of debugging
**Severity:** Critical - Silent crashes with no error message

### Problem
Test was crashing mysteriously. Output would appear up to a certain point, then nothing. Crash point seemed to move around between rebuilds.

### Symptoms
- Test prints some output (e.g., "Max fitness: 5.0") then stops
- No error message, no crash dump
- LED/NeoPixel may or may not continue
- Crash point appears random but is actually at function entry
- Clean rebuild sometimes "fixes" it temporarily (different stack layout)

### Root Cause
`CompassCalibrator calibrator;` declared as local variable in `main()`. The CompassCalibrator object is ~3KB+, exceeding available stack space.

**Key insight:** Compiler pre-allocates ALL stack space at function entry, not when variables are used. So even though the crash "appears" to happen after several printf statements, the stack overflow actually occurs immediately when main() is entered.

### Solution
Use static allocation for large objects:

```cpp
// At file scope (outside any function)
static CompassCalibrator g_calibrator;

int main() {
    // Use g_calibrator instead of local variable
    g_calibrator.start(...);
}
```

### How to Identify This Issue
1. Crash point seems to move around between builds
2. Adding/removing printf changes where crash appears
3. Object being created is large (>1KB)
4. Crash happens "between" two printf statements

### Prevention
- Any object >1KB should be static or heap-allocated
- Known large objects: CompassCalibrator, any object with large buffers
- When debugging mysterious crashes, check for large local variables

---

## Entry 2: Version Strings for Iterative Debugging

**Date:** 2026-01-26
**Context:** Debugging stack overflow issue above

### Problem
During iterative debugging, it was unclear if the latest code was actually running. Sometimes old binaries were being executed, wasting debugging time.

### Solution
Add a version string to debug output that changes with each significant code change:

```cpp
printf("Build: v3-static-calibrator\n");
```

Update the version string whenever making changes during debugging sessions. This immediately confirms whether the expected code is running.

### Prevention
- Always add version strings when doing iterative debugging
- Update version string on each significant change
- Check version string first before analyzing other output

---

## Entry 3: BASEPRI Register Blocking USB Interrupts

**Date:** 2026-01-26 (earlier in same session)
**Time Spent:** ~30 minutes

### Problem
USB CDC output not working after HAL initialization. Terminal shows nothing or partial output.

### Root Cause
FreeRTOS or HAL init leaves BASEPRI register elevated (typically to `configMAX_SYSCALL_INTERRUPT_PRIORITY = 16`), which blocks USB interrupts (priority 0x80).

### Solution
Clear BASEPRI after hal.init(), before stdio_init_all():

```cpp
hal.init();
__asm volatile ("mov r0, #0\nmsr basepri, r0" ::: "r0");
stdio_init_all();
```

### Prevention
- Always clear BASEPRI after HAL init in test code
- Template this pattern in all smoke tests

---

## Entry 4: HAL Init Order with USB

**Date:** 2026-01-26
**Related to:** RP2350_FULL_AP_PORT.md PD1

### Problem
USB CDC breaks after flash operations in storage.init().

### Root Cause
Flash operations make entire flash inaccessible. TinyUSB interrupt handlers are in flash. If USB is active during flash ops, handlers can't execute and USB breaks.

### Solution
Initialize HAL (which does flash ops) BEFORE enabling USB:

```cpp
hal.init();           // Flash ops happen here, USB not yet active
stdio_init_all();     // NOW enable USB
```

### Prevention
- Follow this init order in ALL tests using HAL + USB
- Document in test file comments

---

## Entry 5: Use Debug Probe Before Manual BOOTSEL

**Date:** 2026-01-26

### Problem
When debugging crashes, kept asking user to manually reset into BOOTSEL mode when the device became unresponsive.

### Better Approach
Debug probe can always flash the device, even when USB is completely broken:

```bash
arm-none-eabi-gdb firmware.elf -batch \
  -ex "target extended-remote localhost:3333" \
  -ex "monitor reset halt" \
  -ex "load" \
  -ex "monitor reset run"
```

### Prevention
- Always try debug probe first when USB is unresponsive
- Only ask for manual BOOTSEL as last resort
- Keep OpenOCD running during debugging sessions

---

## Entry 6: WS2812/NeoPixel Requires begin() Call

**Date:** 2026-01-26
**Time Spent:** ~1 session (issue not immediately identified)

### Problem
NeoPixel LED was not lighting up at all during compass calibration test, despite code appearing to set colors and call show().

### Symptoms
- LED completely dark, no response to any color commands
- No errors or warnings
- Code runs to completion without issues
- Other test code (without HAL) may show working NeoPixel

### Root Cause
The RocketChip WS2812 class follows standard pattern: construct → `begin()` → use. The `begin()` method:
1. Allocates the pixel buffer
2. Allocates a PIO state machine
3. Loads the PIO program
4. Sets `m_initialized = true`

Without `begin()`, both `setPixel()` and `show()` silently return early:
```cpp
void WS2812::setPixel(uint16_t index, const RGB& color) {
    if (!m_initialized || index >= m_num_leds) {
        return;  // Silent early return!
    }
    // ...
}
```

### Solution
Always call `begin()` after constructing WS2812:

```cpp
g_statusLed = new rh::WS2812(kNeoPixelPin, 1);
if (!g_statusLed->begin()) {
    // Handle init failure
}
g_statusLed->fill(rh::Colors::YELLOW);
g_statusLed->show();
```

### Prevention
- Always follow the construct → begin → use pattern for hardware classes
- Check return value of begin() for failure handling
- When LED doesn't respond, verify begin() was called before assuming hardware issue

---

## Entry 7: ArduPilot Expects Zeroed Memory Allocations

**Date:** 2026-01-28
**Time Spent:** ~2 hours of debugging
**Severity:** Critical - Hardfault on device restart

### Problem
Device would complete tests successfully on first boot, but hardfault on restart (pressing RESET button). Core 1 crashed in `isr_hardfault`.

### Symptoms
- First boot: test runs to completion
- After pressing RESET: device hangs immediately
- GDB shows hardfault on Core 1 (rp2350.cm1)
- Crash in `_free_r()` with garbage pointer (e.g., `0xd063dfd2`)

### Root Cause
ArduPilot's `Device` class has an uninitialized pointer `_checked.regs`:
```cpp
// In AP_HAL/Device.h
struct {
    uint8_t n_regs;
    struct checkreg *regs;  // NEVER initialized to nullptr!
} _checked;

~Device() {
    delete[] _checked.regs;  // Deletes garbage pointer!
}
```

ArduPilot assumes malloc/new returns zeroed memory (like ChibiOS does). Standard malloc/new on Pico SDK does NOT zero memory.

When the OwnPtr containing an I2CDevice goes out of scope, the Device destructor runs and calls `delete[]` on garbage, causing hardfault.

### Solution
Override C++ `operator new` to use `calloc` (which zeros memory):

```cpp
// In malloc_wrapper.cpp
void* operator new(std::size_t size) {
    if (size == 0) size = 1;
    void* ptr = calloc(1, size);  // calloc zeros memory
    if (ptr == nullptr) {
        ptr = malloc(size);  // Fallback (will panic if OOM)
    }
    return ptr;
}
```

Also requires CMake define to disable Pico SDK's default operator new/delete:
```cmake
add_compile_definitions(
    PICO_CXX_DISABLE_ALLOCATION_OVERRIDES=1
)
```

### How to Identify This Issue
1. First boot works, restart crashes
2. Hardfault on Core 1
3. Crash in `_free_r()` or `delete`
4. Stack trace shows destructor path
5. Using ArduPilot code with OwnPtr/unique_ptr

### Prevention
- Always use the calloc-based operator new when integrating ArduPilot
- This is documented in RP2350_FULL_AP_PORT.md as PD11
- The Pico SDK's `pico_malloc` wraps malloc for thread safety but does NOT zero memory
- The Pico SDK's `pico_cxx_options` provides operator new/delete that calls malloc (not calloc)

### Key Files
- `lib/ap_compat/AP_HAL_RP2350/malloc_wrapper.cpp` - Zero-initializing operator new
- CMakeLists.txt - `PICO_CXX_DISABLE_ALLOCATION_OVERRIDES=1`

---

## Entry 8: FreeRTOS SMP Multi-Core Memory Visibility

**Date:** 2026-01-28
**Time Spent:** ~3 hours of debugging
**Severity:** Critical - init() hangs indefinitely

### Problem
`AP_InertialSensor::init()` hangs forever. Monitor task shows callbacks are running, I2C transfers succeed, gyro is registered with correct rate (1125Hz), but `wait_for_sample()` never sees the `_new_gyro_data` flag.

### Symptoms
- init() blocks indefinitely in `wait_for_sample()` loop
- Monitor shows: gyro_count=1, accel_count=1, rate=1125Hz
- I2C transfer count increases (3000+/sec)
- Callback count increases (1000+/sec)
- But `_new_gyro_data` flag reads as false in main loop

### Root Cause
FreeRTOS SMP on dual-core RP2350 allows tasks to run on different cores. The DeviceBus callback thread (setting flags) may run on Core 1, while the main task (reading flags) runs on Core 0. ARM's weak memory model means writes on one core aren't immediately visible to the other core without memory barriers.

ArduPilot assumes single-core memory semantics throughout. This worked on ESP32 because they pin all ArduPilot tasks to the same core.

### Solution
Two-pronged approach:

1. **Core pinning** (primary fix):
```cpp
// Pin bus thread to Core 0 to match main task
vTaskCoreAffinitySet(bus_thread_handle, (1 << 0));
```

2. **Memory barriers** (defense in depth):
```cpp
void Scheduler::delay_microseconds_boost(uint16_t us) {
    __sync_synchronize();  // See latest values
    // ... delay ...
    __sync_synchronize();  // Publish our writes
}
```

### How to Identify This Issue
1. Polling loop never sees flag changes from another task
2. Both tasks clearly running (debug output confirms)
3. Using FreeRTOS SMP on multi-core MCU
4. Works fine in single-core mode

### Prevention
- Pin all related ArduPilot tasks to the same core
- Add memory barriers around any cross-task flag polling
- Documented as PD12 in `RP2350_FULL_AP_PORT.md`

---

## Entry 9: FreeRTOS Priority Inversion with Busy-Wait

**Date:** 2026-01-28
**Time Spent:** ~1 hour (discovered during PD12 debugging)
**Severity:** Critical - Lower priority tasks starve

### Problem
Even after fixing memory visibility (Entry 8), `wait_for_sample()` still hung. The DeviceBus callback thread was supposed to run and set flags, but it wasn't getting CPU time.

### Symptoms
- Main task spinning in `wait_for_sample()` at priority 2
- DeviceBus thread at priority 5 (higher)
- But DeviceBus callbacks not running frequently enough
- `taskYIELD()` calls not helping

### Root Cause
DeviceBus was using `hal.scheduler->delay_microseconds()` between callbacks, which busy-waits for short delays. The thread would:
1. Run callbacks (takes ~100us)
2. Busy-wait for 1000us until next callback
3. Repeat

During the busy-wait, `taskYIELD()` was called, but **`taskYIELD()` only yields to tasks of equal or higher priority**. Since DeviceBus is at priority 5 and main task at priority 2, the yield does nothing useful.

Result: DeviceBus hogs CPU during busy-wait, starving lower-priority tasks.

### Solution
Use `vTaskDelay()` instead of busy-wait:

```cpp
// CRITICAL: Must use vTaskDelay() to yield to lower-priority tasks
TickType_t ticks = pdMS_TO_TICKS((delay + 500) / 1000);
if (ticks == 0) {
    ticks = 1;  // Always yield at least 1 tick
}
vTaskDelay(ticks);  // Properly yields to ALL tasks
```

This accepts 1ms timing granularity but ensures the scheduler can run other tasks.

### How to Identify This Issue
1. Higher-priority task using `taskYIELD()` in a loop
2. Lower-priority tasks not getting CPU time
3. System appears "stuck" but high-priority task is running

### Prevention
- Never busy-wait in higher-priority tasks when lower-priority tasks need CPU
- `taskYIELD()` only yields to same-or-higher priority
- Use `vTaskDelay(1)` to yield to ALL tasks (minimum 1 tick)
- Documented as PD13 in `RP2350_FULL_AP_PORT.md`

---

## Entry 10: USB CDC Requires FreeRTOS Scheduler Running

**Date:** 2026-01-28
**Time Spent:** ~1 hour of iterative debugging
**Severity:** Critical - USB completely breaks (error 2)

### Problem
USB CDC serial would break with "error 2" when adding USB wait loops in `main()` before `vTaskStartScheduler()`.

### Symptoms
- Any `stdio_usb_connected()` check before scheduler starts breaks USB
- Indefinite `while (!stdio_usb_connected())` loop breaks USB
- Bounded wait loops (e.g., 6 seconds) also break USB
- USB enumeration fails completely ("error 2" on host)
- Works fine if no USB I/O done before scheduler

### Root Cause
TinyUSB on RP2350 with FreeRTOS SMP runs background tasks on Core 1. These tasks handle USB enumeration and I/O. Before `vTaskStartScheduler()` is called, these tasks aren't running.

Calling `stdio_usb_connected()` or doing any USB I/O in main() before the scheduler starts causes USB to malfunction because the necessary TinyUSB background processing isn't happening.

### Solution
Move ALL USB I/O to FreeRTOS tasks that run after scheduler starts:

```cpp
int main() {
    gpio_init(kLedPin);
    gpio_set_dir(kLedPin, GPIO_OUT);
    stdio_init_all();  // Initialize USB, but DON'T do any I/O yet!

    // Do non-USB init here (sensors, etc.)
    SensorTask_Init();
    SensorTask_Create();

    // Create UITask that will handle USB I/O
    xTaskCreate(UITask, "UI", ...);

    vTaskStartScheduler();  // NOW USB works
}

static void UITask(void* params) {
    // Safe to do USB I/O here - scheduler is running
    while (true) {
        printf("\rPress any key to start...");  // Works!
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) break;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

### How to Identify This Issue
1. USB "error 2" on host when connecting
2. USB worked before adding wait loop
3. Wait loop or USB checks are in main() before scheduler

### Prevention
- **Rule: Never do USB I/O before `vTaskStartScheduler()`**
- Move all "wait for terminal" patterns to FreeRTOS tasks
- Use LED blink patterns for pre-scheduler status (not serial)
- This applies to FreeRTOS SMP on RP2350 with TinyUSB

---

## Entry 11: Prioritize Debug Probe Over LED Debugging

**Date:** 2026-01-29
**Context:** Multiple debugging sessions where USB was broken
**Severity:** Process Improvement - Saves hours of debugging time

### Problem
When encountering issues that break output (USB enumeration failures, hardfaults, early crashes), default instinct was to add LED blink patterns for diagnostics. This provided minimal information and required many flash/test cycles.

### Better Approach
**Always use the debug probe first.** It provides:
- Register and memory inspection via GDB
- Stack traces showing exact crash location
- Ability to flash even when USB is completely broken
- Variable inspection without modifying code

### Occasional Connection Quirks (Expected)
The probe connection may have transient issues:
- OpenOCD timeout on first attempt
- "Remote communication error" from GDB
- Need to run `monitor reset halt` multiple times

**These are normal** - retry and it works. The probe has proven reliable in extensive debugging sessions. Don't give up after one failed attempt.

### When LED Debugging Makes Sense
Only resort to LED-based debugging when:
- Debug probe is not physically connected
- Observing runtime behavior without breakpoints
- Testing production firmware without probe

### Key Takeaway
The few seconds spent reconnecting a flaky probe connection saves hours compared to iterating with LED blink codes. The probe gives you answers; LEDs give you hints.

---

## Entry 12: USB CDC Init Order Critical for Enumeration

**Date:** 2026-01-29
**Time Spent:** ~1 hour
**Severity:** Critical - USB completely broken (error 2)

### Problem
USB CDC (COM6) failed to enumerate. Device showed "error 2" in Device Manager. Serial output completely broken.

### Symptoms
- USB device doesn't appear in Device Manager
- "error 2" when device tries to enumerate
- LED blink patterns visible (code is running)
- Manual BOOTSEL required to flash

### Root Cause
Init order in `src/main.cpp` was wrong:

**BROKEN order:**
```cpp
stdio_init_all();           // USB starts first
SensorTask_Init();          // HAL/flash ops break USB!
```

Flash operations in `SensorTask_Init()` → `hal_init()` → `Storage.init()` make entire flash inaccessible. TinyUSB interrupt handlers are in flash. When USB is active during flash ops, handlers can't execute and USB breaks permanently.

### Solution
Reorder init in `src/main.cpp`:

**WORKING order:**
```cpp
// 1. HAL init (flash ops) BEFORE USB
SensorTask_Init();          // Flash ops complete here

// 2. Clear BASEPRI (may be elevated, blocking USB IRQs)
__asm volatile ("mov r0, #0\nmsr basepri, r0" ::: "r0");

// 3. NOW safe to start USB
stdio_init_all();
```

### Key File
`src/main.cpp` - init order at boot (lines 250-294 after fix)

### Prevention
- **Rule:** HAL init (flash ops) MUST happen before `stdio_init_all()`
- Always clear BASEPRI after HAL init, before USB
- This is documented in RP2350_FULL_AP_PORT.md PD1 (Flash/USB interaction)

### Related
- Entry 3: BASEPRI blocking USB
- Entry 4: HAL init order
- RP2350_FULL_AP_PORT.md PD1

---

## Entry 13: Adafruit ICM-20948 Default I2C Address is 0x69, not 0x68

**Date:** 2026-01-29
**Time Spent:** ~20 minutes
**Severity:** High - IMU not detected, silent failure

### Problem
IMU (ICM-20948) not being detected by ArduPilot backends. Barometer worked fine (proving I2C bus functional), but zero I2C traffic to the IMU address.

### Symptoms
- Barometer samples: 2500+ (working at 0x77)
- IMU samples: 0 (no traffic to 0x68)
- `AP_INS: accel=0, gyro=0, rate=0Hz`
- I2C debug output shows only 0x77 transfers, never 0x68

### Root Cause
`hwdef.h` was configured with **0x68** as the ICM-20948 address, but **Adafruit boards default to 0x69**.

The ICM-20948 has an AD0 pin that controls I2C address:
- AD0 = LOW (grounded) → 0x68
- AD0 = HIGH (or floating) → 0x69

**Adafruit boards pull AD0 HIGH by default**, giving address 0x69. You must bridge the SDO/ADR solder jumper on the back to change to 0x68.

Comments in `hwdef.h` incorrectly stated "AD0 pulled LOW on Adafruit board" - this was wrong.

### Solution
Update all hwdef.h files:
```cpp
// WRONG:
constexpr uint8_t ICM20948 = 0x68;  // AD0=LOW
#define HAL_INS_PROBE_LIST PROBE_IMU_I2C(Invensensev2, 0, 0x68, ROTATION_NONE)

// CORRECT:
constexpr uint8_t ICM20948 = 0x69;  // AD0=HIGH (Adafruit default)
#define HAL_INS_PROBE_LIST PROBE_IMU_I2C(Invensensev2, 0, 0x69, ROTATION_NONE)
```

### Files Fixed
- `lib/ap_compat/AP_HAL_RP2350/hwdef.h`
- `lib/ap_compat/RocketChip/hwdef.h`
- `src/hal/IMU_ICM20948.h`
- `docs/HARDWARE.md`

### Prevention
1. **Always check vendor documentation for default I2C addresses**
2. Run I2C scan test to confirm actual device addresses before configuring probes
3. For Adafruit boards: check product page or schematic for default pin states
4. Add comments noting whether addresses are vendor defaults vs modified

### Reference
Adafruit ICM-20948 product page confirms 0x69 as default:
https://learn.adafruit.com/adafruit-tdk-invensense-icm-20948-9-dof-imu/pinouts

---

## Entry 14: Don't Delete "Duplicate" Files Without Checking for Platform Fixes

**Date:** 2026-01-29
**Time Spent:** ~2 hours rediscovering the issue
**Severity:** Critical - IMU completely broken

### Problem
IMU stopped working with "INS: unable to initialise driver" after a "cleanup" commit removed `lib/ap_compat/AP_InertialSensor/` files, thinking they were stale duplicates of ArduPilot code.

### Symptoms
- `AP_InertialSensor::init()` hangs indefinitely or fails
- Baro works fine (same I2C bus, same hardware)
- I2C scan shows device responding at correct address
- Callbacks running, but `wait_for_sample()` never sees flag changes

### Root Cause
The files in `lib/ap_compat/AP_InertialSensor/` were NOT duplicates. They contained the critical **PD12 fix**: `std::atomic<bool>` for cross-core memory visibility on dual-core RP2350.

Commit `f8b85cb` deleted them with message "Removed stale AP_InertialSensor copy" during a cleanup/sync operation. Without the `std::atomic` fix, writes on one core aren't visible to reads on the other core.

### Solution
Restore the files from the commit that added them:
```bash
git checkout dd454e6 -- lib/ap_compat/AP_InertialSensor/
```

And ensure CMakeLists.txt uses `lib/ap_compat/AP_InertialSensor/` instead of `lib/ardupilot/libraries/AP_InertialSensor/` for the core .cpp files.

### Prevention
1. **Never assume "duplicate" files are identical** - check for platform-specific modifications
2. **Files in `lib/ap_compat/` exist for a reason** - they override ArduPilot with platform fixes
3. **Before deleting any ap_compat file**, search for comments containing "fix", "PD", "RP2350", "FreeRTOS", "atomic"
4. **Add README.md files** to directories with critical fixes (done for AP_InertialSensor)
5. **When asking an agent to "clean up" repos**, explicitly exclude `lib/ap_compat/` from deletion

### Key Files
- `lib/ap_compat/AP_InertialSensor/AP_InertialSensor.h` - std::atomic flags
- `lib/ap_compat/AP_InertialSensor/AP_InertialSensor_Backend.cpp` - memory_order writes
- `lib/ap_compat/AP_InertialSensor/README.md` - documents why files exist

---

## Entry 15: USB Terminal Connection Affecting Program State

**Date:** 2026-01-30
**Status:** RESOLVED (v0.3.1)
**Severity:** High - CLI unusable
**Time Spent:** ~4 hours (architecture redesign + debugging)

### Problem
CLITask stops executing when USB terminal connects. Program behavior should NOT depend on terminal connection - only output buffering should be affected.

### Symptoms
- LED flashes rapidly (50ms) while waiting for sensor init - CORRECT
- When terminal connects to COM6, LED goes SOLID - WRONG
- Pressing Enter does nothing
- Program appears frozen but scheduler is still running (verified via probe)

### Root Cause Analysis
The old CLITask architecture was **too tightly coupled** to sensor code:
1. CLI ran regardless of terminal connection state (wasting cycles)
2. CLI called `SensorTask_*()` functions directly (coupling CLI to sensor implementation)
3. USB I/O operations (printf/getchar) were attempted even when terminal wasn't properly connected

The user's key insight: "the CLI menu needs to be completely divorced from any internally running code by its very nature."

### Solution: RC_OS Decoupled Architecture (v0.3)

**Principle:** CLI should be a "local GCS" that translates keystrokes → MAVLink commands internally.

1. **CLITask only runs when terminal connected** (`stdio_usb_connected()`):
   - When disconnected: slow LED blink (1Hz), NO USB I/O whatsoever
   - When connected: show banner, process CLI commands

2. **All calibration commands route through MAVLink** via `RC_OS::cmd_*()` functions:
   ```cpp
   // CLI command 'l' → MAVLink internally
   MAV_RESULT result = RC_OS::cmd_level_cal();
   // This calls GCS::send_local_command(MAV_CMD_PREFLIGHT_CALIBRATION, 0,0,0,0,1,0,0)
   // Which routes through same handler as Mission Planner would use
   ```

3. **Benefits:**
   - CLI and GCS use identical code paths for all operations
   - CLI completely decoupled from SensorTask internals
   - Terminal connection can't affect flight code
   - Easy to test: CLI commands = MAVLink commands

### Files Modified
- `src/main.cpp` - Rewrote CLITask with terminal-connected check
- `src/cli/RC_OS.h` - New file with CLI→MAVLink command mappings
- `lib/ap_compat/GCS_MAVLink/GCS.h` - Added `send_local_command()` method
- `lib/ap_compat/GCS_MAVLink/GCS.cpp` - Implemented `send_local_command()`

### Key Code Pattern
```cpp
static void CLITask(void* pvParameters) {
    bool wasConnected = false;
    while (true) {
        bool isConnected = stdio_usb_connected();

        if (!isConnected) {
            // DISCONNECTED: Just blink LED, NO USB I/O
            gpio_put(kLedPin, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_put(kLedPin, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
            wasConnected = false;
            continue;  // Skip all USB I/O
        }

        // CONNECTED: Normal CLI operation
        if (!wasConnected) {
            wasConnected = true;
            printf("RocketChip OS v0.3\n");  // Safe - terminal IS connected
        }
        // ... process CLI commands via RC_OS::cmd_*() ...
    }
}
```

### Additional Fix: USB Input Buffer Drain

Initial implementation still showed phantom menu transitions - calibration menu appeared without user pressing 'c'. Root cause: **USB CDC buffer had garbage bytes** when terminal connected.

**Fix:** Drain input buffer immediately after terminal connects, before processing commands:
```cpp
// After printing banner and sensor status...
// Drain any garbage from USB input buffer before accepting commands
while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {
    // Discard all pending input
}
g_menuMode = MenuMode::Main;
```

### Prevention
1. **Never do USB I/O unless `stdio_usb_connected()` returns true**
2. **CLI should speak MAVLink internally**, not call sensor functions directly
3. **Terminal connection should only affect output buffering**, not program flow
4. **Drain USB input buffer after terminal connects** - garbage bytes from connection handshake can trigger unwanted commands

### Verification (PASSED 2026-01-30)
- [x] LED blinks slowly (1Hz) when terminal not connected
- [x] Banner appears on terminal connection with sensor status
- [x] CLI commands work ('h', 's', 'c')
- [x] Calibration commands route through MAVLink (verified: `[GCS] Accel calibration requested`)
- [x] Level cal completes quickly (~2 seconds)
- [x] No phantom menu transitions

---

## How to Use This Document

1. **Before debugging crashes:** Check if symptoms match any entry here
2. **After significant debugging:** Add new entry with lessons learned
3. **During context recovery:** Read this to restore debugging knowledge
4. **When writing new code:** Apply prevention measures from relevant entries
