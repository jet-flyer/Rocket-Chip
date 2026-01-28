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

## How to Use This Document

1. **Before debugging crashes:** Check if symptoms match any entry here
2. **After significant debugging:** Add new entry with lessons learned
3. **During context recovery:** Read this to restore debugging knowledge
4. **When writing new code:** Apply prevention measures from relevant entries
