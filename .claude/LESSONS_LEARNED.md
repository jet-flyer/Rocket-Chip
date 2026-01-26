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

## How to Use This Document

1. **Before debugging crashes:** Check if symptoms match any entry here
2. **After significant debugging:** Add new entry with lessons learned
3. **During context recovery:** Read this to restore debugging knowledge
4. **When writing new code:** Apply prevention measures from relevant entries
