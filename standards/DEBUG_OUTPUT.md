# Debug Output Standards

**Status:** Draft
**Applies to:** All firmware development
**Referenced by:** `docs/SAD.md` Section 11 (Tools and Validation)

---

## Overview

Serial debug output is essential during development for diagnosing timing issues, state transitions, and sensor behavior. This document defines conventions to ensure debug code doesn't impact flight performance and compiles out cleanly for release builds.

---

## Implementation

### Debug Macros

Debug output uses direct `printf()` calls wrapped in macros that compile out for release builds.

```cpp
// In include/rocketchip/config.h

#ifdef DEBUG
    #include "pico/time.h"
    #include <stdio.h>
    #define DBG_PRINT(fmt, ...) printf("[%lu] " fmt "\n", (unsigned long)time_us_32(), ##__VA_ARGS__)
    #define DBG_STATE(from, to) DBG_PRINT("State: %s -> %s", from, to)
    #define DBG_ERROR(fmt, ...) printf("[%lu] ERROR: " fmt "\n", (unsigned long)time_us_32(), ##__VA_ARGS__)
#else
    #define DBG_PRINT(fmt, ...) ((void)0)
    #define DBG_STATE(from, to) ((void)0)
    #define DBG_ERROR(fmt, ...) ((void)0)
#endif
```

**Key file:** `include/rocketchip/config.h` - Macro definitions

### Build Configuration for DEBUG

**CRITICAL:** The Pico SDK defaults to Release builds. To enable DBG_* macros:

```cmake
# In CMakeLists.txt - Force Debug build for development
set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)

# Add DEBUG define for debug builds
if(CMAKE_BUILD_TYPE MATCHES Debug)
    add_compile_definitions(DEBUG=1)
endif()
```

After changing CMakeLists.txt, do a **clean rebuild**:
```bash
rm -rf build && mkdir build && cd build && cmake .. -G Ninja && ninja
```

### Serial Configuration

- **Interface:** USB-CDC
- **Baud rate:** 115200 (ignored for USB-CDC but set for compatibility)
- **Format:** `[timestamp_us] message`

### USB CDC Connection Handling

USB CDC serial disconnects when the device resets, causing most terminal programs to freeze or lose data. To ensure reliable output:

**Pattern:** Run program logic immediately, wait for connection before printing results.

```cpp
// Initialize stdio
stdio_init_all();

// Run tests/logic immediately (don't block on serial)
runTests();
showStatusLED(passed);  // Visual feedback works without serial

// Wait for USB CDC connection before printing
while (!stdio_usb_connected()) {
    gpio_put(LED_PIN, 1);
    sleep_ms(100);
    gpio_put(LED_PIN, 0);
    sleep_ms(100);
}
sleep_ms(500);  // Brief settle time

// Now print results
printf("Test results: ...\n");
```

**Key principles:**
- Never block program execution waiting for serial connection
- Visual indicators (LEDs, NeoPixel) provide immediate feedback without serial
- Startup/diagnostic output waits for connection so terminals receive complete data
- LED blink while waiting indicates "connect terminal now"
- Boot button can trigger result reprint for late-connecting terminals

---

## Per-Module Debug Guidelines

| Module | Debug Policy | Rationale |
|--------|--------------|-----------|
| Sensor sampling | **Minimal** — LED blink patterns only | 1kHz rate; printf affects timing |
| Control loop | **Minimal** — error conditions only | 500Hz rate; timing-critical |
| Sensor fusion | State changes, errors | 200Hz allows occasional prints |
| Mission engine | State transitions, events | 100Hz; primary debugging target |
| Data logger | Write confirmations, errors | 50Hz; I/O bound anyway |
| Telemetry | TX confirmations, RSSI | 10Hz; low rate |
| UI/CLI | Button events, display updates | 30Hz; non-critical |

---

## Build Configurations

| CMake Build Type | DEBUG defined | Output |
|------------------|---------------|--------|
| `Debug` | Yes | Full debug output |
| `Release` | No | Silent (macros compile to nothing) |
| `RelWithDebInfo` | No | Silent (debug symbols only, no serial output) |

---

## Flashing and Debugging Tools

### picotool (Preferred for Flashing)

Use `picotool` for routine build-flash-test cycles:

```bash
picotool load firmware.uf2 --force
```

The `--force` flag sends a USB vendor command to reboot the device into BOOTSEL mode automatically, flashes the firmware, then reboots back to application mode. No debug probe required.

### Debug Probe (For Debugging Only)

The Raspberry Pi Debug Probe is only needed for:
- GDB debugging (breakpoints, stepping, variable inspection)
- Flashing when USB is non-functional (e.g., firmware bug blocking USB enumeration)
- Low-level register inspection via OpenOCD

For normal development, prefer `picotool` - it's faster and doesn't require additional hardware connections.

---

## Future Enhancements

- **Dedicated debug ring buffer:** Capture debug messages to PSRAM, dump post-flight via USB. Avoids runtime printf overhead.
- **Severity levels:** Add DBG_WARN, DBG_INFO, DBG_VERBOSE with compile-time filtering.
- **Binary debug protocol:** Structured packets instead of strings for automated parsing.

---

## CLI Testing via Python Serial

**Use Python serial for reliable CLI testing.** PuTTY and VSCode Serial Monitor have quirks with USB CDC that cause truncated output and missed input. Python's pyserial provides consistent, scriptable testing.

### Quick CLI Test
```bash
# Run all non-destructive tests
python scripts/cli_test.py all

# Test specific functionality
python scripts/cli_test.py status    # Sensor status
python scripts/cli_test.py help      # Help menu
python scripts/cli_test.py menu      # Calibration menu
python scripts/cli_test.py level_cal # Level calibration (device must be flat)
```

### Interactive 6-Position Calibration
```bash
python scripts/accel_cal_6pos.py
```
This script guides you through each position, you move the device when prompted.

### Manual Serial Connection
```bash
python -m serial.tools.miniterm COM6 115200
# Ctrl+] to exit
```

### Programmatic Testing
```python
import serial
import time

port = serial.Serial('COM6', 115200, timeout=1)
time.sleep(0.5)
port.read(1000)  # Drain buffer

# Send command and read response
port.write(b's')
time.sleep(0.5)
response = port.read(4000).decode('utf-8', errors='replace')
print(response)

port.close()
```

### Terminal Compatibility Notes
| Terminal | Input | Output | Notes |
|----------|-------|--------|-------|
| Python miniterm | OK | OK | **Recommended** |
| Python serial | OK | OK | Best for scripted tests |
| PuTTY/plink | OK | Truncated banner | ~63 char cutoff on connect |
| VSCode Serial Monitor | OK | OK | Must use "Terminal" mode, not "Text" mode |

The banner truncation in PuTTY is a display quirk, not a firmware bug - CLI commands work fine.

---

## Test Script Design Guidelines

### Pause Output for Manual Input

When test scripts require user action (power cycle, cable disconnect, key press), **pause serial output streaming** before prompting. Use `input()` to block the script so the prompt stays visible on screen. If the script continues printing serial data while waiting for input, the prompt scrolls away and the user misses it.

```python
# GOOD: Script pauses, prompt stays visible
log_and_print(logf, "[script] >>> POWER CYCLE the device, then press ENTER <<<\n")
input("[script] Press ENTER after power cycle...")

# BAD: Prompt scrolls away while serial data streams
print("Power cycle the device")
while True:
    chunk = port.read(4096)  # keeps printing, prompt is gone
```

### Build Iteration Tags for Debug Sessions

During extended debugging sessions with multiple build-flash-test cycles, add a **monotonic build iteration tag** to the firmware banner. `__DATE__ __TIME__` alone is insufficient — timestamps blur together and the same binary flashed twice looks identical.

```cpp
// In firmware — increment on EVERY rebuild during debug iteration
static const char *kBuildTag = "IVP30-fix-3";
printf("Build: %s (%s %s)\n", kBuildTag, __DATE__, __TIME__);
```

**Always verify the build tag in serial output before starting a test cycle.** A 5-minute soak on the wrong binary is 5 minutes wasted. *(LL Entry 2)*

---

*See also: `standards/CODING_STANDARDS.md` for general code conventions*
