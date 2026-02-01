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

```cpp
// In include/rocketchip/config.h

#ifdef DEBUG
  #define DBG_PRINT(fmt, ...) printf("[%lu] " fmt "\n", time_us_32(), ##__VA_ARGS__)
  #define DBG_TASK(name) DBG_PRINT("[%s] tick", name)
  #define DBG_STATE(from, to) DBG_PRINT("State: %s -> %s", from, to)
  #define DBG_ERROR(fmt, ...) printf("[%lu] ERROR: " fmt "\n", time_us_32(), ##__VA_ARGS__)
#else
  #define DBG_PRINT(fmt, ...) ((void)0)
  #define DBG_TASK(name) ((void)0)
  #define DBG_STATE(from, to) ((void)0)
  #define DBG_ERROR(fmt, ...) ((void)0)
#endif
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

## Per-Task Guidelines

| Task | Debug Policy | Rationale |
|------|--------------|-----------|
| SensorTask | **Minimal** — LED blink patterns only | 1kHz rate; printf affects timing |
| ControlTask | **Minimal** — error conditions only | 500Hz rate; timing-critical |
| FusionTask | State changes, errors | 200Hz allows occasional prints |
| MissionTask | State transitions, events | 100Hz; primary debugging target |
| LoggerTask | Write confirmations, errors | 50Hz; I/O bound anyway |
| TelemetryTask | TX confirmations, RSSI | 10Hz; low rate |
| UITask | Button events, display updates | 30Hz; non-critical |

---

## Build Configurations

| Environment | DEBUG defined | Output |
|-------------|---------------|--------|
| `env:dev` | Yes | Full debug output |
| `env:main` | No | Silent (macros compile to nothing) |
| `env:core` | No | Silent |
| `env:titan` | No | Silent |

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

*See also: `standards/CODING_STANDARDS.md` for general code conventions*
