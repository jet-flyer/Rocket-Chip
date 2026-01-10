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

## Future Enhancements

- **Dedicated debug ring buffer:** Capture debug messages to PSRAM, dump post-flight via USB. Avoids runtime printf overhead.
- **Severity levels:** Add DBG_WARN, DBG_INFO, DBG_VERBOSE with compile-time filtering.
- **Binary debug protocol:** Structured packets instead of strings for automated parsing.

---

*See also: `standards/CODING_STANDARDS.md` for general code conventions*
