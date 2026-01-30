# Standards Deviations Log

**Purpose**: Track deviations from project coding standards (JSF AV C++, DEBUG_OUTPUT.md, CODING_STANDARDS.md) with severity, remediation difficulty, and rationale.

**Last Updated**: 2026-01-21 (Revision 3 - Council-approved full resolution)

---

## Severity Levels

| Level | Description |
|-------|-------------|
| **Critical** | Must fix before flight code; safety or reliability impact |
| **High** | Should fix before production; may cause subtle issues |
| **Medium** | Fix when touching affected code; technical debt |
| **Low** | Acceptable for test/debug code; cosmetic or stylistic |
| **Accepted** | Intentional deviation with documented rationale |

## Difficulty Levels

| Level | Description |
|-------|-------------|
| **Trivial** | Simple search-and-replace or single-line change |
| **Easy** | Localized changes, no API impact |
| **Moderate** | Multiple files or requires testing |
| **Hard** | Architectural changes or external dependencies |

---

## Active Deviations

### 1. ArduPilot Library Deviations

**Location**: `lib/ardupilot/` (external code)

**Standard Violated**: Multiple JSF rules

**Severity**: Accepted
**Difficulty**: N/A
**Status**: Permanently Accepted

**Rationale**: ArduPilot is external, battle-tested code. Per `CODING_STANDARDS.md` "ArduPilot Library Integration" section, we accept ArduPilot's coding style within their libraries and use the shim layer (`lib/ap_compat/`) to bridge to RocketChip conventions.

**Policy**:
- No modifications to ArduPilot source files
- Shim layer follows RocketChip standards
- ArduPilot deviations are not tracked individually

---

### 2. Dynamic Memory Allocation in Test Code

**Location**: [calibration_test.cpp:155](tests/smoke_tests/calibration_test.cpp#L155)
```cpp
statusLed = new WS2812(NEOPIXEL_PIN, 1);
```

**Standard Violated**: JSF AV Rule 206 - "Allocation/deallocation from/to the free store shall not occur after initialization"

**Severity**: Low
**Difficulty**: Easy
**Status**: Accepted for test code

**Rationale**: This is test/smoke code, not flight code. Dynamic allocation at startup is acceptable for interactive test utilities. Production code in `src/` uses static allocation.

**Remediation**: If this code moves to production, change to:
```cpp
static WS2812 statusLedInstance(NEOPIXEL_PIN, 1);
WS2812* statusLed = &statusLedInstance;
```

---

### 3. Use of Primitive Types in Test Code

**Location**: [calibration_test.cpp](tests/smoke_tests/calibration_test.cpp) (multiple)
```cpp
uint8_t orientation = 0;       // Line 226
bool collectingSamples = false; // Line 229
float brightness = ...;         // Line 85
```

**Standard Violated**: JSF AV Rule 209 - "The basic types of char, int, short, long, float, double shall not be used"

**Severity**: Low
**Difficulty**: Trivial
**Status**: Accepted for test code

**Rationale**:
- `uint8_t` is a fixed-width type and compliant
- `bool` is explicitly allowed in C++
- `float` usage in test code is acceptable; ArduPilot library uses `float` internally

**Remediation**: For production code, define project-wide typedefs:
```cpp
typedef float  float32_t;
typedef double float64_t;
```

---

### 4. HAVE_PAYLOAD_SPACE Always Returns True

**Location**: [GCS.h:48](lib/ap_compat/GCS_MAVLink/GCS.h#L48)
```cpp
#define HAVE_PAYLOAD_SPACE(chan, id) (true)
```

**Standard Violated**: ArduPilot design pattern - should check actual TX buffer space

**Severity**: Medium
**Difficulty**: Moderate
**Status**: Temporary simplification - fix when implementing radio telemetry

**Rationale**: USB CDC uses blocking writes, so buffer space is effectively unlimited. For initial calibration MAVLink integration, this simplification is acceptable.

**Risk**: When radio telemetry (RFM69, etc.) is added with limited TX buffers:
- Messages could be dropped silently
- Buffer overruns possible under high message rates

**Remediation**: Implement proper buffer space checking when adding radio telemetry:
1. Track TX buffer space per channel
2. Implement `comm_get_txspace()` for each channel type
3. Replace macro with proper ArduPilot-style check:
```cpp
#define HAVE_PAYLOAD_SPACE(chan, id) (comm_get_txspace(chan) >= MAVLINK_MSG_ID_##id##_LEN + overhead)
```

---

## Resolved Deviations

### R1. USB CDC Wait Pattern (FIXED 2026-01-21)

**Location**: [calibration_test.cpp](tests/smoke_tests/calibration_test.cpp)

**Standard Violated**: DEBUG_OUTPUT.md USB CDC handling pattern

**Resolution**: Updated to match DEBUG_OUTPUT.md pattern:
- Visual feedback during USB wait (magenta blink)
- 500ms settle time after connection
- LED indicates "connect terminal now" state

---

### R2. Naming Convention Deviations (FIXED 2026-01-21)

**Location**: [calibration_test.cpp](tests/smoke_tests/calibration_test.cpp), `src/main.cpp`, `src/services/SensorTask.cpp`

**Standard Violated**: JSF AV Rule 50-53 (naming conventions)

**Resolution**: Updated all identifiers to follow naming conventions:
- Constants now use `k` prefix: `kNeoPixelPin`, `kLedPin`, `kUiTaskPriority`, `kI2cSda`, etc.
- Global variables use `g_` prefix: `g_statusLed`, `g_sensorData`, `g_sensorDataMutex`, etc.
- Module state clearly sectioned with comments

---

### R3. Magic Numbers (FIXED 2026-01-21)

**Location**: [calibration_test.cpp](tests/smoke_tests/calibration_test.cpp) (multiple)

**Standard Violated**: JSF AV Rule 151 - "Magic numbers shall not be used"

**Resolution**: Added configuration constants section at top of file:
```cpp
// Timing constants (in milliseconds)
static constexpr uint32_t kUsbWaitBlinkMs = 200;
static constexpr uint32_t kUsbSettleTimeMs = 500;
// ... etc.
```
All magic numbers throughout the code now reference these constants.

---

### R4. Raw printf Usage (FIXED 2026-01-21)

**Location**: `src/main.cpp`, `src/services/SensorTask.cpp`

**Standard Violated**: DEBUG_OUTPUT.md - diagnostic output should be compile-time guardable

**Resolution**: Introduced compile-time guarded debug macros in `include/debug.h`:
```cpp
#ifdef CONFIG_DEBUG
#define DBG_PRINT(fmt, ...) printf(fmt, ##__VA_ARGS__)
#define DBG_ERROR(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define DBG_PRINT(fmt, ...) do {} while(0)
#define DBG_ERROR(fmt, ...) do {} while(0)
#endif
```
All diagnostic printf in production src/ code replaced with DBG_PRINT/DBG_ERROR.
Fatal hooks (malloc/stack failure) retain minimal printf for post-mortem debugging plus LED blink codes.

---

### R5. Error Handling Strategy Undefined (FIXED 2026-01-21)

**Location**: [AP_InternalError.h](lib/ap_compat/AP_InternalError/AP_InternalError.h)

**Standard Violated**: CODING_STANDARDS.md "Error Handling" section (implicit)

**Previous Status**: Pending council decision

**Council Decision (2026-01-21)**: Approved Option A - Full ArduPilot AP_InternalError integration

**Resolution**: Implemented full AP_InternalError with:
- Error accumulation bitmask (`AP_InternalError::errors()`)
- Error count tracking (`AP_InternalError::count()`)
- Integration with DBG_ERROR debug macro
- Fatal paths (malloc/stack hooks, scheduler return) routed to `AP_InternalError::error()` with appropriate enum values
- Placeholder for MAVLink STATUS_TEXT integration when telemetry available

---

## Deviations by File

| File | Critical | High | Medium | Low | Accepted | Fixed |
|------|:--------:|:----:|:------:|:---:|:--------:|:-----:|
| `src/main.cpp` | 0 | 0 | 0 | 0 | 0 | 2 |
| `src/services/SensorTask.cpp` | 0 | 0 | 0 | 0 | 0 | 2 |
| `tests/smoke_tests/calibration_test.cpp` | 0 | 0 | 0 | 2 | 0 | 3 |
| `lib/ap_compat/*` | 0 | 0 | 0 | 0 | 0 | 1 |
| `lib/ardupilot/*` | 0 | 0 | 0 | 0 | 1 | 0 |

**Summary**: 8 deviations resolved, 3 remaining (2 accepted for test code, 1 permanently accepted for external library).

---

## Review Schedule

- **Before Flight Code**: Review all Low severity items in production paths
- **Quarterly**: Audit new code for standards compliance

---

## Notes

1. Test code (`tests/`) has relaxed standards enforcement to enable rapid validation
2. Production code (`src/`) must strictly follow all standards
3. External libraries are accepted as-is with shim layer bridging
4. New deviations should be logged here before merging to main
5. Debug output can be disabled for release builds via `-DCONFIG_DEBUG=OFF`
