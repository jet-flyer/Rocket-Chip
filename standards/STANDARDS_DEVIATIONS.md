# Standards Deviations Log

**Purpose**: Track deviations from project coding standards (JSF AV C++, DEBUG_OUTPUT.md, CODING_STANDARDS.md) with severity, remediation difficulty, and rationale.

**Last Updated**: 2026-01-21 (Revision 2 - Post-remediation)

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

### 1. Dynamic Memory Allocation in Test Code

**Location**: [calibration_test.cpp:155](tests/smoke_tests/calibration_test.cpp#L155)
```cpp
statusLed = new WS2812(NEOPIXEL_PIN, 1);
```

**Standard Violated**: JSF AV Rule 206 - "Allocation/deallocation from/to the free store shall not occur after initialization"

**Severity**: Low
**Difficulty**: Easy
**Status**: Deferred

**Rationale**: This is test/smoke code, not flight code. Dynamic allocation at startup is acceptable for interactive test utilities. Production code in `src/` uses static allocation.

**Remediation**: If this code moves to production, change to:
```cpp
static WS2812 statusLedInstance(NEOPIXEL_PIN, 1);
WS2812* statusLed = &statusLedInstance;
```

---

### 2. Use of Primitive Types Instead of Fixed-Width Types

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

### 3. Naming Convention Deviations

**Location**: [calibration_test.cpp](tests/smoke_tests/calibration_test.cpp) (multiple)

**Standard Violated**: JSF AV Rule 50-53 (naming conventions)

**Severity**: Low
**Difficulty**: Trivial
**Status**: FIXED (2026-01-21)

**Resolution**: Updated all identifiers to follow naming conventions:
- Constants now use `k` prefix: `kNeoPixelPin`, `kOrientationNames`, `kI2cSpeedHz`, etc.
- Global variables now use `g_` prefix: `g_statusLed`, `g_lastAnimUpdate`, `g_pulsePhase`, etc.
- Module state clearly sectioned with comments

---

### 4. Magic Numbers

**Location**: [calibration_test.cpp](tests/smoke_tests/calibration_test.cpp) (multiple)

**Standard Violated**: JSF AV Rule 151 - "Magic numbers shall not be used"

**Severity**: Low
**Difficulty**: Easy
**Status**: FIXED (2026-01-21)

**Resolution**: Added configuration constants section at top of file:
```cpp
// Timing constants (in milliseconds)
static constexpr uint32_t kUsbWaitBlinkMs = 200;
static constexpr uint32_t kUsbSettleTimeMs = 500;
static constexpr uint32_t kBlinkSlowPeriodMs = 500;
static constexpr uint32_t kBlinkFastPeriodMs = 100;
static constexpr uint32_t kAnimUpdatePeriodMs = 20;
static constexpr uint32_t kProgressIndicatorMs = 200;

// I2C configuration
static constexpr uint32_t kI2cSpeedHz = 400000;

// LED brightness
static constexpr uint8_t kLedBrightnessPercent = 50;

// Calibration parameters
static constexpr uint8_t kNumOrientations = 6;
static constexpr float kSampleDurationSec = 2.0f;
```
All magic numbers throughout the code now reference these constants.

---

### 5. ArduPilot Library Deviations

**Location**: `lib/ardupilot/` (external code)

**Standard Violated**: Multiple JSF rules

**Severity**: Accepted
**Difficulty**: N/A
**Status**: Accepted

**Rationale**: ArduPilot is external, battle-tested code. Per `CODING_STANDARDS.md` "ArduPilot Library Integration" section, we accept ArduPilot's coding style within their libraries and use the shim layer (`lib/ap_compat/`) to bridge to RocketChip conventions.

**Policy**:
- No modifications to ArduPilot source files
- Shim layer follows RocketChip standards
- ArduPilot deviations are not tracked individually

---

### 6. Error Handling Strategy Undefined

**Location**: [AP_InternalError.h](lib/ap_compat/AP_InternalError/AP_InternalError.h)

**Standard Violated**: CODING_STANDARDS.md "Error Handling" section (implicit)

**Severity**: Medium
**Difficulty**: Moderate
**Status**: Pending Decision

**Rationale**: Current stub just prints errors. Before EKF3/fusion integration, must decide:
- **Option A**: Full ArduPilot error accumulation/reporting
- **Option B**: RocketChip-specific error system aligned with state machine

**Remediation**: Council decision required. Documented in code as TODO.

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

## Deviations by File

| File | Critical | High | Medium | Low | Accepted | Fixed |
|------|:--------:|:----:|:------:|:---:|:--------:|:-----:|
| `tests/smoke_tests/calibration_test.cpp` | 0 | 0 | 0 | 2 | 0 | 2 |
| `lib/ap_compat/*` | 0 | 0 | 1 | 0 | 0 | 0 |
| `lib/ardupilot/*` | 0 | 0 | 0 | 0 | 1 | 0 |

**Summary**: 2 deviations fixed, 2 remaining (1 accepted for test code, 1 pending decision), 1 accepted (external library).

---

## Review Schedule

- **Before Phase 3 (EKF3)**: Resolve error handling strategy (Medium #6)
- **Before Flight Code**: Review all Low severity items in production paths
- **Quarterly**: Audit new code for standards compliance

---

## Notes

1. Test code (`tests/`) has relaxed standards enforcement to enable rapid validation
2. Production code (`src/`) must strictly follow all standards
3. External libraries are accepted as-is with shim layer bridging
4. New deviations should be logged here before merging to main
