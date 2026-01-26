# RocketChip Standards and Protocols

## Coding Standards

**Coding standards are mandatory.** All code must adhere to these standards. Deviations require explicit approval and must be documented in the changelog with rationale. Approved exceptions are tracked in the Exceptions Table below.

### Foundation

This project follows the **JSF AV C++ Coding Standards** (Lockheed Martin, 2005) as the primary reference. JSF is the foundation for NASA/JPL's institutional C++ standard.

**References:**
- [JSF AV C++ Standards (PDF)](http://www.stroustrup.com/JSF-AV-rules.pdf) - Primary reference
- [JPL C Coding Standard (PDF)](https://yurichev.com/mirrors/C/JPL_Coding_Standard_C.pdf) - Additional reference for C-specific guidance
- Power of 10 Rules (Holzmann, JPL) - Distilled safety-critical rules

### JPL Additions (Pro Tier Goals)

These additional rules from JPL's standard are stretch goals for Pro tier:
- Mandatory `static_assert` for compile-time validation
- No uninitialized local variables
- Stricter `const`/`constexpr` usage
- Triple-voting on safety-critical variables

### Prior Art Research

**Before implementing any hardware interface, driver, or novel functionality:**

1. **Check ArduPilot first** - Search existing AP_HAL implementations for similar functionality
   - How do ChibiOS/Linux/SITL HALs handle this?
   - Are there relevant PRs or issues discussing the approach?
   - Example: ADC voltage scaling (see [PR #18754](https://github.com/ArduPilot/ardupilot/pull/18754))

2. **Check Adafruit/SparkFun libraries** - These are our preferred hardware vendors
   - CircuitPython implementations often have clear, well-documented approaches
   - Arduino libraries show common patterns for similar MCUs
   - Check for errata workarounds they've already implemented

3. **Check Pico SDK examples** - For RP2350-specific implementations
   - SDK examples show idiomatic usage patterns
   - Hardware-specific quirks are often documented in examples

4. **Document findings** - When prior art influences implementation:
   - Reference source in code comments (e.g., "// Per ArduPilot PR #18754...")
   - Note any deviations from prior art and rationale

**Rationale:** Proven solutions from established projects save time and reduce bugs. When millions of devices use a particular approach, that's strong evidence it works.

### Exceptions Table

Approved deviations from coding standards. Each exception requires documented rationale.

| ID | Rule | Exception | Rationale | Approved By | Date |
|----|------|-----------|-----------|-------------|------|
| *No exceptions yet* | | | | | |

*When exceptions are approved, add them here with full context. If this table grows large, it may be broken out into a separate EXCEPTIONS.md file.*

---

## Communication Protocols

### Telemetry

| Protocol | Use Case | Notes |
|----------|----------|-------|
| **MAVLink v2** | Primary telemetry | Compatible with QGroundControl, Mission Planner |
| **RFM69HCW** | Physical layer for long-range | 915MHz ISM band (US), current testing board |
| **WiFi** | Short-range / ground testing | ESP32 variants only |
| **Bluetooth/BLE** | Config and data download | ESP32 variants, close-range |
| **NRF24L01** | Budget short-range option | 2.4GHz, mentioned as alternative |

### Sensor Buses

| Bus | Speed | Use Case | Notes |
|-----|-------|----------|-------|
| **I2C** | 400kHz (Fast Mode) | Most sensors | STEMMA QT connectors, easy daisy-chain |
| **SPI** | Up to 10MHz+ | High-speed sensors, SD card | Lower latency than I2C |
| **UART** | 9600-115200 baud | GPS, debug serial | GPS typically 9600 default |

### GPS Protocols

| Protocol | Notes |
|----------|-------|
| **NMEA** | Standard ASCII sentences, human-readable |
| **UBX** | u-blox binary protocol, higher precision/rate |

---

## Data Formats

### Logging

| Format | Use Case | Notes |
|--------|----------|-------|
| **CSV** | Human-readable logs | Easy post-flight analysis, larger file size |
| **Binary** | High-rate logging | Compact, requires parser, preserves precision |
| **MAVLink** | Structured telemetry logs | Compatible with existing analysis tools |

### Configuration

| Format | Use Case | Notes |
|--------|----------|-------|
| **YAML** | Changelogs, mission configs | Human-readable, good for structured data |
| **Header files** | Compile-time config | `config.h` style for build-time settings |

---

## Hardware Interface Standards

| Standard | Description |
|----------|-------------|
| **Feather** | Adafruit board form factor (50.8mm x 22.8mm) |
| **FeatherWing** | Expansion board standard for Feather |
| **STEMMA QT / Qwiic** | 4-pin JST SH connector for I2C (3.3V) |
| **SWD** | Serial Wire Debug interface for programming/debugging |

### Platform-Specific Coding Rules

| Rule | Correct | Incorrect | Rationale |
|------|---------|-----------|-----------|
| LED Pin | `PICO_DEFAULT_LED_PIN` | `7` or other hardcoded value | SDK defines correct pin per board variant |
| Delay in main | `sleep_ms()` | `busy_wait_ms()` | `sleep_ms` is SDK-safe before RTOS |
| FreeRTOS delay | `vTaskDelay()` | `sleep_ms()` | Must use RTOS primitives after scheduler starts |
| Large objects | `static ClassName g_obj;` | `ClassName obj;` in function | Stack space is limited; large objects cause overflow |
| HAL init order | `hal.init()` before `stdio_init_all()` | `stdio_init_all()` first | Flash ops in HAL conflict with USB if USB is already running |
| BASEPRI after HAL | Clear BASEPRI after `hal.init()` | Leave BASEPRI elevated | FreeRTOS/HAL may block USB interrupts via BASEPRI |

### Memory Allocation Rules

**Large Object Allocation:** Objects larger than ~1KB should use static allocation, not stack allocation:

```cpp
// CORRECT: Static allocation at file scope
static CompassCalibrator g_calibrator;

int main() {
    g_calibrator.start(...);  // Use the static instance
}

// INCORRECT: Stack allocation in function
int main() {
    CompassCalibrator calibrator;  // May cause stack overflow!
    calibrator.start(...);
}
```

**Why:** RP2350 default stack size is limited. The compiler pre-allocates stack space at function entry, so crashes appear to happen before any code executes. Symptoms include:
- Random-looking crash points (actually at function entry)
- Crash happens after some printf output, before the next
- No error message - device just stops responding

**Objects known to require static allocation:**
- `CompassCalibrator` (ArduPilot) - ~3KB+
- Any object with large internal buffers

**Debug tip:** When a crash appears random, try moving large local variables to static allocation.

---

## Safety and Regulatory

### RF Regulations (US)

| Band | Regulation | Limits |
|------|------------|--------|
| **915MHz ISM** | FCC Part 15 | Power and duty cycle limited, license-free |
| **2.4GHz ISM** | FCC Part 15 | WiFi, Bluetooth, NRF24L01 |

*Check local regulations for non-US deployments.*

### Rocketry Safety Codes

| Organization | Applies To |
|--------------|------------|
| **NAR** (National Association of Rocketry) | Low/mid power |
| **TRA** (Tripoli Rocketry Association) | High power |

### Pyro Channel Safety

Firmware handling pyro channels must implement:
- Software arm state (cannot fire unless armed)
- Physical arm switch recommended (Pro tier)
- Continuity checking before arm
- Timeout/auto-disarm after landing
- All pyro code requires council review before merge

### Watchdog Requirements

- Watchdog timer must be enabled in flight-critical code
- Watchdog timeout appropriate for loop rate
- Watchdog kick only in main loop (not in interrupts)

---

## Software Architecture

### RTOS

| Tier | RTOS | Notes |
|------|------|-------|
| Core | Evaluating | May not need RTOS for simple logging |
| Main | Evaluating | Depends on telemetry complexity |
| Pro | **FreeRTOS** | Required for TVC, pyro timing, multi-task |

### State Machines

Flight phases managed via explicit state machine:
- States: IDLE, ARMED, BOOST, COAST, DESCENT, LANDED, ERROR
- All transitions logged
- Timeout fallbacks for stuck states

### Mission Engine

Event-action-state architecture (separate module/library):
- **Mission** defines sensors, states, events, actions, logging
- Same hardware runs different Missions for different applications
- See Mission Engine documentation for details

### ArduPilot Library Integration

Using ArduPilot libraries via compatibility shim (not full ArduPilot firmware):
- **AP_Math** - Vectors, matrices, quaternions
- **AP_AHRS** - Attitude estimation (DCM)
- **Filter** - LowPass, Notch filters
- **AP_AccelCal / AP_Compass** - Calibration routines

Full ArduPilot port (ArduRocket) is a stretch goal requiring ChibiOS HAL.

### Dependency Bypassing Policy

**MANDATORY: Explicit approval required before bypassing any ArduPilot dependencies.**

When integrating ArduPilot libraries, the default approach is to use ArduPilot's proven implementations directly. Writing custom wrappers, simplified versions, or workarounds that bypass ArduPilot's actual code is **not permitted without explicit user approval**.

**Sparse Checkout Review:**

When you encounter an ArduPilot dependency that is NOT in our sparse checkout (`lib/ardupilot/.git/info/sparse-checkout`), you MUST:
1. **Flag it for review** - Tell the user which library is missing
2. **Propose adding it** - Suggest: `git sparse-checkout add libraries/AP_LibraryName`
3. **Wait for decision** - Do not create a simplified alternative without explicit approval

When you find yourself writing code that mimics or copies ArduPilot logic (e.g., copying `correct_field()` implementation), STOP and flag this for review. The real ArduPilot code should be used instead.

**Before proposing to bypass a dependency:**

1. **Identify the exact dependency** - What ArduPilot code are you considering not using?
2. **Explain the blocker** - What specifically prevents using the real implementation?
   - Missing HAL component (specify which)
   - ChibiOS-specific code that cannot be ported
   - Circular dependency that cannot be resolved
3. **Propose alternatives** - Can we implement the missing HAL component instead? Can we add the library to sparse checkout?
4. **Wait for approval** - Do not proceed until the user explicitly approves the bypass

**Rationale:** ArduPilot code has been battle-tested on millions of devices. Custom implementations introduce bugs that ArduPilot already solved (e.g., sign conventions, edge cases, calibration algorithms). The time "saved" by shortcuts is inevitably lost debugging issues that proven code handles correctly.

**Examples of what requires approval:**
- Writing a simplified `AP_Compass` wrapper instead of using `AP_Compass_Backend`
- Implementing custom calibration logic instead of using `CompassCalibrator` directly
- Stubbing out functionality that ArduPilot implements

**What does NOT require approval:**
- Implementing missing AP_HAL components (Storage, I2C, SPI, etc.)
- Creating compatibility shims that call real ArduPilot code
- Adding platform-specific implementations in `lib/ap_compat/`

---

## Code Verification Process

All code changes must pass the verification checklist before merge to main.

### Pre-Commit Checklist

1. **Build Verification**
   - [ ] Code compiles without errors
   - [ ] Code compiles without warnings (`-Wall -Wextra`)
   - [ ] All affected targets build successfully

2. **Standards Compliance**
   - [ ] Naming conventions followed (JSF AV Rule 50-53):
     - Constants use `k` prefix: `kSampleRate`, `kMaxRetries`
     - Global variables use `g_` prefix: `g_sensorData`, `g_calibrationState`
     - Pointers use `p_` suffix in name: `p_buffer`, or `g_p_` for global pointers
   - [ ] No magic numbers (JSF AV Rule 151) - use named constants
   - [ ] No dynamic allocation after initialization (JSF AV Rule 206) for production code
   - [ ] Fixed-width types used (JSF AV Rule 209): `uint32_t`, `int16_t`, not `int`, `long`
   - [ ] No exceptions, RTTI disabled for embedded targets
   - [ ] If deviation required, documented in `standards/STANDARDS_DEVIATIONS.md`

3. **DEBUG_OUTPUT.md Compliance** (for code with serial output)
   - [ ] USB CDC wait pattern followed (visual feedback, settle time)
   - [ ] Debug macros used (`DBG_PRINT`, `DBG_ERROR`) not raw `printf` in production
   - [ ] NeoPixel/LED status patterns match documented conventions

4. **Documentation**
   - [ ] CHANGELOG.md updated for significant changes
   - [ ] Code comments explain "why", not "what"
   - [ ] New modules have header file documentation

### Pre-Merge Review

- [ ] Self-review completed using checklist above
- [ ] Any new standards deviations logged with severity assessment
- [ ] Safety-critical code (pyro, state machine) requires council review

### Deviations Tracking

All standards deviations must be logged in `standards/STANDARDS_DEVIATIONS.md` with:
- Location (file:line)
- Standard violated
- Severity level (Critical/High/Medium/Low/Accepted)
- Difficulty to fix (Trivial/Easy/Moderate/Hard)
- Rationale for deviation
- Remediation plan (if applicable)

See `standards/STANDARDS_DEVIATIONS.md` for current deviation log and review schedule.
