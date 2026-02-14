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

### Code Classification

All source files are classified by their role in the flight stack. Standards rigor
scales with criticality — flight-critical code gets full JSF AV + JPL C enforcement,
while ground-only code has relaxed rules for stdio and diagnostics.

**Same binary principle:** There is no compile-time flight flag. Flight and ground code
coexist in the same binary. The flight state machine controls runtime access — when the
system transitions from IDLE to ARMED, ground-only code paths (CLI, diagnostics, USB I/O)
are locked out at runtime, not compiled out.

| Classification | Standards Rigor | stdio | Runtime Lockout |
|---------------|----------------|-------|----------------|
| **Flight-Critical** | Full JSF AV + JPL C. No stdio except bounded `snprintf` with `sizeof()` | `snprintf` only (MISRA-C safe subset) | Always active |
| **Flight-Support** | Full JSF AV. No stdio in hot path | None | Always active |
| **Ground** | JSF AV with stdio relaxation. `printf`/`getchar` permitted behind `stdio_usb_connected()` guard | Permitted with guards | Locked out when state != IDLE |

#### Current File Classification

| File | Classification | Notes |
|------|---------------|-------|
| `src/drivers/icm20948.cpp` | Flight-Critical | IMU driver — zero printf in read path |
| `src/drivers/baro_dps310.cpp` | Flight-Critical | Barometer driver |
| `src/drivers/i2c_bus.cpp` | Flight-Critical (core) | Bus read/write/probe are flight-critical; `i2c_bus_scan()` is ground-only |
| `src/drivers/gps_pa1010d.cpp` | Flight-Critical | 3 bounded `snprintf` for NMEA command formatting (documented MISRA deviation) |
| `src/drivers/ws2812_status.cpp` | Flight-Support | Status LED — non-critical but active in flight |
| `src/calibration/calibration_data.cpp` | Ground | Pre-flight calibration storage |
| `src/calibration/calibration_manager.cpp` | Ground | Calibration algorithms — run pre-flight only |
| `src/calibration/calibration_storage.cpp` | Ground | Flash storage — pre-flight only |
| `src/cli/rc_os.cpp` | Ground | CLI / local GCS — locked out in flight |
| `src/main.cpp` | Ground (mixed) | Contains flight loop (Core 1 sensor sampling, watchdog) + ground-only CLI dispatch |

### RP2350 Bare-Metal Platform Constraints

These constraints are non-negotiable. They exist because violations produce silent
crashes or USB failures that take hours to diagnose.
See `.claude/LESSONS_LEARNED.md` for the full debugging narratives behind each rule.

#### Memory

- **Large local variables (>1KB) must be `static` or heap-allocated.** Large locals
  cause silent stack corruption at function entry. *(LL Entry 1)*

#### USB CDC

- **Guard all USB I/O with `stdio_usb_connected()`.** When disconnected, do
  nothing — no printf, no getchar. *(LL Entry 15)*

- **Drain USB input buffer on terminal connection.** Garbage bytes from USB
  handshake trigger phantom commands. *(LL Entry 15)*

- **Let SDK handle USB.** Do not override SDK USB handling.

#### Flash

- **Flash ops make ALL flash inaccessible** — including USB IRQ handlers and code
  on both cores. Use `flash_safe_execute()` from `pico/flash.h` for all flash
  access. *(LL Entries 4, 12)*

#### Build System

- **CMake + Pico SDK only.** No PlatformIO, no Arduino.

- **All `#define` macros affecting class layout must be in global
  `add_compile_definitions()`.** Inconsistent macros across compilation units
  cause ODR violations — inline getters return garbage.

- **Board type before SDK import:**
  `set(PICO_BOARD "adafruit_feather_rp2350" CACHE STRING "Board type")`

#### Hardware

- **Verify I2C addresses against Adafruit defaults.** ICM-20948 is 0x69
  (AD0=HIGH), not 0x68. Run I2C scan to confirm. *(LL Entry 13)*

- **WS2812 requires `begin()` after construction.** Without it, `setPixel()` and
  `show()` silently no-op. *(LL Entry 6)*

#### Debugging

- **Use debug probe first** when USB is broken. Don't waste time on LED blink
  patterns. *(LL Entries 5, 11)*

- **Always include a version string** that changes with each significant build.
  During extended debugging sessions with multiple builds, use a monotonic build
  iteration tag (e.g., `Build: IVP30-fix-3`) and verify it in serial output
  before testing. `__DATE__ __TIME__` alone is insufficient — timestamps blur
  together across rapid rebuilds and the same binary flashed twice looks
  identical. *(LL Entry 2)*

#### General

- **No arbitrary numerical values.** Sample rates, buffer sizes, thresholds,
  timeouts — all must be justified by a source (datasheet, SDK docs, reference
  implementation, confirmed forum post). If no source exists, flag and ask.

- **Research before implementing.** Before writing code that touches hardware
  interfaces or sensor drivers, check relevant documentation, datasheets, and
  recent forum posts for known issues.

- **Don't assume — ask.** If a task or design choice isn't covered by existing
  docs and research doesn't definitively answer it, ask before implementing.

### Prior Art Research

**Before implementing any hardware interface, driver, or novel functionality:**

1. **Check Pico SDK examples first** - For RP2350-specific implementations
   - SDK examples show idiomatic usage patterns
   - Hardware-specific quirks are often documented in examples

2. **Check Adafruit/SparkFun libraries** - These are our preferred hardware vendors
   - CircuitPython implementations often have clear, well-documented approaches
   - Arduino libraries show common patterns for similar MCUs
   - Check for errata workarounds they've already implemented

3. **Check ArduPilot as reference** - Useful for algorithms and sensor handling patterns
   - Calibration algorithms, sensor fusion approaches, flight state logic
   - Not a direct dependency — use as reference material, not imported code
   - Full ArduPilot integration (ArduRocket) is a future goal once ChibiOS RP2350 support ships

4. **Document findings** - When prior art influences implementation:
   - Reference source in code comments (e.g., "// Based on Pico SDK example...")
   - Note any deviations from prior art and rationale

**Rationale:** Proven solutions from established projects save time and reduce bugs. When millions of devices use a particular approach, that's strong evidence it works.

**See also:** `standards/VENDOR_GUIDELINES.md` — vendor-specific constraints, datasheet-sourced values, and OEM recommendations for all hardware components.

### Exceptions and Deviations

Approved deviations from coding standards are tracked in `standards/STANDARDS_DEVIATIONS.md`. Compliance audit results are tracked in `standards/STANDARDS_AUDIT.md`.

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
| Delay | `sleep_ms()` | `busy_wait_ms()` | `sleep_ms` works with SDK timers |
| Large objects | `static ClassName g_obj;` | `ClassName obj;` in function | Stack space is limited; large objects cause overflow |

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
- Any object with large internal buffers (calibration matrices, sensor history, etc.)

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
