# RocketChip Standards and Protocols

## Coding Standards

**Coding standards are mandatory.** All code must adhere to these standards. Deviations require explicit approval and must be documented in the changelog with rationale. Approved exceptions are tracked in the Exceptions Table below.

### Foundation

This project draws from multiple safety-critical coding standards. None is treated as the supreme authority; each has a defined scope and the project applies a precedence rule when they conflict.

**Standards-precedence rule:**

Standards apply in chronological order — **newer standards take precedence over older ones** unless an older standard's text explicitly governs the specific case. The chain by publication date:

1. **Power of 10 Rules** (Holzmann/JPL, 2006 → JPL institutional 2008) — newest distilled safety-critical rules. Default authority when it speaks.
2. **JPL Institutional Coding Standard for C** (2009) — builds on P10, adds C-language-specific guidance. Default authority for C-language matters.
3. **JSF AV C++ Coding Standards** (Lockheed Martin, December 2005) — foundational catalog of 221 C++ rules. Applies where the newer standards are silent. Some JSF rules have been absorbed into modern C++ practice (C++ Core Guidelines, HIC++ V4, AUTOSAR) and no longer require separate enforcement.
4. **MISRA C** (1998 / 2004) — C-language foundation, **absorbed by JPL C at LOC-5 and LOC-6** (see "MISRA-C chain-of-custody" below). Cited where it remains the explicit authority (e.g., MISRA-C 2012 accepted safe subset for bounded `snprintf`).

**MISRA-C chain-of-custody** (added 2026-05-13 per audit-coverage gap-fill cycle; council-approved with amendments):

MISRA-C is not directly audited as a separate standard. It is **absorbed by JPL C at LOC-5 and LOC-6** — the two MISRA-absorption tiers of JPL's six-level compliance framework:

- **LOC-5 (MISRA "shall" rules not already covered by LOC-1..4)** — 73 mandatory MISRA rules.
- **LOC-6 (MISRA "should" rules not already covered by LOC-1..4)** — 16 advisory MISRA rules.

Per JPL's institutional documentation, LOC-5 and LOC-6 rule text is MIRA Ltd. copyrighted and not included in the publicly distributed JPL Institutional Coding Standard PDF. Project policy 2026-05-13: **LOC-5 and LOC-6 are DEFERRED-WITH-RATIONALE.** Rationale:

1. **Paywalled rule source contradicts the open-source audit principle** stated in `standards/AUDIT_GUIDANCE.md` Appendix B.5 ("no proprietary or paywalled material required for routine audit work").
2. **~90% overlap with JSF AV C++ rules already fully audited** at Tier 3.1 — marginal new coverage relative to the paywall cost.
3. **Project tier is hobbyist / educational rocketry** (NAR / TRA scope, not airworthiness-certified). IEC 61508 HFT=0 framing applies; fault detection + safe-mode pivot, not formal fault tolerance.

**MISRA-C remains a candidate for full inclusion in any future formally-certified-code variant.** Specifically: a hypothetical Pro-tier / Gemini-class certification path (multi-MCU dual-core observer per `AGENT_WHITEBOARD.md` "In-flight fault recovery architecture") would naturally adopt LOC-5 and LOC-6 as part of formal verification work. See `docs/PROJECT_STATUS.md` "Side Projects & Future Product Lines" for the forward-looking pointer.

The deferral is **re-evaluated each cycle** at Tier 2.5a (deferred-with-rationale row walk) — 3-cycle stale-rationale threshold triggers explicit user re-disposition.

**Conflict resolution:**
- **Newer standard explicit + older standard silent** → newer applies.
- **Older standard explicit + newer standard silent** → older applies (default coverage).
- **Both speak, agree** → either citation works.
- **Both speak, conflict** → newer wins.
- **Silent conflict** (one standard's silence vs. another's explicit rule) → **default to the more recent OR more restrictive rule** (IRL aviation-engineering practice). Time is the tiebreaker only when neither document references the other; if Standard B explicitly says "this rule overrides Standard A's section X," that explicit override governs regardless of date.

**Practical examples:**
- P10 Rule 9 (no function pointers, 2006) vs. JSF Rule 176 (use typedef when declaring function pointers, 2005): **P10 wins** — newer and more restrictive. Function pointers are an accepted deviation in our project, not "compliant via JSF."
- P10 Rule 2 (loops must have fixed upper bound, 2006) — Holzmann's own paper carves out the **inverted-rule exemption** for non-terminating scheduler loops (prove the loop *cannot* terminate). That explicit exemption is part of P10 itself, not an external override. The project's compliant non-terminating loops (QF_run scheduler, Core 1 sensor loop, fault halt) are enumerated in `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` "Note on P10 Rule 2"; those satisfy the inverted rule and therefore aren't deviations.
- JSF Rule 22 (no `<stdio.h>`, 2005) — newer standards don't disagree, so JSF applies.

**References:**
- [JSF AV C++ Standards (PDF, 2005)](https://www.stroustrup.com/JSF-AV-rules.pdf) — foundational C++ catalog
- [JPL C Coding Standard (PDF, 2009)](https://yurichev.com/mirrors/C/JPL_Coding_Standard_C.pdf) — C-language refinement, newer
- Power of 10 Rules (Holzmann/JPL, 2006) — distilled top-10 safety-critical rules, newest
- [NASA Software Engineering Handbook (SWEHB)](https://swehb.nasa.gov/) — NASA-wide guidance (handbook, not a hard standard). Good general background for safety-critical software practice across the full project lifecycle (coding, safety, V&V, configuration management, etc.). The whole handbook is the reference; specific section citations live at the point where they're actually applied (e.g., a particular accepted-deviation row, an audit-procedure step, or a design-decision doc) rather than in this overview list.

### JPL Additions (Pro Tier Goals)

These additional rules from JPL's standard are stretch goals for Pro tier:
- Mandatory `static_assert` for compile-time validation
- No uninitialized local variables
- Stricter `const`/`constexpr` usage
- Triple-voting on safety-critical variables

### Single-binary flight code

All code in `src/` on a production build is flight code. There is no compile-time flight/non-flight split and no per-file rigor tier. The same standards (this document + `standards/ACCEPTED_STANDARDS_DEVIATIONS.md`) apply uniformly. Pre-commit gates (host ctest, clang-tidy, bench_sim) run on any firmware-affecting change per the "categories not enumerations" policy in `scripts/ci/pre_commit_matrix.py` — they do not branch on per-file classification.

**Per-file deviations are case-by-case.** When a particular file genuinely needs a documented exception to the project standards, the deviation is logged in `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` with severity assessed at the time, not pre-allocated by a classification tier. The `src/fusion/eskf_codegen.cpp` CG-1 entry there (auto-generated codegen exceeding JSF AV Rule 1's L-SLOC limit) is the canonical example: the deviation is justified by the *specific* facts of that file (NASA SWEHB §8.11 auto-generated code), not by tagging it with a tier.

**Runtime access control** is the flight state machine's job, not a per-file label. When the system transitions from IDLE to ARMED, state-mutating CLI / diagnostic / test-mode paths are gated at runtime by `rc::test_mode_active()` and the state machine's phase checks (see the "R-25-exec test-mode audit invariant" section below). Read-only diagnostic paths remain available regardless of phase.

**stdio retired in src/*.cpp (2026-05-17, R-5 closure).** `<stdio.h>` is banned project-wide by the pre-commit hook. All diagnostic output uses `rc::rc_log` / `rc::rc_snprintf` / `rc::strbuf` from `include/rocketchip/rc_log.h`. `getchar_timeout_us` and `stdio_usb_connected` come from `pico/stdio.h` (SDK header, not libc `<stdio.h>`) and remain available for input/connection-state polling — they do not invoke `<stdio.h>`'s implementation-defined behaviors.

**History note (2026-05-21):** A three-tier table (Flight-Critical / Flight-Support / Ground) with per-tier rigor levels and a 12-row file roster was previously published in this section. Retired this commit. Original prescriptive levers (Ground stdio relaxation) were eliminated by R-5 (stdio removal, 2026-05-17). The remaining lever (per-tier descriptive labeling) was doing more documentation work than the table's accuracy / maintenance overhead justified — file role is better documented in per-file header doxygen and `docs/SCAFFOLDING.md` directory layout. The single-binary principle absorbs what the table was actually doing; case-by-case deviation logging handles the exception cases.

### R-25-exec test-mode audit invariant (2026-05-13)

Per council Approach A, the project enforces a mechanical audit invariant: **every state-mutating test affordance must check `rc::test_mode_active()` at entry**. Grep targets:

```
grep -rn "fault_force_" src/safety/      # every entry calls fi_test_mode_gate() (or fis_)
grep -rn "test_mode_active()" src/cli/   # every state-mutating Debug-menu branch
```

The host ctest `scripts_fault_force_gate_audit` (`scripts/audit/check_fault_force_gates.py`, landed 2026-05-21) mechanizes this: it walks every `fault_force_*` definition in `safety/fault_inject.cpp` + `safety/station_fault_inject.cpp` and verifies the first body line matches the `if (!fi_test_mode_gate("...")) { return; }` (or station-prefix `fis_`) pattern, with a documented allowlist for recovery actions (`fault_force_core0_stall_clear`, `fault_force_station_gps_restore`). It also verifies `rc_os_debug.cpp` state-mutating branches (`case 'l'/'L'`, `case '0'..'5'`) call `rc::test_mode_active()` before executing. Pre-commit hook runs ctest; the audit invariant cannot rot silently.

### RP2350 Bare-Metal Platform Constraints

These constraints are non-negotiable. They exist because violations produce silent
crashes or USB failures that take hours to diagnose.
See `.claude/LESSONS_LEARNED.md` for the full debugging narratives behind each rule.
For silicon-errata compliance status (which errata affect us and how each is handled),
see `standards/RP2350_ERRATA.md`.

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

Approved deviations from coding standards are tracked in `standards/ACCEPTED_STANDARDS_DEVIATIONS.md`. Compliance audit results are tracked in `standards/STANDARDS_AUDIT.md`.

### Comment Density

Target band: **15-25% comment density in `.cpp` files** (function-internal comment lines vs. total non-blank function lines).

**Why these numbers:**
- MathWorks Polyspace (MISRA/safety-critical static-analysis tool) defines `commentdensityfallsbelowthreshold` with a recommended **lower limit of 20%**.
- Arafati & Riehle 2009 ("The Comment Density of Open Source Software Code," ICSE NIER, 5,229 active OSS projects): mean = 19%, median = 17%, 15-25% is the empirically-observed healthy band.
- Elish & Offutt (100 Java OSS classes): mean = 15.2%, σ = 12.2.

**What counts in the measurement:**
- **Numerator:** lines starting with `//`, `/*`, or `*` (in-block).
- **Denominator:** total non-blank lines.
- **Scope:** `.cpp` files. **Header files (`.h`) are excluded** — Doxygen-heavy interface declarations naturally run 60-85% comments, and that's appropriate use of comments for API documentation.

**Per-context guidance:**
- **Flight-critical code** (per the File Classification table above): higher density is acceptable when the comment captures a hardware quirk, safety rationale, or load-bearing invariant. No upper limit codified for safety-critical contexts.
- **Ground / dev / test code**: 15-25% target applies strictly. Above ~30% in a single function is a smell — move multi-paragraph rationale to a decision doc (`docs/decisions/`), leave a 1-line pointer comment in the function.
- **Comments that paraphrase what well-named code already says** are noise regardless of density. Comments answer "why," not "what."

**When over-density is unjustified, do:**
1. Pull the multi-paragraph rationale out to a decision doc + a single-line pointer comment.
2. Delete comments that restate self-documenting code (well-named variables, obvious control flow).
3. Keep comments that explain hardware quirks, standards citations, lived-experience lessons (LL entry refs), or surprising invariants.

**Reference:** project policy 2026-05-13 (R-3 inlined fault handler shipped at ~41% density, refactored to extract rationale to `FAULT_HANDLER_DESIGN.md` and bring inline density down). Cycle-end audit 2026-05-13: src/ overall `.cpp` density = 21.8% (within band); all subdirs healthy.

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

### Flight Director & Mission Profiles

The **Flight Director** is the runtime engine — event-action-state architecture that tracks flight state, processes events, and executes transitions. It contains a hierarchical state machine at its core:
- States: IDLE, ARMED, BOOST, COAST, DESCENT, LANDED, ERROR
- All transitions logged
- Timeout fallbacks for stuck states

The **Mission Profile** is the configuration data that feeds the Flight Director — it defines phases, transitions, guard conditions, actions, and hardware mappings for a specific vehicle type. Same hardware runs different Mission Profiles for different applications.

The Flight Director follows the Mission Profile. See `docs/flight_director/` and `docs/mission_profiles/` for details.

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
   - [ ] If deviation required, documented in `standards/ACCEPTED_STANDARDS_DEVIATIONS.md`

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

All standards deviations must be logged in `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` with:
- Location (file:line)
- Standard violated
- Severity level (Critical/High/Medium/Low/Accepted)
- Difficulty to fix (Trivial/Easy/Moderate/Hard)
- Rationale for deviation
- Remediation plan (if applicable)

See `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` for current deviation log and review schedule.
