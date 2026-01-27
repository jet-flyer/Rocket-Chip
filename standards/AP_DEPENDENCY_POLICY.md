# ArduPilot Dependency Policy

**Purpose:** Define which ArduPilot libraries must use upstream code vs. when custom implementations are approved.

**Last Updated:** 2026-01-26

---

## Core Principles

### 1. Use ArduPilot's Proven Code

ArduPilot code is battle-tested. Custom implementations introduce risk. The burden of proof is on bypassing, not on using.

**Example of risk:** A custom AP_Compass wrapper used `raw - offset` instead of `raw + offset`, introducing a sign convention bug that ArduPilot's code (AP_Compass_Backend.cpp:85) already handles correctly.

### 2. Development Order is Flexible

Phases are guidelines, not constraints. If implementing Feature X properly requires Component Y that was "planned for later", **pull Component Y forward**. Don't create broken bridge solutions to maintain arbitrary schedules.

### 3. Flag Dependencies Immediately

When encountering a missing ArduPilot dependency:

1. **Flag it:** "This needs AP_XYZ which isn't in sparse checkout"
2. **Suggest pulling it up:** "Should we add AP_XYZ now instead of a workaround?"
3. **Wait for decision** - don't assume workarounds are acceptable

### 4. Integrate into Main Codebase

New features should be integrated into the main `rocketchip` target, not isolated in separate smoke tests. Components need to work together:

- Mag calibration needs AHRS
- AHRS needs calibrated sensors
- Calibration needs storage
- Everything needs to integrate

**Don't** create separate test targets for each feature. **Do** integrate into main codebase and verify there. Smoke tests are for initial bring-up of hardware interfaces, not for feature development.

---

## Approved Custom Implementations

### Category A: Legitimate HAL (Required for Platform)

| Component | Reason |
|-----------|--------|
| `AP_HAL_RP2350/*` | RP2350-specific hardware abstraction - ChibiOS doesn't support RP2350 |

This is required because ArduPilot's official HAL implementations (ChibiOS, Linux, SITL) don't support RP2350.

### Category B: Provisional (Pending Investigation)

These implementations exist but require investigation to determine if ArduPilot's code can be used instead.

| Component | Status | Required Investigation |
|-----------|--------|------------------------|
| AP_AHRS (bootstrap) | PROVISIONAL | Can AP_AHRS_DCM work with current HAL? |
| AP_InertialSensor (bridge) | PROVISIONAL | What blocks full AP_InertialSensor? |
| GCS_MAVLink (minimal) | PROVISIONAL | Should we pull up full GCS? |

**Provisional means:** May be replaced with ArduPilot's code after investigation. Do not assume these are permanent.

---

## Must Use ArduPilot's Code

These components must use ArduPilot's implementation directly. No custom alternatives permitted.

| Component | Reason |
|-----------|--------|
| AP_Math | Math is math - no platform dependency |
| Filter | Filters are generic algorithms |
| AccelCalibrator | Calibration algorithm is complex, proven |
| CompassCalibrator | Calibration algorithm is complex, proven |
| AP_FlashStorage | Storage algorithm with wear-leveling is complex |
| AP_InternalError | Simple enough to use directly, provides better debugging |
| AP_Declination | WMM model lookup is complex, data tables are large |

---

## Process for Requesting Custom Implementation

If you believe a custom implementation is needed:

1. **Identify exact ArduPilot code being considered**
   - Which library/class would you be reimplementing?
   - Is it in the sparse checkout already?

2. **Investigate what it actually requires** - not assumptions
   - Read the ArduPilot code
   - Identify specific dependencies (HAL components, other libraries)
   - Don't say "too complex" - identify specific blockers

3. **Document specific blockers**
   - "Needs AP_Scheduler which we don't have" (specific)
   - "Requires ChibiOS thread model" (specific)
   - NOT: "Too heavy" or "Too complex" (vague)

4. **Propose pulling up the dependency first**
   - Can we add the missing HAL component?
   - Can we add the missing library to sparse checkout?
   - Don't assume workaround is preferred

5. **Get explicit user approval** before creating any custom implementation

6. **If approved, document:**
   - Exact scope of custom implementation
   - What triggers upgrade to ArduPilot's version
   - Validation tests comparing output to ArduPilot's

---

## Sparse Checkout Management

### Current Libraries in Checkout

```
lib/ardupilot/libraries/
├── AP_AccelCal/       - IN USE
├── AP_Common/         - Headers only
├── AP_Compass/        - IN USE (CompassCalibrator)
├── AP_Declination/    - Available (Phase 4)
├── AP_FlashStorage/   - IN USE
├── AP_HAL/            - Headers only
├── AP_InternalError/  - IN USE (replacing custom)
├── AP_Math/           - IN USE
├── AP_Param/          - Available (recently added)
├── Filter/            - IN USE
└── StorageManager/    - Not used (AP_FlashStorage preferred)
```

### Adding Libraries to Sparse Checkout

```bash
cd lib/ardupilot
git sparse-checkout add libraries/AP_LibraryName
```

### Before Creating Custom Code

Always check:
1. Is the library already in sparse checkout?
2. If not, can we add it?
3. What does the ArduPilot library actually require?

---

## Review Schedule

- **When adding custom code:** Must follow process above
- **Quarterly:** Review all provisional implementations
- **Before each major milestone:** Verify no unapproved bypasses exist

---

## Change History

| Date | Change |
|------|--------|
| 2026-01-26 | Initial policy created after AP_Compass wrapper bug discovery |
