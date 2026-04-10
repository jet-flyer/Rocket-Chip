# Health Monitor Contract — Decision Record

**Date:** 2026-04-09
**Stage:** 13 (Health Monitor), IVP-104
**Council:** NASA/JPL, Professor, ArduPilot, Hobbyist, Cubesat — Unanimous GO
**Plan:** `.claude/plans/wild-wishing-pinwheel.md`

---

## Decision

Promote health monitor from FD module to standalone AO_HealthMonitor. Use 2-bit per-subsystem encoding with fault latch during flight.

## Encoding

| Value | Name | Meaning |
|-------|------|---------|
| `0b00` | Absent | Not present / not initialized. Fail-safe default (uninitialized = all absent) |
| `0b01` | Fault | Present but failing — errors, divergence, no data |
| `0b10` | Degraded | Working but reduced quality — intermittent errors, no GPS fix |
| `0b11` | OK | Fully operational |

### Primary byte: 4 subsystems x 2 bits

| Bits | Subsystem | Absent | Fault | Degraded | OK |
|------|-----------|--------|-------|----------|-----|
| [1:0] | IMU | Not initialized | 10/10 invalid | 5+/10 invalid (sliding window) | accel_valid steady |
| [3:2] | Baro | Not initialized | 10/10 invalid | 3+/10 invalid (sliding window) | baro_valid steady |
| [5:4] | ESKF | Not initialized | `healthy()` false | `healthy()` true, confidence gate uncertain | Healthy + confident |
| [7:6] | GPS | Not initialized | Init but `gps_read_count == 0` | NMEA flowing, `fix_type < 2` or `sats < 4` | 2D/3D fix, 4+ sats |

### Secondary byte: 1-bit flags

| Bit | Flag | Condition for OK |
|-----|------|-----------------|
| 0 | Radio | `AO_Radio_get_state()->initialized` |
| 1 | Flash | Flight table loaded + space remaining |
| 2 | Watchdog | No safe-mode AND no ESKF disabled |
| 3 | PIO | `pio_watchdog_fault_detected()` false |

## State Machine

```
absent -> healthy   (on successful init)
absent -> fault     (on init failure with attempted init)
healthy -> degraded (sliding window threshold breached)
healthy -> fault    (all readings invalid, or critical failure)
degraded -> healthy (window recovers below threshold)
degraded -> fault   (sustained degradation, all invalid)
fault -> healthy    (ONLY in IDLE or LANDED phases — auto-recover)
fault -> fault      (LATCHED during ARMED through DESCENT/ABORT)
```

### Fault Latch Behavior

- **ARMED, BOOST, COAST, DROGUE_DESCENT, MAIN_DESCENT, ABORT:** Faults latch. Once a subsystem enters fault, it stays in fault regardless of measured state until the flight phase returns to IDLE or LANDED.
- **IDLE, LANDED:** Auto-recovery permitted. Fault latch cleared on phase transition.
- **Rationale (Council):** Mid-flight fault oscillation confuses telemetry analysis. A sensor that faults and "recovers" 3 times during flight is not trustworthy. Post-landing GPS recovery is important for beacon/location.

### Fault-to-Healthy Logging

Every fault-to-healthy transition emits a `DBG_PRINT` log event. This is the minimum for traceability per JPL + Cubesat council requirements.

## Sliding Windows

- **Window size:** 10 ticks (1 second at 10Hz)
- **IMU degrade threshold:** 5/10 invalid ticks
- **Baro degrade threshold:** 3/10 invalid ticks
- **Pre-filled valid at boot** (optimistic: assume sensors work until proven otherwise)

## Go/No-Go Derivation

Go/No-Go ready requires:
- IMU >= Degraded (OK or Degraded)
- Baro >= Degraded
- ESKF >= Degraded
- Flash OK (1-bit)
- Watchdog OK (1-bit)
- No launch_abort

GPS and Radio are Tier 2 (profile-dependent, not hard requirements for all mission types).

## AO Priority

**Finding:** `AO_ARCHITECTURE.md` documents priorities as Radio=8, FD=6, etc. but `main.cpp` uses Radio=6, FD=5, Logger=4, Telem=3, LED=2, RCOS=1. The document is wrong — code is authoritative.

**New priority assignment (IVP-105):** HealthMonitor slots between FD and Logger.

## Staleness Counter

`uint8_t tick_counter` incremented every `health_monitor_tick()` call. If the watchdog path reads this counter and it hasn't advanced, the health monitor itself is stalled. One byte, wraps at 255 — the watchdog only needs to detect "not advancing," not the absolute value.
