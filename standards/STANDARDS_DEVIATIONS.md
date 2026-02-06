# Standards Deviations Log

**Purpose**: Track deviations from project coding standards (JSF AV C++, DEBUG_OUTPUT.md, CODING_STANDARDS.md) with severity, remediation difficulty, and rationale.

**Last Updated**: 2026-02-02 (Fresh start - post branch reorganization)

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

### ArduPilot Algorithm Deviations (IVP-17: 6-Position Accel Calibration)

These are implementation deviations from ArduPilot's `AccelCalibrator` class. The core math (Jacobian formulas, residual computation, parameter update) is identical.

| ID | Location | ArduPilot Approach | Our Approach | Severity | Difficulty | Rationale |
|----|----------|-------------------|--------------|----------|-----------|-----------|
| AP-1 | `calibration_manager.c` | Heap-allocated `std::vector` for samples | Static array `g_6pos_samples[300][3]` (3.6KB) | Accepted | N/A | LL Entry 1: no heap after init; RP2350 has 264KB RAM |
| AP-2 | `calibration_manager.c` | `mat_inverseN()` with LU decomposition | Gaussian elimination with partial pivoting | Accepted | N/A | Simpler, no separate L/U/P arrays; adequate for 9x9 |
| AP-3 | `rc_os.c` | Progress callback + async state machine | Blocking function with sensor read callback | Accepted | N/A | Bare-metal CLI doesn't need async; matches existing `cmd_reset_cal()` pattern |
| AP-4 | `rc_os.c` | Trusts user orientation (implicit) | Explicit orientation confirmation per position | Accepted | N/A | Catches positioning errors immediately vs failing at fit time |

---

## Resolved

*Previous deviations archived with `AP_FreeRTOS` branch.*

---

## Review Schedule

- **Before Flight Code**: Review all Low severity items in production paths
- **Quarterly**: Audit new code for standards compliance

---

## Notes

1. Test code (`tests/`) has relaxed standards enforcement to enable rapid validation
2. Production code (`src/`) must strictly follow all standards
3. New deviations should be logged here before merging to main
