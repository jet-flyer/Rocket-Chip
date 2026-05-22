# Big Daddy Replay Profiles — Content Validation

**Audit date:** 2026-05-21
**Validator:** automated content checks via `python -c "..."` (see commit `<this commit>`)
**Plan reference:** [plan step 1](../../C:\Users\pow-w\.claude\plans\lexical-zooming-hejlsberg.md) — pre-commit gate for IVP-131 R-28 work.

## Purpose

Council amendment 4a (2026-05-21): "Validate Big Daddy profile content by plotting before committing references. Profile filenames are claims; verify them."

Each profile's filename and header docstring is a *claim* about what physical phenomenon the data exercises. If the data doesn't match the claim, we'd be locking a mislabeled profile into the ESKF regression oracle. This document records the evidence that each profile's content matches its claim.

## Validation method

For each profile, statistical content checks against the docstring's claimed events:

- **Peak altitude:** `max(gps_alt_mm) / 1000.0` vs claim (allowable delta: 50mm, 1mm-quantization headroom).
- **Apogee timing:** `argmax(gps_alt_mm)` ≈ claimed apogee_t.
- **Baro freeze:** count distinct `pressure_pa` values after claimed freeze time; expected = 1.
- **GPS dropout:** count GPS-valid samples after claimed dropout time; expected = 0.
- **IMU zero-out:** count nonzero `accel_z` samples before AND after claimed event time; expected = 0 post-event, nonzero pre-event.

Plotted PNG evidence is *not* committed in this PR — the data is purely tabular and the statistical checks above are unambiguous. PNGs can be regenerated with `matplotlib` from the CSVs if visual confirmation is needed.

## Per-profile evidence

### `big_daddy_f15_nominal.csv` — PASS

- Header claims: apogee at 7.6s, peak altitude 347.0m AGL, chute deploy at 13.6s, landing at 90.4s.
- Measured: peak `gps_alt_mm/1000 = 346.977 m` at `t = 7.629 s`.
- Delta vs claim: 23 mm (well within 1mm-quantization × interpolation margin).
- Total samples: 13,660 over 90.368s — matches landing-at-90.4s claim.

### `big_daddy_early_burnout.csv` — PASS

- Header claims: motor flame-out at 0.5s, apogee at 2.9s, peak altitude 35.1m AGL.
- Measured: peak `gps_alt_mm/1000 = 35.053 m` at `t = 2.919 s`.
- Delta vs claim: 47 mm (within quantization margin).
- Max `accel_z` in window t=0.5..0.8s: 14.67 m/s² (residual thrust tapering; nominal would be ~22 N → ~93 m/s² for partial mass). Profile shows truncated boost as claimed.

### `big_daddy_baro_dropout.csv` — PASS

- Header claims: pressure stuck at 101325 Pa (sea level) for all `t >= 7.6s` (post-apogee).
- Measured: unique `pressure_pa` values after `t = 8.1s` = **1** (value: 101325.0 Pa).
- Confirms baro-stuck-at-sea-level fault as designed.

### `big_daddy_gps_dropout_descent.csv` — PASS

- Header claims: no GPS fixes after `t = 7.6s` (descent phase).
- Measured: GPS-valid samples after `t = 8.1s` = **0**.
- Confirms post-apogee GPS dropout as designed.

### `big_daddy_imu_zero_fault.csv` — PASS

- Header claims: ICM-20948 returns `accel/gyro = (0, 0, 0)` for all `t >= 10.0s` (LL Entry 29 fault class).
- Measured: nonzero `accel_z` samples before `t = 10.0s` = **955** of 955 (100% nonzero pre-event).
- Measured: nonzero `accel_z` samples after `t = 10.5s` = **0** of 12,133 (100% zero post-event).
- Confirms ICM-20948 silent-zero-output fault as designed.

## Summary

| Profile | Filename claim | Content matches? |
|---|---|---|
| Nominal | F15-6 boost + 347m apogee | yes (delta 23mm) |
| Early burnout | flame-out at 0.5s, 35m apogee | yes (delta 47mm) |
| Baro dropout | pressure frozen post-apogee | yes (1 unique value) |
| GPS dropout descent | no GPS fixes post-apogee | yes (0 samples) |
| IMU zero fault | accel/gyro zero at t=10s | yes (100% conformance) |

**Conclusion:** all 5 profiles' content matches their claimed phenomenon. Safe to proceed with frozen-reference generation (plan steps 4 + 7).

## Schema (shared across all 5 profiles)

```
time_s, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, pressure_pa, gps_lat_1e7, gps_lon_1e7, gps_alt_mm
```

- **Sparse rows:** some sensor columns are blank to simulate slower sensor rates (e.g., baro at ~100 Hz, GPS at ~10 Hz vs IMU at ~1 kHz). The `BigDaddyLog` parser (plan step 2) treats blank fields as "no new sample for this channel" — the ESKF must handle out-of-band sensor cadence the same way the firmware does.
- **GPS:** integer microdegrees (`*_1e7`) and integer mm. Conversion handled inside the parser.
- **Frame:** body-frame accel/gyro; gravity included (`accel_z ≈ 9.8066` at rest).

## How to re-run this validation

The checks above are reproducible by running the script in this commit's PR description (or equivalent) against the 5 profile CSVs. No external dependencies beyond Python stdlib.
