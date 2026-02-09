# RocketChip Project Status

**Last Updated:** 2026-02-08

## Current Phase

**Stage 4 COMPLETE** — GPS Integration

All Stage 4 IVPs (31/32/33) hardware-verified. GPS fix confirmed outdoors, CLI displays all GPS data from seqlock. Next: Stage 5 ESKF sensor fusion (IVP-34+).

## Completed

### Stage 1: Foundation (IVP-01 through IVP-08) ✅

Hardware-verified 2026-02-04:
- [x] IVP-01: Clean build from source
- [x] IVP-02: Red LED blink (heartbeat 100ms on / 900ms off)
- [x] IVP-03: NeoPixel rainbow via PIO
- [x] IVP-04: USB CDC serial output
- [x] IVP-05: Debug macros functional (timestamps, compile out in Release)
- [x] IVP-06: I2C bus init at 400kHz
- [x] IVP-07: I2C scan (ICM-20948, DPS310, PA1010D all detected)
- [x] IVP-08: Heartbeat superloop with uptime

### Stage 2: Single-Core Sensors (IVP-09 through IVP-18) ✅

Hardware-verified 2026-02-06:
- [x] IVP-09: ICM-20948 IMU initialization (WHO_AM_I, accel ±4g, gyro ±500dps, AK09916 mag 100Hz)
- [x] IVP-10: IMU data validation (10Hz, all gates pass across reboot cycles)
- [x] IVP-11: DPS310 barometer initialization (ruuvi driver, 8Hz/8x oversampling, continuous mode)
- [x] IVP-12: Barometer data validation (10Hz, 100 samples, all gates pass)
- [x] IVP-13: Multi-sensor polling (IMU 100Hz/774us, Baro 50Hz/251us, 0 errors/60s)
- [x] IVP-13a: I2C bus recovery (disconnect/reconnect verified, lazy baro reinit, no hang)
- [x] IVP-14: Calibration storage (flash persistence, all 4 gates pass, power cycle verified)
- [x] IVP-15: Gyro bias calibration (2s stationary, auto-applied)
- [x] IVP-16: Level calibration (gravity-based level offset, flash-persisted)
- [x] IVP-17: 6-position accel calibration (Gauss-Newton solver, 6/6 gates pass, 3/3 power cycles)
- [x] IVP-18: CLI menu (RC_OS with main + calibration menus, sensor status, I2C rescan)

### Stage 3 Session A: Core 1 Alive (IVP-19/20) ✅

Hardware-verified 2026-02-06:
- [x] IVP-19: Core 1 launched, NeoPixel blinks cyan/magenta at 2Hz from Core 1
- [x] IVP-20: Cross-core atomic counter (std::atomic<uint32_t>, relaxed/acquire, 5-min soak, 4/s stable)

### Stage 3 Session B: Inter-Core Primitives (IVP-21/22/23) ✅

Hardware-verified 2026-02-06:
- [x] IVP-21: Spinlock soak (SW spinlock ID=26, 5-min soak, 0/2999 inconsistent, max 5us hold, 29.4M writes)
- [x] IVP-22: FIFO message passing (1000/1000 echo, 0 lost, 1000/1000 sequential, FIFO depth 4 per SDK docs)
- [x] IVP-23: Doorbell signals (1000/1000 detected, 0-5us latency, clear isolation confirmed)

### Stage 3 Session C: Seqlock Single-Buffer (IVP-24) ✅

Hardware-verified 2026-02-06:
- [x] IVP-24: Seqlock soak (124-byte struct, ~1kHz writer/200Hz reader, 0 torn/59852 reads, 56 retries 0.09%, 5-min soak)

### Stage 3 Session D: Sensor Migration to Core 1 (IVP-25/26) ✅

Hardware-verified 2026-02-06:
- [x] IVP-25: Core 1 IMU sampling (998.4 Hz, avg 1002us cycle, 51.7us jitter stddev, 0 errors/5-min, 0 stale reads)
- [x] IVP-26: Core 1 baro sampling (7.9 Hz matching DPS310 8Hz hw rate, data-ready gated, 0 errors, both datasets valid on Core 0)

### Stage 3 Session E: USB Stress + Flash Under Dual-Core (IVP-27/28) ✅

Hardware-verified 2026-02-07:
- [x] IVP-27: USB stability soak (10-min, CLI responsive, disconnect/reconnect, 2660-key mash survived, 0 baro errors)
- [x] IVP-28: Flash under dual-core (5/5 saves, ~63ms/op, +1-2 err/save within tolerance, readback OK, power-cycle persistence confirmed)

### Stage 3 Session F: MPU Stack Guard + Watchdog (IVP-29/30) ✅

Hardware-verified 2026-02-07:
- [x] IVP-29: PMSAv8 MPU stack guard on both cores (64-byte guard regions, MemManage fault handler, no false faults over 5-min soak)
- [x] IVP-30: Hardware watchdog (5s timeout, dual-core kick pattern, 300k+ IMU reads/0 errors during soak, Core 0 stall resets in ~5s, Core 1 stall resets in ~5s, WATCHDOG RESET banner on reboot, stack overflow triggers MPU fault with LED pattern)

Session plan complete: A (Core 1 alive) ✅ → B (spinlock/FIFO/doorbell) ✅ → C (seqlock) ✅ → D (sensor migration) ✅ → E (USB/flash stress) ✅ → F (MPU/watchdog) ✅

### Standards Audit & Remediation ✅

Completed 2026-02-08:
- [x] Full JSF AV / JPL C / Power of 10 audit (249 rules, 245 applicable)
- [x] Tier 1: Quick fixes (braces, hex case, default cases, precedence, return checks)
- [x] Tier 2: Moderate fixes (multi-expression lines, variable declarations, int→fixed-width, -Werror/-Wpedantic)
- [x] Tier 3: Deviation review (8 accepted deviations documented in STANDARDS_DEVIATIONS.md)
- [x] Tier 4: RC_ASSERT macro, goto elimination, bare-metal loop/stdio/preprocessor deviation docs
- **Result:** 220 PASS / 25 PARTIAL/FAIL (90% compliance). Remaining 25 are accepted deviations or deferred to production architecture refactoring.

### Stage 4: GPS Integration (IVP-31/32/33) ✅

Hardware-verified 2026-02-08:
- [x] IVP-31: PA1010D GPS on Core 1 (10Hz read, lwGPS NMEA parsing, seqlock publish, 0% IMU/baro/GPS errors over 5-min soak with 500us I2C settling delay)
- [x] IVP-32: GPS outdoor fix validation (PPS lock confirmed, lat/lon accuracy verified, IMU/baro rates unchanged)
- [x] IVP-33: GPS CLI integration (CLI `s` shows fix/no-fix/not-detected states, all data from seqlock, no Core 0 I2C traffic)

## In Progress

**Stage 4 COMPLETE.** Next: Stage 5 ESKF sensor fusion (IVP-34: Vector3/Quaternion math library)

## What Exists

**Active (in build):**
- `src/main.cpp` — Full Stage 2 firmware with sensor polling + CLI
- `src/drivers/ws2812_status.cpp/h` — NeoPixel PIO driver
- `src/drivers/i2c_bus.cpp/h` — I2C bus wrapper with recovery
- `src/drivers/icm20948.cpp/h` — ICM-20948 IMU driver with I2C master control
- `src/drivers/baro_dps310.cpp/h` — DPS310 barometer wrapper
- `src/cli/rc_os.cpp/h` — CLI menu system with calibration integration
- `src/calibration/calibration_data.cpp/h` — Calibration data structures
- `src/calibration/calibration_manager.cpp/h` — 6-pos accel calibration (Gauss-Newton)
- `src/calibration/calibration_storage.cpp/h` — Flash persistence
- `scripts/accel_cal_6pos.py` — Interactive serial calibration script
- `scripts/ivp27_28_test.py` — IVP-27/28 soak + flash test monitor
- `scripts/ivp29_30_test.py` — IVP-29/30 watchdog soak + manual test monitor

- `src/drivers/gps_pa1010d.cpp/h` — GPS driver (PA1010D via I2C, lwGPS NMEA parser)

## Blockers

None.

## Reference

- `docs/IVP.md` — Full 64-step integration plan with verification gates
- `docs/SAD.md` — Software Architecture Document
- `standards/CODING_STANDARDS.md` — Platform constraints
- `.claude/LESSONS_LEARNED.md` — Debugging journal
