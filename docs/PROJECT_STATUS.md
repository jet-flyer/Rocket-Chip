# RocketChip Project Status

**Last Updated:** 2026-02-06

## Current Phase

**IVP Stage 3 IN PROGRESS** — Dual-Core Integration (v0.2.0)

Sessions A+B+C+D (IVP-19 through IVP-26) hardware-verified. Next: Session E (IVP-27/28)

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

## In Progress

**Stage 3 Session E next (IVP-27/28 — USB stress + flash under dual-core)**

Session plan on whiteboard: A (Core 1 alive) ✅ → B (spinlock/FIFO/doorbell) ✅ → C (seqlock) ✅ → D (sensor migration) ✅ → E (USB/flash stress) → F (MPU/watchdog)

## What Exists

**Active (in build):**
- `src/main.cpp` — Full Stage 2 firmware with sensor polling + CLI
- `src/drivers/ws2812_status.c/h` — NeoPixel PIO driver
- `src/drivers/i2c_bus.c/h` — I2C bus wrapper with recovery
- `src/drivers/icm20948.c/h` — ICM-20948 IMU driver with I2C master control
- `src/drivers/baro_dps310.c/h` — DPS310 barometer wrapper
- `src/cli/rc_os.c/h` — CLI menu system with calibration integration
- `src/calibration/calibration_data.c/h` — Calibration data structures
- `src/calibration/calibration_manager.c/h` — 6-pos accel calibration (Gauss-Newton)
- `src/calibration/calibration_storage.c/h` — Flash persistence
- `scripts/accel_cal_6pos.py` — Interactive serial calibration script

**Ready (commented out in CMakeLists.txt, re-enable at their IVP step):**
- `src/drivers/gps_pa1010d.c/h` — GPS wrapper (IVP-31)

## Blockers

None.

## Reference

- `docs/IVP.md` — Full 64-step integration plan with verification gates
- `docs/SAD.md` — Software Architecture Document
- `standards/CODING_STANDARDS.md` — Platform constraints
- `.claude/LESSONS_LEARNED.md` — Debugging journal
