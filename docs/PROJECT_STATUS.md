# RocketChip Project Status

**Last Updated:** 2026-02-06

## Current Phase

**IVP Stage 2 COMPLETE** — Minimum Viable Demo milestone reached (v0.1.1)

Next: Stage 3 (Dual-Core Architecture) or Stage 4 (GPS Integration)

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

## In Progress

None — awaiting next stage decision.

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
