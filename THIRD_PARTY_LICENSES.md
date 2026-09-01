# Third-Party Licenses

This file lists all third-party components used by RocketChip, their licenses,
and attribution. All licenses listed are compatible with GPLv3.

Last updated: 2026-08-22

---

## Submodule Dependencies (shipped in binary)

### Raspberry Pi Pico SDK

- **Location:** `pico-sdk/`
- **Version:** 2.2.0 (commit `a1438df`)
- **License:** BSD-3-Clause
- **Copyright:** (c) 2020 Raspberry Pi (Trading) Ltd.
- **URL:** https://github.com/raspberrypi/pico-sdk
- **License file:** `pico-sdk/LICENSE.TXT`

The Pico SDK includes bundled dependencies under their own licenses:
- TinyUSB (MIT) — `pico-sdk/lib/tinyusb/LICENSE`
- lwIP (BSD) — used only if networking is enabled (not used by RocketChip)
- mbedTLS (Apache-2.0 OR GPL-2.0-or-later) — `pico-sdk/lib/mbedtls/LICENSE`
- BTStack (custom) — `pico-sdk/lib/btstack/LICENSE` (not used by RocketChip)
- cyw43-driver (BSD-3-Clause + proprietary RP addendum) — `pico-sdk/lib/cyw43-driver/LICENSE` (not used by RocketChip)

### ruuvi.dps310.c — DPS310 Barometer Driver

- **Location:** `lib/ruuvi.dps310.c/`
- **Version:** v0.3.0+ (commit `edacb9f`)
- **License:** MIT
- **Copyright:** (c) 2019 Otso Jousimaa
- **URL:** https://github.com/ruuvi/ruuvi.dps310.c
- **License file:** `lib/ruuvi.dps310.c/LICENSE`

### lwGPS — Lightweight GPS NMEA Parser

- **Location:** `lib/lwgps/`
- **Version:** v2.2.0+ (commit `e35f27a`)
- **License:** MIT
- **Copyright:** (c) 2025 Tilen MAJERLE
- **URL:** https://github.com/MaJerle/lwgps
- **License file:** `lib/lwgps/LICENSE`

### ICM-20948 Driver

- **Location:** `lib/icm20948/`
- **Version:** v1.0.0 (commit `9fce4ef`)
- **License:** MIT
- **Copyright:** (c) 2020 Stephen Murphy
- **URL:** https://github.com/stephendpmurphy/icm20948
- **License file:** `lib/icm20948/LICENSE`

### Embedded Template Library (ETL)

- **Location:** `EXTERNAL/etl-20.47.1/`
- **Version:** 20.47.1
- **License:** MIT
- **Copyright:** (c) 2021 John Wellbelove
- **URL:** https://github.com/ETLCPP/etl
- **License text:** `EXTERNAL/etl-20.47.1/etl/version.h` (this vendor tree has no standalone LICENSE file)
- **Usage:** `rc_log` formatter backend (`etl::to_string`). Header-only; compiled into the target firmware.

### QP/C Real-Time Event Framework (subset)

- **Location:** `lib/qep/`
- **Version:** 8.1.3 (vendored 2026-03-15)
- **License:** GPL-3.0-or-later OR LicenseRef-QL-commercial
- **Copyright:** (c) 2005 Quantum Leaps, LLC
- **URL:** https://github.com/QuantumLeaps/qpc
- **License file:** `lib/qep/LICENSE.txt`
- **Usage:** QEP hierarchical state machine plus the QF/QV subset listed in `CMakeLists.txt`. Dual-licensed upstream; RocketChip uses the GPL-3.0-or-later term (same family as this project). Commercial closed-source use of QP/C requires a Quantum Leaps license: https://www.state-machine.com/licensing

### MAVLink C headers

- **Location:** `lib/mavlink/`
- **Version:** MAVLink 2.0 generated headers (`MAVLINK_BUILD_DATE` Thu Mar 05 2026)
- **License:** MIT (message-definition XML and generated C headers)
- **URL:** https://github.com/mavlink/c_library_v2
- **Docs:** https://mavlink.io/en/ (License: MIT for XML + generated C; the *mavgen* toolchain itself is LGPLv3 and is not vendored here)
- **Usage:** Vehicle/GCS telemetry (IVP-61). Header-only.

---

## Build-Time Dependencies (not shipped in binary)

### Google Test

- **Version:** v1.14.0 (fetched via CMake FetchContent)
- **License:** BSD-3-Clause
- **Copyright:** (c) 2008 Google Inc.
- **URL:** https://github.com/google/googletest
- **Usage:** Host-side unit tests only. Not compiled into the target firmware.

---

## Derived Data

### WMM Magnetic Field Tables

- **Files:** `src/fusion/wmm_tables.h`, `src/fusion/wmm_tables.cpp`
- **Generator:** `scripts/generate_wmm_table.py` from `lib/wmm/WMM.COF`
- **Source:** NOAA/NCEI World Magnetic Model 2025 coefficients (public domain)
- **URL:** https://www.ncei.noaa.gov/products/world-magnetic-model
- **Epoch:** 2025.0, evaluated at 2027.5, valid 2025–2030
- **Description:** 19×37 grids of declination, inclination, and total intensity
  at 10° resolution. Interpolation (`wmm_get_field`, `wmm_get_declination`,
  `wmm_get_earth_field_ned`) is original RocketChip code (GPL-3.0-or-later).
  This replaced the earlier ArduPilot IGRF13 `wmm_declination.cpp` table.

---

## Algorithm References

The following are academic papers and reference implementations consulted during
development. RocketChip's implementations are original code, not copies. These
are listed for attribution and academic citation, not license obligation.

### Error-State Kalman Filter (ESKF)

- **Paper:** Solà, J. (2017). "Quaternion kinematics for the error-state Kalman
  filter." arXiv:1711.02508.
- **Usage:** Core ESKF architecture, quaternion operations, state propagation,
  measurement update equations.
- **Files:** `src/fusion/eskf.cpp`, `src/fusion/eskf.h`, `src/math/quat.cpp`

### Mahony AHRS Complementary Filter

- **Paper:** Mahony, R., Hamel, T., Pflimlin, J.-M. (2008). "Nonlinear
  complementary filters on the special orthogonal group." IEEE Trans. Automatic
  Control, 53(5), 1203-1218.
- **Usage:** Independent attitude cross-check alongside ESKF.
- **Files:** `src/fusion/mahony_ahrs.cpp`, `src/fusion/mahony_ahrs.h`

### ArduPilot (algorithmic reference)

- **URL:** https://github.com/ArduPilot/ardupilot
- **License:** GPL-3.0-or-later
- **Usage:** Consulted as reference for sensor fusion patterns (EKF3 gating,
  fuseEulerYaw heading model, zeroPosVelUpdate ZUPT, alignYawAngle reset),
  calibration algorithms (AccelCalibrator 6-position fit), sensor driver
  patterns (AP_InertialSensor_Invensensev2 bypass mode), and tuning parameters
  (GPS noise, bias process noise). No ArduPilot source code is included in
  this repository. Specific algorithmic deviations from ArduPilot are documented
  in `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` (entries AP-1 through AP-4).

### I2C Bus Recovery

- **Specification:** NXP UM10204 "I2C-bus specification and user manual",
  Section 3.1.16 (Bus clear procedure).
- **Usage:** Stretch-wait in `src/drivers/i2c_bus.cpp`. Bit-bang 9-clock
  bus-clear is not implemented (wedges PA1010D on STEMMA).

---

## License Compatibility Summary

| Component | License | GPLv3 Compatible |
|---|---|---|
| Pico SDK | BSD-3-Clause | Yes (permissive) |
| TinyUSB (via Pico SDK) | MIT | Yes (permissive) |
| ruuvi.dps310.c | MIT | Yes (permissive) |
| lwGPS | MIT | Yes (permissive) |
| ICM-20948 driver | MIT | Yes (permissive) |
| ETL 20.47.1 | MIT | Yes (permissive) |
| QP/C 8.1.3 | GPL-3.0-or-later OR QL-commercial | Yes (GPL term) |
| MAVLink C headers | MIT | Yes (permissive) |
| NOAA WMM2025 coefficients | Public domain | Yes |
| Google Test | BSD-3-Clause | Yes (permissive, test-only) |
