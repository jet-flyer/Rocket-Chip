# Seqlock Shared Sensor Data — Current Layout

**Purpose:** Track the current `shared_sensor_data_t` struct layout as it evolves. The original council-approved design is in `docs/decisions/SEQLOCK_DESIGN.md` (124 bytes). This document reflects the current implementation.

**Last Updated:** 2026-02-10 (wizard-7)
**Current Size:** 140 bytes (static_assert in main.cpp)

---

## Struct Layout

```cpp
struct shared_sensor_data_t {
    // --- IMU (68 bytes: 13 floats + 3 uint32_t + 3 bool + 1 pad) ---
    float accel_x;              // m/s^2, calibrated, body frame
    float accel_y;
    float accel_z;
    float gyro_x;               // rad/s, calibrated, body frame
    float gyro_y;
    float gyro_z;
    float mag_x;                // uT calibrated (when mag_valid)
    float mag_y;
    float mag_z;
    float mag_raw_x;            // uT raw (for mag recalibration)
    float mag_raw_y;
    float mag_raw_z;
    float imu_temperature_c;    // ICM-20948 die temperature
    uint32_t imu_timestamp_us;  // time_us_32() at read
    uint32_t imu_read_count;    // Monotonic, for stale detection
    uint32_t mag_read_count;    // Monotonic, increments only on new mag data
    bool accel_valid;           // false if I2C read failed
    bool gyro_valid;
    bool mag_valid;             // false if no new mag data
    uint8_t _pad_imu;

    // --- Barometer (20 bytes) ---
    float pressure_pa;          // Pascals (raw, no altitude conversion)
    float baro_temperature_c;
    uint32_t baro_timestamp_us;
    uint32_t baro_read_count;
    bool baro_valid;
    uint8_t _pad_baro[3];

    // --- GPS (36 bytes, IVP-31 PA1010D on Core 1) ---
    int32_t gps_lat_1e7;        // Latitude * 1e7 (ArduPilot convention)
    int32_t gps_lon_1e7;        // Longitude * 1e7
    float gps_alt_msl_m;        // Altitude MSL in meters
    float gps_ground_speed_mps;
    float gps_course_deg;       // Course over ground
    uint32_t gps_timestamp_us;
    uint32_t gps_read_count;
    uint8_t gps_fix_type;       // 0=none, 2=2D, 3=3D
    uint8_t gps_satellites;
    bool gps_valid;
    uint8_t gps_gga_fix;        // GGA fix quality (0=none, 1=GPS, 2=DGPS)
    uint8_t gps_gsa_fix_mode;   // GSA fix mode (1=none, 2=2D, 3=3D)
    bool gps_rmc_valid;         // RMC status ('A')
    uint8_t _pad_gps[2];

    // --- Health (16 bytes, cumulative, never reset during flight) ---
    uint32_t imu_error_count;
    uint32_t baro_error_count;
    uint32_t gps_error_count;
    uint32_t core1_loop_count;  // Monotonic, Core 1 main loop iterations
};
// Total: 140 bytes
```

## Changes From Original Design (124 bytes)

| Change | Size Delta | When | Why |
|--------|-----------|------|-----|
| GPS diagnostic fields (`gps_gga_fix`, `gps_gsa_fix_mode`, `gps_rmc_valid`, extra pad) | +4 bytes | IVP-31 | Debug GPS fix detection — raw lwGPS fields visible in CLI |
| Raw mag fields (`mag_raw_x/y/z`) | +12 bytes | wizard-7 | Ellipsoid solver needs uncorrected data for recalibration |

## Seqlock Copy Budget

At 140 bytes, `memcpy` cost is ~35 cycles on Cortex-M33 (4 bytes/cycle). Well within the 1kHz Core 1 budget. No concern until struct exceeds ~512 bytes.
