# AP_Cal - Calibration System Documentation

**Status:** Phase 1 Complete
**Last Updated:** 2026-02-02

## Overview

RocketChip's calibration system follows ArduPilot patterns while being independent of ArduPilot code dependencies. The goal is to achieve ArduPilot-equivalent calibration quality using the same algorithms and approaches.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     SensorTask (Core 0)                     │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │
│  │  ICM-20948  │  │   DPS310    │  │  PA1010D    │         │
│  │  (IMU)      │  │   (Baro)    │  │  (GPS)      │         │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘         │
│         │                │                │                 │
│         ▼                ▼                ▼                 │
│  ┌──────────────────────────────────────────────┐          │
│  │           Calibration Manager                │          │
│  │  - Gyro bias calibration                     │          │
│  │  - Accel level calibration                   │          │
│  │  - Baro ground reference                     │          │
│  │  - [Future] 6-position accel cal             │          │
│  │  - [Future] Magnetometer cal                 │          │
│  └──────────────────┬───────────────────────────┘          │
│                     │                                       │
│                     ▼                                       │
│  ┌──────────────────────────────────────────────┐          │
│  │           Calibration Storage                │          │
│  │  - Dual-sector wear leveling                 │          │
│  │  - Power-safe writes                         │          │
│  │  - CRC16 validation                          │          │
│  └──────────────────────────────────────────────┘          │
└─────────────────────────────────────────────────────────────┘
```

## Implemented Features

### Phase 1: Basic Calibration (Complete)

| Feature | Status | ArduPilot Equivalent |
|---------|--------|---------------------|
| Gyro bias calibration | ✅ Done | `AP_InertialSensor::_init_gyro()` |
| Accel level calibration | ✅ Done | `AP_InertialSensor::_calibrate_accel()` (simple) |
| Baro ground reference | ✅ Done | `AP_Baro::calibrate()` |
| Dual-sector flash storage | ✅ Done | `AP_FlashStorage` pattern |
| CRC16 validation | ✅ Done | Standard across ArduPilot |
| Motion detection | ✅ Done | Used in all ArduPilot cal routines |

### Phase 2: Advanced Calibration (Planned)

| Feature | Status | ArduPilot Equivalent |
|---------|--------|---------------------|
| 6-position accel calibration | 🔲 Planned | `AccelCalibrator` |
| Magnetometer calibration | 🔲 Planned | `CompassCalibrator` |
| Temperature compensation | 🔲 Planned | `AP_InertialSensor_tempcal` |
| In-flight learning | 🔲 Future | `AP_InertialSensor::update_gyro_learning()` |

## Calibration Data Structure

```c
typedef struct {
    // Header
    uint32_t magic;         // 0x52434341 "RCCA"
    uint16_t version;       // Schema version
    uint16_t crc16;         // CRC of payload

    // Accelerometer
    accel_cal_t accel;      // offset + scale per axis

    // Gyroscope
    gyro_cal_t gyro;        // bias per axis

    // Magnetometer
    mag_cal_t mag;          // hard iron + soft iron matrix

    // Barometer
    baro_cal_t baro;        // ground pressure reference

    // Metadata
    uint32_t cal_flags;     // Which calibrations complete
    uint32_t cal_timestamp; // When last calibrated
} calibration_store_t;
```

Total size: <512 bytes (fits in single flash page)

## Flash Storage Layout

Uses last 8KB of 8MB flash:

```
Flash End (0x800000)
├── Sector B: 0x7FF000 - 0x7FFFFF (4KB)
│   └── [Header][Entry][Calibration Data]
├── Sector A: 0x7FE000 - 0x7FEFFF (4KB)
│   └── [Header][Entry][Calibration Data]
└── Application code ends before 0x7FE000
```

### Wear Leveling Strategy

Following ArduPilot's AP_FlashStorage pattern:

1. **Dual-sector alternation**: Writes go to alternate sector
2. **Sequence numbers**: Higher sequence = more recent
3. **Atomic state transitions**: Uses flash bit-clearing property
4. **Power-safe**: Any power loss leaves recoverable state

### Sector States

```
ERASED    (0xFFFFFFFF) - Fresh sector
IN_USE    (0x52435355) - Active sector
FULL      (0x52435346) - Sector complete
```

## Calibration Routines

### Gyro Bias Calibration

**Purpose:** Remove gyroscope zero-rate offset

**ArduPilot Reference:** `AP_InertialSensor::_init_gyro()`

**Algorithm:**
1. Device must be stationary
2. Collect 200 samples (~2 seconds at 100Hz)
3. Check for motion (max-min < threshold)
4. Compute average as bias
5. Store bias and reference temperature

**Motion Threshold:** 0.05 rad/s (same as ArduPilot)

**Usage:**
```c
sensor_task_start_gyro_cal();
// Wait for completion
while (sensor_task_is_calibrating()) {
    vTaskDelay(pdMS_TO_TICKS(100));
}
sensor_task_save_calibration();
```

### Accelerometer Level Calibration

**Purpose:** Establish gravity reference when device is level

**ArduPilot Reference:** Simple level cal in `AP_InertialSensor`

**Algorithm:**
1. Device must be flat and stationary
2. Collect 100 samples (~1 second)
3. Check for motion (max-min < threshold)
4. Compute offset from expected gravity vector
5. Expected: X≈0, Y≈0, Z≈±9.81 m/s²

**Motion Threshold:** 0.5 m/s²

**Offset Calculation:**
```c
offset.x = avg_x;  // Expected 0
offset.y = avg_y;  // Expected 0
offset.z = avg_z - sign(avg_z) * GRAVITY;  // Preserve sign
```

### Barometer Ground Reference

**Purpose:** Set current location as altitude zero reference

**ArduPilot Reference:** `AP_Baro::calibrate()`

**Algorithm:**
1. Collect 50 samples (~1 second at 50Hz)
2. Average pressure as ground reference
3. Store reference temperature

**Altitude Calculation:**
```c
// Barometric formula
h = 44330 * (1 - (P/P0)^0.1903)
```

## Applying Calibration

Calibration is applied automatically in SensorTask:

```c
// Gyro: subtract bias
calibrated = raw - bias

// Accel: offset then scale
calibrated = (raw - offset) * scale

// Altitude: use ground reference
altitude_agl = 44330 * (1 - (pressure / ground_pressure)^0.1903)
```

## CLI Commands

Via sensor_task API:
- `sensor_task_start_gyro_cal()` - Start gyro calibration
- `sensor_task_start_accel_level_cal()` - Start level cal
- `sensor_task_start_baro_cal()` - Start baro cal
- `sensor_task_save_calibration()` - Save to flash
- `sensor_task_reset_calibration()` - Factory reset

## Future: 6-Position Accelerometer Calibration

**ArduPilot Reference:** `AccelCalibrator` class

**Algorithm:**
1. Place device in 6 orientations (±X, ±Y, ±Z up)
2. Collect samples at each position
3. Fit ellipsoid to data points
4. Solve for offset and scale per axis

**Positions:**
- LEVEL
- LEFT
- RIGHT
- NOSE_DOWN
- NOSE_UP
- BACK (inverted)

## Future: Magnetometer Calibration

**ArduPilot Reference:** `CompassCalibrator` class

**Algorithm:**
1. Rotate device through all orientations
2. Collect samples as sphere of data points
3. Fit ellipsoid to find hard iron offsets
4. Compute soft iron matrix from ellipsoid axes
5. Validate against expected field strength

**Hard Iron:** Constant offset from nearby ferrous materials
**Soft Iron:** Distortion matrix from nearby conductors

## Files

| File | Purpose |
|------|---------|
| `src/calibration/calibration_data.h` | Data structures |
| `src/calibration/calibration_data.c` | CRC, validation |
| `src/calibration/calibration_manager.h` | Calibration routines |
| `src/calibration/calibration_manager.c` | Algorithm implementation |
| `src/calibration/calibration_storage.h` | Flash interface |
| `src/calibration/calibration_storage.c` | Dual-sector wear leveling |
| `src/tasks/sensor_task.h` | Public API |
| `src/tasks/sensor_task.c` | Task integration |

## References

- ArduPilot `AP_InertialSensor` - IMU calibration
- ArduPilot `AP_FlashStorage` - Wear leveling pattern
- ArduPilot `AccelCalibrator` - 6-position algorithm
- ArduPilot `CompassCalibrator` - Magnetometer algorithm
- TDK ICM-20948 Datasheet - Sensor specifications
- Infineon DPS310 Datasheet - Barometer specifications
