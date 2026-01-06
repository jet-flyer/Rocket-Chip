# ArduPilot Integration Plan for RocketChip

## Overview
This document outlines a phased approach to integrating ArduPilot modules into the RocketChip flight computer project. The plan prioritizes essential modules first, with each phase building on the previous one.

## Phased Integration Plan

### Phase 1: Mathematical Foundation (MUST HAVE - Week 1)
**Why**: AccelCalibrator won't compile without these basic math libraries
```
├── AP_Math/
│   ├── vector2.h/cpp          # Basic 2D vector math
│   ├── vector3.h/cpp          # Required for AccelCalibrator
│   ├── matrix3.h/cpp          # Required for calibration
│   ├── quaternion.h/cpp       # Better than raw Madgwick
│   ├── AP_Math.h              # Common math functions
│   └── rotations.h            # Sensor orientation helpers
```
**Test**: Compile AccelCalibrator successfully

### Phase 2: Calibration Suite (MUST HAVE - Week 2)
**Why**: Professional calibration for all sensors
```
├── AP_AccelCal/               # You're already doing this
│   ├── AccelCalibrator.*      # Core algorithm
│   └── AP_AccelCal.*          # Management layer
├── AP_Compass/
│   ├── CompassCalibrator.*    # Similar to AccelCalibrator
│   └── AP_Compass_Backend.h   # Mag calibration
```
**Test**: Full 6-position accel cal working via Serial

### Phase 3: MAVLink Enhancement (HIGH PRIORITY - Week 3)
**Why**: GCS compatibility, professional telemetry
```
├── GCS_MAVLink/
│   ├── GCS_Common.cpp         # Common MAVLink handlers
│   ├── GCS_Param.cpp          # Parameter protocol
│   └── GCS_MAVLink.h          # Full protocol support
```
**Test**: QGroundControl can connect and calibrate

### Phase 4: Sensor Abstraction (MEDIUM PRIORITY - Week 4-5)
**Why**: Better sensor handling, multiple IMU support
```
├── AP_Baro/
│   ├── AP_Baro_DPS310.*       # Your existing sensor
│   └── AP_Baro_Backend.*      # Altitude calculations
├── Filter/
│   ├── LowPassFilter.*        # Better than raw data
│   ├── NotchFilter.*          # Vibration rejection
│   └── HarmonicNotchFilter.*  # Advanced filtering
```
**Test**: Cleaner sensor data, better altitude

### Phase 5: Advanced State Estimation (NICE TO HAVE - Week 6+)
**Why**: Professional attitude/position estimation
```
├── AP_AHRS/
│   ├── AP_AHRS_DCM.*          # Simple, reliable
│   ├── AP_AHRS_EKF3.*         # Full EKF (complex!)
│   └── AP_AHRS_Backend.*      # Sensor fusion
├── AP_NavEKF3/                # Only if you want full EKF
│   └── [Many files]           # Very complex
```
**Test**: Better attitude estimation than Madgwick

### Phase 6: Professional Logging (NICE TO HAVE - Week 8+)
**Why**: Industry-standard log format, analysis tools
```
├── AP_Logger/
│   ├── AP_Logger_File.*       # SD card backend
│   ├── AP_Logger_MAVLink.*    # Telemetry backend
│   └── LogStructure.h         # Message definitions
```
**Test**: Logs readable in Mission Planner/MAVExplorer

## Priority-Ordered Module List

### 🔴 MUST HAVE (Blocking current development)
1. **AP_Math** - Vector3f, Matrix3f for AccelCalibrator
2. **AP_AccelCal** - Professional accelerometer calibration
3. **AP_Common** - Basic macros and definitions

### 🟠 HIGH PRIORITY (Major improvements)
4. **AP_Compass** - Magnetometer calibration (yours is basic)
5. **GCS_MAVLink** - Full GCS compatibility
6. **AP_Param** - Runtime parameter adjustment
7. **Filter** - LowPassFilter, NotchFilter for cleaner data

### 🟡 MEDIUM PRIORITY (Good improvements)
8. **AP_Baro** - Better altitude, temperature compensation
9. **AP_AHRS_DCM** - Better than Madgwick, simpler than EKF
10. **AP_GPS** - When you add GPS module
11. **AP_BattMonitor** - Voltage/current monitoring

### 🟢 NICE TO HAVE (Future enhancements)
12. **AP_Logger** - Professional logging system
13. **AP_Scheduler** - Better timing management
14. **AP_NavEKF3** - Full EKF (overkill for rockets?)
15. **AP_Mission** - Waypoint/mission support
16. **AP_Terrain** - Terrain following (for recovery?)

### 🔵 PROBABLY SKIP (Not applicable to rockets)
- AP_Motors - For multicopters
- AP_Servo - For control surfaces
- AC_AttitudeControl - For active stabilization
- AP_Landing - For planes
- AP_Avoidance - For obstacle avoidance

## Implementation Strategy for Each Phase

### Phase 1 Implementation:
```cpp
// 1. Create AP_Math directory in your project
// 2. Copy minimal files
// 3. Create compatibility header:

// AP_HAL_Compat.h
#pragma once
#include <Arduino.h>

namespace AP_HAL {
    inline uint32_t millis() { return ::millis(); }
    inline uint32_t micros() { return ::micros(); }
}

// Math compatibility
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))

// 4. Test with simple program:
#include <AP_Math/AP_Math.h>
void setup() {
    Vector3f v(1,2,3);
    Matrix3f m(1,0,0, 0,1,0, 0,0,1);
    Vector3f result = m * v;
}
```

### Phase 2 Testing Checklist:
```
□ AccelCalibrator compiles
□ Can collect 6 samples
□ Calibration converges
□ Results are reasonable (offset < 1g, scale 0.8-1.2)
□ Calibration saves/loads from EEPROM
```

### Phase 3 Verification:
```
□ QGroundControl connects
□ Parameters visible in GCS
□ Calibration works from GCS
□ Telemetry streams work
□ Both Serial and future LoRa compatible
```

## Risk Mitigation

1. **Test each phase independently** - Don't move to next until current works
2. **Keep fallback options** - Can revert to your original code
3. **Use compiler flags** to enable/disable ArduPilot modules:
```cpp
#define USE_ARDUPILOT_MATH 1
#define USE_ARDUPILOT_AHRS 0  // Not ready yet

#if USE_ARDUPILOT_MATH
    #include <AP_Math/AP_Math.h>
    typedef Vector3f Vec3;
#else
    #include "SimpleVector3.h"
    typedef SimpleVector3 Vec3;
#endif
```

This phased approach lets you get immediate benefits (working calibration) while keeping future options open. Each phase is independently valuable and tested before moving on.

## Notes on Arduino IDE Compatibility

- ArduPilot modules will need adaptation for Arduino IDE
- May need to create wrapper libraries
- Consider using PlatformIO for better library management in later phases
- Keep core functionality Arduino IDE compatible for the base model

## RocketChip-Specific Considerations

- Focus on modules that make sense for rocket flight
- Skip airplane/multicopter specific modules
- Prioritize data quality and logging over real-time control
- Keep memory footprint reasonable for RP2350

## Timeline Estimates

- **Phase 1-2**: 2 weeks (Essential math and calibration)
- **Phase 3**: 1 week (MAVLink enhancement)
- **Phase 4-5**: 2-3 weeks (Sensor filtering and state estimation)
- **Phase 6+**: Optional future enhancements

Total estimated time for core functionality: 5-6 weeks