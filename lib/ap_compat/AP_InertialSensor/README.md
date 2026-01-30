# AP_InertialSensor - Platform-Specific Fix

**DO NOT DELETE THESE FILES - THEY CONTAIN CRITICAL PLATFORM FIXES**

## Why This Copy Exists

These files are **NOT duplicates** of ArduPilot's AP_InertialSensor. They contain a critical fix for RP2350 dual-core operation (PD12).

## The Fix: std::atomic for Cross-Core Memory Visibility

ArduPilot's original AP_InertialSensor uses plain `bool` for flags like `_new_gyro_data` and `_new_accel_data`. This works on single-core STM32 where all tasks share the same memory space.

On RP2350 with FreeRTOS SMP (dual Cortex-M33 cores), writes on one core may not be immediately visible to the other core. This causes `AP_InertialSensor::init()` to hang indefinitely because:
- Callback thread runs on Core 1, sets `_new_gyro_data = true`
- Main thread runs on Core 0, checks `_new_gyro_data` in `wait_for_sample()`
- Main thread never sees the flag as `true` due to memory visibility

## The Solution

In `AP_InertialSensor.h`:
```cpp
#include <atomic>

// PD12 fix: Use std::atomic for cross-core memory visibility
std::atomic<bool> _new_accel_data[INS_MAX_INSTANCES];
std::atomic<bool> _new_gyro_data[INS_MAX_INSTANCES];
```

In `AP_InertialSensor_Backend.cpp`:
```cpp
// Use memory_order_release when writing
_imu._new_gyro_data[instance].store(true, std::memory_order_release);

// Use memory_order_acquire when reading (in AP_InertialSensor.cpp)
_new_gyro_data[instance].load(std::memory_order_acquire);
```

## Files Modified

- `AP_InertialSensor.h` - std::atomic flag declarations
- `AP_InertialSensor.cpp` - memory_order_acquire reads
- `AP_InertialSensor_Backend.cpp` - memory_order_release writes

## Reference

- See `docs/RP2350_FULL_AP_PORT.md` PD12 for full documentation
- See `.claude/LESSONS_LEARNED.md` Entry 8 for debugging history

## History

- 2026-01-28: Fix implemented after 3+ hours debugging
- 2026-01-29: Files accidentally deleted in commit f8b85cb ("Removed stale copy")
- 2026-01-29: Files restored after rediscovering the issue
