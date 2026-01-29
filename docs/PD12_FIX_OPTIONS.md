# PD12 Memory Visibility Fix Options

**Issue:** ArduPilot's `AP_InertialSensor` reads `_new_gyro_data`/`_new_accel_data` flags WITHOUT semaphore protection, causing cross-core visibility issues on RP2350 + FreeRTOS SMP.

**Root Cause:** Flags are plain `bool` - no volatile, no atomic. Callback writes inside semaphore, main loop reads outside semaphore.

---

## Option A: Add `volatile` to Flags

**Changes:** 2 words in `AP_InertialSensor.h:546-547`

```cpp
volatile bool _new_accel_data[INS_MAX_INSTANCES];
volatile bool _new_gyro_data[INS_MAX_INSTANCES];
```

**Pros:**
- Minimal change (2 words)
- Prevents compiler from caching in register
- May work on RP2350 due to uncached SRAM

**Cons:**
- NOT technically correct per ARM memory model
- `volatile` doesn't provide CPU memory barriers
- Only works because RP2350 SRAM happens to be uncached
- Not portable to other platforms with data caches

**Verdict:** Quick hack, may work, but not recommended for production.

---

## Option B: Use `std::atomic<bool>` (RECOMMENDED)

**Changes:** ~30 lines across header and implementation files

**Header (`AP_InertialSensor.h:546-547`):**
```cpp
#include <atomic>

std::atomic<bool> _new_accel_data[INS_MAX_INSTANCES];
std::atomic<bool> _new_gyro_data[INS_MAX_INSTANCES];
```

**Reads (`AP_InertialSensor.cpp` in `wait_for_sample()`):**
```cpp
// Before:
if (_new_gyro_data[i]) { ... }

// After:
if (_new_gyro_data[i].load(std::memory_order_acquire)) { ... }
```

**Writes (`AP_InertialSensor_Backend.cpp`):**
```cpp
// Before:
_imu._new_gyro_data[instance] = true;

// After:
_imu._new_gyro_data[instance].store(true, std::memory_order_release);
```

**Clears (`AP_InertialSensor_Backend.cpp` in `update_gyro()`/`update_accel()`):**
```cpp
// Before:
_imu._new_gyro_data[instance] = false;

// After:
_imu._new_gyro_data[instance].store(false, std::memory_order_relaxed);
```

**Pros:**
- Correct per C++11 memory model
- Explicit acquire/release semantics
- Portable across all platforms
- Self-documenting intent
- No additional runtime overhead vs volatile on ARM

**Cons:**
- More lines changed (~30)
- Requires touching multiple files

**Verdict:** Correct solution. Recommended for production.

---

## Option C: Add Explicit `__dmb()` Barriers

**Changes:** ~5 lines in `AP_InertialSensor.cpp`

```cpp
#include "hardware/sync.h"  // Pico SDK

// In wait_for_sample(), before reading flags:
__dmb();  // Data Memory Barrier
for (uint8_t i=0; i<_gyro_count; i++) {
    if (_new_gyro_data[i]) { ... }
}
```

**Pros:**
- Correct per ARM memory model
- Minimal changes to ArduPilot code
- Explicit CPU barrier

**Cons:**
- Still needs `volatile` to prevent compiler caching (so really A+C)
- Platform-specific (`__dmb()` is ARM-only)
- Less self-documenting than `std::atomic`

**Verdict:** Correct but less clean than Option B.

---

## Option D: Wrap Flag Reads in Semaphore

**Changes:** ~15 lines in `AP_InertialSensor.cpp`

```cpp
// In wait_for_sample() polling loop:
while (true) {
    {
        WITH_SEMAPHORE(_sem);
        if (_new_gyro_data[i]) {
            break;
        }
    }  // Release before sleeping
    hal.scheduler->delay_microseconds_boost(100);
}
```

**Pros:**
- Uses existing synchronization primitive
- Semaphore release includes memory barrier

**Cons:**
- Major performance overhead (semaphore per iteration)
- ~100 cycles per iteration vs ~10 for atomic
- At 10kHz polling, this adds significant CPU load

**Verdict:** Correct but wasteful. Not recommended.

---

## Option E: Replace Polling with FreeRTOS Queue

**Changes:** ~200+ lines (major refactor)

```cpp
// In callback:
SensorSample sample = {...};
xQueueSend(g_sensor_queue, &sample, 0);

// In wait_for_sample():
SensorSample sample;
xQueueReceive(g_sensor_queue, &sample, portMAX_DELAY);
```

**Pros:**
- Eliminates flag entirely
- Proper producer-consumer pattern
- FreeRTOS handles all synchronization

**Cons:**
- Major architectural change
- Incompatible with ArduPilot's data flow model
- Would need to change how `update_gyro()`/`update_accel()` work

**Verdict:** Clean but too invasive. Consider for custom EKF only.

---

## Option F: Custom EKF (Bypass ArduPilot Entirely)

**Changes:** ~1000 lines new code

Write a custom 13-state EKF using AP_Math, with native FreeRTOS queue-based data flow.

**Pros:**
- No ArduPilot threading assumptions
- Full dual-core utilization
- Simpler state (13 vs 22)

**Cons:**
- Significant development effort
- Requires EKF expertise
- No ArduPilot community support

**Verdict:** Best long-term solution if ArduPilot integration proves too problematic.

---

## Recommendation

**For immediate fix:** Option B (`std::atomic<bool>`)
- Correct by design
- ~30 lines is manageable
- One-time change when bringing AP_InertialSensor into codebase

**If Option B proves insufficient:** Option F (Custom EKF)
- Same development timeline as deep ArduPilot debugging
- No threading conflicts
- Full control

---

## Files to Modify (Option B)

1. `AP_InertialSensor.h:546-547` - Declaration (2 lines)
2. `AP_InertialSensor.cpp` - wait_for_sample() reads (~10 lines)
3. `AP_InertialSensor_Backend.cpp` - Flag writes and clears (~18 lines)

---

## Testing

After applying fix, verify with:
1. `wait_for_sample()` returns within expected time
2. Gyro/accel sample rates match expected (~1125Hz)
3. No hangs during `init()` or calibration
4. Works with tasks on different cores (no core pinning)
