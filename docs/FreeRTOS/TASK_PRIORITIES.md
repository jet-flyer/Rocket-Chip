# FreeRTOS Task Priority Hierarchy

**Status:** Draft
**Last Updated:** 2026-02-02
**Reference:** `docs/SAD.md` Section 5 (Task Architecture)

---

## Overview

This document defines the task priority hierarchy for RocketChip's FreeRTOS implementation. Priorities are assigned based on timing criticality and the consequences of deadline misses.

**FreeRTOS Priority Convention:** Higher number = higher priority. Priority 0 is the idle task.

---

## Priority Levels

| Level | Priority | Category | Timing Tolerance | Example Tasks |
|-------|----------|----------|------------------|---------------|
| 5 | Highest | Hard Real-Time | <1ms jitter | SensorTask, ControlTask |
| 4 | High | Soft Real-Time | <5ms jitter | FusionTask, MissionTask |
| 3 | Medium | Buffered I/O | <20ms jitter | LoggerTask |
| 2 | Low | Background Comm | <100ms jitter | TelemetryTask |
| 1 | Lowest | User Interface | Best effort | UITask, CLITask |

---

## Current Task Assignments

| Task | Priority | Rate | Stack | Core | Purpose |
|------|----------|------|-------|------|---------|
| SensorTask | 5 | 1kHz | 1KB | 0 | IMU/baro sampling, DMA triggers |
| ControlTask | 5 | 500Hz | 1KB | 0 | TVC PID loops (Titan only) |
| FusionTask | 4 | 200Hz | 2KB | 1 | AHRS computation, altitude estimation |
| MissionTask | 4 | 100Hz | 2KB | 1 | Flight state machine, event detection |
| LoggerTask | 3 | 50Hz | 2KB | 1 | Flash writes, log buffering |
| TelemetryTask | 2 | 10Hz | 1KB | 1 | MAVLink TX/RX over radio |
| UITask | 1 | 30Hz | 1KB | 1 | NeoPixel, buttons, display |
| CLITask | 1 | N/A | 1KB | 1 | USB serial command interface |

---

## Priority Assignment Rationale

### Priority 5: Hard Real-Time

Tasks that directly affect flight safety or control loop stability.

**SensorTask:**
- IMU data must be sampled at consistent intervals for accurate integration
- Missed samples cause drift accumulation in attitude estimation
- Jitter directly impacts sensor fusion quality

**ControlTask (Titan):**
- TVC servo commands must be timely to maintain rocket stability
- Control loop latency translates to phase lag in the feedback system
- Missing deadlines during boost could cause loss of vehicle

### Priority 4: Soft Real-Time

Tasks with timing requirements but some tolerance for jitter.

**FusionTask:**
- Consumes sensor data, produces attitude/altitude estimates
- Can tolerate occasional delays if sensor timestamps are preserved
- Output feeds MissionTask and TelemetryTask

**MissionTask:**
- Flight state transitions (IDLE → ARMED → BOOST → COAST → etc.)
- Event detection (apogee, landing, anomalies)
- Must be responsive but not sample-rate critical

### Priority 3: Buffered I/O

Tasks that buffer data and can catch up after delays.

**LoggerTask:**
- Writes to flash/SD in bursts
- Double-buffering allows sensor sampling to continue during writes
- Occasional delays acceptable if buffer doesn't overflow

### Priority 2: Background Communication

Tasks with relaxed timing, typically limited by external factors.

**TelemetryTask:**
- Radio TX is slow compared to CPU (LoRa: ~250kbps max)
- 10Hz update rate is sufficient for ground station display
- Packet loss is expected; protocol handles retransmission

### Priority 1: User Interface

Best-effort tasks that can be preempted freely.

**UITask:**
- LED animations, button polling, display updates
- Humans can't perceive sub-100ms delays
- Must never block higher-priority tasks

**CLITask:**
- USB serial command processing
- Only active during ground testing/configuration
- Inherently interactive (human typing speed)

---

## Core Affinity Strategy

### Core 0: Real-Time Core

Reserved for timing-critical tasks:
- SensorTask
- ControlTask

**Rules:**
- Minimal ISR usage (prefer DMA)
- No blocking I/O (flash, USB)
- No printf or logging from this core

### Core 1: Application Core

All other processing:
- FusionTask, MissionTask, LoggerTask, TelemetryTask, UITask, CLITask

**Rules:**
- Can perform blocking operations
- Handles all I/O (flash, radio, USB)
- May have higher aggregate CPU usage

---

## Priority Inversion Prevention

### Known Risks

1. **Shared sensor data:** SensorTask (P5) writes, FusionTask (P4) reads
   - **Mitigation:** Lock-free ring buffer or brief mutex with priority inheritance

2. **Flash access:** LoggerTask (P3) holds flash mutex, MissionTask (P4) may need storage
   - **Mitigation:** Separate storage pools or queue-based access

3. **USB CDC:** CLITask (P1) uses USB, but TinyUSB IRQs run at hardware priority
   - **Mitigation:** USB I/O only from designated task, never from high-priority tasks

### FreeRTOS Configuration

```c
// Enable priority inheritance for mutexes
#define configUSE_MUTEXES                1
#define configUSE_RECURSIVE_MUTEXES      1

// Recommend using xSemaphoreCreateMutex() over binary semaphores
// for shared resources to get automatic priority inheritance
```

---

## Adding New Tasks

When adding a new task, determine priority by answering:

1. **What happens if this task misses its deadline?**
   - Vehicle crash/safety issue → Priority 5
   - Data quality degradation → Priority 4
   - Delayed logging/comms → Priority 2-3
   - User annoyance → Priority 1

2. **What is the task's rate requirement?**
   - >100Hz → Likely Priority 4-5
   - 10-100Hz → Likely Priority 3-4
   - <10Hz → Likely Priority 1-2

3. **Does this task do blocking I/O?**
   - Yes → Core 1, Priority ≤3
   - No → Core assignment flexible

4. **Does this task need to preempt existing tasks?**
   - If yes, must be higher priority than those tasks
   - Consider whether those tasks can tolerate preemption

---

## Implementation Checklist

When implementing a new task:

- [ ] Assign priority based on criteria above
- [ ] Set core affinity if required (`vTaskCoreAffinitySet()`)
- [ ] Size stack appropriately (use `uxTaskGetStackHighWaterMark()` to tune)
- [ ] Document in this table
- [ ] Update SAD.md Section 5 if task is part of core architecture
- [ ] Verify no priority inversion with existing tasks

---

## Future Considerations

### Dynamic Priority

Some tasks may benefit from dynamic priority adjustment:
- **ControlTask:** Only active during BOOST phase; could be suspended otherwise
- **TelemetryTask:** Higher priority during descent for recovery beacon

### Watchdog Integration

- Each task should kick a per-task watchdog flag
- Supervisor task at Priority 1 monitors all flags
- System watchdog reset if any critical task stops responding

---

## References

- FreeRTOS Task Priorities: https://www.freertos.org/RTOS-task-priority.html
- FreeRTOS SMP on RP2350: https://www.freertos.org/symmetric-multiprocessing-introduction.html
- `docs/SAD.md` Section 5: Task Architecture

---

*This document is intentionally sparse. Expand as tasks are implemented.*
