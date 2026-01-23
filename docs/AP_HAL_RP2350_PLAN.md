# AP_HAL_RP2350 Implementation Plan

> **Multi-conversation tracking document** - Claude Code uses this to maintain context across sessions.
> Last updated: 2026-01-22

## Executive Summary

Implement `AP_HAL` for RP2350 to enable use of ArduPilot libraries unchanged. First deliverable is **CalibrationStore** for persistent sensor calibration, which requires Scheduler, Semaphores, Util, and Storage as prerequisites.

**Architecture:**
```
ArduPilot Libraries (unchanged) → AP_HAL Interface → AP_HAL_RP2350 (our impl) → Pico SDK + FreeRTOS
```

## Current Status

| Phase | Component | Status | Notes |
|-------|-----------|--------|-------|
| 1 | Scheduler | **VALIDATED** | FreeRTOS task/timer mapping, callbacks |
| 1 | Semaphores | **VALIDATED** | Mutex + BinarySemaphore wrappers |
| 1 | Util | **VALIDATED** | Memory info, system ID, arming state |
| 1 | HAL Singleton | **VALIDATED** | HAL_RP2350 class, global `hal` instance |
| 1 | Storage | NOT STARTED | Flash persistence via AP_FlashStorage |
| 1 | CalibrationStore | NOT STARTED | Depends on Storage |
| 2 | I2CDevice | NOT STARTED | Full rewrite (replaces Bus::I2C) |
| 2 | SPIDevice | NOT STARTED | Full rewrite (replaces Bus::SPI) |
| 2 | UARTDriver | NOT STARTED | USB CDC + hardware UARTs |
| 2 | GPIO | NOT STARTED | Digital I/O |
| 2 | AnalogIn | NOT STARTED | ADC channels |
| 2 | RCOutput | NOT STARTED | PWM output |
| 3 | AP_Notify | NOT STARTED | NeoPixel backend |

## Decisions (Resolved 2026-01-22)

### D1: Migration Strategy for `ap_compat`

**Decision:** **(A) Replace** - Delete `ap_compat` entirely, create fresh `lib/AP_HAL_RP2350/`

No temporary fixes, no parallel abstractions. The old shim layer will be removed once AP_HAL_RP2350 is functional.

### D2: Sensor Driver Conversion Scope

**Decision:** **(A) Convert all** - Rewrite all sensor drivers to use `AP_HAL::I2CDevice`/`AP_HAL::SPIDevice`

Per the plan: "Don't keep parallel abstractions (old `Bus` AND new `I2CDevice`)". The existing `Bus::I2C` and `Bus::SPI` classes will be deleted once conversion is complete.

### D3: ArduPilot Library Scope

**Decision:** Follow the plan's library list. Investigate dependencies as encountered.

**Phase 1:** AP_FlashStorage, StorageManager
**Phase 3:** AP_Notify
**Future:** EKF3, AP_InertialSensor (address dependencies when we get there)

---

## Directory Structure (Proposed)

```
lib/
├── ardupilot/                    # Unchanged AP code (versioned copy)
│   ├── libraries/
│   │   ├── AP_HAL/               # Interface headers only
│   │   ├── AP_FlashStorage/
│   │   ├── StorageManager/
│   │   ├── AP_Math/              # Already present
│   │   ├── Filter/               # Already present
│   │   ├── AP_AccelCal/          # Already present
│   │   └── AP_Notify/
│   └── VERSION.txt               # Track AP commit hash
│
├── AP_HAL_RP2350/                # Our HAL implementation
│   ├── AP_HAL_RP2350.h           # Main include
│   ├── HAL_RP2350_Class.h/.cpp   # HAL singleton
│   ├── Scheduler.h/.cpp          # FreeRTOS task/timer mapping
│   ├── UARTDriver.h/.cpp         # USB CDC, UART0, UART1
│   ├── I2CDevice.h/.cpp          # I2C bus
│   ├── SPIDevice.h/.cpp          # SPI bus
│   ├── GPIO.h/.cpp               # Digital I/O
│   ├── AnalogIn.h/.cpp           # ADC
│   ├── RCOutput.h/.cpp           # PWM/servo output
│   ├── Storage.h/.cpp            # Flash persistence
│   ├── Util.h/.cpp               # System utilities
│   ├── Semaphores.h/.cpp         # FreeRTOS semaphore wrapper
│   └── hwdef.h                   # Board pin/peripheral mapping
│
└── ap_compat/                    # Keep for feature flags & board defs
    └── (existing files)

src/hal/                          # RocketChip-specific (no AP equivalent)
├── PIO.h/.cpp                    # Keep: No AP equivalent
├── NeoPixel_PIO.h/.cpp          # Keep: Backend for AP_Notify
├── BoardDetect.h/.cpp           # Keep: Runtime board detection
└── (existing files)             # Keep until migration complete
```

---

## Implementation Plan

### Phase 1: Core HAL + Storage

**Goal:** CalibrationStore working with persistent storage

#### Step 1.1: Scheduler

Implement `AP_HAL::Scheduler` wrapping FreeRTOS:

| AP_HAL Method | FreeRTOS Implementation |
|---------------|------------------------|
| `delay()` | `vTaskDelay()` |
| `delay_microseconds()` | Hardware timer busy-wait |
| `millis()` | `xTaskGetTickCount()` conversion |
| `micros()` | Hardware timer read |
| `register_timer_process()` | FreeRTOS software timer |
| `register_io_process()` | Low-priority task callback |
| `in_main_thread()` | Check current task handle |
| `system_initialized()` | Static flag |

**Files to create:**
- `lib/AP_HAL_RP2350/Scheduler.h`
- `lib/AP_HAL_RP2350/Scheduler.cpp`

**Tests:**
- Timer callbacks fire at correct rate
- delay() yields properly
- micros() has microsecond resolution

#### Step 1.2: Semaphores

Implement `AP_HAL::Semaphore` wrapping FreeRTOS:

| AP_HAL Method | FreeRTOS Implementation |
|---------------|------------------------|
| `take()` | `xSemaphoreTake(handle, timeout)` |
| `take_nonblocking()` | `xSemaphoreTake(handle, 0)` |
| `give()` | `xSemaphoreGive(handle)` |

**Files to modify:**
- `lib/ap_compat/AP_HAL_Compat.h` - Replace stub with real impl
OR
- `lib/AP_HAL_RP2350/Semaphores.h/.cpp` - New files

**Tests:**
- Mutex protects critical section
- Take with timeout returns false after timeout

#### Step 1.3: Util

Implement `AP_HAL::Util`:

| AP_HAL Method | RP2350 Implementation |
|---------------|----------------------|
| `available_memory()` | `xPortGetFreeHeapSize()` |
| `get_soft_armed()` | Static flag |
| `safety_switch_state()` | Return SAFETY_ARMED (no switch) |

**Files to create:**
- `lib/AP_HAL_RP2350/Util.h`
- `lib/AP_HAL_RP2350/Util.cpp`

#### Step 1.4: Storage

Implement `AP_HAL::Storage` using Pico SDK flash:

**Key constraints:**
- RP2350 flash: 8MB total, erase size 4KB
- Reserve last N sectors for storage
- Wear leveling via AP_FlashStorage library

**Design:**
```cpp
class Storage : public AP_HAL::Storage {
public:
    void read_block(void *dst, uint16_t loc, size_t n) override;
    void write_block(uint16_t loc, const void *src, size_t n) override;

private:
    AP_FlashStorage flash_storage;  // Handles wear leveling
    uint8_t buffer[HAL_STORAGE_SIZE];  // RAM mirror
    bool dirty;
};
```

**Files to create:**
- `lib/AP_HAL_RP2350/Storage.h`
- `lib/AP_HAL_RP2350/Storage.cpp`

**Files to copy from ArduPilot:**
- `libraries/AP_FlashStorage/AP_FlashStorage.h`
- `libraries/AP_FlashStorage/AP_FlashStorage.cpp`

**Tests:**
- Write data, reboot, verify read
- Multiple writes don't corrupt
- Power-loss safety (verify via deliberate reset)

#### Step 1.5: HAL Singleton

Create HAL singleton that ties everything together:

```cpp
class HAL_RP2350 : public AP_HAL::HAL {
public:
    HAL_RP2350();
    void init();

    // Subsystem pointers (set in constructor)
    Scheduler scheduler;
    UARTDriver serial0;  // USB CDC
    UARTDriver serial1;  // UART0
    // ... etc
};

extern HAL_RP2350 hal;  // Global instance
```

**Files to create:**
- `lib/AP_HAL_RP2350/HAL_RP2350_Class.h`
- `lib/AP_HAL_RP2350/HAL_RP2350_Class.cpp`
- `lib/AP_HAL_RP2350/AP_HAL_RP2350.h` (main include)

#### Step 1.6: CalibrationStore

RocketChip wrapper using AP Storage:

```cpp
class CalibrationStore {
public:
    bool load(SensorCalibration& cal);
    bool save(const SensorCalibration& cal);

private:
    static constexpr uint16_t STORAGE_OFFSET = 0;
    static constexpr uint32_t MAGIC = 0x52434341;  // "RCCA"
};
```

**Files to create:**
- `src/calibration/CalibrationStore.h`
- `src/calibration/CalibrationStore.cpp`

**Tests (smoke_calibration):**
- Save calibration, verify magic
- Reboot, load calibration, verify values match
- Corrupt magic, verify load returns false

---

### Phase 2: Bus & Peripherals

#### Step 2.1: I2CDevice

Implement `AP_HAL::I2CDevice` using existing `Bus::I2C`:

```cpp
class I2CDevice : public AP_HAL::I2CDevice {
public:
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;
    AP_HAL::Semaphore *get_semaphore() override;

private:
    rocketchip::hal::I2CBus* bus;  // Reuse existing impl
    Semaphore sem;
};
```

#### Step 2.2: SPIDevice

Same pattern as I2CDevice, wrap `Bus::SPI`.

#### Step 2.3: UARTDriver

Implement `AP_HAL::UARTDriver` for:
- `serial0`: USB CDC
- `serial1`: UART0 (pins TX=0, RX=1)
- `serial2`: UART1 (if needed)

#### Step 2.4: GPIO, AnalogIn, RCOutput

Thin wrappers around existing HAL implementations.

---

### Phase 3: Notifications

#### Step 3.1: NeoPixel Backend for AP_Notify

```cpp
class NeoPixel_PIO : public AP_Notify::NotifyDevice {
public:
    bool init() override;
    void update() override;  // Called by AP_Notify

private:
    void set_color(uint8_t r, uint8_t g, uint8_t b);
    // Reuse existing PIO WS2812 code
};
```

Register with AP_Notify; get state-based LED patterns automatically.

---

## Conversion Mapping Reference

| Current `src/hal/` | AP_HAL Equivalent | Migration Action |
|-------------------|-------------------|------------------|
| `Bus::I2C` | `AP_HAL::I2CDevice` | Wrap or replace |
| `Bus::SPI` | `AP_HAL::SPIDevice` | Wrap or replace |
| `GPIO` | `AP_HAL::GPIO` | Thin wrapper |
| `ADC` | `AP_HAL::AnalogIn` | Thin wrapper |
| `PWM` | `AP_HAL::RCOutput` | Thin wrapper |
| `Timing` | `AP_HAL::Scheduler` | Replace timing functions |
| `UART` | `AP_HAL::UARTDriver` | Wrap or replace |
| `PIO`, `StatusLED` | *Keep as-is* | No AP equivalent |

---

## ArduPilot Files to Copy

From https://github.com/ArduPilot/ardupilot (track commit hash in VERSION.txt):

**Phase 1 (Required for Storage):**
```
libraries/AP_HAL/AP_HAL.h
libraries/AP_HAL/AP_HAL_Namespace.h
libraries/AP_HAL/AP_HAL_Boards.h
libraries/AP_HAL/Scheduler.h
libraries/AP_HAL/UARTDriver.h
libraries/AP_HAL/Storage.h
libraries/AP_HAL/Util.h
libraries/AP_HAL/Semaphores.h

libraries/AP_FlashStorage/AP_FlashStorage.h
libraries/AP_FlashStorage/AP_FlashStorage.cpp
```

**Phase 2 (Required for Peripherals):**
```
libraries/AP_HAL/I2CDevice.h
libraries/AP_HAL/SPIDevice.h
libraries/AP_HAL/GPIO.h
libraries/AP_HAL/AnalogIn.h
libraries/AP_HAL/RCOutput.h
```

**Phase 3 (Required for Notifications):**
```
libraries/AP_Notify/* (subset TBD)
```

---

## Standards Checklist

- [ ] AP code in `lib/ardupilot/` copied verbatim—never modify
- [ ] HAL implements AP_HAL interfaces exactly
- [ ] No stubs or throwaway code
- [ ] RocketChip-specific code uses project naming (`kConstant`, `g_global`)
- [ ] No dynamic allocation after init
- [ ] Fixed-width types throughout
- [ ] All tests pass before marking step complete

---

## Session Log

### Session 1 (2026-01-22)

**Actions:**
- Created this tracking document
- Explored codebase structure
- Identified existing `ap_compat` layer
- **Resolved all pending decisions** (see Decisions section above)
- Fetched AP_HAL interface specifications (Scheduler, Semaphores, Util)
- Implemented Phase 1 core components:
  - `Scheduler.h/.cpp` - FreeRTOS task/timer mapping, 1kHz callbacks
  - `Semaphores.h/.cpp` - Mutex + BinarySemaphore with FreeRTOS
  - `Util.h/.cpp` - Memory info, system ID, arming state
  - `HAL_RP2350_Class.h/.cpp` - HAL singleton
  - `hwdef.h` - Board definitions and feature flags
  - `AP_HAL_RP2350.h` - Main include header
- Created `smoke_ap_hal` test target
- **Build verified:** `smoke_ap_hal.uf2` (109KB) compiles successfully

**Files created:**
```
lib/AP_HAL_RP2350/
├── AP_HAL_RP2350.h
├── HAL_RP2350_Class.h/.cpp
├── Scheduler.h/.cpp
├── Semaphores.h/.cpp
├── Util.h/.cpp
└── hwdef.h

tests/smoke_tests/
└── ap_hal_test.cpp
```

**Build output:**
- Code: 58KB
- BSS: 88KB (FreeRTOS stacks)
- Total flash: ~109KB

**Next session should:**
1. Implement Storage (AP_FlashStorage integration)
2. Implement CalibrationStore
3. ~~Hardware validation of smoke_ap_hal on actual RP2350~~ **DONE**

### Session 2 (2026-01-22)

**Actions:**
- Hardware validated all Phase 1 components on Feather RP2350
- All 7 smoke tests passing:
  - Timing (millis/micros accuracy)
  - Scheduler delays
  - Semaphores (recursive mutex)
  - Binary semaphores
  - Util functions (memory, system ID, arming)
  - Timer callbacks (1kHz timer, 100Hz I/O)
  - System state

**Fixes applied:**
- Fixed `available_memory()` for static-allocation-only config (returns `configTOTAL_HEAP_SIZE` when heap reports 0)
- Refactored smoke test to follow DEBUG_OUTPUT.md pattern (run tests immediately, store results, wait for USB, print)
- Fixed file-scope static allocations for FreeRTOS task buffers and semaphores
- Corrected LED pin in hwdef.h for Feather RP2350 (GPIO 7)

**Next session should:**
1. Implement Storage (AP_FlashStorage integration)
2. Implement CalibrationStore

---

## References

- ArduPilot AP_HAL: https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL
- AP_HAL_ChibiOS (reference impl): https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL_ChibiOS
- AP_HAL_ESP32 (non-STM32 reference): https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL_ESP32
- AP_FlashStorage: https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_FlashStorage
