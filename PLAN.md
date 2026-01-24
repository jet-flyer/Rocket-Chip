# AP_HAL_RP2350 Phase 2 Implementation Plan

## Overview

Convert RocketChip HAL drivers to AP_HAL interfaces. This enables ArduPilot libraries to work unchanged while using our proven RocketChip HAL implementations underneath.

**Architecture:**
```
ArduPilot Libraries (AP_AHRS, etc.)
        ↓
AP_HAL Interfaces (AP_HAL::GPIO, AP_HAL::UARTDriver, etc.)
        ↓
AP_HAL_RP2350 Implementations (lib/ap_compat/AP_HAL_RP2350/)
        ↓
RocketChip HAL (src/hal/) ← proven, tested code
        ↓
Pico SDK + FreeRTOS
```

## Implementation Order

Starting with simpler components, progressing to those with platform caveats:

1. **GPIO** - Straightforward wrapper, document E9 erratum
2. **AnalogIn** - Simple ADC wrapper, handle E9 erratum
3. **UARTDriver** - More complex but no platform issues
4. **I2CDevice** - Needs timeout handling (PD7)
5. **SPIDevice** - Polling-only due to DMA issues (PD8)
6. **RCOutput** - Lower priority (TVC not in current scope)

---

## Component 1: GPIO

### Files to Create
- `lib/ap_compat/AP_HAL_RP2350/GPIO.h`
- `lib/ap_compat/AP_HAL_RP2350/GPIO.cpp`

### Stub to Replace
- `lib/ap_compat/AP_HAL/GPIO.h` - Replace with forward to real impl

### Interface Mapping

| AP_HAL::GPIO | RocketChip hal::GPIO |
|--------------|---------------------|
| `init()` | No-op (SDK init handled elsewhere) |
| `pinMode(pin, output)` | `GPIO::pinMode(pin, mode)` |
| `read(pin)` | `GPIO::readBool(pin)` |
| `write(pin, value)` | `GPIO::write(pin, value)` |
| `toggle(pin)` | `GPIO::toggle(pin)` |
| `usb_connected()` | `stdio_usb_connected()` |
| `channel(n)` | Create `DigitalSource` wrapper |
| `attach_interrupt()` | `GPIO::attachInterrupt()` |

### DigitalSource Implementation
Wrap a single pin for AP_HAL's object-oriented GPIO access.

### Platform Caveat (PD9)
Document in header: "RP2350-E9 erratum - floating pins may latch at ~2V. Use external pull-downs on critical inputs."

---

## Component 2: AnalogIn

### Files to Create
- `lib/ap_compat/AP_HAL_RP2350/AnalogIn.h`
- `lib/ap_compat/AP_HAL_RP2350/AnalogIn.cpp`

### Stub to Replace
- `lib/ap_compat/AP_HAL/AnalogIn.h` - Replace with forward

### Interface Mapping

| AP_HAL::AnalogIn | RocketChip hal::ADC |
|------------------|---------------------|
| `init()` | `ADC::init()` |
| `channel(n)` | Create `AnalogSource` for channel |
| `board_voltage()` | Return 3.3V (or measure VSYS if available) |

| AP_HAL::AnalogSource | RocketChip hal::ADC |
|---------------------|---------------------|
| `read_average()` | `ADC::readAveraged(channel)` |
| `read_latest()` | `ADC::readRaw(channel)` |
| `voltage_average()` | `ADC::readAveragedVoltage(channel)` |
| `voltage_latest()` | `ADC::readVoltage(channel)` |
| `set_pin(p)` | Store pin, call `ADC::configurePin()` |

### Platform Caveat (PD9)
Before ADC read, disable GPIO input on the pin to avoid E9 erratum leakage:
```cpp
gpio_set_input_enabled(pin, false);  // Disable digital input
uint16_t raw = ADC::readRaw(channel);
```

---

## Component 3: UARTDriver

### Files to Create
- `lib/ap_compat/AP_HAL_RP2350/UARTDriver.h`
- `lib/ap_compat/AP_HAL_RP2350/UARTDriver.cpp`

### Stub to Replace
- `lib/ap_compat/AP_HAL/UARTDriver.h` - Replace with forward

### Design
Create multiple UARTDriver instances:
- `serial[0]` - USB CDC (wraps `USBSerial`)
- `serial[1]` - UART0 (hardware UART)
- `serial[2]` - UART1 (hardware UART)

### Interface Mapping (Protected Backend Methods)

| AP_HAL::UARTDriver | RocketChip |
|-------------------|------------|
| `_begin(baud, rx, tx)` | `UART::begin()` or `USBSerial::begin()` |
| `_write(buf, size)` | `UART::write()` / `USBSerial::write()` |
| `_read(buf, count)` | `UART::read()` / `USBSerial::read()` |
| `_available()` | `UART::available()` / `USBSerial::available()` |
| `_flush()` | `UART::flush()` / `USBSerial::flush()` |
| `_end()` | Cleanup |
| `_discard_input()` | `UART::clear()` |
| `is_initialized()` | `UART::isInitialized()` |
| `tx_pending()` | Check TX buffer |

### BetterStream Base
UARTDriver inherits from BetterStream. Base class provides `printf()`, `vprintf()` using virtual `write()`.

### Notes
- Port locking handled by base class (lock_read_key, lock_write_key)
- USB CDC: baud rate ignored, but store for `get_baud_rate()`
- No flow control on RP2350 UARTs initially

---

## Component 4: I2CDevice

### Files to Create
- `lib/ap_compat/AP_HAL_RP2350/I2CDevice.h`
- `lib/ap_compat/AP_HAL_RP2350/I2CDevice.cpp`

### Design
- `I2CDeviceManager` - Creates/manages I2CDevice instances
- `I2CDevice_RP2350` - Wraps `rocketchip::hal::I2CBus`

### Interface Mapping

| AP_HAL::I2CDevice | RocketChip hal::I2CBus |
|-------------------|------------------------|
| `transfer(send, send_len, recv, recv_len)` | Combination of `write()` + `read()` |
| `set_speed(HIGH/LOW)` | Store speed, apply on next transfer |
| `read_registers_multiple()` | Loop calling `readRegisters()` |
| `get_semaphore()` | Return FreeRTOS mutex wrapper |
| `register_periodic_callback()` | Use Scheduler timer system |

### Platform Caveat (PD7)
I2C clock stretching intolerance - use longer timeouts:
```cpp
// Default timeout_ms = 4 in ArduPilot
// We should default to 10ms for RP2350
static constexpr uint32_t kDefaultI2CTimeoutMs = 10;
```

For sensors known to stretch clocks (BNO055), recommend SPI or lower I2C frequency (100kHz).

### I2CDeviceManager
```cpp
I2CDevice* get_device_ptr(uint8_t bus, uint8_t address,
                          uint32_t bus_clock = 400000,
                          bool use_smbus = false,
                          uint32_t timeout_ms = 10);  // Extended default
```

---

## Component 5: SPIDevice

### Files to Create
- `lib/ap_compat/AP_HAL_RP2350/SPIDevice.h`
- `lib/ap_compat/AP_HAL_RP2350/SPIDevice.cpp`

### Design
- `SPIDeviceManager` - Creates/manages SPIDevice instances by name
- `SPIDevice_RP2350` - Wraps `rocketchip::hal::SPIBus`

### Interface Mapping

| AP_HAL::SPIDevice | RocketChip hal::SPIBus |
|-------------------|------------------------|
| `transfer(send, send_len, recv, recv_len)` | `writeRegisters()` + `readRegisters()` |
| `transfer_fullduplex(send, recv, len)` | Direct SPI transfer |
| `set_speed(HIGH/LOW)` | Reconfigure SPI frequency |
| `get_semaphore()` | Return FreeRTOS mutex wrapper |
| `register_periodic_callback()` | Use Scheduler timer system |

### Platform Caveat (PD8) - CRITICAL
**SPI+DMA stops after ~253 cycles.** Implementation MUST use polling only:
```cpp
// DO NOT USE DMA for SPI transfers
// Use blocking spi_write_read_blocking() from Pico SDK
```

Document this limitation prominently. DMA can be added later with watchdog if proven reliable.

### Device Naming
ArduPilot uses string names for SPI devices. Create mapping:
```cpp
// hwdef.h style mapping
struct SPIDeviceDesc {
    const char* name;
    uint8_t bus;
    uint8_t cs_pin;
    uint32_t freq_hz;
    SPIBus::Mode mode;
};

static constexpr SPIDeviceDesc kSPIDevices[] = {
    {"imu0", 1, 9, 10000000, SPIBus::Mode::MODE_3},   // ISM330DHCX
    {"mag0", 1, 10, 10000000, SPIBus::Mode::MODE_3},  // LIS3MDL
    {"baro0", 1, 6, 10000000, SPIBus::Mode::MODE_3},  // DPS310
};
```

---

## Component 6: RCOutput (Lower Priority)

### Files to Create
- `lib/ap_compat/AP_HAL_RP2350/RCOutput.h`
- `lib/ap_compat/AP_HAL_RP2350/RCOutput.cpp`

### Design
Wrap `rocketchip::hal::PwmManager` for servo/ESC output.

### Platform Caveat (PD4)
PWM timer sharing on adjacent pins - document in hwdef.h which pins can have independent frequencies.

### Scope
Implement basic PWM output. DShot/OneShot protocols are stretch goals.

---

## Stub Header Cleanup

After implementing each component, update the stub in `lib/ap_compat/AP_HAL/`:

**Before (stub):**
```cpp
// Stub - GPIO.h
#pragma once
namespace AP_HAL {
class GPIO {};
class DigitalSource {};
}
```

**After (forward):**
```cpp
// Forward to real implementation
#pragma once
#include "../AP_HAL_RP2350/GPIO.h"
```

---

## HAL Singleton Integration

Update `HAL_RP2350_Class.h` to include new subsystems:

```cpp
class HAL_RP2350 {
public:
    // Phase 1 (existing)
    Scheduler scheduler;
    Util util;
    Storage storage;

    // Phase 2 (new)
    GPIO_RP2350 gpio;
    AnalogIn_RP2350 analogin;
    UARTDriver_RP2350* serial[3];  // USB, UART0, UART1
    I2CDeviceManager_RP2350 i2c_mgr;
    SPIDeviceManager_RP2350 spi_mgr;
    RCOutput_RP2350 rcout;
};
```

---

## Testing Strategy

### Incremental Testing (Test Each Component Immediately)

Each component is tested **before** moving to the next. Add tests to `tests/smoke_tests/smoke_phase2.cpp` incrementally:

| After Completing | Test | Pass Criteria |
|------------------|------|---------------|
| GPIO | Toggle onboard LED, print USB connected status | LED blinks, USB status correct |
| AnalogIn | Read internal temperature sensor | Temperature ~25-35C (reasonable room temp) |
| UARTDriver | USB CDC echo test (type chars, see them back) | Characters echo correctly |
| I2CDevice | Probe I2C bus for known sensor (if connected) | Device responds at expected address |
| SPIDevice | Read WHO_AM_I from ISM330DHCX | Returns 0x6B (ISM330DHCX ID) |
| RCOutput | Generate 1.5ms pulse on servo pin | Scope/servo shows center position |

### Build Verification Points

After each component:
```bash
# Must pass before proceeding
cmake --build build --target smoke_phase2
# Flash and verify on hardware
```

### Integration Test
After all components complete, modify existing `smoke_ap_hal` to exercise Phase 2 components alongside Phase 1.

---

## Files Summary

### New Files to Create
```
lib/ap_compat/AP_HAL_RP2350/
├── GPIO.h
├── GPIO.cpp
├── AnalogIn.h
├── AnalogIn.cpp
├── UARTDriver.h
├── UARTDriver.cpp
├── I2CDevice.h
├── I2CDevice.cpp
├── SPIDevice.h
├── SPIDevice.cpp
├── RCOutput.h          (lower priority)
└── RCOutput.cpp        (lower priority)
```

### Files to Modify
```
lib/ap_compat/AP_HAL/
├── GPIO.h              (replace stub with forward)
├── AnalogIn.h          (replace stub with forward)
├── UARTDriver.h        (replace stub with forward)

lib/ap_compat/AP_HAL_RP2350/
├── HAL_RP2350_Class.h  (add new members)
├── HAL_RP2350_Class.cpp (init new subsystems)
└── hwdef.h             (add SPI device table)

tests/smoke_tests/
└── smoke_phase2.cpp    (new test file)
```

---

## Estimated Scope

| Component | Complexity | Key Challenge |
|-----------|------------|---------------|
| GPIO | Low | E9 erratum documentation |
| AnalogIn | Low | E9 erratum handling in code |
| UARTDriver | Medium | BetterStream inheritance, buffer management |
| I2CDevice | Medium | Timeout handling, semaphore integration |
| SPIDevice | Medium | Device naming, polling-only constraint |
| RCOutput | Medium | PWM slice awareness |

---

## Success Criteria

Phase 2 is complete when:
1. All smoke tests pass on hardware
2. Existing sensor drivers can be ported to use AP_HAL interfaces
3. No regression in Phase 1 functionality
4. Platform caveats (PD7, PD8, PD9) are handled or documented
