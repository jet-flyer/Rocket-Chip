# RocketChip Hardware Abstraction Layer (HAL)

Platform-independent interfaces for RP2350/Pico SDK with FreeRTOS integration.

## Modules

### HAL.h
Master header and initialization. Include this to get all HAL functionality.
- `initHAL()` - Bootstrap all subsystems
- `getPlatformInfo()` - Chip/board/clock info
- `getResetReason()` - Why the system last reset

### Bus.h
I2C and SPI bus abstractions.
- `I2CBus` - I2C with probe/read/write, register helpers (400kHz/1MHz)
- `SPIBus` - SPI with configurable clock, mode, bit order
- `SensorBus` - Base class for sensor drivers

### GPIO.h
Digital I/O with interrupts.
- `OutputPin` - Digital output with set/clear/toggle
- `InputPin` - Digital input with pull-up/down configuration
- `InterruptPin` - Edge-triggered interrupts with callbacks
- `DebouncedInput` - Hardware debouncing for buttons/switches

### ADC.h
Analog input (12-bit).
- `ADC` - Raw/voltage readings, averaging, internal temperature
- `BatteryMonitor` - Voltage divider support, SOC estimation
- `PyroContinuity` - Ematch continuity checking

### PWM.h
PIO-based PWM for servos and ESCs.
- `PwmManager` - State machine allocation, channel configuration
- `Servo` - Angle/position control with presets
- `ESC` - Throttle control with arm/disarm safety

Presets: `SERVO_STANDARD`, `SERVO_DIGITAL`, `ESC_STANDARD`, `ESC_FAST`, `ESC_ONESHOT125`

### Timing.h
Microsecond/millisecond timing.
- `Timing` - `micros()`, `millis()`, delays
- `IntervalTimer` - Periodic task scheduling
- `StopWatch` - Execution time measurement
- `RateStats` - Sample rate tracking
- `ScopedTimer` - RAII profiling

### PIO.h
Programmable I/O utilities.
- `WS2812` - NeoPixel driver (CPU-offloaded timing)
- `StatusLED` - Predefined patterns (idle, armed, flight, error)
- `PIOManager` - State machine allocation
- `RGB` - Color struct with utilities

### UART.h
Serial communication.
- `UART` - Hardware UART with buffered I/O
- `USBSerial` - TinyUSB CDC wrapper (when enabled)

Presets: `DEBUG`, `GPS_DEFAULT`, `GPS_FAST`, `MAVLINK`

## Usage

```cpp
#include "HAL.h"

using namespace rocketchip::hal;

int main() {
    // Initialize all HAL subsystems
    HALInitResult result = initHAL();
    if (!result.success) {
        // Handle error
    }

    // Use HAL components
    OutputPin led(13, PinState::LOW);
    led.set();

    I2CBus imu(i2c0, 0x6A, 2, 3, 400000);
    imu.begin();

    // ...
}
```

## Build

The HAL is built as a static library (`libhal.a`) and linked automatically when using the `hal` target in CMakeLists.txt:

```cmake
target_link_libraries(your_target hal)
```
