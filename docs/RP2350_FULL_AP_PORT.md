# RP2350 Full ArduPilot Port - Platform Differences

**Purpose:** Track fundamental platform differences between RP2350 and STM32 (ArduPilot's primary target) that must be addressed for a full ArduPilot port.

**When to add entries:** When implementing AP_HAL_RP2350 components, document any behavior that differs from STM32 and requires platform-specific handling. These entries serve as a reference for future full-port work.

---

## Entry Format

```
### PDn: Short Title

**Category:** [Memory | Flash | Timing | Peripherals | RTOS | Other]
**Discovered:** YYYY-MM-DD
**Severity:** [Critical | High | Medium | Low]
**Status:** [Documented | Workaround | Resolved]

**STM32 Behavior:**
How ArduPilot expects this to work on STM32.

**RP2350 Behavior:**
How RP2350 actually behaves.

**Solution:**
What we did to handle this difference.

**Files Affected:**
List of files implementing the workaround/solution.

**References:**
Links to documentation, datasheets, or relevant code.

**Source URLs:**
External links where this issue was discovered or discussed (forums, GitHub issues, etc.).
```

---

## Platform Differences

### PD1: Flash Operations Disable XIP (Execute-in-Place)

**Category:** Flash
**Discovered:** 2026-01-23
**Severity:** Critical
**Status:** Resolved

**STM32 Behavior:**
Flash write/erase operations stall the CPU briefly but code can continue executing from other flash banks or from RAM. ArduPilot's `AP_FlashStorage` assumes flash operations are "blocking but safe" - the CPU waits, then continues.

**RP2350 Behavior:**
RP2350 uses XIP (Execute-in-Place) where code runs directly from flash via the XIP cache. During flash write/erase operations, the **entire flash becomes inaccessible**. Any code attempting to execute from flash will crash instantly.

**IMPORTANT UPDATE (2026-01-24):** The Pico SDK's `flash_safe_execute()` uses `multicore_lockout` internally, which **conflicts with FreeRTOS SMP**. When FreeRTOS SMP is managing both cores, calling `flash_safe_execute()` causes a hard fault because the lockout mechanism and scheduler conflict on Core 1 signaling.

**Solution:**
Use direct flash operations with interrupt disable and XIP cache invalidation:

1. **RAM-resident flash wrappers:** Use `__not_in_flash_func()` on free functions (not class methods - the attribute doesn't work on C++ member functions):
   ```cpp
   static void __not_in_flash_func(do_flash_erase)(uint32_t offset, size_t length) {
       flash_range_erase(offset, length);
       xip_cache_invalidate_all();  // SDK 2.2.0+ API
   }
   ```

2. **Disable interrupts during flash ops:**
   ```cpp
   uint32_t saved = save_and_disable_interrupts();
   do_flash_erase(offset, length);
   restore_interrupts(saved);
   ```

3. **XIP cache invalidation:** Call `xip_cache_invalidate_all()` (from `hardware/xip_cache.h`) after flash operations so subsequent reads see the new flash contents.

4. **Page alignment requirement (CRITICAL):** `flash_range_program()` requires:
   - Offset aligned to 256 bytes (FLASH_PAGE_SIZE)
   - Length must be a multiple of 256 bytes

   AP_FlashStorage writes small chunks (4-byte headers, small log entries) which causes **silent failures** if passed directly. Solution: read-modify-write full pages:
   ```cpp
   // Read existing page, modify bytes, write full page
   const uint8_t* xip_page = (const uint8_t*)(0x10000000 + page_start);
   memcpy(page_buffer, xip_page, 256);
   memcpy(page_buffer + offset_in_page, data, bytes_to_write);
   flash_range_program(page_start, page_buffer, 256);
   ```

5. **Init order in HAL:** Storage.init() is called inside hal.init() before scheduler.init() creates additional HAL tasks. This works because FreeRTOS SMP allows flash ops as long as interrupts are disabled and flash functions are in RAM.

**DO NOT USE:** `flash_safe_execute()` with FreeRTOS SMP. See `REBUILD_CONTEXT.md` for debugging history.

**Files Affected:**
- `lib/ap_compat/AP_HAL_RP2350/Storage.cpp`
- `lib/ap_compat/AP_HAL_RP2350/HAL_RP2350_Class.cpp` (init order)
- `lib/ap_compat/AP_HAL_RP2350/REBUILD_CONTEXT.md` (debugging notes)

**References:**
- Pico SDK `hardware/xip_cache.h` - `xip_cache_invalidate_all()` (SDK 2.2.0+)
- Pico SDK `hardware/flash.h` - `flash_range_erase()`, `flash_range_program()`
- RP2350 Datasheet Section 2.8 (XIP)
- ArduPilot `libraries/AP_FlashStorage/AP_FlashStorage.cpp`

**Source URLs:**
- https://forums.raspberrypi.com/viewtopic.php?t=388734 (Flash/multicore conflicts)
- https://forum.chibios.org/viewtopic.php?t=5800&start=60 (ChibiOS XIP discussion)

---

### PD2: SIO FIFO IRQ Shared Between Cores (RP2350 Only)

**Category:** RTOS
**Discovered:** 2026-01-24
**Severity:** High
**Status:** Documented

**STM32 Behavior:**
Inter-core communication on dual-core STM32 (e.g., H7 series) uses dedicated hardware mailboxes or HSEM (hardware semaphores) with separate interrupt lines per core.

**RP2350 Behavior:**
The SIO FIFO IRQ number is **identical on both cores** (differs from RP2040 which had separate IRQ numbers). This causes conflicts when:
1. Using `irq_set_exclusive_handler()` for FIFO-based inter-core communication
2. Simultaneously calling `flash_safe_execute()` which also uses SIO FIFO internally

Using both together causes hard faults. The Pico SDK docs state: "it is suggested that you do not use the FIFO for your own purposes" due to SDK functionality dependencies.

**Solution:**
Use **doorbells + software queues** for inter-core communication instead of raw SIO FIFO. Doorbells are RP2350-specific (not available on RP2040). When both cores share the same vector table, install IRQ handler once and check `get_core_num()` to determine which core received the interrupt.

**Files Affected:**
- Any future inter-core communication code
- `lib/ap_compat/AP_HAL_RP2350/Scheduler.cpp` (if adding core affinity)

**References:**
- Pico SDK `hardware/sync.h` - doorbell documentation
- RP2350 Datasheet Section 3.8 (SIO)

**Source URLs:**
- https://forums.raspberrypi.com/viewtopic.php?t=388734 (Primary source - RPi engineer confirmation)

---

### PD3: TinyUSB Expects Core 1 for USB Tasks

**Category:** RTOS
**Discovered:** 2026-01-24
**Severity:** Medium
**Status:** Documented

**STM32 Behavior:**
USB stack runs on the single core or on whichever core the application assigns it to. No implicit core affinity.

**RP2350 Behavior:**
When FreeRTOS is detected, the Pico SDK automatically attempts to run TinyUSB tasks on Core 1. If you configure FreeRTOS for single-core operation (`configNUMBER_OF_CORES 1`), TinyUSB stops working because Core 1 isn't running FreeRTOS tasks.

Additionally, high-rate USB traffic (messages every ≤4ms) can cause ISR hardfaults - this is a TinyUSB issue, not FreeRTOS.

**Solution:**
1. Keep FreeRTOS SMP enabled with both cores
2. Keep USB CDC traffic infrequent during flight (telemetry at 10Hz max)
3. Follow DEBUG_OUTPUT.md USB wait pattern for reliable startup

**Files Affected:**
- `src/main.cpp` (FreeRTOS config)
- Any telemetry/logging code using USB

**References:**
- Pico SDK `tusb_config.h` defaults
- DEBUG_OUTPUT.md USB CDC handling section

**Source URLs:**
- https://forums.raspberrypi.com/viewtopic.php?p=2305819 (ISR hardfault reports)
- https://forums.raspberrypi.com/viewtopic.php?t=389724 (FreeRTOS SMP discussion)

---

### PD4: PWM Timer Sharing on Adjacent Pins

**Category:** Peripherals
**Discovered:** 2026-01-24
**Severity:** Low
**Status:** Documented

**STM32 Behavior:**
PWM timers are explicitly configured per channel. Timer sharing is visible in CubeMX and hwdef files.

**RP2350 Behavior:**
Consecutive even/odd GPIO pins (e.g., GPIO2/3, GPIO10/11) share the same PWM slice and therefore the same timer/frequency. You cannot set independent PWM frequencies on adjacent even/odd pin pairs.

**Solution:**
When assigning PWM outputs (servos, motor ESCs), ensure channels requiring different frequencies are not on adjacent even/odd pins. Document pin assignments in hwdef.h.

**Files Affected:**
- `lib/ap_compat/AP_HAL_RP2350/hwdef.h` (pin assignments)
- Future `RCOutput.cpp`

**References:**
- RP2350 Datasheet Section 4.5 (PWM)

**Source URLs:**
- https://madflight.com/Board-RP2040/ (MadFlight documentation)

---

### PD5: Second-Stage Bootloader Requirement

**Category:** Flash
**Discovered:** 2026-01-24
**Severity:** Medium
**Status:** Resolved (Pico SDK handles)

**STM32 Behavior:**
Bootloader is optional. Code can execute directly from flash at reset vector.

**RP2350 Behavior:**
RP2040/RP2350 ROM bootloader searches the first 256 bytes of flash for a valid second-stage bootloader by verifying a checksum. If checksum fails, the chip enters USB bootloader mode. The second-stage bootloader initializes XIP hardware before jumping to application code.

Early ChibiOS ports failed because they had the wrong entry point and couldn't boot from flash - only RAM loading via SWD worked.

**Solution:**
Pico SDK handles this automatically via `boot_stage2`. The SDK's linker scripts and CMake build system include the correct bootloader. No action needed as long as we use Pico SDK build system.

**Files Affected:**
- None (handled by Pico SDK)

**References:**
- Pico SDK `boot_stage2/` directory
- RP2350 Datasheet Section 5.1 (Boot Sequence)

**Source URLs:**
- https://forum.chibios.org/viewtopic.php?t=5800&start=60 (ChibiOS flash boot struggles)
- https://forum.chibios.org/viewtopic.php?t=6034 (Stage2 bootloader discussion)

---

### PD6: No Official ChibiOS Support

**Category:** RTOS
**Discovered:** 2026-01-24
**Severity:** High
**Status:** Documented (Architecture Decision)

**STM32 Behavior:**
ArduPilot uses ChibiOS as its RTOS. Full HAL support exists for STM32F4/F7/H7 families.

**RP2350 Behavior:**
No official ChibiOS support exists for RP2350. The RP2040 ChibiOS port is incomplete:
- USB HAL not implemented ("there is not yet a driver in the HAL")
- Flash boot required community patches
- XIP execution has cache contention issues

ArduPilot's official position: "Until there is an official version of ChibiOS on Pi Pico, we won't support them."

**Solution:**
RocketChip uses FreeRTOS SMP + Pico SDK instead of ChibiOS. This is why we're building AP_HAL_RP2350 rather than waiting for upstream ArduPilot support. MadFlight and Betaflight both use this same approach successfully.

**Files Affected:**
- Entire `lib/ap_compat/AP_HAL_RP2350/` directory
- Build system (CMake + Pico SDK, not waf + ChibiOS)

**References:**
- ArduPilot ChibiOS fork: https://github.com/ArduPilot/ChibiOS

**Source URLs:**
- https://github.com/ArduPilot/ardupilot/issues/28266 (ArduPilot RP2350 request)
- https://discuss.ardupilot.org/t/pi-pico-porting-steps/125846 (Porting discussion)
- https://github.com/betaflight/betaflight/issues/13943 (Betaflight's successful approach)

---

### PD7: I2C Clock Stretching Intolerance

**Category:** Peripherals
**Discovered:** 2026-01-24
**Severity:** High
**Status:** Documented (Phase 2 - I2CDevice)

**STM32 Behavior:**
STM32 I2C peripherals handle clock stretching transparently. Sensors like BNO055 that stretch clocks during processing work without special configuration.

**RP2350 Behavior:**
The RP2040/RP2350 hardware I2C peripheral is intolerant of clock stretching. Sensors that stretch clocks (BNO055, some barometers during conversion) cause "Clock stretch too long" timeouts. Additionally, requested I2C frequencies may not be achieved - requesting 1MHz yields only ~516kHz due to clock stretching overhead.

When RP2350 is used as I2C slave, clock stretching is not enabled by default (`IC_CON.RX_FIFO_FULL_HLD_CTRL` must be set).

**Solution:**
1. Use longer timeouts when configuring I2C: `busio.I2C(SCL, SDA, frequency=400000, timeout=10000)`
2. Lower I2C frequency (40kHz-100kHz) for problematic sensors
3. Consider PIO-based software I2C for sensors requiring clock stretching
4. Our current sensors (ISM330DHCX, LIS3MDL, DPS310) use SPI - preferred path

**Files Affected:**
- Future `lib/ap_compat/AP_HAL_RP2350/I2CDevice.cpp`

**References:**
- RP2350 Datasheet Section 4.3 (I2C)
- Pico SDK `hardware/i2c.h`

**Source URLs:**
- https://github.com/micropython/micropython/issues/8167 (RP2 I2C clock stretching issue)
- https://github.com/raspberrypi/pico-sdk/issues/456 (I2C slave clock stretch default)
- https://forums.raspberrypi.com/viewtopic.php?t=376498 (RP2350 I2C frequency issues)

---

### PD8: SPI+DMA Stops After ~253 Cycles

**Category:** Peripherals
**Discovered:** 2026-01-24
**Severity:** Critical
**Status:** Documented (Phase 2 - SPIDevice)

**STM32 Behavior:**
SPI with DMA runs indefinitely without intervention. ArduPilot sensor drivers rely on continuous DMA transfers at high rates (1kHz+).

**RP2350 Behavior:**
SPI+DMA on RP2350 has been observed to stop completely after exactly 253 cycles (with 2KB buffer) or ~508 cycles (with 1KB buffer). The failure count scales proportionally with buffer size. After failure, interrupts still fire but the main loop stops executing. This occurs regardless of SPI clock frequency.

Additionally, RP2350 errata E5 requires clearing the DMA channel enable bit before abort to prevent re-triggering.

**Solution:**
1. Test extensively with actual sensor read patterns before trusting DMA
2. Consider polling-based SPI for critical sensors initially
3. Implement DMA watchdog to detect stalls and reinitialize
4. Keep DMA transfer sizes small and verify completion
5. Clear DMA enable bit before abort per errata E5

**Files Affected:**
- Future `lib/ap_compat/AP_HAL_RP2350/SPIDevice.cpp`

**References:**
- RP2350 Datasheet errata E5
- Pico SDK `hardware/dma.h`

**Source URLs:**
- https://forums.raspberrypi.com/viewtopic.php?t=385042 (SPI+DMA stops after 253 cycles)
- https://github.com/tinygo-org/tinygo/issues/4690 (TinyGo SPI DMA issue)

---

### PD9: RP2350-E9 Erratum (GPIO/ADC Leakage)

**Category:** Peripherals
**Discovered:** 2026-01-24
**Severity:** High
**Status:** Documented (Phase 2 - AnalogIn, GPIO)

**STM32 Behavior:**
GPIO inputs with internal pull-downs enabled behave predictably. ADC readings are stable with high-impedance sources.

**RP2350 Behavior:**
RP2350-E9 erratum causes GPIO pins to get "stuck" at ~2V even without pulldown enabled. When a floating input is driven high and then released, it remains stuck high until grounded directly. This affects:
1. **ADC accuracy**: Pins can latch to intermediate voltages
2. **Digital inputs**: Floating pins may read unpredictably
3. **High-impedance sources**: ~100µA leakage current affects readings

ADC also has INL/DNL (linearity) issues with jumps at multiples of 512 (at 12-bit resolution).

**Solution:**
1. Use external pull-downs (4.7kΩ) on critical GPIO inputs - costs 0.7mA when high
2. For ADC: use low-impedance sources or add buffer amplifier
3. For ADC: use external voltage reference (e.g., LM4040) to improve noise
4. Clear GPIO input enable when using ADC (`gpio_set_input_enabled(pin, false)`)
5. Never rely on floating pins or internal pull-downs alone

**Files Affected:**
- Future `lib/ap_compat/AP_HAL_RP2350/AnalogIn.cpp`
- Future `lib/ap_compat/AP_HAL_RP2350/GPIO.cpp`

**References:**
- RP2350 Datasheet Erratum RP2350-E9
- RP2350 Datasheet Section 4.9 (ADC)

**Source URLs:**
- https://hackaday.com/2024/09/04/the-worsening-raspberry-pi-rp2350-e9-erratum-situation/
- https://github.com/raspberrypi/pico-feedback/issues/401 (E9 stuck at 2V)
- https://github.com/earlephilhower/arduino-pico/issues/2534 (ADC INL/DNL)

---

### PD10: AP_InertialSensor Backend Rate Requirements

**Category:** Timing
**Discovered:** 2026-01-24
**Severity:** Medium
**Status:** Documented (Phase 4 - AP_InertialSensor)

**STM32 Behavior:**
ChibiOS provides deterministic timing for 1kHz+ sensor polling. DMA transfers complete reliably.

**RP2350 Behavior:**
ArduPilot's AP_InertialSensor requires gyro backend rate ≥ 1.8× main loop rate. With a 400Hz main loop, this means 720Hz+ gyro sampling. Pre-arm checks will fail if this isn't met: "Gyro rate < loop rate*1.8".

Combined with PD8 (SPI+DMA stops), this creates risk for high-rate sensor polling.

**Solution:**
1. Validate gyro sample rates early in Phase 4 development
2. Use SPI polling initially until DMA is proven reliable
3. Consider lower main loop rates (200Hz) if timing is unstable
4. ISM330DHCX supports up to 6.66kHz ODR - verify we can actually read that fast

**Files Affected:**
- Future AP_InertialSensor integration
- `lib/ap_compat/AP_HAL_RP2350/SPIDevice.cpp`

**References:**
- ArduPilot `libraries/AP_InertialSensor/AP_InertialSensor.cpp`

**Source URLs:**
- https://ardupilot.org/dev/docs/code-overview-sensor-drivers.html

---

### PD11: Memory Allocation Must Zero (ESP32 HAL Lesson)

**Category:** Memory
**Discovered:** 2026-01-24
**Severity:** Medium
**Status:** Documented (All Phases)

**STM32 Behavior:**
ArduPilot on ChibiOS uses `chibios_malloc` which may or may not zero memory. Code often assumes zeroed memory.

**RP2350 Behavior:**
Standard `malloc()` does not zero memory. ArduPilot libraries may assume zeroed allocations.

The ESP32 HAL team learned this lesson: "malloc needs to be over-ridden so that it zero's all memory - changed to use calloc."

**Solution:**
1. Override `malloc()` to use `calloc()` or zero memory after allocation
2. Alternatively, ensure all ArduPilot library allocations happen at init time when memory is fresh
3. Review any ArduPilot code that uses malloc without initialization

**Files Affected:**
- `lib/ap_compat/AP_HAL_RP2350/Util.cpp` (memory hooks)
- FreeRTOS heap configuration

**References:**
- ArduPilot `libraries/AP_HAL_ESP32/README.md`

**Source URLs:**
- https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ESP32/README.md

---

### PD12: Multi-Core Memory Visibility (FreeRTOS SMP)

**Category:** RTOS
**Discovered:** 2026-01-28
**Severity:** Critical
**Status:** Resolved

**STM32 Behavior:**
ArduPilot runs on single-core STM32 processors. Memory writes by one task are immediately visible to other tasks. Code patterns like setting a flag in a callback and checking it in a polling loop work without explicit synchronization beyond the semaphore protecting the write.

Example: `AP_InertialSensor::wait_for_sample()` checks `_new_gyro_data[i]` in a loop. The callback sets this flag inside `WITH_SEMAPHORE`, but the polling loop reads it WITHOUT holding the semaphore. On single-core, this works because all threads share the same core's cache.

**RP2350 Behavior:**
RP2350 is dual-core Cortex-M33. With FreeRTOS SMP, tasks can migrate between cores or run simultaneously on different cores. Memory writes on Core 0 may not be visible to Core 1 without explicit memory barriers or `volatile` access.

Symptoms observed:
- Callback runs successfully (verified via debug logs)
- Callback sets `_new_gyro_data = true` inside semaphore
- Polling loop on different core never sees the flag as true
- `AP_InertialSensor::init()` hangs indefinitely despite gyro registered (count=1, rate=1125Hz)

The ESP32 HAL (also dual-core) solves this by pinning related tasks to the same core using `xTaskCreatePinnedToCore()`. This sidesteps the visibility issue entirely.

**Solution:**
Combined two approaches:

1. **Pin DeviceBus thread to Core 0** using `vTaskCoreAffinitySet(bus_thread_handle, (1 << 0))`. This matches ESP32 HAL's pattern and ensures callback thread and main task share the same core's cache.

2. **Memory barriers in `delay_microseconds_boost()`** - Added `__sync_synchronize()` at start and end. This function is called by `wait_for_sample()` in a tight loop, so barriers ensure flag visibility even if cores differ.

Both solutions together provide defense-in-depth. The core pinning is the primary fix; barriers provide additional safety.

**Files Affected:**
- `lib/ap_compat/AP_HAL_RP2350/Scheduler.cpp` - memory barriers in `delay_microseconds_boost()`
- `lib/ap_compat/AP_HAL_RP2350/DeviceBus.cpp` - `vTaskCoreAffinitySet()` to pin bus thread

**References:**
- FreeRTOS SMP documentation on core affinity
- ARM Cortex-M33 memory model
- ArduPilot `libraries/AP_HAL_ESP32/Scheduler.cpp` (xTaskCreatePinnedToCore usage)

**Source URLs:**
- https://forums.raspberrypi.com/viewtopic.php?t=388734 (RP2040/RP2350 multi-core issues)
- https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/13-Symmetric-multiprocessing-SMP/01-Symmetric-multiprocessing-introduction

---

### PD13: FreeRTOS Priority Inversion with Busy-Wait Delays

**Category:** RTOS
**Discovered:** 2026-01-28
**Severity:** Critical
**Status:** Resolved

**STM32 Behavior:**
ChibiOS on STM32 uses a different threading model. Timer callbacks run in ISR context or dedicated threads with proper yielding. The scheduler has fine-grained timing (sub-ms).

**RP2350 Behavior:**
FreeRTOS with 1kHz tick rate (1ms resolution) has coarse timing. The DeviceBus pattern from ESP32 HAL uses a polling thread that:
1. Runs callbacks when due
2. Calculates next callback time
3. Delays until next callback

The delay implementation matters critically:
- `busy_wait_us_32()` - CPU spins, task stays "running", lower-priority tasks starve
- `vTaskDelay()` - Task blocks, yields to other tasks, but rounds to 1ms ticks

With DeviceBus at priority 5 and main task at priority 2:
- If DeviceBus busy-waits between callbacks, main task NEVER runs
- `wait_for_sample()` in main task cannot execute to check flags
- System appears hung even though callbacks are running

Symptoms:
- Callbacks running at correct rate (~500/sec with I2C overhead)
- Main task's `wait_for_sample()` never sees flags set
- Adding debug prints shows callbacks execute but main task is starved

**Solution:**
DeviceBus must use `vTaskDelay()` for inter-callback delays, even for short periods:

```cpp
// CRITICAL: Must use vTaskDelay() to yield to lower-priority tasks.
// DeviceBus runs at priority 5, main task at priority 2-5.
// If we busy-wait, lower-priority tasks (like wait_for_sample) starve.
TickType_t ticks = pdMS_TO_TICKS((delay + 500) / 1000);
if (ticks == 0) {
    ticks = 1;  // Always yield at least 1 tick
}
vTaskDelay(ticks);
```

This accepts 1ms timing granularity but ensures proper task scheduling. Callback rate is limited to ~500-700/sec due to FreeRTOS tick + I2C overhead, but this is sufficient for IMU operation.

**Files Affected:**
- `lib/ap_compat/AP_HAL_RP2350/DeviceBus.cpp` - Use `vTaskDelay()` instead of `hal.scheduler->delay_microseconds()`

**References:**
- FreeRTOS task scheduling and priority preemption
- ArduPilot `libraries/AP_HAL_ESP32/DeviceBus.cpp`

**Source URLs:**
- https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/01-Tasks-and-co-routines/03-Task-priorities

---

### PD14: vTaskDelay() Timing Significantly Slower Than Expected

**Category:** RTOS
**Discovered:** 2026-02-01
**Severity:** High
**Status:** Resolved (workaround)

**STM32 Behavior:**
`vTaskDelay()` and ChibiOS equivalents provide predictable delays. A 1ms delay takes approximately 1ms. ArduPilot's `wait_for_sample()` uses short delays (~100µs) in a polling loop, expecting total loop time to be predictable.

**RP2350 Behavior:**
`vTaskDelay()` on RP2350 with FreeRTOS SMP takes significantly longer than expected. Community reports (RPi Forums, Jan 2026):
- USB device took 15 seconds to appear after boot with `vTaskDelay()` calls
- Removing `vTaskDelay()` or replacing with `portYIELD()` made it instantaneous
- The issue is specific to RP2350 - RP2040 behaves correctly

Combined with tick quantization (1ms ticks), each `vTaskDelay(1)` call can block for 1-2ms:
- Worst case: Just missed a tick → blocks for 1.999ms
- Best case: Just made a tick → blocks for 1.001ms

**Impact on RocketChip:**
ArduPilot's `AP_InertialSensor::update()` calls `wait_for_sample()`, which has a polling loop with `vTaskDelay(1)` calls. In a 500Hz (2ms) SensorTask loop:
- `wait_for_sample()` consumes 1-2ms per iteration
- Remaining time budget: 0-1ms for all other work
- Result: 35-40% of loop iterations missed their deadline (overruns)

**Solution:**
Don't call `update()` from high-frequency periodic tasks. The DeviceBus callback thread already populates sensor data at 1125Hz. Just read the latest available data:

```cpp
static void readIMU() {
    // DON'T call rocket.ins()->update() - it blocks!
    // Just read the latest data directly
    if (rocket.ins()->get_gyro_health(0)) {
        const ::Vector3f& accel = rocket.ins()->get_accel(0);
        const ::Vector3f& gyro = rocket.ins()->get_gyro(0);
        // ... copy to shared data
    }
}
```

**Files Affected:**
- `src/services/SensorTask.cpp` - Non-blocking `readIMU()` implementation
- `lib/ap_compat/AP_HAL_RP2350/Scheduler.cpp` - `delay_microseconds_boost()` uses `vTaskDelay(1)`

**References:**
- FreeRTOS tick timing and task scheduling
- ArduPilot `libraries/AP_InertialSensor/AP_InertialSensor.cpp` - `wait_for_sample()` implementation

**Source URLs:**
- https://forums.raspberrypi.com/viewtopic.php?p=2358678 (rp2350 with tinyusb - "slow" with FreeRTOS)
- https://forums.raspberrypi.com/viewtopic.php?t=389724 (RP2350 FreeRTOS SMP discussion)

---

## Future Entries Template

Copy this template when adding new platform differences:

```markdown
### PDn: Title

**Category:**
**Discovered:**
**Severity:**
**Status:**

**STM32 Behavior:**


**RP2350 Behavior:**


**Solution:**


**Files Affected:**


**References:**


**Source URLs:**

```

---

## Categories Reference

| Category | Description |
|----------|-------------|
| Memory | RAM layout, heap, stack, PSRAM differences |
| Flash | Flash architecture, XIP, wear-leveling, erase granularity |
| Timing | Clock sources, timer resolution, tick rates |
| Peripherals | GPIO, I2C, SPI, UART, DMA behavioral differences |
| RTOS | FreeRTOS vs ChibiOS, task model, interrupt handling |
| Other | Anything not fitting above categories |

## Severity Levels

| Level | Description |
|-------|-------------|
| Critical | Will crash or corrupt data if not handled |
| High | Significant behavioral difference affecting correctness |
| Medium | Noticeable difference requiring adaptation |
| Low | Minor difference, mostly cosmetic or optimization |
