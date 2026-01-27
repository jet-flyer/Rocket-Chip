# ChibiOS RP2350 Known Issues

**Purpose:** Document known issues, bugs, errata, and gotchas specific to ChibiOS on RP2350/RP2040 hardware. This complements `docs/RP2350_FULL_AP_PORT.md` which documents general platform differences.

**Last Updated:** 2026-01-27

---

## Severity Levels

| Level | Description |
|-------|-------------|
| **Critical** | Blocks functionality; must address before that feature works |
| **High** | Significant impact; requires workaround for reliable operation |
| **Medium** | Noticeable limitation; workaround available |
| **Low** | Minor issue; document for awareness |

---

## Critical Issues

### CI1: USB Not Working on ChibiOS RP2350

**Category:** Peripherals
**Severity:** Critical
**Status:** BLOCKED - No working path identified

**Issue:**
Neither ChibiOS native USB nor TinyUSB integration works on RP2350 with ChibiOS:

1. **ChibiOS USB HAL**: Driver exists at `os/hal/ports/RP/LLD/USBv1/hal_usb_lld.c` but causes hard crashes when enabled. Device becomes unresponsive (all registers zero, requires BOOTSEL recovery).

2. **TinyUSB Integration**: TinyUSB requires Pico SDK runtime functions (spin locks, interrupt handling, etc.) which fundamentally conflict with ChibiOS:
   - Pico SDK `sync.c`, `platform.c` conflict with ChibiOS initialization
   - Removing SDK sources and stubbing functions still crashes
   - Interrupt priority handling differs between ChibiOS and SDK
   - RP2350 Cortex-M33 unaligned access handling requires SDK runtime

**UPDATE (2026-01-27):** Extensive testing confirmed both paths fail:
- ChibiOS USB HAL: Hard crash on enable
- TinyUSB + ChibiOS: Hard crash at boot, even with all Pico SDK functions stubbed
- ChibiOS forum patches (SDK 2.2.0 integration) contain no USB-related changes

**Impact:**
- Cannot use USB CDC for debug output or telemetry
- Must use UART via debug probe for all debug communication
- USB functionality blocked until ChibiOS implements working driver

**Workaround:**
Use UART via debug probe (GPIO0/1) for debug output. This is working and reliable. When ChibiOS USB becomes available, switching from UART to USB stream should be straightforward - only the stream source changes, not application code.

**Root Cause Analysis (2026-01-27):**
Examination of the ChibiOS USB driver source reveals a critical bug:

1. **Missing IRQ handler wrapper**: The RP USB driver exports `usb_lld_serve_interrupt()` but has no actual `OSAL_IRQ_HANDLER()` wrapper to connect it to the interrupt vector (Vector78 for RP2350). Compare with STM32 USB driver which defines `OSAL_IRQ_HANDLER(STM32_USB1_LP_HANDLER)`.

2. **Mismatched prologue/epilogue**: The `usb_lld_serve_interrupt()` function at line 394 in `hal_usb_lld.c` has `OSAL_IRQ_EPILOGUE()` at line 470 but no `OSAL_IRQ_PROLOGUE()` at the start.

3. **No `rp_usb.inc` file**: Unlike UART which has `rp_uart0.inc`/`rp_uart1.inc` included from `rp_isr.c` to register handlers, there is no `rp_usb.inc` for USB.

The driver defines all register layouts and logic but the interrupt plumbing is incomplete, explaining why enabling USB crashes the device.

**Future Development:**
ChibiOS forum shows active work on Pico SDK 2.2.0 integration (Jan 2026), but no USB driver work mentioned. The driver exists but needs the IRQ handler implementation completed. Monitor ChibiOS trunk for updates.

**Source URLs:**
- https://forum.chibios.org/viewtopic.php?f=3&t=5800&start=60 (USB HAL discussion)
- https://forum.chibios.org/viewtopic.php?f=3&t=6631 (SDK 2.2.0 patches - no USB)
- ChibiOS source: `os/hal/ports/RP/LLD/USBv1/hal_usb_lld.c` line 394, 470

---

### CI2: Flash XIP Issues During Writes

**Category:** Flash
**Severity:** Critical
**Status:** Hardware Limitation

**Issue:**
RP2350 uses XIP (Execute-in-Place) where code runs directly from flash. During flash write/erase operations, the **entire flash becomes inaccessible**. Any code attempting to execute from flash will crash.

This is identical to PD1 in `RP2350_FULL_AP_PORT.md` but particularly relevant for ChibiOS because:
- ChibiOS typically runs from flash
- Interrupt handlers may be in flash
- Both cores may be executing from flash

**Impact:**
- Flash writes cause crashes if not handled carefully
- Cannot use standard ChibiOS flash drivers without modification

**Workaround:**
1. Place flash operation functions in RAM using `__attribute__((section(".ramfunc")))`
2. Disable all interrupts during flash operations
3. Stall the other core during flash operations
4. Invalidate XIP cache after flash operations

**Source URLs:**
- https://forum.chibios.org/viewtopic.php?t=6398
- https://forums.raspberrypi.com/viewtopic.php?t=388734

---

### CI3: Stage2 Bootloader Required for Flash Boot

**Category:** Boot
**Severity:** Critical (for flash execution)
**Status:** ChibiOS Limitation

**Issue:**
RP2040/RP2350 requires a 256-byte stage2 bootloader at the start of flash to initialize the QSPI interface before code can execute from flash. ChibiOS trunk does not include this bootloader integration - it primarily supports RAM-only execution.

**Impact:**
- Default ChibiOS build runs from RAM only
- Flash boot requires custom linker scripts and bootloader integration
- UF2 file generation requires bootloader awareness

**Workaround:**
1. Use Pico SDK's `boot_stage2` bootloader
2. Use community patches (e.g., electroniceel's ChibiOS fork)
3. Integrate bootloader into linker script:
   - Bootloader at 0x10000000 (256 bytes)
   - Application at 0x10000100

**Source URLs:**
- https://forum.chibios.org/viewtopic.php?t=6034
- https://github.com/electroniceel/ChibiOS/tree/RP2040-flash

---

## High-Severity Issues

### CI4: SPI+DMA Stops After ~253 Cycles

**Category:** Peripherals
**Severity:** High
**Status:** Hardware/Driver Issue

**Issue:**
SPI with DMA on RP2350 has been observed to stop after approximately 253 cycles (with 2KB buffer) or ~508 cycles (with 1KB buffer). After failure, interrupts still fire but the main loop stops executing.

This is identical to PD8 in `RP2350_FULL_AP_PORT.md`.

**Impact:**
- High-rate sensor polling (1kHz+) may fail
- DMA transfers become unreliable over time

**Workaround:**
1. Link two DMA channels together for double-buffering
2. Implement DMA watchdog to detect stalls and reinitialize
3. Clear DMA enable bit before abort (per RP2350 errata E5)
4. Consider polling-based SPI for critical sensors initially

**Source URLs:**
- https://forums.raspberrypi.com/viewtopic.php?t=385042
- https://github.com/tinygo-org/tinygo/issues/4690

---

### CI5: I2C Clock Stretching Intolerance

**Category:** Peripherals
**Severity:** High
**Status:** Hardware Limitation

**Issue:**
RP2040/RP2350 hardware I2C is intolerant of clock stretching. Sensors that stretch clocks (BNO055, some barometers during conversion) cause "Clock stretch too long" timeouts.

This is identical to PD7 in `RP2350_FULL_AP_PORT.md`.

**Impact:**
- Some I2C sensors may timeout or fail
- Requested I2C frequencies may not be achieved (~516kHz when requesting 1MHz)

**Workaround:**
1. Use longer timeouts: 10000µs instead of default
2. Lower I2C frequency: 40kHz-100kHz for problematic sensors
3. Use PIO-based software I2C if necessary
4. Prefer SPI sensors when available (RocketChip uses SPI for IMU)

**Source URLs:**
- https://github.com/micropython/micropython/issues/8167
- https://github.com/raspberrypi/pico-sdk/issues/456
- https://learn.adafruit.com/raspberry-pi-i2c-clock-stretching-fixes

---

### CI6: Multicore SMP Synchronization Complexity

**Category:** RTOS
**Severity:** High
**Status:** Architectural Consideration

**Issue:**
ChibiOS SMP on RP2350 requires careful synchronization:
- `chSysLock()` on one core doesn't block the other core
- Virtual timer has 64-bit microsecond counter but 32-bit alarm comparisons
- Hardware divider state must be saved in ISR context
- Inter-core communication via SIO FIFO has shared IRQ (see CI10)

**Impact:**
- Race conditions possible if not using proper SMP primitives
- Timer wraparound issues possible with long delays

**Workaround:**
1. Use ChibiOS SMP-aware synchronization primitives
2. Use `chSysLockFromISR()` / `chSysUnlockFromISR()` in interrupt context
3. Protect shared resources with mutexes, not just critical sections
4. Test thoroughly with both cores active

**Source URLs:**
- https://forum.chibios.org/viewtopic.php?t=6398
- https://forum.chibios.org/viewtopic.php?t=5891
- https://forum.chibios.org/viewtopic.php?f=3&t=5107&start=20

---

### CI13: I2C Hardware Driver is Stub Only

**Category:** Peripherals
**Severity:** High
**Status:** ChibiOS Limitation

**Issue:**
The ChibiOS I2C low-level driver for RP2350 (`os/hal/ports/RP/LLD/I2Cv1/hal_i2c_lld.c`) contains only empty function bodies - it's a stub placeholder with no actual implementation. The driver is also NOT included in `platform.mk`.

```c
void i2c_lld_start(I2CDriver *i2cp) {
  if (i2cp->state == I2C_STOP) {
#if PLATFORM_I2C_USE_I2C1 == TRUE
    if (&I2CD1 == i2cp) {
      // EMPTY - no implementation
    }
#endif
  }
}
```

**Impact:**
- Cannot use ChibiOS HAL I2C driver with hardware I2C peripheral
- Sensors on I2C bus (STEMMA QT / QWIIC) not accessible via standard HAL

**Workaround:**
Use ChibiOS fallback/software I2C driver at `os/hal/lib/fallback/I2C/`:
1. Include the fallback driver source in Makefile instead of I2Cv1
2. Configure pins as GPIO with pull-ups
3. Use `SW_I2C_USE_I2C1 = TRUE` in halconf.h

The software I2C bit-bangs the protocol via GPIO. Slower than hardware but fully functional for sensor communication.

**Alternative:** ChibiOS-Contrib (used by QMK) has a hardware I2C driver for RP2040 that could potentially be adapted for RP2350.

**Files Affected:**
- `chibios/phase0_validation/test_i2c_scan/` - Uses fallback driver

**Source URLs:**
- https://forum.chibios.org/viewtopic.php?t=5800&start=80 (I2C via SDK mention)
- ChibiOS source: `os/hal/ports/RP/LLD/I2Cv1/hal_i2c_lld.c` (stub)
- ChibiOS source: `os/hal/lib/fallback/I2C/hal_i2c_lld.c` (working)

---

## Medium-Severity Issues

### CI7: ADC Linearity Issues (DNL Error)

**Category:** Peripherals
**Severity:** Medium
**Status:** Hardware Erratum (RP2350-E11)

**Issue:**
Differential Non-Linearity (DNL) error causes non-monotonic behavior at certain ADC codes. Specifically affects readings around multiples of 512 at 12-bit resolution.

**Impact:**
- ±1 LSB accuracy issues at certain voltage levels
- May affect precise voltage measurements

**Workaround:**
1. Use 8-bit conversions if linearity is critical
2. Accept ±1 LSB error at certain codes
3. Apply software calibration/lookup table if needed

**Source URLs:**
- https://github.com/micropython/micropython/issues/10947
- RP2350 Datasheet Errata

---

### CI8: GPIO/ADC Interaction

**Category:** Peripherals
**Severity:** Medium
**Status:** Hardware Quirk

**Issue:**
GPIO digital input enable can affect ADC readings. If a pin is configured as both GPIO input and ADC input, the digital input circuitry can interfere.

**Impact:**
- Inaccurate ADC readings on shared pins

**Workaround:**
Disable GPIO input before using ADC:
```c
gpio_set_input_enabled(pin, false);
// Now use ADC
```

**Source URLs:**
- https://forums.raspberrypi.com/viewtopic.php?t=390805

---

### CI9: RP2350-E9 Erratum (GPIO Stuck at ~2V)

**Category:** Peripherals
**Severity:** Medium
**Status:** Hardware Erratum

**Issue:**
GPIO pins can get "stuck" at approximately 2V even without internal pulldown enabled. When a floating input is driven high and then released, it may remain at an intermediate voltage.

This is identical to PD9 in `RP2350_FULL_AP_PORT.md`.

**Impact:**
- Floating inputs read unpredictably
- ~100µA leakage current on affected pins
- ADC readings affected on high-impedance sources

**Workaround:**
1. Use external pull-downs (4.7kΩ) on critical GPIO inputs
2. Add buffer amplifier for high-impedance ADC sources
3. Never rely on floating pins or internal pull-downs alone

**Source URLs:**
- https://hackaday.com/2024/09/04/the-worsening-raspberry-pi-rp2350-e9-erratum-situation/
- https://github.com/raspberrypi/pico-feedback/issues/401

---

## Low-Severity Issues

### CI10: SIO FIFO Shared Between Cores (RP2350)

**Category:** RTOS
**Severity:** Low
**Status:** Hardware Design (RP2350 only)

**Issue:**
On RP2350, the SIO FIFO IRQ number is identical on both cores (differs from RP2040). This causes conflicts when using FIFO for inter-core communication while also using SDK functions that rely on SIO FIFO internally (like `flash_safe_execute()`).

**Impact:**
- Cannot use both raw SIO FIFO and SDK flash functions
- IRQ handler conflicts possible

**Workaround:**
Use doorbells + software queues for inter-core communication instead of raw SIO FIFO. Doorbells are RP2350-specific.

**Source URLs:**
- https://forums.raspberrypi.com/viewtopic.php?t=388734

---

### CI11: PWM Timer Sharing on Adjacent Pins

**Category:** Peripherals
**Severity:** Low
**Status:** Hardware Design

**Issue:**
Consecutive even/odd GPIO pins (e.g., GPIO2/3, GPIO10/11) share the same PWM slice and therefore the same timer/frequency. Cannot set independent PWM frequencies on adjacent even/odd pin pairs.

**Impact:**
- Pin assignment constraints for PWM outputs

**Workaround:**
When assigning PWM outputs (servos, motor ESCs), ensure channels requiring different frequencies are not on adjacent even/odd pins. Document pin assignments in board configuration.

**Source URLs:**
- RP2350 Datasheet Section 4.5 (PWM)

---

### CI12: malloc() Does Not Zero Memory

**Category:** Memory
**Severity:** Low
**Status:** Implementation Detail

**Issue:**
Standard `malloc()` does not zero memory. ArduPilot libraries may assume zeroed allocations, causing subtle initialization bugs.

This lesson was learned during ESP32 HAL integration.

**Impact:**
- Uninitialized memory bugs possible

**Workaround:**
Override `malloc()` to use `calloc()` or explicitly zero memory after allocation:
```c
void* hal_malloc(size_t size) {
    void* p = malloc(size);
    if (p) memset(p, 0, size);
    return p;
}
```

**Source URLs:**
- ArduPilot ESP32 HAL README

---

## Summary Table

| ID | Issue | Severity | ChibiOS Fix? | Workaround Available |
|----|-------|----------|--------------|---------------------|
| CI1 | USB HAL crashes | Critical | Exists but broken | TinyUSB/bare-metal |
| CI2 | Flash XIP issues | Critical | No | RAM functions |
| CI3 | Stage2 bootloader | Critical | Partial | Community patches |
| CI4 | SPI+DMA stops | High | No | Link channels |
| CI5 | I2C clock stretch | High | No | Lower speed |
| CI6 | SMP complexity | High | Implemented | Careful design |
| CI13 | I2C driver is stub | High | No | Fallback SW I2C |
| CI7 | ADC linearity | Medium | No | 8-bit mode |
| CI8 | GPIO/ADC interact | Medium | No | Disable input |
| CI9 | E9 erratum | Medium | No | External pulldowns |
| CI10 | SIO FIFO shared | Low | N/A | Use doorbells |
| CI11 | PWM sharing | Low | N/A | Pin planning |
| CI12 | malloc zeroing | Low | No | Override malloc |

---

## Relationship to RP2350_FULL_AP_PORT.md

Several issues documented here overlap with `docs/RP2350_FULL_AP_PORT.md`:
- CI2 ≈ PD1 (Flash XIP)
- CI4 ≈ PD8 (SPI+DMA)
- CI5 ≈ PD7 (I2C clock stretching)
- CI9 ≈ PD9 (E9 erratum)

The platform differences doc focuses on FreeRTOS/Pico SDK context while this document focuses on ChibiOS-specific considerations. Both remain relevant as reference material.
