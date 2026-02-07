# RocketChip Integration and Verification Plan (IVP)

**Status:** ACTIVE — Living document
**Last Updated:** 2026-02-06
**Target Platform:** RP2350 (Adafruit Feather HSTX w/ 8MB PSRAM)
**Architecture:** Bare-metal Pico SDK, dual-core AMP (see `PICO_SDK_MULTICORE_DECISION.md`)

---

## 1. Purpose and Scope

This document defines the step-by-step integration order for RocketChip firmware, from LED blink through full ESKF sensor fusion and telemetry. Every step has a concrete verification gate — a pass/fail test that must succeed before proceeding.

**This is a living document:**
- **Stages 1-4** (Foundation, Sensors, Dual-Core, GPS): Fully detailed
- **Stage 5** (Sensor Fusion): Moderate detail, some TBDs
- **Stages 6-9** (Mission Engine, Logging, Telemetry, Integration): Placeholders expanded as earlier stages complete

**Key references (not duplicated here):**
- `docs/SAD.md` — System architecture, data structures, module responsibilities
- `include/rocketchip/config.h` — Pin assignments, I2C addresses, timing constants
- `.claude/LESSONS_LEARNED.md` — Debugging journal (referenced as "LL Entry N")
- `docs/decisions/ESKF/FUSION_ARCHITECTURE.md` — Sensor fusion design
- `PICO_SDK_MULTICORE_DECISION.md` — Council decision on bare-metal dual-core

**Numerical values:** Rates, thresholds, buffer sizes, and timing values throughout this document are **preliminary targets** unless marked `[VALIDATED]`. Each must be justified from a datasheet, SDK measurement, or empirical test before committing to implementation. Do not treat them as specifications — they are starting points for the verification gate to confirm or revise.

**Conventions:**
- Each step has a unique ID (`IVP-XX`) for traceability
- `[GATE]` = verification test. Step is not complete until the gate passes.
- `[DIAG]` = failure diagnosis hints
- `[LL]` = LESSONS_LEARNED cross-reference
- `⚠️ VALIDATE` = numerical value requires measurement/datasheet confirmation before implementation

---

## 2. Test Environment

### Required Tools

| Tool | Purpose | Notes |
|------|---------|-------|
| picotool | Routine flashing | `picotool load build/rocketchip.uf2 --force` |
| Raspberry Pi Debug Probe | GDB, flash when USB broken | Use Pico SDK OpenOCD — see `.claude/DEBUG_PROBE_NOTES.md` |
| Python pyserial | Scripted serial testing | More reliable than PuTTY for USB CDC |
| VSCode Serial Monitor | Interactive serial | Recommended over PuTTY (LL Entry 16) |
| Multimeter | Voltage checks | 3.3V rail, battery voltage |

### Serial Connection

USB CDC at 115200 baud. Connect via:
```bash
python -m serial.tools.miniterm COM6 115200
```

### Debug Probe Commands

```bash
# Start OpenOCD (always use Pico SDK version, NOT system version)
taskkill //F //IM openocd.exe 2>/dev/null; sleep 2; \
/c/Users/pow-w/.pico-sdk/openocd/0.12.0+dev/openocd \
-s /c/Users/pow-w/.pico-sdk/openocd/0.12.0+dev/scripts \
-f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000" &

# GDB backtrace
/c/Users/pow-w/.pico-sdk/toolchain/14_2_Rel1/bin/arm-none-eabi-gdb.exe \
build/rocketchip.elf -batch \
-ex "target extended-remote localhost:3333" \
-ex "monitor reset halt" -ex "bt"

# Flash via probe
# ... same as above but add: -ex "load" -ex "monitor reset run"
```

### Build Commands

```bash
cmake -B build -G Ninja
cmake --build build/
```

---

## 3. Stage Overview

| Stage | Name | SAD Phase | Steps | Detail | Milestone |
|-------|------|-----------|-------|--------|-----------|
| 1 | Foundation | Phase 1 | IVP-01 — IVP-08 | Full | |
| 2 | Single-Core Sensors | Phase 2 | IVP-09 — IVP-18 | Full | **Minimum Viable Demo** |
| 3 | Dual-Core Integration | Phase 2 | IVP-19 — IVP-30 | Full | |
| 4 | GPS Navigation | Phase 3 | IVP-31 — IVP-33 | Full | |
| 5 | Sensor Fusion | Phase 4 | IVP-34 — IVP-43 | Moderate | |
| 6 | Mission Engine | Phase 5 | IVP-44 — IVP-48 | Placeholder | **Crowdfunding Demo Ready** |
| 7 | Data Logging | Phase 6 | IVP-49 — IVP-53 | Placeholder | |
| 8 | Telemetry | Phase 7 | IVP-54 — IVP-58 | Placeholder | |
| 9 | System Integration | Phase 9 | IVP-59 — IVP-63 | Placeholder | **Flight Ready** |

---

## Stage 1: Foundation

**Purpose:** Board boots, blinks LED, outputs to USB serial, scans I2C bus. All single-core, Core 0 only. Validates build system, flashing, and basic Pico SDK functionality.

---

### IVP-01: Clean Build from Source

**Prerequisites:** None (first step)

**Implement:** Create minimal `src/main.cpp` with an empty `main()` that returns 0. Verify CMakeLists.txt has no FreeRTOS references and compiles cleanly.

**[GATE]:**
- `cmake -B build -G Ninja` completes without errors
- `cmake --build build/` completes with zero errors in `src/` and `include/`
- `build/rocketchip.uf2` exists

**[DIAG]:** If CMake fails, verify `PICO_BOARD` is set to `adafruit_feather_rp2350` before SDK import. If driver files fail to compile, temporarily comment them out of CMakeLists.txt — they'll be re-added incrementally.

---

### IVP-02: Red LED Blink (Board Alive)

**Prerequisites:** IVP-01

**Implement:** In `main()`, initialize GPIO for red LED (`PICO_DEFAULT_LED_PIN` = GPIO 7 on Feather RP2350). Blink at 1Hz (500ms on, 500ms off) using `sleep_ms()`.

**[GATE]:**
- Flash via `picotool load build/rocketchip.uf2 --force`
- Red LED blinks at 1Hz, visible within 1 second of power-on

**[DIAG]:** If no blink, flash via debug probe and use GDB to verify `main()` is reached. Use `PICO_DEFAULT_LED_PIN`, not hardcoded `7` (per CODING_STANDARDS.md).

**[LL]:** Entry 5 (debug probe before manual BOOTSEL), Entry 2 (version strings)

---

### IVP-03: NeoPixel Status LED

**Prerequisites:** IVP-02

**Implement:** Initialize WS2812 via `ws2812_status_init(pio0, 21)`. Set solid green via `ws2812_set_mode(WS2812_MODE_SOLID, WS2812_COLOR_GREEN)`. Red LED continues 1Hz heartbeat.

**[GATE]:**
- NeoPixel shows solid green
- Red LED still blinks 1Hz
- Change to `WS2812_MODE_RAINBOW` — smooth color cycling

**[DIAG]:** NeoPixel dark = `ws2812_status_init()` not called or PIO allocation failed. Check return value. Colors wrong = GRB vs RGB ordering issue in PIO program.

**[LL]:** Entry 6 (WS2812 requires begin/init call)

---

### IVP-04: USB CDC Serial Output

**Prerequisites:** IVP-02

**Implement:** Call `stdio_init_all()` in `main()`. Blink LED fast while waiting for USB connection, then print version banner:

```cpp
stdio_init_all();
while (!stdio_usb_connected()) {
    gpio_put(PICO_DEFAULT_LED_PIN, 1); sleep_ms(100);
    gpio_put(PICO_DEFAULT_LED_PIN, 0); sleep_ms(100);
}
sleep_ms(500);  // Settle time
printf("RocketChip v%s\n", ROCKETCHIP_VERSION_STRING);
printf("Build: " __DATE__ " " __TIME__ "\n");
```

**[GATE]:**
- Connect terminal: `python -m serial.tools.miniterm COM6 115200`
- Banner appears within 1 second of connection
- Version matches `ROCKETCHIP_VERSION_STRING` in config.h ("0.1.0")

**[DIAG]:** No COM port = check `pico_enable_stdio_usb(rocketchip 1)` in CMakeLists.txt. COM port but no output = try debug probe, check that `stdio_init_all()` completes. Use VSCode Serial Monitor — PuTTY may truncate initial output (LL Entry 16).

**[LL]:** Entry 3 (BASEPRI — not applicable in bare-metal but settle time pattern still useful), Entry 16 (PuTTY truncation)

---

### IVP-05: Debug Macros Functional

**Prerequisites:** IVP-04

**Implement:** Verify `DBG_PRINT`, `DBG_ERROR` from config.h work in Debug builds and compile out in Release.

**[GATE]:**
- Debug build: `DBG_PRINT("test %d", 42)` outputs `[timestamp] test 42`
- Release build (`cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release`): same code produces no output, binary is smaller

**[DIAG]:** Check that `add_compile_definitions(DEBUG=1)` is gated on `CMAKE_BUILD_TYPE` in CMakeLists.txt.

---

### IVP-06: I2C Bus Initialization

**Prerequisites:** IVP-04

**Implement:** Call `i2c_bus_init()`. Print result. Bus is I2C1 on GPIO 2 (SDA) / GPIO 3 (SCL) at 400kHz per `i2c_bus.h`.

**[GATE]:**
- Print: `I2C1 initialized at 400kHz on SDA=2 SCL=3`
- `i2c_bus_init()` returns `true`

**[DIAG]:** Check GPIO pins match `I2C_BUS_SDA_PIN` (2) and `I2C_BUS_SCL_PIN` (3) in `i2c_bus.h`. Check that STEMMA QT cable is connected to at least one sensor (provides I2C pull-ups).

---

### IVP-07: I2C Device Scan

**Prerequisites:** IVP-06

**Implement:** Call `i2c_bus_scan()`. Print all detected devices.

**[GATE]:**
- With ICM-20948 + DPS310 connected:
  ```
  0x69: ACK  (ICM-20948)
  0x77: ACK  (DPS310)
  ```
- If PA1010D GPS connected: also `0x10: ACK`
- Addresses match config.h: `i2c::kIcm20948 = 0x69`, `i2c::kDps310 = 0x77`
- No false positives

**[DIAG]:** 0x69 not found = check LL Entry 13 (Adafruit ICM-20948 defaults to 0x69, AD0=HIGH). No devices = check cable seating, verify 3.3V on sensor VCC with multimeter.

**[LL]:** Entry 13 (ICM-20948 address is 0x69 not 0x68)

---

### IVP-08: Heartbeat Superloop + Uptime

**Prerequisites:** IVP-04, IVP-03

**Implement:** Restructure `main()` into the polling superloop pattern. Red LED heartbeat (100ms on, 900ms off). NeoPixel in rainbow mode. Print uptime every 5 seconds when terminal connected. Guard all USB I/O with `stdio_usb_connected()`.

**[GATE]:**
- Red LED: 100ms on, 900ms off heartbeat
- NeoPixel: rainbow cycling
- Uptime prints every 5s: `Uptime: 5000 ms`, `Uptime: 10000 ms`, ...
- Values monotonically increasing
- Disconnect terminal, wait 30s, reconnect: output resumes with correct uptime (program did not stall without terminal)

**[DIAG]:** If program stops when terminal disconnects, USB I/O is not guarded. Wrap all `printf`/`getchar` in `stdio_usb_connected()` checks.

**[LL]:** Entry 15 (terminal-connected pattern)

---

## Stage 2: Single-Core Sensors

**Purpose:** Bring up each sensor individually on Core 0, validate data quality, integrate calibration and CLI. All single-core superloop. Validates drivers before adding dual-core complexity.

---

### IVP-09: ICM-20948 IMU Initialization

**Prerequisites:** IVP-07 (I2C scan shows 0x69)

**Implement:** Call `icm20948_init(&dev, ICM20948_ADDR_DEFAULT)`. Verify WHO_AM_I. Set accel +/-4g, gyro +/-500dps, mag continuous 100Hz.

**[GATE]:**
- Print: `ICM-20948 init OK (WHO_AM_I=0xEA)`
- `dev.mag_initialized == true` (AK09916 accessible)
- `icm20948_ready(&dev)` returns true

**[DIAG]:** WHO_AM_I mismatch = wrong address or wrong device. Mag init fails = I2C bypass mode not configured in `icm20948_init()`.

**[LL]:** Entry 13 (I2C address)

---

### IVP-10: IMU Data Validation

**Prerequisites:** IVP-09

**Implement:** Read via `icm20948_read(&dev, &data)` at 10Hz (slow for debug). Print raw and scaled values. Device stationary on flat surface.

**[GATE]:**
- Accel Z: ~+9.8 m/s^2 (+/-0.5). X, Y near zero (+/-0.3)
- Gyro: all axes near zero (+/-0.05 rad/s) when stationary
- Mag: reasonable values (10-60 uT), not all zeros, not saturated
- Temperature: plausible ambient (15-40 C)
- No NaN, no inf, no stuck values
- 30 seconds continuous: stable, no drift beyond noise floor

**[DIAG]:** Accel Z near zero = axes swapped or scale factor wrong. Check `icm20948.c` against datasheet. Large gyro bias = normal pre-calibration. Mag all zeros = AK09916 not in continuous mode.

---

### IVP-11: DPS310 Barometer Initialization

**Prerequisites:** IVP-07 (I2C scan shows 0x77)

**Implement:** Call `baro_dps310_init(BARO_DPS310_ADDR_DEFAULT)` then `baro_dps310_start_continuous()`.

**[GATE]:**
- Print: `DPS310 init OK`
- `baro_dps310_ready()` returns true
- `baro_dps310_start_continuous()` returns true

**[DIAG]:** Init fails = verify address matches scan result. May need delay after init before starting continuous mode.

---

### IVP-12: Barometer Data Validation

**Prerequisites:** IVP-11

**Implement:** Read via `baro_dps310_read(&data)` at 10Hz. Print pressure, temperature, altitude.

**[GATE]:**
- Pressure: ~101325 Pa (+/-3000 Pa for typical altitudes)
- Temperature: plausible ambient (15-40 C)
- `data.valid == true` on every read
- Noise: pressure variation < +/-5 Pa over 10 seconds
- No stuck values, no NaN

**[DIAG]:** Pressure reads zero = continuous mode not started. Temperature wildly wrong = ruuvi library coefficient handling.

---

### IVP-13: Multi-Sensor Polling (Single Core)

**Prerequisites:** IVP-10, IVP-12

**Implement:** Read IMU and baro in the main loop using `time_us_64()` interval tracking. Print combined status every second. Target rates: IMU at `⚠️ VALIDATE 100Hz` (config.h `kSensorPollMs`), baro at `⚠️ VALIDATE 50Hz` (config.h `kBaroDivider`). These are Core 0 development rates — Core 1 will run faster in Stage 3.

**[GATE]:**
- IMU actual rate: within 5% of target
- Baro actual rate: within 10% of target
- Zero I2C errors over 60 seconds
- Print: `IMU: N/s, Baro: N/s` (actual measured rates)
- **Measure and record per-read I2C time**: IMU read = Xus, baro read = Xus (these measurements inform Stage 3 timing budgets)

**[DIAG]:** Rates low = I2C bus saturated. Verify `I2C_BUS_FREQ_HZ` is 400000 in `i2c_bus.h`. I2C errors = bus contention, only one transaction at a time in polling mode.

---

### IVP-13a: I2C Bus Recovery

**Prerequisites:** IVP-13

**Implement:** Implement I2C bus timeout and recovery strategy. When an I2C read times out (`I2C_TIMEOUT_US = 10000` in `i2c_bus.h`): skip the sample, increment error counter, attempt bus recovery (toggle SCL to clear stuck SDA). If errors persist beyond a threshold, reinitialize the I2C bus.

**[GATE]:**
- Simulate bus hang (disconnect sensor during reads): recovery occurs, no permanent lockup
- After recovery: sensor reads resume successfully
- Error counter increments and is visible via CLI (`s` command)
- Recovery does not crash or hang either core

**[DIAG]:** Bus permanently locked = SDA stuck low. SCL toggling (9 clock pulses) should release it. If not, full `i2c_bus_deinit()` + `i2c_bus_init()` cycle needed.

---

### IVP-14: Calibration Storage (Flash Persistence)

**Prerequisites:** IVP-04

**Implement:** Call `calibration_manager_init()`. Test write/read/power-cycle persistence via `calibration_save()` / `calibration_load()`.

**[GATE]:**
- After reset: `calibration_load()` returns `CAL_RESULT_OK` or initializes defaults
- Save test data, read back: matches
- **Power cycle** (unplug USB, replug): data persists
- Save 10 times consecutively: all succeed (dual-sector wear leveling works)

**[DIAG]:** Flash write fails = `flash_safe_execute()` timeout. Check `pico_flash` linked in CMakeLists.txt. Data doesn't persist = sector offsets overlap firmware image.

**[LL]:** Entry 4 (flash ops), Entry 12 (flash/USB interaction)

---

### IVP-15: Gyro Bias Calibration

**Prerequisites:** IVP-10, IVP-14

**Implement:** Call `calibration_start_gyro()`. Feed samples via `calibration_feed_gyro()` for 2-3 seconds while device is stationary.

**[GATE]:**
- Returns `CAL_RESULT_OK`
- Bias values: each axis < 0.1 rad/s
- After applying via `calibration_apply_gyro()`: readings near zero when stationary
- Persists across power cycle

**[DIAG]:** `CAL_RESULT_MOTION_DETECTED` = device was not stationary. Bias > 0.5 rad/s = check gyro scale factor.

---

### IVP-16: Level Calibration

**Prerequisites:** IVP-10, IVP-14

**Implement:** Call `calibration_start_accel_level()` with device flat on table. Feed samples via `calibration_feed_accel()`.

**[GATE]:**
- Completes in < 3 seconds
- After applying via `calibration_apply_accel()`: Z reads +9.81 +/-0.05, X and Y < 0.05 m/s^2
- Persists across power cycle

**[DIAG]:** Wrong values = check accelerometer axis orientation. "Flat" must have Z pointing up.

---

### IVP-17: 6-Position Accel Calibration

**Prerequisites:** IVP-16, IVP-14

**Implement:** Call `calibration_start_accel_6pos()`. Guide through 6 positions via `calibration_get_6pos_position_name()` and `calibration_accept_6pos_position()`.

**[GATE]:**
- All 6 positions collected (progress reaches 100% for each)
- Ellipsoid fit converges (result = `CAL_RESULT_OK`)
- Offset values: each axis < 5 m/s^2 (sanity bound)
- Scale values: each axis 0.8–1.2 (sanity bound)
- After applying: gravity magnitude 9.81 +/-0.02 m/s^2 in all orientations
- Persists across power cycle

**[DIAG]:** Fit doesn't converge = positions not distinct enough. Hangs at ellipsoid fit = stack overflow. `run_ellipsoid_fit()` uses ~400+ bytes of local arrays.

**[LL]:** Entry 1 (large local variables), Entry 19 (stack overflow during ellipsoid fit)

---

### IVP-18: CLI Menu (Single Core) — 🎯 MINIMUM VIABLE DEMO

**Prerequisites:** IVP-13, IVP-15, IVP-16, IVP-17

> **Milestone:** At IVP-18 completion, the board is a working calibrated sensor platform with USB CLI. This is the earliest point at which the hardware can be demonstrated as a functional data logger.

**Implement:** CLI in main polling loop. Terminal-connected pattern. Main menu: `h` (help/status), `s` (sensor data), `c` (calibration submenu). Calibration submenu: `l` (level), `a` (6-pos accel), `g` (gyro), `b` (baro), `x` (exit).

**[GATE]:**
- `h`: prints version, uptime, calibration state
- `s`: prints calibrated accel, gyro, mag, baro values
- `c`: enters calibration menu, `x` exits
- Calibration commands complete successfully
- Drain USB input buffer on terminal connect (no phantom commands)
- CLI does **not** block sensor polling (rates maintained during interaction)
- Scripted test:
  ```python
  port.write(b'h')
  time.sleep(0.5)
  response = port.read(4000).decode()
  assert 'RocketChip' in response
  ```

**[DIAG]:** CLI freezes = `getchar_timeout_us(0)` must be non-blocking. Phantom commands on connect = need input buffer drain.

**[LL]:** Entry 15 (terminal-connected pattern, buffer drain), Entry 16 (PuTTY truncation — use VSCode)

---

## Stage 3: Dual-Core Integration

**Purpose:** Move sensor sampling to Core 1, implement cross-core data sharing, validate RP2350 inter-core hardware primitives, and verify timing and USB stability. This is the most critical integration boundary in the project.

### RP2350 Inter-Core Hardware Primitives

The RP2350 provides several hardware mechanisms for inter-core coordination. This stage exercises each one explicitly before relying on them in production code. These primitives live in the SIO (Single-cycle I/O) block and operate independently of any RTOS.

| Primitive | Pico SDK Header | Purpose in RocketChip |
|-----------|----------------|----------------------|
| **Hardware spinlocks** | `hardware/sync/spin_lock.h` | Short critical sections protecting shared state |
| **Multicore FIFO** | `pico/multicore.h` | Message passing between cores (4-entry, 32-bit words) |
| **Doorbell interrupts** | `pico/multicore.h` | RP2350-specific: fast inter-core signaling without FIFO overhead |
| **Multicore lockout** | `pico/multicore.h` | Pause one core during flash operations |
| **Atomics** | `<stdatomic.h>` | Lock-free counters, flags, seqlock sequence numbers |
| **MPU (PMSAv8)** | ARM CMSIS | Per-core memory protection, stack guard regions |

**Note on spinlocks:** The RP2350 has 32 hardware spinlock instances (IDs 0-31). IDs 0-15 are reserved by the SDK and Pico runtime. IDs 16-23 are used for striped spin locks. IDs 24-31 are available for application use via `spin_lock_claim_unused()`. Additionally, due to RP2350 errata E17, the SDK may default to software spinlocks (`PICO_USE_SW_SPIN_LOCKS=1`) — verify which mode is active and document the implications.

---

### IVP-19: Core 1 LED Blink (Dual-Core Alive)

**Prerequisites:** IVP-08

**Implement:** Launch Core 1 via `multicore_launch_core1()`. Core 1 blinks NeoPixel. Core 0 continues red LED heartbeat + USB output. No shared data yet.

**[GATE]:**
- Red LED: heartbeat (Core 0)
- NeoPixel: visibly different blink rate (Core 1)
- Both stable simultaneously for 60 seconds
- USB serial output from Core 0 uninterrupted
- Print: `Core 0: main loop` / `Core 1: running`

**[DIAG]:** Core 1 doesn't start = check `multicore_launch_core1()`. NeoPixel doesn't blink but Core 1 running (verified via probe) = PIO state machine conflict.

---

### IVP-20: Cross-Core Shared Counter (Atomics)

**Prerequisites:** IVP-19

**Implement:** Core 1 increments an `_Atomic uint32_t` counter in a tight loop. Core 0 reads it every second and prints. Use `memory_order_relaxed` for the increment, `memory_order_acquire` for the read.

**[GATE]:**
- Counter increments monotonically (never decreases, no torn reads)
- Counter never reads as 0 after first second
- 5 minutes continuous: no glitches, no stalls
- Print actual increment rate (documents Core 1 loop throughput)

**[DIAG]:** Counter reads stale/zero = memory barriers missing. Plain `volatile` is NOT sufficient for cross-core on ARM Cortex-M33. Must use `<stdatomic.h>` or `__atomic_*` builtins.

**[LL]:** Entry 8 (cross-core memory visibility — volatile insufficient)

---

### IVP-21: Hardware Spinlock Validation

**Prerequisites:** IVP-20

**Implement:** Claim a hardware spinlock via `spin_lock_claim_unused()`. Core 1 acquires lock, updates a shared struct (non-atomic, multi-field), releases lock. Core 0 acquires same lock, reads struct, releases lock. Verify mutual exclusion.

**[GATE]:**
- `spin_lock_claim_unused()` succeeds (returns ID 24-31)
- Document whether SDK is using HW or SW spinlocks (`PICO_USE_SW_SPIN_LOCKS`)
- Shared struct never shows inconsistent state (partial update)
- Measure lock hold time: print min/max/avg in microseconds
- 5 minutes continuous: no deadlocks, no stalls
- Verify IRQ behavior: `spin_lock_blocking()` disables IRQs on the locking core — confirm USB IRQs on Core 0 are not disrupted when Core 1 holds a lock

**[DIAG]:** Deadlock = both cores trying to acquire same lock in nested fashion. Spinlocks are NOT recursive. IRQ disruption = lock held too long on Core 0. Keep critical sections < `⚠️ VALIDATE 10us`.

**[NOTE — Research 2026-02-06]:** RP2350 errata **E2** (SIO register aliasing) breaks hardware spinlocks — writes to SIO registers above offset `+0x180` alias spinlock registers, causing spurious releases. The SDK defaults to `PICO_USE_SW_SPIN_LOCKS=1` on RP2350, using `LDAEXB`/`STREXB` instead. This is transparent to all SDK APIs. Previous references to "E17" were incorrect — E17 was not found in SDK source.

---

### IVP-22: Multicore FIFO Message Passing

**Prerequisites:** IVP-19

**Implement:** Use `multicore_fifo_push_blocking()` and `multicore_fifo_pop_blocking()` to send 32-bit command words between cores. Core 0 sends commands, Core 1 acknowledges.

**[GATE]:**
- Send 1000 messages Core 0 → Core 1: all received, none lost
- Send 1000 messages Core 1 → Core 0: all received, none lost
- Round-trip latency: measure and record (expected `⚠️ VALIDATE <5us`)
- FIFO overflow test: send 5 messages without reading — 5th blocks (FIFO is 4-deep on RP2350, verify)
- No interference with USB or sensor polling

**[DIAG]:** Messages lost = FIFO overflow. `multicore_fifo_push_blocking()` will spin until space available. Use `_timeout_us` variant if blocking is unacceptable. Note: `multicore_lockout` uses the FIFO internally — cannot use FIFO for application messages while lockout is active.

**[NOTE — Research 2026-02-06]:** This step exercises the FIFO primitive but the FIFO **cannot be used for app messaging in the final architecture**. `multicore_lockout_victim_init()` (called by `flash_safe_execute`, needed for IVP-28) registers an exclusive handler on the FIFO IRQ, claiming it permanently. RP2350 FIFO is also only 4-deep (halved from RP2040's 8), making it unsuitable for high-rate notification.

---

### IVP-23: Doorbell Interrupts (RP2350-Specific)

**Prerequisites:** IVP-19

**Implement:** Use RP2350 doorbell registers for lightweight inter-core signaling. Claim a doorbell via `multicore_doorbell_claim_unused()`. Core 1 sets doorbell after completing a sensor read cycle; Core 0 checks/clears doorbell to know new data is available (alternative to polling the seqlock sequence counter).

**[GATE]:**
- `multicore_doorbell_claim_unused()` succeeds
- Core 1 sets doorbell: Core 0 detects it via `multicore_doorbell_is_set_current_core()`
- Latency from set to detect: measure and record
- 1000 doorbell signals: none missed
- Doorbell clear on Core 0 does not affect Core 1
- No interference with FIFO or lockout mechanisms

**[DIAG]:** Doorbell not detected = wrong core mask in claim, or not checking correct doorbell number. Doorbells are RP2350-only — will not compile for RP2040 targets.

**[NOTE — Research 2026-02-06]:** Doorbells are validated here but **polling the seqlock sequence counter is the chosen notification mechanism** for production. At 200Hz Core 0 loop rate, the sequence check costs 5-7 cycles (0.003% of loop budget) and always finds fresh IMU data. Doorbells save ~5ms latency the fusion loop can't use. Doorbells become relevant if RocketChip moves to a sleep-based power architecture (WFI/WFE).

---

### IVP-24: Seqlock Single-Buffer

**Prerequisites:** IVP-20

**Design reference:** `docs/decisions/SEQLOCK_DESIGN.md` — council-reviewed struct layout (~116 bytes), barrier requirements, and rationale.

**Implement:** Single-buffer seqlock for cross-core sensor data sharing. `shared_sensor_data_t` (~116 bytes) in static SRAM, wrapped in `sensor_seqlock_t` with `_Atomic uint32_t sequence`:
- Writer (Core 1): increment seq to odd (`memory_order_release`), `__dmb()`, write data, `__dmb()`, increment seq to even (`memory_order_release`)
- Reader (Core 0): read seq (`memory_order_acquire`), `__dmb()`, copy data, `__dmb()`, read seq again, retry if mismatch or odd

**[GATE]:**
- Core 1 writes test struct (incrementing counter + known pattern) at `⚠️ VALIDATE 1kHz`
- Core 0 reads at `⚠️ VALIDATE 200Hz`, verifies data consistency
- **Zero torn reads** over 5 minutes
- Retry count < 1% of reads
- Print: `Seqlock: N reads, N retries, 0 torn`

**[DIAG]:** Torn reads = barriers wrong or seqlock logic bug. Test with intentional delay in writer to widen the write window and verify retry logic.

**[LL]:** Entry 8 (memory barriers)

---

### IVP-25: Core 1 Sensor Sampling

**Prerequisites:** IVP-24, IVP-13

**Implement:** Move IMU reads to Core 1 in a tight polling loop using `time_us_64()`. Write sensor data to seqlock buffer. Core 0 reads and prints. Target rate: `⚠️ VALIDATE 1kHz` — actual achievable rate depends on I2C transaction time measured in IVP-13.

**[GATE]:**
- **Measure actual I2C transaction time**: IMU full read = Xus (record this — informs max achievable rate)
- IMU sample rate: within 1% of target (target derived from measured I2C time)
- Jitter: record 1000 consecutive sample timestamps, report std deviation
- Core 0 reads valid data from seqlock
- USB output uninterrupted
- 5 minutes continuous: no hangs, no USB disconnects, no I2C errors

**[DIAG]:** Rate lower than expected = I2C transaction time longer than estimated. A full ICM-20948 read (accel+gyro+temp+mag via bypass) may take `⚠️ VALIDATE 400-500us` at 400kHz. Jitter high = check for interrupt contention. USB breaks = see LL Entry 12.

**[LL]:** Entry 12 (USB/flash interaction)

---

### IVP-26: Baro on Core 1

**Prerequisites:** IVP-25

**Implement:** Add `baro_dps310_read()` to Core 1 loop. Baro reads interleaved with IMU at a lower rate divider. Both sensors publish through seqlock.

**[GATE]:**
- IMU rate unchanged from IVP-25
- Baro rate: within 10% of target
- No I2C bus errors over 60 seconds (both devices share I2C1)
- **Measure total I2C time per cycle**: IMU + baro combined = Xus (record — this is the hard budget limit)
- Both datasets valid on Core 0

**[DIAG]:** I2C errors = bus arbitration. Reads must be sequential on same I2C bus. If total I2C time exceeds the cycle period, reduce baro frequency or reduce IMU rate.

---

### IVP-27: USB Stability Under Dual-Core Load

**Prerequisites:** IVP-25

**Implement:** Stress test: Core 1 samples sensors, Core 0 prints sensor summary while CLI remains responsive.

**[GATE]:**
- **10 minutes continuous** — no USB disconnect
- CLI responsive: press `h`, get response within 1 second
- Disconnect terminal, wait 60s, reconnect: output resumes, no crash
- Rapidly press keys during sensor output: no crash, no hang

**[DIAG]:** USB disconnects = check for flash operations blocking interrupts. CLI hangs = `getchar_timeout_us(0)` must be non-blocking. Always guard with `stdio_usb_connected()`.

**[LL]:** Entry 10 (USB/scheduler dependency — bare-metal avoids this), Entry 15 (terminal guard)

---

### IVP-28: Flash Operations Under Dual-Core (Multicore Lockout)

**Prerequisites:** IVP-27, IVP-14

**Implement:** Test `calibration_save()` while Core 1 is actively sampling. Use `flash_safe_execute()` which internally uses `multicore_lockout` to coordinate both cores during flash erase/program.

Core 1 must call `multicore_lockout_victim_init()` early in its entry function. This registers Core 1 as a lockout participant so it enters a RAM-resident spin loop when Core 0 initiates a flash operation.

**[GATE]:**
- `calibration_save()` returns `CAL_RESULT_OK` while Core 1 is sampling
- Core 1 pauses during flash op (expected behavior of `multicore_lockout`)
- After flash op: Core 1 resumes, no data loss, no crash
- **Measure flash op duration** (record — this is the sensor data gap)
- USB remains functional after flash op
- Power cycle: data persists
- Repeat save 5 times: all succeed
- Note: `multicore_lockout` uses the inter-core FIFO internally — verify FIFO is not used for application messages simultaneously

**[DIAG]:** Crash during flash = `multicore_lockout_victim_init()` not called on Core 1. USB breaks = LL Entries 4, 12. Core 1 doesn't resume = lockout release bug or FIFO conflict.

**[LL]:** Entry 4 (flash makes ALL flash inaccessible), Entry 12 (flash/USB ordering)

---

### IVP-29: MPU Stack Guard Regions

**Prerequisites:** IVP-19

**Implement:** Configure Cortex-M33 PMSAv8 MPU on each core: no-access guard region at bottom of each core's stack. Stack overflow triggers synchronous MemManage fault instead of silent corruption.

PMSAv8 configuration:
- Guard region size: 64 bytes minimum (PMSAv8 requires 32-byte alignment minimum; 64 bytes provides margin)
- Region attributes: no-access (read/write disabled)
- Each core has its own MPU — configure independently
- RP2350 supports 8 MPU regions per core — one region per core for stack guard

**[GATE]:**
- Intentional overflow on Core 0 (recursive function): MemManage fault triggers
- Intentional overflow on Core 1 (recursive function): MemManage fault triggers
- Fault handler: NeoPixel solid red, red LED distinct pattern (3 fast + 1 slow)
- Normal operation: no false faults over 5 minutes
- MPU regions verified via GDB: `info mem` shows guard regions

**[DIAG]:** MPU config fails = check region doesn't conflict with PSRAM XIP mapping. Fault handler doesn't trigger = guard region may not cover the actual stack bottom. Check stack placement with linker map file. RP2350 PSRAM is memory-mapped via XIP — ensure MPU region doesn't overlap.

**[LL]:** Entry 1 (stack overflow), Entry 19 (stack overflow during calibration)

---

### IVP-30: Hardware Watchdog

**Prerequisites:** IVP-27

**Implement:** Enable RP2350 hardware watchdog via `watchdog_enable()` with `⚠️ VALIDATE 5000ms` timeout and `pause_on_debug=1`. Dual-core kick pattern: both cores must set a flag; only when both flags set does the watchdog get kicked via `watchdog_update()`.

**[GATE]:**
- Normal operation: watchdog does not fire over 5 minutes
- Stall Core 0 (infinite loop): watchdog resets within timeout
- Stall Core 1 (infinite loop): watchdog resets within timeout
- After reset: `watchdog_caused_reboot()` returns true
- Debug probe connected: watchdog pauses (allows GDB without spurious resets)

**[DIAG]:** Fires during normal operation = kick interval too long or one core not setting its flag. One core stalling doesn't trigger = dual-core flag logic bug. Note: if OpenOCD disconnects uncleanly, `pause_on_debug` may not function — power cycle if watchdog fires unexpectedly during debug.

---

## Stage 4: GPS Navigation

**Purpose:** Bring up PA1010D GPS module, parse NMEA, validate fix quality. GPS measurement data is required as an input to the ESKF (Stage 5), so it must be integrated and validated first.

**Architecture note (revised 2026-02-06):** The original Stage 4 IVPs (4 steps) were written before Stage 3 established that Core 1 owns the I2C bus exclusively. The PA1010D shares the I2C bus with IMU and baro — Core 0 cannot read GPS without bus collisions (see LL Entry 20). These IVPs are restructured to 3 steps: GPS runs on Core 1 from the start, publishing data to Core 0 via the seqlock established in IVP-24.

---

### IVP-31: PA1010D GPS Init + Core 1 Integration

**Prerequisites:** IVP-26 (baro on Core 1 via seqlock), IVP-07 (I2C bus functional)

**Implement:**

1. **GPS init on Core 0, before `multicore_launch_core1()`:** Call `gps_pa1010d_init()` which probes 0x10, initializes lwGPS parser, and sends PMTK314 to enable only RMC+GGA sentences (reduces parsing overhead, ~139 bytes/sec vs ~449 default). GPS init must complete before Core 1 launches because both share the I2C bus with no mutual exclusion. Record init result; do not block boot if GPS is absent.

2. **GPS polling on Core 1 sensor loop:** Add `gps_pa1010d_update()` call to the Core 1 loop, interleaved with IMU and baro reads. Poll at 10Hz (every ~100 IMU cycles). Read the full 255-byte MT3333 TX buffer per vendor recommendation (GlobalTop/Quectel app notes: "read full buffer, partial reads not recommended"). Pico SDK `i2c_read_blocking()` has no upper limit — the 32-byte Arduino pattern was a Wire.h software limitation. At 400kHz, 255 bytes takes ~5.8ms; at 10Hz this affects 10 of 1000 IMU cycles/sec. The lwGPS parser is stateful and handles partial NMEA sentences across calls. Minimum 2ms between successive GPS reads (MT3333 TX buffer refill time per GlobalTop app note).

3. **Publish GPS data via seqlock:** After `gps_pa1010d_update()`, copy parsed GPS fields into `shared_sensor_data_t` GPS section (`gps_lat_1e7`, `gps_lon_1e7`, `gps_alt_msl_m`, etc.) and publish via the existing `seqlock_write()`. Convert `double` lat/lon to `int32_t * 1e7` (ArduPilot convention, avoids soft-float on Cortex-M33). Increment `gps_read_count` on each successful parse. Set `gps_valid` based on fix type >= 2D.

4. **Track previous `gps_read_count`** on Core 0 — compare against seqlock snapshot to detect genuinely new GPS data vs repeated reads (council modification).

**[GATE]:**
- Print: `PA1010D init OK at 0x10` (or `PA1010D not detected` — non-fatal)
- Core 1 loop rate remains within 5% of pre-GPS baseline (~1kHz)
- **Measure GPS I2C read time** (record — expected ~5.8ms for 255 bytes at 400kHz)
- IMU and baro rates unaffected by GPS reads
- Raw NMEA data received: print first 10 NMEA sentences to confirm `$GPRMC` and `$GPGGA` arriving
- If indoors (no fix): `gps_fix_type = 0`, `gps_satellites = 0` — this is expected
- `gps_read_count` increments on Core 0's seqlock snapshot
- GPS poll interval is ~100ms (10Hz), measured from Core 1 timestamps

**[DIAG]:** No response at 0x10 = GPS not plugged into Qwiic chain (physically removed during Stage 2, reconnect now). NMEA garbled = 0x0A padding not filtered. IMU rate drops more than expected = measure actual GPS read time, verify 10Hz poll rate. `gps_read_count` stuck at 0 = `gps_pa1010d_update()` not being called on Core 1, or PMTK314 command failed.

**[LL]:** Entry 20 (PA1010D bus interference), Entry 21 (I2C master race condition — same shared-bus principle)

---

### IVP-32: GPS Fix and Outdoor Validation

**Prerequisites:** IVP-31, outdoor testing required

**Implement:** Take the device outdoors with open sky view. Core 0 reads GPS data from seqlock (not direct I2C). Print parsed fields from the seqlock snapshot: lat/lon, altitude, speed, satellite count, fix type, HDOP, time/date. No new driver code needed — this validates the end-to-end pipeline from PA1010D → Core 1 I2C read → lwGPS parse → seqlock → Core 0 display.

**[GATE]:**
- Satellite count > 0 (outdoor, open sky)
- Fix type: 2 (2D) or 3 (3D)
- Latitude/longitude: plausible for test location (read from seqlock `gps_lat_1e7`/`gps_lon_1e7`, convert back to degrees for display)
- Altitude MSL: plausible for test location (+/-50m)
- Ground speed: near zero when stationary
- Time to first fix: record (expected `⚠️ VALIDATE <60s` cold start)
- `gps_read_count` incrementing at ~10Hz
- No NaN, no wildly jumping values between consecutive fixes
- Core 1 IMU/baro rates unchanged from IVP-31 indoor test

**[DIAG]:** No fix outdoors = antenna obstructed or GPS module not configured for correct constellation. Zero satellites = check antenna connection. Position jumps = multipath (expected near buildings). `gps_valid` stays false despite satellites = fix_mode not reaching 2D, check GSA sentence parsing.

---

### IVP-33: GPS CLI Integration

**Prerequisites:** IVP-32, IVP-18 (RC_OS CLI)

**Implement:** Add GPS data to CLI `s` (status) command output. Core 0 reads GPS fields from its seqlock snapshot (already being read for IMU/baro status). Show fix status, satellite count, position (degrees with 7 decimal places), altitude, ground speed, HDOP, UTC time. Handle graceful degradation when GPS module is absent (init failed) or has no fix.

**[GATE]:**
- CLI `s` command shows GPS data when module connected and has fix
- CLI `s` command shows `GPS: no fix (N sats)` when module connected but no fix
- CLI `s` command shows `GPS: not detected` when module absent
- Position data matches known test location (outdoor)
- Displayed `gps_read_count` increments confirm live data flow
- No additional I2C traffic from Core 0 — all GPS data comes from seqlock

---

## Stage 5: Sensor Fusion

**Purpose:** ESKF sensor fusion, incrementally: math library, then simple baro filter, then full 15-state ESKF, then MMAE bank. See `docs/decisions/ESKF/FUSION_ARCHITECTURE.md`.

**Prerequisite decision:** All numerical parameters in this stage (state counts, filter counts, process noise values, measurement noise values, convergence thresholds) are preliminary. Each must be derived from datasheets, Sola (2017), and empirical tuning. Do not hardcode without justification.

---

### IVP-34: Vector3 and Quaternion Math Library

**Prerequisites:** IVP-01

**Implement:** `src/math/Vector3.h`, `src/math/Quaternion.h`. Operations: add, subtract, cross, dot, normalize, quaternion multiply/conjugate/rotate/from_euler/to_euler.

**[GATE]:** Unit tests (host-compiled, with ASan/UBSan) pass with float tolerance. Edge cases: zero vector normalize, identity quaternion, 90-degree rotations. Run on host first, then verify on target.

---

### IVP-35: Matrix Operations

**Prerequisites:** IVP-34

**Implement:** `src/math/Matrix.h` — multiply, transpose, add, subtract, inverse (up to 15x15), Cholesky decomposition. May wrap CMSIS-DSP `arm_matrix_instance_f32`.

**[GATE]:** Unit tests for known matrices (host-compiled first, then target). **Benchmark on target:** 15x15 multiply = Xus (record — informs ESKF timing budget). Cholesky of known positive-definite matrix matches expected result.

---

### IVP-36: 1D Barometric Altitude Kalman Filter

**Prerequisites:** IVP-35, IVP-12, IVP-25

**Implement:** 2-state (altitude, vertical velocity) Kalman filter using baro. Runs on Core 0. Process noise and measurement noise values: `⚠️ VALIDATE` — derive from DPS310 datasheet noise floor and empirical baro variance measured in IVP-12.

**[GATE]:** Raise device 1 meter — filter reports ~1m change with less noise than raw baro. Velocity transitions from zero to positive to zero. Converges within reasonable time after boot (record actual convergence time).

---

### IVP-37: ESKF Propagation (IMU-Only)

**Prerequisites:** IVP-35, IVP-34, IVP-25

**Implement:** ESKF nominal state propagation (accel + gyro). `⚠️ VALIDATE 15-state` error covariance prediction. No measurement updates yet. Process noise Q matrix values: `⚠️ VALIDATE` — derive from ICM-20948 datasheet noise density specs.

**[GATE]:**
- Stationary: attitude drift measured and recorded (expected `⚠️ VALIDATE <5 degrees` over 30 seconds)
- Rotate 90 degrees: tracks correctly (within `⚠️ VALIDATE 10 degrees`)
- **Benchmark execution time per step** on target (record — informs fusion rate)
- No NaN in state vector or covariance matrix

**Reference:** Sola (2017) "Quaternion kinematics for the error-state Kalman filter"

---

### IVP-38: ESKF Barometric Altitude Update

**Prerequisites:** IVP-37, IVP-36

**Implement:** Baro measurement update for altitude and vertical velocity correction. Measurement noise R: `⚠️ VALIDATE` — derive from DPS310 spec and IVP-12 measurements.

**[GATE]:** Altitude tracks raw baro with reduced noise. Raise 1m: error measured and recorded. No divergence over 5 minutes.

---

### IVP-39: ESKF Magnetometer Heading Update

**Prerequisites:** IVP-37

**Implement:** Mag measurement update for yaw correction. Requires completed mag calibration. Mag noise values: `⚠️ VALIDATE` — derive from AK09916 noise spec.

**[GATE]:** Yaw drift measured when stationary (record rate). 360-degree rotation returns to approximate original. Recovers from nearby magnetic interference.

---

### IVP-40: Mahony AHRS Cross-Check

**Prerequisites:** IVP-37

**Implement:** Independent Mahony AHRS on same sensor data, running alongside ESKF. Kp/Ki gains: `⚠️ VALIDATE` — reference Mahony (2008) and tune empirically.

**[GATE]:** Attitude difference ESKF vs Mahony measured during normal operation (record). Reconverges after fast rotation. Alarm threshold: `⚠️ VALIDATE` — determine empirically from divergence during known-good operation.

---

### IVP-41: GPS Measurement Update

**Prerequisites:** IVP-38, IVP-33

**Implement:** GPS position + velocity measurement updates to ESKF. GPS noise R matrix: `⚠️ VALIDATE` — derive from PA1010D position accuracy spec and HDOP.

**[GATE]:** TBD — requires outdoor testing. Position converges to GPS. Velocity matches GPS ground speed. Record actual convergence time.

---

### IVP-42: MMAE Bank Manager (Titan Tier)

**Prerequisites:** IVP-37, IVP-38, IVP-39

**Implement:** `⚠️ VALIDATE 4-6` parallel ESKFs with different process model hypotheses. Weighting via innovation likelihood.

**[GATE]:** TBD — requires motion profiles. Nominal hypothesis dominates during static conditions. Anomaly (covered baro port) shifts weight to non-nominal hypothesis.

---

### IVP-43: Confidence Gate (Titan Tier)

**Prerequisites:** IVP-42, IVP-40

**Implement:** Evaluate MMAE weights + AHRS divergence. Output confidence flag to mission engine.

**[GATE]:** TBD — define "uncertain" threshold. Gate correctly flags uncertainty during intentional anomalies.

---

## Stage 6: Mission Engine — *TBD after Stage 5*

**Purpose:** Flight state machine and event-driven actions. See SAD.md Section 6.

**Prerequisite decision:** Resolve SAD Open Question #4 (Mealy vs Moore state machine) before implementing.

| Step | Title | Brief Description |
|------|-------|------------------|
| IVP-44 | State Machine Core | IDLE/ARMED/BOOST/COAST/DESCENT/LANDED states and transitions |
| IVP-45 | Event Engine | Condition evaluator for launch, apogee, landing detection |
| IVP-46 | Action Executor | LED, beep, logging trigger actions on state transitions |
| IVP-47 | Confidence-Gated Actions | Irreversible actions (pyro) held when confidence flag low |
| IVP-48 | Mission Configuration | Load mission definitions (rocket, freeform) |

> **Milestone:** At IVP-48 completion, the system has calibrated sensors, GPS, sensor fusion, and a working flight state machine — **Crowdfunding Demo Ready**.

---

## Stage 7: Data Logging — *TBD after Stage 6*

**Purpose:** Flight data storage. See SAD.md Sections 8, 9.

| Step | Title | Brief Description |
|------|-------|------------------|
| IVP-49 | LittleFS Integration | Mount filesystem on remaining flash |
| IVP-50 | Logger Service | Buffered writes at `⚠️ VALIDATE 50Hz` |
| IVP-51 | Pre-Launch Ring Buffer | PSRAM ring buffer for pre-launch capture |
| IVP-52 | Log Format | MAVLink binary format, readable by Mission Planner/QGC |
| IVP-53 | USB Data Download | Download flight logs via CLI |

---

## Stage 8: Telemetry — *TBD after Stage 7*

**Purpose:** MAVLink over LoRa radio. See SAD.md Sections 3.2, 8.

| Step | Title | Brief Description |
|------|-------|------------------|
| IVP-54 | RFM95W Radio Driver | SPI-based LoRa radio init, TX/RX |
| IVP-55 | MAVLink Encoder | Heartbeat, attitude, GPS, system status messages |
| IVP-56 | Telemetry Service | `⚠️ VALIDATE 10Hz` downlink with message prioritization |
| IVP-57 | GCS Compatibility | QGroundControl / Mission Planner connection |
| IVP-58 | Bidirectional Commands | GCS sends calibrate, arm, parameter set commands |

---

## Stage 9: System Integration — *TBD after Stage 8*

**Purpose:** Full system verification and flight readiness.

| Step | Title | Brief Description |
|------|-------|------------------|
| IVP-59 | Full System Bench Test | All subsystems running `⚠️ VALIDATE 30 minutes`, no crashes |
| IVP-60 | Simulated Flight Profile | Replay recorded accel/baro through state machine |
| IVP-61 | Power Budget Validation | Battery runtime validation under flight load |
| IVP-62 | Environmental Stress | Temperature range, vibration (if available) |
| IVP-63 | Flight Test | Bungee-launched glider: full data capture + telemetry |

> **Milestone:** IVP-63 — **Flight Ready**.

---

## Regression Test Matrix

Tests to re-run after changes to specific areas.

| Trigger | Re-run | Rationale |
|---------|--------|-----------|
| CMakeLists.txt change | IVP-01 | Build integrity |
| config.h change | IVP-01, IVP-05 | Constants affect all modules |
| Sensor driver change | IVP-09 — IVP-13a, IVP-25 — IVP-26 | Driver + integration |
| Inter-core primitive change | IVP-20 — IVP-24, IVP-27 | Atomics, spinlocks, FIFO, seqlock, USB stability |
| Flash/storage code change | IVP-14, IVP-28 | Persistence + dual-core safety |
| Calibration code change | IVP-15 — IVP-17 | Calibration accuracy |
| USB/CLI code change | IVP-04, IVP-18, IVP-27 | USB stability |
| GPS driver change | IVP-31 — IVP-33 | GPS + integration |
| Fusion algorithm change | IVP-36 — IVP-43 | Filter correctness |
| Mission engine change | IVP-44 — IVP-48 | State machine correctness |
| **Major refactor** | **All Stage 1-3** | Full regression |
| **Before any release** | IVP-01, IVP-27, IVP-28, IVP-30, IVP-59 | Release qualification (build, USB, flash, watchdog, bench test) |

---

## Document Maintenance

- **Completed steps:** Mark with date and commit hash. Do not delete — they form the verification record.
- **Failed steps:** Document failure mode, root cause, resolution. Add LESSONS_LEARNED entry if warranted.
- **New steps:** Assign next available IVP-XX number when TBDs are resolved.
- **Retired steps:** Mark as RETIRED with rationale. Do not renumber.

---

*Document maintained in: `docs/IVP.md`*
*Architecture reference: `docs/SAD.md`*
*Debugging journal: `.claude/LESSONS_LEARNED.md`*
