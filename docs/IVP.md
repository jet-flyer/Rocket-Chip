# RocketChip Integration and Verification Plan (IVP)

**Status:** ACTIVE — Living document
**Last Updated:** 2026-02-24
**Target Platform:** RP2350 (Adafruit Feather HSTX w/ 8MB PSRAM)
**Architecture:** Bare-metal Pico SDK, dual-core AMP (see `docs/decisions/SEQLOCK_DESIGN.md`)

---

## 1. Purpose and Scope

This document defines the step-by-step integration order for RocketChip firmware, from LED blink through full ESKF sensor fusion and telemetry. Every step has a concrete verification gate — a pass/fail test that must succeed before proceeding.

**This is a living document:**
- **Stages 1-4** (Foundation, Sensors, Dual-Core, GPS): Fully detailed
- **Phase M** (Magnetometer Calibration): Fully detailed — added out-of-sequence to correct a missed Phase 3 dependency (see note below)
- **Stage 5** (Sensor Fusion): Fully detailed
- **Stages 6-10** (Flight Director, Adaptive Estimation, Logging, Telemetry, Integration): Placeholders expanded as earlier stages complete

**Key references (not duplicated here):**
- `docs/SAD.md` — System architecture, data structures, module responsibilities
- `include/rocketchip/config.h` — Pin assignments, I2C addresses, timing constants
- `.claude/LESSONS_LEARNED.md` — Debugging journal (referenced as "LL Entry N")
- `docs/decisions/ESKF/FUSION_ARCHITECTURE.md` — Sensor fusion design
- `docs/decisions/SEQLOCK_DESIGN.md` — Council decision on bare-metal dual-core and seqlock pattern

**Numerical values:** Rates, thresholds, buffer sizes, and timing values throughout this document are **preliminary targets** unless marked `[VALIDATED]`. Each must be justified from a datasheet, SDK measurement, or empirical test before committing to implementation. Do not treat them as specifications — they are starting points for the verification gate to confirm or revise.

**Conventions:**
- Each step has a unique ID (`IVP-XX`) for traceability
- `[GATE]` = verification test. Step is not complete until the gate passes.
- `[DIAG]` = failure diagnosis hints
- `[LL]` = LESSONS_LEARNED cross-reference
- `⚠️ VALIDATE` = numerical value requires measurement/datasheet confirmation before implementation

**Stage completion rule:** When all IVP steps in a Stage are verified, **strip or refactor all gate/soak/diagnostic test code** from that Stage out of production source files. `main.cpp` should contain only the verified behavior, not the test scaffolding that proved it. Silencing prints via skip flags is a temporary measure during active development — not a substitute for cleanup. Each Stage boundary is a cleanup checkpoint.

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
| **M** | **Magnetometer Calibration** | **Phase 3** | **IVP-34 — IVP-38** | **Full** | **(out-of-sequence)** |
| 5 | Sensor Fusion | Phase 4 | IVP-39 — IVP-48 | Full | |
| **6** | **Radio & Telemetry** | **Phase 5** | **IVP-63 — IVP-67** | **Placeholder** | **(pulled forward)** |
| 7 | Flight Director | Phase 6 | IVP-49 — IVP-53 | Placeholder | **Crowdfunding Demo Ready** |
| 8 | Adaptive Estimation & Safety | Phase 6 | IVP-54 — IVP-57 | Placeholder | |
| 9 | Data Logging | Phase 7 | IVP-58 — IVP-62 | Placeholder | |
| 10 | System Integration | Phase 9 | IVP-68 — IVP-72 | Placeholder | **Flight Ready** |

> **Stage 6 pull-forward rationale:** Radio & Telemetry was originally Stage 9 but has zero dependencies on Stages 7–9 (Flight Director, Adaptive Estimation, Data Logging). A live radio link enables untethered dynamic validation tests (turntable, pendulum, vehicle GPS-vs-INS) and real-time debugging — both of which accelerate remaining development. SPI bus is independent from I2C sensors. IVP step numbers are unchanged; only stage ordering changed. A full IVP number renumber is deferred to the next substantial restructuring.

> **IVP-64 re-evaluation pending:** Original scope was MAVLink encoding. Project is pivoting to CCSDS telemetry. IVP-64 and IVP-66 scopes will be redefined when Stage 6 reaches those steps.

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

## Phase M: Magnetometer Calibration (Out-of-Sequence)

**Purpose:** Full compass calibration with ArduPilot-parity ellipsoid fit. Corrects a missed Phase 3 dependency — IVP-44 (ESKF Mag Heading Update) requires completed magnetometer calibration, but no mag calibration was included in the original plan. GPS (Stage 4) is already validated; Phase M slots in before sensor fusion begins.

**Why "Phase M":** This phase was added out-of-sequence after Stages 1-4 were completed. The "M" designation marks this as a corrective insertion for a missed dependency. IVP numbering is inline (IVP-34 through IVP-38) with downstream stages renumbered +5 accordingly.

**ArduPilot parity target:** The calibration algorithm matches ArduPilot's `CompassCalibrator` class (two-step Levenberg-Marquardt with sphere-coverage acceptance). This is one of ArduPilot's key reliability features — the sphere-coverage constraint ensures uniform sample distribution, preventing degenerate fits from planar or clustered data.

**Prior art:** Previous implementation on the `AP_FreeRTOS` branch reached milestone 4 of 5 (synthetic test passing, hardware test with LIS3MDL). Key lessons from that branch: CompassCalibrator is ~3KB+ (must be static, LL Entry 1), `report.ofs` is the correction offset (negative of hard iron), `thin_samples()` stalls Step 2 if sample feeding stops, and near-PC magnetic interference causes `fit_acceptable()` rejections.

**Hardware setup for mag calibration:** Disconnect baro (DPS310) and GPS (PA1010D) from the Qwiic chain during calibration. Only the ICM-20948 (IMU + AK09916 magnetometer) is needed. This eliminates I2C bus contention and frees bandwidth for optional OLED display feedback.

**AK09916 magnetometer specs (via ICM-20948 I2C master):**
- Resolution: 0.15 uT/LSB
- Measurement range: +/-4900 uT
- Continuous mode 4: 100Hz
- Noise: ~0.1 uT RMS (per AKM datasheet)
- Expected field magnitude: 25-65 uT (varies by location)

---

### IVP-34: Magnetometer Calibration Data Structure + Storage

**Prerequisites:** IVP-14 (calibration storage), IVP-17 (6-pos accel cal — same model)

**Implement:**

1. **Add `mag_cal_t` struct to `calibration_data.h`:**
   ```cpp
   typedef struct {
       cal_vec3_t offset;      ///< Hard iron offset in uT (add to raw)
       cal_vec3_t scale;       ///< Soft iron diagonal scale factors
       cal_vec3_t offdiag;     ///< Soft iron off-diagonal terms (XY, XZ, YZ)
       float expected_radius;  ///< Expected field magnitude in uT (from fit)
       float temperature_ref;  ///< Temperature at calibration time (C)
       uint8_t status;         ///< CAL_STATUS_MAG if calibrated
       uint8_t _pad[3];
   } mag_cal_t;  // 48 bytes
   ```
   This uses the identical 9-parameter model as `accel_cal_t` (offset + diag + offdiag), matching ArduPilot's `CompassCalibrator::param_t`. The `expected_radius` stores the fitted sphere radius for post-calibration validation.

2. **Add `mag_cal_t mag;` field to `calibration_store_t`:**
   Place after `baro_cal_t baro;`. Total struct size increases from ~132 to ~180 bytes (within 256-byte flash page limit). Bump `kCalibrationVersion` from 2 to 3. The version bump triggers `calibration_init_defaults()` for the mag field on first boot after update — existing accel/gyro/baro calibrations are preserved via the migration path in `calibration_load()`.

3. **Add `calibration_apply_mag()` and `calibration_apply_mag_with()` functions:**
   Same signature pattern as existing `calibration_apply_accel()` / `calibration_apply_accel_with()`:
   ```cpp
   void calibration_apply_mag(float mx_raw, float my_raw, float mz_raw,
                               float* mx_cal, float* my_cal, float* mz_cal);
   void calibration_apply_mag_with(const calibration_store_t* cal,
                                    float mx_raw, float my_raw, float mz_raw,
                                    float* mx_cal, float* my_cal, float* mz_cal);
   ```
   Correction formula: `corrected = M * (raw + offset)` where M is the symmetric 3x3 matrix with scale (diagonal) and offdiag terms — identical to the accel correction in `calibration_apply_accel()`.

**[GATE]:**
- `sizeof(calibration_store_t) <= 256` (static_assert passes)
- `kCalibrationVersion == 3`
- Default mag cal: offset={0,0,0}, scale={1,1,1}, offdiag={0,0,0}, expected_radius=0
- Apply identity cal to raw mag data: output equals input
- Save/load cycle with mag data: values persist across power cycle
- Upgrade from v2 calibration: accel/gyro/baro fields preserved, mag initialized to defaults

**[DIAG]:** Static_assert fails = struct padding. Check alignment of new field. Load fails after version bump = migration path in `calibration_load()` not handling v2→v3.

---

### IVP-35: Sphere-Coverage Sample Collection Engine

**Prerequisites:** IVP-34, IVP-10 (IMU data validated — AK09916 mag data available)

**Implement:**

1. **Static sample buffer:**
   ```cpp
   static float g_mag_samples[300][3];  // 3.6KB — same pattern as g_6pos_samples
   static uint16_t g_mag_sample_count = 0;
   ```
   300 samples matches ArduPilot's `COMPASS_CAL_DEFAULT_BUFFER_LENGTH`. Buffer is static (LL Entry 1).

2. **Sphere-coverage acceptance criterion (ArduPilot parity):**
   Each new sample must have minimum angular separation from ALL existing samples. The threshold is computed from a circle-packing formula:
   ```
   min_angular_separation = acos(1 - 2/N)  where N = current sample count
   ```
   This produces ~8.3 degrees at 300 samples. Samples that are too close to existing samples are rejected — this forces the user to rotate the device to cover the full sphere.

   The angular separation is computed as:
   ```cpp
   float dot = (s1[0]*s2[0] + s1[1]*s2[1] + s1[2]*s2[2]) / (len1 * len2);
   float angle = acosf(fmaxf(-1.0f, fminf(1.0f, dot)));  // clamp for numerical safety
   if (angle < min_sep) reject;
   ```

3. **Geodesic completion mask (80 sections, 10 bytes):**
   ArduPilot uses an icosahedron with 4 subdivisions (20 faces x 4 = 80 sections) to track which regions of the sphere have been covered. Each section maps to 1 bit in a `uint8_t[10]` mask. Progress = popcount(mask) / 80. This provides meaningful progress feedback rather than raw sample count.

   Section lookup: normalize sample vector, map to icosahedron face via dot product with 20 face normals, then subdivide within face.

4. **Sample feed function:**
   ```cpp
   // Returns true if sample was accepted, false if too close to existing
   bool calibration_feed_mag_sample(float mx, float my, float mz);
   uint16_t calibration_get_mag_sample_count(void);
   uint8_t calibration_get_mag_completion_pct(void);  // 0-100 from geodesic mask
   ```

**[GATE]:**
- Feed 300 samples from uniform spherical distribution (synthetic): all accepted, completion 100%
- Feed 300 samples from a plane (e.g., only rotating around Z): rejected after ~20, completion stays low (~15%)
- Feed duplicate samples: rejected (angular separation check)
- Sample count never exceeds 300
- Completion percentage tracks actual sphere coverage (not just sample count)
- Memory: verify `g_mag_samples` is in BSS, not stack

**[DIAG]:** All samples rejected = threshold too high. Check units (radians, not degrees). Completion stuck at low percentage = device not being rotated through enough orientations. Synthetic test should use Fibonacci sphere sampling for uniform distribution.

---

### IVP-36: Two-Step Levenberg-Marquardt Compass Fit

**Prerequisites:** IVP-35, IVP-17 (existing Gauss-Newton solver — reusable Jacobian math)

**Implement:**

ArduPilot's compass calibration uses a two-step process. Both steps use the same Levenberg-Marquardt (damped least-squares) optimizer but with different parameter sets.

1. **Step 1: Sphere fit (4 parameters, 10 iterations)**
   - Parameters: `[offset_x, offset_y, offset_z, radius]`
   - Residual: `r_i = radius - |sample_i + offset|`
   - Initial guess: offset = -mean(samples), radius = mean(|samples + offset|)
   - LM damping: lambda starts at 1.0, multiply by 10 on worse fit, divide by 10 on better fit
   - This finds the hard iron offset (dominant error source)

2. **Fisher-Yates thinning between steps:**
   After Step 1 converges, thin the sample buffer from N to `N * 2/3` using Fisher-Yates shuffle. This removes correlated samples and improves Step 2 convergence. ArduPilot's `thin_samples()` implementation.

3. **Step 2: Ellipsoid fit (9 parameters, 20 iterations)**
   - Parameters: `[offset_x, offset_y, offset_z, diag_x, diag_y, diag_z, offdiag_xy, offdiag_xz, offdiag_yz]`
   - Residual: `r_i = radius - |M * (sample_i + offset)|` where M is symmetric 3x3
   - Initial guess: offsets from Step 1, diag = {1,1,1}, offdiag = {0,0,0}
   - Same LM damping as Step 1
   - **The Jacobian math is identical to `calc_jacobian_6pos()` in `calibration_manager.cpp`** — the 9 partial derivatives are the same for any ellipsoid fit

4. **Parameter bounds checking (ArduPilot parity):**
   ```
   radius:  15.0 - 95.0 uT (equiv. AP's 150-950 mGauss)
   diag:    0.2 - 5.0 (each axis)
   offdiag: |value| < 1.0
   offsets: |value| < 85.0 uT (equiv. AP's 2000 mGauss / 2.39)
   ```
   If any parameter exceeds bounds after convergence, return `CAL_RESULT_FIT_FAILED`.

5. **Fitness metric:**
   ```
   fitness = sqrt(sum(residuals^2) / N)
   ```
   ArduPilot considers fitness < 50 mGauss (≈ 5.0 uT) as acceptable. The fitness value quantifies the RMS deviation from the fitted ellipsoid — lower means better calibration.

**Reusable code from `calibration_manager.cpp`:**
- `forward_eliminate()` and `back_substitute()` — Gaussian elimination for normal equation solve
- Jacobian computation — same 9 partial derivatives
- Static 9x9 matrix operations — no heap allocation

**New code needed:**
- LM damping wrapper (add lambda to diagonal of J^T*J before solving)
- 4-parameter sphere fit (simpler Jacobian than the 9-param version)
- `thin_samples()` Fisher-Yates shuffle
- Parameter bounds validation
- Two-step orchestration: sphere_fit → thin → ellipsoid_fit → validate

**[GATE]:**
- Synthetic test: generate 300 samples on known ellipsoid (offset=[10,-5,20] uT, scale=[1.1,0.95,1.05], offdiag=[0.02,-0.01,0.03]), add 0.1 uT noise
  - Step 1 (sphere) converges in <10 iterations
  - Step 2 (ellipsoid) converges in <20 iterations
  - Recovered offset within 0.5 uT of truth
  - Recovered scale within 0.05 of truth
  - Fitness < 1.0 uT
- Parameter bounds: out-of-range synthetic data returns `CAL_RESULT_FIT_FAILED`
- **Benchmark on target:** Full fit (both steps) < 100ms on Cortex-M33 at 150MHz
- No heap allocation during fit (all static arrays)
- Existing accel 6-pos calibration unaffected (regression test IVP-17)

**[DIAG]:** Step 1 diverges = initial guess bad, check mean computation. Step 2 diverges = LM lambda not growing fast enough, or Step 1 result was poor. Fitness too high = inadequate sphere coverage (IVP-35 acceptance was too lax), or significant magnetic interference during collection. Regression in accel cal = shared code modified incorrectly.

**[LL]:** Entry 1 (static allocation for large objects), Entry 21 (I2C master disable during calibration)

---

### IVP-37: Interactive Compass Calibration CLI

**Prerequisites:** IVP-36, IVP-18 (CLI infrastructure), IVP-35 (sample collection)

**Implement:**

1. **CLI command `m` in calibration submenu:**
   ```
   === Compass Calibration ===
   Rotate the device slowly through all orientations.
   Cover the full sphere - pitch, roll, and yaw in all combinations.
   Progress shows sphere coverage (not just sample count).

   Disconnect baro and GPS from Qwiic chain before starting.
   Only ICM-20948 (IMU + mag) should be connected.

   Press ENTER to start, 'x' to cancel...
   ```

2. **Pre-calibration hook:** Call `icm20948_set_i2c_master_enable(&g_imu, false)` to disable autonomous I2C master mag reads. Then switch AK09916 to single-measurement mode and read mag data directly via I2C master commands. This prevents the bank-switching race condition (LL Entry 21) during the high-rate sampling needed for calibration.

   Alternative (simpler): Keep I2C master enabled in continuous 100Hz mode and just read the EXT_SENS_DATA registers. The bank-switching race only manifests at >10Hz external read rates, and we're reading at ~20Hz for calibration. **Decision: use continuous mode, read at 20Hz** — simpler, sufficient, and the race wasn't triggered at rates below ~50Hz in IVP-17 testing.

3. **Sample collection loop (runs on Core 0 during calibration):**
   ```
   while (sample_count < 300 && !cancelled) {
       // Read mag from Core 1 seqlock (not direct I2C)
       read mag sample from seqlock snapshot;
       bool accepted = calibration_feed_mag_sample(mx, my, mz);
       if (accepted) {
           update NeoPixel progress (blue→green gradient);
           print progress every 10 accepted samples;
       }
       sleep_ms(50);  // ~20Hz collection rate
   }
   ```

   Core 1 continues its normal sensor loop (IMU + mag at 100Hz via I2C master). Core 0 reads mag data from the seqlock at 20Hz for calibration. No I2C bus ownership conflict — Core 1 owns the bus throughout.

4. **NeoPixel progress feedback:**
   - Collecting: blue pulsing, brightness proportional to completion %
   - Sample accepted: brief green flash
   - Fit running: yellow solid
   - Success: green solid for 3 seconds
   - Failed: red solid for 3 seconds

5. **Post-collection: run fit and validate:**
   ```
   printf("Computing fit (%d samples)...\n", sample_count);
   cal_result_t result = calibration_compute_mag();
   if (result == CAL_RESULT_OK) {
       printf("Compass calibration OK!\n");
       printf("  Offset: [%.2f, %.2f, %.2f] uT\n", ...);
       printf("  Scale:  [%.3f, %.3f, %.3f]\n", ...);
       printf("  Fitness: %.2f uT\n", fitness);
       calibration_save();
   } else {
       printf("Calibration FAILED: %s\n", result_string);
   }
   ```

6. **Post-calibration hook:** Restore I2C master to normal operation if it was modified.

**[GATE]:**
- `m` command accessible from calibration menu
- NeoPixel shows progress during collection
- Device rotation through arbitrary orientations → progress increases
- Sitting still → progress stalls (sphere coverage not increasing)
- Full rotation coverage (~1-2 minutes): 300 samples collected, fit runs
- Fit result printed with offset, scale, offdiag, fitness
- Calibration saved to flash, persists across power cycle
- `x` cancels cleanly at any point, restores normal operation
- Core 1 sensor loop uninterrupted throughout calibration
- Scripted test (partial — Python sends `c` then `m`, verifies "Compass Calibration" banner):
  ```python
  port.write(b'c'); time.sleep(0.3)
  port.write(b'm'); time.sleep(0.5)
  response = port.read(4000).decode()
  assert 'Compass Calibration' in response
  port.write(b'x')  # Cancel
  ```

**[DIAG]:** Progress stuck at 0 = mag data all zeros (AK09916 not in continuous mode, or I2C master disabled and not re-enabled). Fit fails consistently = near-PC magnetic interference (move away from computer, monitor, speakers). NeoPixel not updating = `ws2812_status_update()` not called in the collection loop.

**[LL]:** Entry 21 (I2C master bank-switching race), Entry 6 (WS2812 begin() required)

---

### IVP-38: Mag Calibration Applied to Live Data + Validation

**Prerequisites:** IVP-37 (successful calibration stored), IVP-25 (Core 1 sensor loop)

**Implement:**

1. **Core 1 applies mag calibration to live sensor data:**
   In the Core 1 sensor loop, after reading raw mag data from ICM-20948, apply calibration via `calibration_apply_mag_with()` before writing to seqlock. This matches the existing pattern where gyro bias and accel calibration are applied on Core 1 before publishing.

   Core 1 loads calibration data once at startup via `calibration_load_into(&local_cal)` — same as existing accel/gyro calibration loading.

2. **CLI `s` (status) command shows calibrated mag data:**
   ```
   Mag: [12.3, -5.1, 42.7] uT  |M| = 45.2 uT  [CAL OK]
   ```
   Display calibrated X/Y/Z components and total magnitude. Compare magnitude to `expected_radius` from calibration — if within 15%, show `[CAL OK]`, otherwise `[CAL WARN]`.

3. **Heading computation (for display only, not used in ESKF yet):**
   ```cpp
   float heading_deg = atan2f(-my_cal, mx_cal) * 180.0f / M_PI;
   if (heading_deg < 0) heading_deg += 360.0f;
   ```
   This is tilt-uncorrected magnetic heading. Tilt compensation requires accel data and will be done properly in IVP-44 (ESKF mag update). This simple heading is useful for visual validation — rotate the device and heading should track smoothly.

4. **Validation test:** Rotate device 360 degrees around vertical axis. Calibrated magnitude should remain approximately constant (within 5 uT of expected_radius). Heading should increase monotonically through 0-360 degrees. Before calibration, magnitude varies significantly and heading is erratic.

**[GATE]:**
- CLI `s` shows calibrated mag with `[CAL OK]` status
- Calibrated magnitude stable within +/-5 uT during rotation (vs raw which varies +/-15-30 uT uncalibrated)
- Heading tracks 0-360 degrees smoothly during yaw rotation
- Heading returns to approximately same value (+/-5 degrees) after full 360-degree rotation
- Core 1 IMU rate unchanged (mag calibration apply is ~10 multiplies + 6 adds, negligible)
- Power cycle: calibration persists, calibrated output resumes immediately
- Raw vs calibrated comparison (rotate and compare): calibrated clearly superior
- No regressions in accel/gyro calibration application

**[DIAG]:** Magnitude varies widely after cal = poor calibration (insufficient sphere coverage, or strong local interference during cal). Heading jumps = off-diagonal terms wrong sign. Calibration not loading on Core 1 = `calibration_load_into()` failing (check version/CRC).

---

## Stage 5: Sensor Fusion

**Purpose:** ESKF sensor fusion, incrementally: math library, then simple baro filter, then full 24-state ESKF with codegen FPFT optimization. See `docs/decisions/ESKF/FUSION_ARCHITECTURE.md` and `docs/decisions/ESKF/FUSION_ARCHITECTURE_DECISION.md`.

**Prerequisite decision:** All numerical parameters in this stage (state counts, filter counts, process noise values, measurement noise values, convergence thresholds) are preliminary. Each must be derived from datasheets, Sola (2017), and empirical tuning. Do not hardcode without justification.

**Platform constraints (from research):**
- **RP2350 FPU is single-precision only** (FPv5-SP). Double-precision is 20-50x slower (software emulation). All filter math MUST use `float`, not `double`.
- **CMSIS-DSP is NOT included in Pico SDK 2.2.0** — only Core stubs (`arm_math_types.h`). Use plain C/C++ float math initially. CMSIS-DSP can be added later as a submodule if benchmarks warrant it.
- **Total ESKF static RAM budget:** ~4-6KB (P covariance matrix 15x15 = 900 bytes, nominal state = 64 bytes, error state = 60 bytes, F/Q/K working matrices ~3KB).
- **Joseph form covariance update mandatory** for single-precision numerical stability. Standard `P = (I - KH)P` loses positive-definiteness in float32 within minutes.
- **Sequential scalar measurement updates** eliminate need for matrix inverse — process one measurement component at a time, reducing the 15x15 inverse to scalar divisions.

**Execution model:** ESKF runs on Core 0, reading sensor data from the seqlock written by Core 1. Fusion rate target: 200Hz (Core tier), up to 400Hz (Titan tier). Core 1 continues 1kHz IMU / 50Hz baro / 10Hz GPS sampling independently.

**Reference:** Solà, J. "Quaternion kinematics for the error-state Kalman filter" (2017) — primary reference for all ESKF equations in this stage.

---

### IVP-39: Vector3 and Quaternion Math Library

**Prerequisites:** IVP-01

**Implement:** `src/math/vec3.h`, `src/math/quat.h`. All `float` (no `double`). Header-only or header + `.cpp` — keep simple.

1. **Vec3 operations:**
   - Construct, add, subtract, negate
   - Scalar multiply/divide
   - Dot product, cross product
   - Length, length_squared, normalize (handle zero-length gracefully — return zero vector, not NaN)
   - Element access: `.x`, `.y`, `.z`

2. **Quaternion operations (Hamilton convention, scalar-first `[w, x, y, z]`):**
   - Construct from `w,x,y,z` and from axis-angle
   - Multiply (Hamilton product), conjugate, inverse
   - Normalize (required after every propagation step to prevent drift)
   - Rotate Vec3: `q.rotate(v)` = `q * [0,v] * q*`
   - `from_euler(roll, pitch, yaw)` and `to_euler()` (ZYX convention)
   - `from_two_vectors(v1, v2)` — quaternion that rotates v1 to v2 (needed for init)
   - `to_rotation_matrix()` — 3x3 DCM (needed for ESKF F_x matrix)

3. **Error-state specific:**
   - `quat_from_small_angle(Vec3 delta_theta)` — first-order quaternion from rotation vector: `q ≈ [1, δθ/2]` normalized. This is the core ESKF operation for error injection.

**[GATE]:**
- Host-compiled unit tests (with ASan/UBSan) pass with `float` tolerance (1e-5 relative)
- Edge cases: zero vector normalize → zero vector (no NaN), identity quaternion multiply, 90° and 180° rotations
- `quat_from_small_angle([0,0,0])` → identity quaternion
- Euler round-trip: `from_euler(r,p,y).to_euler()` recovers `r,p,y` (avoiding gimbal lock at ±90° pitch)
- Quaternion rotation matches DCM rotation for same vector
- Cross-compile for RP2350: no `double` literals, no `<cmath>` double overloads — use `sqrtf`, `atan2f`, `sinf`, `cosf`
- Binary size delta recorded

**[DIAG]:** NaN in quaternion ops = not normalizing after multiply. Rotation seems backwards = conjugate convention wrong (Hamilton vs JPL). Euler angles wrong = ZYX vs XYZ ordering mismatch.

---

### IVP-40: Matrix Operations

**Prerequisites:** IVP-39

**Implement:** `src/math/mat.h` — compile-time-sized matrix using templates or fixed sizes. Operations needed for 15-state ESKF:

1. **Core operations:**
   - Multiply (MxN * NxP → MxP), transpose, add, subtract
   - Scalar multiply
   - Identity matrix construction
   - Element access: `m(row, col)` or `m[row][col]`

2. **ESKF-specific operations:**
   - **Symmetric matrix multiply:** `A * B * A^T` (used for covariance propagation `P = F*P*F^T + Q`) — optimize for symmetric P
   - **Joseph form update:** `P = (I - K*H) * P * (I - K*H)^T + K*R*K^T` — numerically stable covariance update
   - **Scalar measurement update:** For sequential updates, H is a row vector, R is a scalar. The Kalman gain simplifies to: `K = P*H^T / (H*P*H^T + r)` — no matrix inverse needed, just a scalar division
   - **Cholesky decomposition** (lower triangular): for generating sigma points if UKF cross-check is ever added, and for covariance conditioning checks

3. **Storage:** Static arrays, no heap. Working matrices allocated as `static` locals or file-scope globals.

4. **Size configurations needed:**
   - 15x15 (P covariance, F transition, Q process noise)
   - 15x1 (error state vector, Kalman gain for scalar updates)
   - 3x3 (rotation matrices, small covariance blocks)
   - Arbitrary up to 15x15

**[GATE]:**
- Host-compiled unit tests: known matrix multiplications, transpose, identity
- `A * A_inverse ≈ I` for well-conditioned matrices (float tolerance)
- Joseph form produces same result as standard update for well-conditioned case, stays positive-definite when standard form doesn't (test with ill-conditioned P)
- Cholesky of known positive-definite matrix matches expected L
- **Benchmark on target:** 15x15 multiply = Xµs (record — informs ESKF timing budget). Target: <50µs per 15x15 multiply at 150MHz
- Symmetric multiply `F*P*F^T` for 15x15: Xµs (record — this is the dominant ESKF cost)
- No `double` anywhere — verify with `grep -r "double" src/math/`
- Binary size delta recorded

**[DIAG]:** Slow benchmark = check for accidental `double` promotion (literal `1.0` instead of `1.0f`). Cholesky fails = matrix not positive definite (check inputs). NaN propagation = check for division by zero in scalar update denominator.

---

### IVP-41: 1D Barometric Altitude Kalman Filter

**Prerequisites:** IVP-40, IVP-12, IVP-25

**Implement:** `src/fusion/baro_kf.h/.cpp` — 2-state (altitude, vertical velocity) linear Kalman filter using baro pressure. Runs on Core 0 at 50Hz (matching baro update rate from Core 1 seqlock). This is the educational stepping stone before the full ESKF — proves the math library works end-to-end on real sensor data.

1. **State vector:** `x = [altitude_m, vertical_velocity_m_s]^T` (2x1)

2. **Process model (constant velocity):**
   ```
   F = [1  dt]    Q = [dt^4/4  dt^3/2] * q_accel
       [0   1]        [dt^3/2  dt^2  ]
   ```
   where `dt = 1/50` (50Hz) and `q_accel` is the process noise power spectral density for vertical acceleration. Initial `q_accel`: `⚠️ VALIDATE` — start with 0.1 m/s², tune empirically. This represents expected unmodeled vertical acceleration.

3. **Measurement model:**
   ```
   H = [1  0]     z = barometric_altitude_m (from calibration_get_altitude_agl())
   R = sigma_baro^2
   ```
   `sigma_baro`: `⚠️ VALIDATE` — derive from DPS310 datasheet pressure noise (~0.6 Pa RMS at 64x oversampling) converted to altitude noise via barometric formula (~0.05m at sea level). Measure empirically in IVP-12 if not already recorded.

4. **Altitude from pressure:** Use existing `calibration_get_altitude_agl()` which applies the hypsometric formula with ground pressure reference from baro calibration.

5. **Joseph form update** (not standard form) — practice for the full ESKF.

**[GATE]:**
- Device stationary: altitude estimate stable, noise < raw baro noise (quantify both)
- Raise device ~1 meter slowly: filter tracks altitude change within 0.2m, velocity transitions 0→positive→0
- Drop 0.5m quickly: velocity responds, altitude tracks (may lag slightly — record settling time)
- Convergence after boot: reaches steady-state within `⚠️ VALIDATE 5 seconds`
- No divergence over 5 minutes continuous operation
- No NaN, covariance stays positive (P[0][0] > 0, P[1][1] > 0)
- **Measure filter execution time** per step (record — expected <5µs for 2-state)

**[DIAG]:** Filter diverges = Q too small (trusting model too much) or R too small (trusting measurement too much). Altitude offset = ground pressure reference not set (run baro calibration first). Velocity noisy = Q too large.

---

### IVP-42: ESKF Propagation (IMU-Only)

**Prerequisites:** IVP-40, IVP-39, IVP-25

**Implement:** `src/fusion/eskf.h/.cpp` — 15-state Error-State Kalman Filter, propagation only (no measurement updates). Runs on Core 0, reading IMU data from seqlock. This step establishes the nominal state propagation and error covariance prediction.

1. **Nominal state (16 elements, propagated nonlinearly):**
   ```
   x_nom = [q(4), p(3), v(3), a_bias(3), g_bias(3)]
   ```
   - `q`: attitude quaternion (body-to-NED, Hamilton convention, scalar-first)
   - `p`: position in NED frame (m) — initialized to [0,0,0], updated by GPS later
   - `v`: velocity in NED frame (m/s) — initialized to [0,0,0]
   - `a_bias`: accelerometer bias (m/s²) — initialized from calibration
   - `g_bias`: gyroscope bias (rad/s) — initialized from calibration

2. **Nominal state propagation (per Sola §5.3):**
   ```
   q ← q ⊗ q{(ω - g_bias) * dt}     // integrate angular velocity
   v ← v + (R(q) * (a - a_bias) - g) * dt   // rotate accel to NED, subtract gravity
   p ← p + v * dt + 0.5 * (R(q) * (a - a_bias) - g) * dt²
   ```
   where `R(q)` = rotation matrix from body to NED, `g = [0, 0, 9.81]^T`.

3. **Error state (15 elements):**
   ```
   δx = [δθ(3), δp(3), δv(3), δa_bias(3), δg_bias(3)]
   ```
   Note: attitude error uses rotation vector (3 elements), NOT quaternion (4 elements). This is the key advantage of ESKF — the error state is minimal and avoids quaternion constraint.

4. **Error state transition matrix F_x (15x15, per Sola §5.3.3):**
   ```
   F_x = I + F_δ * dt
   ```
   where F_δ contains the Jacobian blocks:
   - `∂δθ/∂δθ = -[ω]_×` (skew-symmetric of angular rate)
   - `∂δv/∂δθ = -R(q) * [a - a_bias]_×` (accel sensitivity to attitude error)
   - `∂δv/∂δa_bias = -R(q)` (accel bias effect on velocity)
   - `∂δθ/∂δg_bias = -I` (gyro bias effect on attitude)
   - All other blocks are zero or identity

5. **Process noise Q (15x15):**
   Derived from IMU noise density specs (ICM-20948 datasheet):
   - Gyro noise: `σ_g = 0.015 °/s/√Hz` = `⚠️ VALIDATE 2.62e-4 rad/s/√Hz`
   - Accel noise: `σ_a = 230 µg/√Hz` = `⚠️ VALIDATE 2.26e-3 m/s²/√Hz`
   - Gyro bias random walk: `⚠️ VALIDATE 1e-5 rad/s²/√Hz` (estimated — no datasheet spec, tune empirically)
   - Accel bias random walk: `⚠️ VALIDATE 1e-4 m/s³/√Hz` (estimated — tune empirically)
   ```
   Q = diag(σ_g² * dt, 0, σ_a² * dt, σ_ab² * dt, σ_gb² * dt)
   ```
   (Actual Q construction per Sola §5.2.4 — continuous-time noise integrated over dt)

6. **Covariance propagation:**
   ```
   P ← F_x * P * F_x^T + Q
   ```

7. **NED frame initialization:** On first valid accel reading (device stationary), compute initial quaternion:
   - Pitch from accel: `pitch = atan2(-ax, sqrt(ay² + az²))`
   - Roll from accel: `roll = atan2(ay, az)`
   - Yaw: set to 0 initially (mag heading correction comes in IVP-44)
   - Initial P: large diagonal values for attitude (`⚠️ VALIDATE 0.1 rad²`), position (`⚠️ VALIDATE 100 m²`), velocity (`⚠️ VALIDATE 1 m²/s²`), biases (`⚠️ VALIDATE` from calibration uncertainty)

8. **Error state reset:** After measurement updates (future IVPs), inject error into nominal and reset:
   ```
   q ← q ⊗ q{δθ}
   p ← p + δp
   v ← v + δv
   a_bias ← a_bias + δa_bias
   g_bias ← g_bias + δg_bias
   δx ← 0
   ```
   Also apply the reset Jacobian G to covariance: `P ← G * P * G^T` (Sola §7.2). For small angles, G ≈ I (can skip initially but must add before flight).

**Memory:**
- Nominal state: 16 floats = 64 bytes
- Error state: 15 floats = 60 bytes
- P matrix: 15x15 = 900 bytes
- F_x matrix: 15x15 = 900 bytes (can be computed in-place)
- Q matrix: 15x15 = 900 bytes (sparse — can optimize to diagonal)
- Working space: ~1KB
- **Total: ~3.8KB static** (within 4-6KB budget)

**[GATE]:**
- Stationary on flat surface: attitude estimate = [0, 0, 0] roll/pitch/yaw (within ±2°). Drift rate measured and recorded over 30 seconds — expected `⚠️ VALIDATE <5° total` (gyro-only, no corrections)
- Rotate 90° around one axis: tracks correctly within `⚠️ VALIDATE 10°`
- Velocity integrates correctly: move device ~1m, velocity transitions from zero to positive to zero (will drift without GPS — expected and acceptable for this step)
- P matrix diagonal stays positive throughout (no negative variance)
- No NaN or Inf in any state or covariance element
- **Benchmark execution time per propagation step** on target: record µs. Target: `⚠️ VALIDATE <100µs` per step at 150MHz (informing fusion rate feasibility)
- Quaternion norm stays within 1.0 ± 1e-4 after 10,000 steps (normalization working)
- CLI `s` command displays fused attitude (roll/pitch/yaw in degrees)

**[DIAG]:** Attitude immediately wrong = NED frame init bad (check accel signs, gravity direction convention). Rapid divergence = F_x Jacobian error (check skew-symmetric construction). P grows unboundedly = Q too large or F_x incorrect. NaN = quaternion not normalized, or division by zero in rotation matrix. Benchmark too slow = accidental `double` math.

**[LL]:** Entry 1 (static allocation for matrices)

---

### IVP-43: ESKF Barometric Altitude Update

**Prerequisites:** IVP-42, IVP-41

**Implement:** Add barometric altitude measurement update to the ESKF. This is the first measurement update — uses the sequential scalar technique to avoid matrix inverse.

1. **Measurement model:**
   ```
   z = altitude_agl (from baro)
   h(x) = -p_d (down component of NED position, negated for altitude-up)
   H = [0 0 0 | 0 0 -1 | 0 0 0 | 0 0 0 | 0 0 0]  (1x15 row vector)
   ```
   H selects the down-position error state and negates it.

2. **Sequential scalar update (no matrix inverse):**
   ```
   innovation = z - h(x_nom)
   S = H * P * H^T + R        // scalar (1x1)
   K = P * H^T / S             // 15x1 column vector
   δx = K * innovation         // 15x1 error state correction
   ```
   Then apply Joseph form:
   ```
   IKH = I - K * H             // 15x15
   P = IKH * P * IKH^T + K * R * K^T
   ```

3. **Innovation gating:** Reject baro measurements where `|innovation| > 3 * sqrt(S)`. Logs rejection count. Prevents bad baro data (e.g., occluded port) from corrupting the filter.

4. **Measurement noise R (scalar):**
   `R = sigma_baro_alt²` — `⚠️ VALIDATE` from DPS310 noise spec and empirical measurement. Expected ~0.05-0.1m altitude noise at 64x oversampling. Can use the variance measured in IVP-41's standalone baro KF.

5. **Error state injection and reset** (per IVP-42 step 8): apply δx to nominal state, reset δx and covariance.

**[GATE]:**
- Device stationary: altitude tracks baro with reduced noise (compare ESKF altitude variance vs raw baro variance, ESKF should be lower)
- Raise device 1m: filter reports ~1m change, settles within `⚠️ VALIDATE 2 seconds`
- Innovation gate: cover baro port briefly — innovations grow, gate rejects measurements, filter coasts on IMU, P grows. Uncover port — filter reconverges
- No divergence over 5 minutes
- P stays positive-definite throughout (check diagonal elements)
- Combined propagation + update benchmark: Xµs (record)
- Compare against standalone baro KF (IVP-41) — ESKF altitude should be comparable or better

**[DIAG]:** Altitude offset = NED frame sign convention wrong (NED down is positive, altitude up is negative position). Innovation always large = H vector wrong or measurement model wrong. P collapses to zero = Joseph form not implemented correctly. Filter doesn't respond to baro = K is zero (P too small, or R too large).

---

### IVP-44: ESKF Magnetometer Heading Update

**Prerequisites:** IVP-42, IVP-38 (calibrated mag data on Core 1)

**Implement:** Add magnetometer heading measurement update to the ESKF. This corrects yaw drift that accel and baro cannot observe.

1. **Measurement model (heading-only, not full vector):**
   Use tilt-compensated magnetic heading as the measurement, not the raw 3D mag vector. This decouples mag from roll/pitch (which accel already observes):
   ```
   // Rotate calibrated mag vector to level frame using current attitude
   m_level = R(q)^T * m_cal     // or equivalently, apply roll/pitch correction
   heading_measured = atan2(-m_level.y, m_level.x)
   heading_predicted = yaw from q
   innovation = wrap_pi(heading_measured - heading_predicted)
   ```
   The `wrap_pi()` ensures the innovation stays in [-π, π] to avoid discontinuity at ±180°.

2. **H vector (1x15):** Jacobian of heading with respect to error state. Only the yaw component of δθ is non-zero:
   ```
   H ≈ [0 0 1 | 0...0]  (simplified — full derivation in Sola §6.2)
   ```
   The exact H depends on current attitude but for small errors and level flight, the yaw-only approximation is adequate.

3. **Measurement noise R (scalar):**
   `R = sigma_mag_heading²` — `⚠️ VALIDATE` — derive from AK09916 noise (0.1 µT RMS, ~0.15 µT/LSB) converted to heading noise at current field strength. At 45 µT total field: σ_heading ≈ 0.1/45 ≈ 0.002 rad ≈ 0.13°. In practice, soft iron residuals dominate — start with `⚠️ VALIDATE σ_heading = 5°` (0.087 rad) and tune.

4. **Magnetic interference detection:** If calibrated mag magnitude deviates >25% from `expected_radius` stored in calibration, increase R by 10x (reduce trust in mag when in anomalous field). Log occurrences.

5. **Update rate:** `⚠️ VALIDATE 10Hz` — mag is low-rate, high-latency. Running at IMU rate wastes compute and risks overweighting mag.

**[GATE]:**
- Stationary: yaw drift rate with mag updates < `⚠️ VALIDATE 0.1°/min` (vs several degrees/min without)
- Rotate 360° around vertical: heading tracks smoothly, returns to start within `⚠️ VALIDATE ±5°`
- Bring magnet near device: interference detected (R inflated), filter coasts, yaw drifts slowly. Remove magnet: filter reconverges within `⚠️ VALIDATE 10 seconds`
- Innovation gate rejects gross outliers
- Compare heading from ESKF vs simple `atan2` heading from IVP-38 — ESKF should be smoother

**[DIAG]:** Heading 90° or 180° off = axis convention mismatch (NED vs ENU, or body-frame X/Y swap). Heading oscillates = R too small (overweighting noisy mag). Heading doesn't converge = R too large or H Jacobian wrong. Innovation always wraps = `wrap_pi()` not applied.

---

### IVP-45: Mahony AHRS Cross-Check

**Prerequisites:** IVP-42

**Implement:** `src/fusion/mahony_ahrs.h/.cpp` — independent attitude estimator running alongside ESKF. Lightweight PI controller on orientation error. Uses same IMU data from seqlock but maintains its own quaternion. Provides the independent cross-check required by the confidence gate (IVP-55).

1. **Algorithm (Mahony 2008):**
   ```
   // Error from gravity reference
   v_hat = R(q)^T * [0, 0, 1]           // predicted gravity in body frame
   e_accel = a_meas × v_hat              // cross product = rotation error

   // Error from mag reference (optional, when mag calibrated)
   h = R(q) * m_cal                       // mag in NED frame
   b = [sqrt(hx² + hy²), 0, hz]          // reference field (horizontal + vertical)
   e_mag = m_cal × (R(q)^T * b)          // mag rotation error

   // PI controller
   integral += Ki * (e_accel + e_mag) * dt
   omega_corrected = omega_meas + Kp * (e_accel + e_mag) + integral

   // Integrate corrected angular rate
   q ← q ⊗ q{omega_corrected * dt}
   normalize(q)
   ```

2. **Gains:** `Kp = ⚠️ VALIDATE 2.0`, `Ki = ⚠️ VALIDATE 0.005`. Start with Mahony's recommended values. Higher Kp = faster convergence but more noise. Lower Ki = less bias correction.

3. **Accel gating:** Skip accel correction when `|a| > ⚠️ VALIDATE 1.2g` or `|a| < ⚠️ VALIDATE 0.8g`. During boost, accel does not point at gravity — using it corrupts attitude. This is critical for flight.

4. **Output:** Quaternion → roll/pitch/yaw. Compare against ESKF attitude every update cycle. Report maximum divergence.

5. **Divergence metric:**
   ```
   angle_diff = 2 * acos(|q_eskf · q_mahony|)   // quaternion angular distance
   ```

**[GATE]:**
- Stationary: Mahony attitude within `⚠️ VALIDATE ±2°` of ESKF attitude
- Slow rotation: divergence < `⚠️ VALIDATE 5°`
- Fast rotation: divergence may temporarily grow, then reconverges within `⚠️ VALIDATE 5 seconds`
- Accel gating: move device rapidly (>1.2g): accel correction disabled, attitude still tracks via gyro integration
- **Benchmark:** Mahony update time = Xµs (record — expected <10µs, much lighter than ESKF)
- Divergence printed in CLI `s` output: `AHRS diff: X.X°`

**[DIAG]:** Large steady-state divergence = gain mismatch or different gravity convention between ESKF and Mahony. Mahony oscillates = Kp too high. Mahony drifts = Ki too low or gyro bias not tracked.

---

### IVP-46: GPS Measurement Update

**Prerequisites:** IVP-43, IVP-33

**Implement:** Add GPS position and velocity measurement updates to the ESKF. GPS provides the only absolute position reference — without it, position drifts indefinitely on IMU integration alone. Requires outdoor testing.

1. **NED frame origin establishment:**
   On first valid 3D GPS fix, record the WGS84 geodetic coordinates as the NED frame origin:
   ```
   lat0 = gps_lat_1e7 / 1e7 * DEG_TO_RAD
   lon0 = gps_lon_1e7 / 1e7 * DEG_TO_RAD
   alt0 = gps_alt_msl_m
   ```
   All subsequent GPS positions are converted to NED relative to this origin using the standard geodetic-to-NED conversion (flat-earth approximation is adequate for model rocketry distances):
   ```
   north = (lat - lat0) * R_earth
   east  = (lon - lon0) * R_earth * cos(lat0)
   down  = -(alt - alt0)
   ```
   Store origin in filter state. Reset origin on ARM (flight state machine, Stage 6) or on user command.

2. **Position measurement update (3 sequential scalar updates):**
   ```
   H_north = [0 0 0 | 1 0 0 | 0 0 0 | ...]  // selects north position error
   H_east  = [0 0 0 | 0 1 0 | 0 0 0 | ...]
   H_down  = [0 0 0 | 0 0 1 | 0 0 0 | ...]
   ```
   Process each component sequentially (north, then east, then down), updating P between each. R for each component: `sigma_pos² = ⚠️ VALIDATE (HDOP * CEP95_to_sigma)²`. PA1010D CEP 95% = ~3m. At HDOP=1: σ ≈ 1.5m horizontal, σ_vert ≈ 2x horizontal.

3. **Velocity measurement update (2 sequential scalar updates):**
   GPS provides ground speed and track angle, not NED velocity components directly. Convert:
   ```
   v_north = ground_speed * cos(track_angle)
   v_east  = ground_speed * sin(track_angle)
   ```
   Vertical velocity from GPS is unreliable — skip it, let baro + IMU handle vertical velocity. R for velocity: `sigma_vel² = ⚠️ VALIDATE 0.5² m²/s²` (derived from GPS velocity accuracy spec).

4. **Innovation gating:** Reject GPS measurements where any innovation exceeds `⚠️ VALIDATE 5 * sqrt(S)`. Log rejections. Prevents multipath-corrupted fixes from damaging the filter.

5. **Update rate:** Match GPS rate from Core 1 seqlock — 10Hz. Only update when `gps_read_count` has incremented since last update (new fix available).

6. **Stationary detection pseudo-measurement:** When `ground_speed < ⚠️ VALIDATE 0.3 m/s` AND accel magnitude is within ±0.1g of gravity AND gyro rates < 0.02 rad/s for > 2 seconds: apply zero-velocity pseudo-measurement `v = [0,0,0]` with tight R (`⚠️ VALIDATE 0.01 m²/s²`). This dramatically reduces position drift when stationary and accelerates filter convergence after boot.

**[GATE]:** (outdoor testing required)
- First GPS fix: NED origin established, position starts tracking
- Walk 10m north: ESKF position tracks GPS with less noise than raw GPS
- Walk in a square: return to start within `⚠️ VALIDATE ±3m` of origin
- Stationary: position drift < `⚠️ VALIDATE 1m` over 5 minutes (with GPS)
- Innovation gate: enter building (GPS degrades) — gate rejects, filter coasts on IMU
- Return outdoors: filter reconverges to GPS
- Zero-velocity detection: stationary device shows near-zero velocity and minimal position drift
- GPS dropout: disable GPS, verify ESKF continues on IMU only (position drifts but attitude stable)
- All position and velocity values displayed in CLI `s` output

**[DIAG]:** Position jumps = innovation gate threshold too loose. Slow convergence = R too large. Position spirals = NED frame convention wrong (lat/lon to north/east mapping). Velocity noisy = GPS velocity noise underestimated.

---

### IVP-47: ESKF Health Tuning & Diagnostics

**Prerequisites:** IVP-46 (all measurement feeds live)

**Rationale:** With all measurement feeds live, tune the ESKF for robust real-world operation before optimizing performance or layering MMAE on top. A poorly-tuned single ESKF will produce poorly-tuned MMAE hypotheses. Currently the magnetometer feed is non-functional (mNIS stuck at 124.99, every update rejected) — this must be fixed before performance optimization, which needs all feeds actually fusing to produce meaningful benchmarks.

**Implement:**

1. **Magnetometer innovation gate tuning.** Current mNIS is stuck at 124.99 (gate rejecting every update when board not level). The heading measurement model assumes approximately level orientation. Options: (a) widen the mag innovation gate, (b) add a tilt-compensated heading model, (c) gate mag updates on pitch/roll magnitude. Research ArduPilot EKF3 `magFuseMethod` and PX4 ECL mag fusion for prior art.

2. **Innovation gate threshold review.** Audit all innovation gates (baro, mag, GPS pos, GPS vel, ZUPT) for appropriate σ thresholds. Currently using `⚠️ VALIDATE` placeholders in several places. Derive thresholds from sensor datasheets and outdoor test data.

3. **Process noise Q tuning.** Review Q diagonal values against real sensor data. Current values are from Solà (2017) defaults — may need adjustment for ICM-20948 noise characteristics (datasheet: gyro noise density 0.015 dps/√Hz, accel noise density 230 µg/√Hz).

4. **CLI health dashboard.** Add ESKF health summary to `e` display: per-sensor NIS statistics (min/max/mean over window), covariance diagonal magnitudes, innovation gate rejection counts.

**[GATE]:**
- mNIS accepts updates when board is reasonably level (< 30° tilt)
- All innovation gate thresholds justified by source (datasheet, reference implementation, or empirical)
- 60s indoor soak: all NIS values within expected bounds, zero UNHEALTHY
- Outdoor soak: GPS NIS reasonable during walk, mag NIS reasonable when level
- CLI `e` shows meaningful health diagnostics

**[DIAG]:** mNIS still stuck after tuning = measurement model fundamentally wrong for non-level operation (need tilt compensation). GPS NIS consistently high = R values too tight for actual GPS accuracy. Baro NIS spikes after GPS origin set = altitude reference mismatch between baro and GPS.

---

### IVP-48: Sparse FPFT Optimization

**Prerequisites:** IVP-47 (ESKF tuned with all feeds fusing correctly)

**Rationale:** `predict()` originally used dense FPFT (three matrix multiplies), benchmarked at ~496µs on target (IVP-42d). At 24 states, dense O(N³) is not viable (1,747µs from SRAM, 15.7× slower than codegen). SymPy codegen with CSE eliminates ~90% of operations by expanding only non-zero symbolic terms at compile time.

**Implement:** Exploit F_x block structure (many zero/identity blocks) to avoid the full O(N³) triple product FPFT = F P F^T + Q.

1. **Identify F_x sparsity pattern.** The 15×15 state transition matrix has identity blocks (attitude→attitude, bias→bias), zero blocks (bias→position), and small dense blocks (attitude→velocity via specific force). Map which 3×3 sub-blocks are zero, identity, or dense.

2. **Replace dense `mat_mul_15x15()` triple product** with block-structured multiply that skips zero blocks and substitutes identity blocks with direct copy. Target: <100µs per predict call.

3. **Also optimize measurement update Joseph form** — `update_baro()`, `update_mag()`, `update_gps_pos/vel()` each have two dense 15×15 multiplies. H matrices are extremely sparse (1–3 non-zero elements per row). Exploit H sparsity for Kalman gain and Joseph form.

4. **Benchmark** all paths on target with `time_us_32()`. Record before/after for predict, each measurement update, and total `eskf_tick()` cycle time.

**[GATE]:**
- `predict()` < 100µs (down from ~496µs)
- All measurement updates profiled and optimized where beneficial
- Total `eskf_tick()` cycle time recorded
- Host tests still pass (identical numerical output — sparse is an algebraic optimization, not an approximation)
- HW soak: no regression in bNIS/gNIS/mNIS behavior

**[DIAG]:** Numerical drift between sparse and dense paths = block boundary error in sparse implementation. Benchmark not improving = profiling wrong code path or memory-bound not compute-bound.

---

*Stage 5 complete at IVP-48. MMAE (originally planned as IVP-49) was replaced by phase-scheduled Q/R after research demonstrated that deterministic flight-phase switching via the state machine captures the same benefit at near-zero computational cost. See `docs/decisions/ESKF/ESKF_RESEARCH_SUMMARY.md` for full rationale.*

---

## Stage 6: Radio & Telemetry

**Purpose:** LoRa radio link for telemetry downlink and real-time debugging. Pulled forward from original Stage 9 position — no dependencies on Flight Director, Adaptive Estimation, or Data Logging. Enables untethered dynamic validation tests and live sensor streaming.

**Hardware:** Adafruit LoRa Radio FeatherWing #3231 (RFM95W, 915 MHz ISM). SPI0 bus (independent from I2C sensors). FeatherWing jumpers: CS=GPIO10, RST=GPIO11, IRQ=GPIO6 (M0 defaults).

| Step | Title | Brief Description |
|------|-------|------------------|
| IVP-63 | RFM95W Radio Driver | SPI bus init, LoRa radio init, TX/RX with polling |
| IVP-64 | Telemetry Encoder | ~~MAVLink~~ → CCSDS TM packet encoding (scope TBD) |
| IVP-65 | Telemetry Service | `⚠️ VALIDATE 10Hz` downlink with message prioritization |
| IVP-66 | GCS Compatibility | ~~QGroundControl / Mission Planner~~ → re-eval for CCSDS (scope TBD) |
| IVP-67 | Bidirectional Commands | GCS sends calibrate, arm, parameter set commands |

---

## Stage 7: Flight Director

**Purpose:** Flight state machine and event-driven actions. See SAD.md Section 6.

**Prerequisite decisions:**
- Resolve SAD Open Question #4 (Mealy vs Moore state machine) before implementing IVP-50.
- IVP-49 (Watchdog Recovery Policy) must be completed first — the state machine depends on knowing how the system recovers from a mid-flight reboot.

| Step | Title | Brief Description |
|------|-------|------------------|
| IVP-49 | Watchdog Recovery Policy | Scratch register persistence, reboot counting, state-aware recovery path |
| IVP-50 | State Machine Core | IDLE/ARMED/BOOST/COAST/DESCENT/LANDED states and transitions |
| IVP-51 | Event Engine | Condition evaluator for launch, apogee, landing detection |
| IVP-52 | Action Executor | LED, beep, logging trigger actions on state transitions |
| IVP-53 | Mission Configuration | Load mission definitions (rocket, freeform, HAB) |

---

### IVP-49: Watchdog Recovery Policy

**Prerequisites:** IVP-30 (hardware watchdog mechanism), Stage 5 complete

**Rationale:** IVP-30 implemented the watchdog *mechanism* (dual-core heartbeat, 5s timeout, kick pattern). The mechanism is correct for ground/IDLE — a full reboot recovers cleanly. But a full reboot mid-flight (ARMED through DESCENT) is catastrophic: ESKF state lost, pyro timers reset, position/velocity knowledge gone. The state machine (IVP-50) cannot be designed without first defining what happens when the system wakes up after an unexpected reset.

**Bug fix (included):** Replace `watchdog_enable_caused_reboot()` with `watchdog_caused_reboot()` in `init_hardware()`. The `_enable_` variant checks a scratch register magic value that persists across picotool flashes and soft resets, causing false watchdog warnings on clean boots. The base `watchdog_caused_reboot()` checks `rom_get_last_boot_type() == BOOT_TYPE_NORMAL` on RP2350, correctly distinguishing true WDT timeouts from reflash reboots.

**Design principle: Subsystem restart, not full reboot.** A hardware watchdog reset is unavoidable (it's a chip-level reset), but the *software response* should restart only the failed subsystem when possible. If ESKF diverges, restart ESKF — don't reboot the whole system. If Core 1 sensor loop hangs, attempt Core 1 restart before triggering a full WDT reset. The watchdog is the last resort when targeted recovery fails, not the first response to any anomaly.

**Implement:**

1. **Scratch register persistence.** Before enabling the watchdog, write diagnostic context to `watchdog_hw->scratch[5..7]` (scratch[4] is reserved by the SDK for reboot magic). On each main loop iteration, update scratch registers with: current flight state enum (scratch[5] low byte), last tick function ID (scratch[5] high byte), monotonic reboot counter (scratch[6]), and a CRC or magic to validate the data (scratch[7]). These survive a WDT reset but not a power-on reset.

2. **Reboot counter with safe-mode lockout.** On boot, read scratch[6] reboot counter. If counter > `⚠️ VALIDATE 3` within a detection window (use scratch[7] timestamp or magic to detect rapid reboots vs. widely-spaced ones), enter safe mode: NeoPixel solid red, beacon-only operation, no ESKF, no pyro. Log reboot count to flash if storage is available. Counter resets on clean power-on (scratch registers cleared by POR).

3. **Ground-side launch abort policy.** Any watchdog reset while in IDLE or ARMED state sets a persistent `LAUNCH_ABORT` flag. This flag requires explicit operator acknowledgement (CLI command or physical reset sequence) before the system can transition to ARMED. Rationale: a WDT reset on the ground indicates a software fault that must be investigated before flight. Even a single ground-side WDT event should abort launch preparation and require a restart of pre-flight procedure. The flag persists across soft resets (stored in scratch registers) and clears only on POR or explicit operator command.

4. **ESKF failure backoff.** Add a consecutive-failure counter for ESKF init→diverge cycles. After `⚠️ VALIDATE 5` consecutive failures (init succeeds but `healthy()` returns false within N seconds), disable ESKF until manual CLI reset (`'e'` key or similar). This prevents the init→diverge→re-init churn observed during ESKF development without requiring a full system reboot. This is the model for subsystem-level recovery: detect failure, disable the failing subsystem, continue operating in degraded mode.

5. **Recovery boot path.** In `init_hardware()` / `init_application()`, after reading scratch registers: if this is a WDT reboot (not POR), log the event, print the diagnostic context from scratch registers, and set a `g_recoveryBoot` flag. The state machine (IVP-50) will use this flag to determine post-reboot behavior per flight state:
   - **IDLE/LANDED:** Normal reboot + LAUNCH_ABORT flag set.
   - **ARMED:** LAUNCH_ABORT + automatic DISARM.
   - **BOOST/COAST/DESCENT:** Recovery mode — skip ESKF (can't re-init under acceleration), resume pyro timers from flash checkpoint, activate beacon. Degrade to baro-only altitude with timer-based backup deployment.

**[GATE]:**
- `watchdog_caused_reboot()` returns false on clean power-on (no more persistent false warnings)
- `watchdog_caused_reboot()` returns true after genuine WDT timeout, with correct scratch register diagnostics
- Reboot counter increments across WDT resets, resets on POR
- After `⚠️ VALIDATE 3` rapid WDT reboots: system enters safe mode (NeoPixel solid red, prints "[SAFE] Reboot loop detected")
- ESKF disables after N consecutive divergences, CLI can re-enable
- Ground WDT reset sets LAUNCH_ABORT flag, prevents ARMED transition until acknowledged
- Normal 5-minute soak with ESKF running: zero WDT fires, zero false warnings
- Scratch register data survives WDT reset, verified via GDB `x/4w &watchdog_hw->scratch[4]`

**[DIAG]:** False WDT warning persists = still using `watchdog_enable_caused_reboot()` or scratch registers not cleared on POR. Safe mode triggers unexpectedly = reboot counter threshold too low or detection window too tight. ESKF never re-enables = backoff counter not resettable via CLI. Scratch data garbled after WDT = CRC/magic validation logic bug. LAUNCH_ABORT blocks arming unexpectedly = flag not cleared on POR or CLI command not working.

> **Milestone:** At IVP-53 completion, the system has calibrated sensors, GPS, sensor fusion, and a working flight state machine — **Crowdfunding Demo Ready**.

---

## Stage 8: Adaptive Estimation & Safety

**Purpose:** Phase-aware ESKF tuning and confidence-gated safety. Integrates with the state machine (IVP-50) for flight phase detection and with the Flight Director (Stage 6) for safety-gated actions.

**Background:** Extended research (2026-02-24) demonstrated that MMAE/IMM is the wrong tool for RocketChip's flight regime switching. Real aerospace navigation (X-43A, SpaceX Grasshopper, ArduPilot EKF3, PX4 ECL) universally uses single kinematic filters with deterministic regime adaptation — not multi-model banks. Simple phase-scheduled Q/R captures the same benefit at near-zero computational cost. See `docs/decisions/ESKF/ESKF_RESEARCH_SUMMARY.md` for full rationale and benchmark data.

| Step | Title | Brief Description |
|------|-------|------------------|
| IVP-54 | Phase-Scheduled Q/R + Innovation Adaptation | Per-phase noise models tied to state machine, innovation ratio fine-tuning, optional Bierman measurement updates |
| IVP-55 | Confidence Gate | ESKF health + AHRS cross-check → binary confidence flag for Flight Director |
| IVP-56 | Confidence-Gated Actions | Irreversible actions (pyro) held when confidence flag low |
| IVP-57 | Vehicle Parameter Profiles | Mission-specific Q/R presets per vehicle type (rocket, HAB, drone) |

---

### IVP-54: Phase-Scheduled Q/R + Innovation Adaptation

**Prerequisites:** IVP-50 (state machine provides flight phase detection), IVP-48 (ESKF tuned)

**Implement:** Per-phase Q and R matrices selected from vehicle-specific parameter profiles, with thin innovation-ratio adaptation layer for fine-tuning.

1. **Q/R profiles per flight phase:**
   - **IDLE/ARMED:** Low Q_velocity, low Q_position (stationary), tight R_baro, normal R_mag
   - **BOOST:** High Q_velocity (rapid acceleration), increased R_baro (vibration/transonic effects), relaxed R_mag
   - **COAST:** Moderate Q_velocity (drag deceleration), gate R_baro during transonic transition, normal R_mag
   - **DESCENT:** Moderate Q, restore R_baro trust, enable accel corrections for attitude (1g gate passes)

2. **Transition smoothing:** Exponential ramp between Q/R values over `⚠️ VALIDATE 5-10` filter steps during detected phase transitions. Avoids covariance discontinuities from hard switching.

3. **Innovation ratio adaptation (thin layer on top):** Per-channel scalar monitor: compute α = ν²/S over a `⚠️ VALIDATE 50-100` sample sliding window. If α consistently exceeds 1.0, scale up relevant Q diagonal elements. Constraints:
   - Adapt only Q, never R simultaneously (ill-conditioned)
   - Only diagonal Q elements
   - Floor at 10% of phase-scheduled Q value
   - Freeze adaptation for `⚠️ VALIDATE 0.5-1s` around phase transitions

4. **Bierman measurement updates (optional, GPS-conditional):** When GPS is detected at boot, use Bierman scalar measurement updates instead of Joseph form (43µs vs 81µs per scalar update, benchmarked on hardware). Factorize P→UD before measurement epoch, Bierman updates on U/D, reconstruct after. Only activated when GPS provides ≥6 scalar measurements per epoch (break-even point for factorize overhead). Boot-time flag — not per-measurement switching. Implementation in `ud_factor.h/.cpp` already benchmarked.

**[GATE]:**
- Static bench: IDLE Q/R active, innovation ratios near 1.0
- Simulated boost (accel stimulus): Q transitions to boost profile within 1-2 filter steps
- Return to rest: Q transitions back within 5-10 steps
- Innovation adaptation: artificially increase sensor noise → adaptation increases Q → innovations return to ~1.0
- No discontinuities in state estimates during phase transitions (smooth ramp)
- Benchmark: phase-scheduled Q/R adds <5µs overhead per predict
- CLI shows: current phase, active Q/R profile name, innovation ratios per channel, adaptation state
- If Bierman enabled: verify cycle time improvement matches benchmark (486µs hybrid vs 851µs Joseph)

**[DIAG]:** Innovation ratios stuck high = Q too low for actual sensor noise. Phase transitions cause state jumps = ramp too fast or Q/R delta too large. Adaptation oscillates = window too short or floor too low. Bierman slower than Joseph = measurement count below break-even threshold (disable Bierman for that epoch pattern).

---

### IVP-55: Confidence Gate

**Prerequisites:** IVP-54, IVP-45 (Mahony AHRS)

**Implement:** `src/fusion/confidence_gate.h/.cpp` — evaluates ESKF innovation consistency and AHRS cross-check to produce a binary confidence flag consumed by the Flight Director. This is the platform safety layer — NOT configurable by Mission Profiles.

1. **Confidence conditions (ALL must be true for `confident = true`):**
   - **AHRS agreement:** quaternion angular distance between ESKF and Mahony AHRS < `⚠️ VALIDATE 15°`
   - **Covariance health:** max diagonal of P < `⚠️ VALIDATE` threshold (position < 100m², velocity < 10 m²/s², attitude < 0.5 rad²)
   - **Innovation consistency:** no sensor channel has sustained innovation > 3σ for more than `⚠️ VALIDATE 5 seconds`
   - **Phase agreement:** ESKF innovation behavior is consistent with state machine's detected phase (e.g., if state machine says BOOST but baro innovations suggest stationary, flag disagreement)

2. **Output:**
   ```cpp
   struct confidence_output_t {
       bool confident;                   // safe to execute irreversible actions
       float ahrs_divergence_deg;        // ESKF vs Mahony angle
       uint32_t time_since_confident_ms; // 0 when confident, counts up when not
       bool phase_agreement;             // sensor behavior matches detected phase
   };
   ```

3. **Hysteresis:** Transition from confident→uncertain requires `⚠️ VALIDATE 3` consecutive failing evaluations (debounce). Transition back requires `⚠️ VALIDATE 5` consecutive passing evaluations (conservative).

**[GATE]:**
- Normal operation: confidence flag = true
- Cover baro port: innovation consistency fails, confidence drops. Actions locked
- Uncover: confidence recovers within `⚠️ VALIDATE 15 seconds`
- Bring magnet near: AHRS divergence grows, may trip confidence gate
- Both ESKF and Mahony normal: confident = true
- Introduce sustained innovation outlier: confidence transitions to false after debounce
- CLI shows confidence state, AHRS divergence, time since last confident, phase agreement
- **No false confidence losses** during normal bench operation over 10 minutes

**[DIAG]:** Always uncertain = thresholds too tight. Never uncertain = thresholds too loose or test conditions not anomalous enough. Flapping = hysteresis too short.

---

### IVP-56: Confidence-Gated Actions

**Prerequisites:** IVP-55 (confidence gate), IVP-52 (action executor)

**Implement:** Wire confidence gate output into the Flight Director's action executor. When `confident = false`:
- Pyro channels LOCKED (cannot fire)
- TVC commands ZEROED (neutral position)
- Status LED: orange pulsing
- Telemetry: UNCERTAIN flag set
- If uncertain for > `⚠️ VALIDATE 30 seconds` during descent: execute safe fallback (deploy drogue if not already deployed)

**[GATE]:**
- Confidence loss → pyro locked within 1 tick
- Recovery → pyro re-enabled after hysteresis window
- Safe fallback triggers after timeout during simulated descent
- No pyro firing possible when `confident = false`

**[DIAG]:** Safe fallback doesn't trigger = Flight Director not consuming the flag. Pyro fires despite low confidence = action executor not checking gate.

---

### IVP-57: Vehicle Parameter Profiles

**Prerequisites:** IVP-54 (phase-scheduled Q/R framework)

**Implement:** Mission-specific Q/R presets per vehicle type. The Q/R scheduling (IVP-54) selects values per flight phase — this step provides the actual phase definitions and noise parameters per vehicle type.

1. **Profile structure:**
   ```cpp
   struct vehicle_profile_t {
       const char* name;                           // e.g., "model_rocket"
       phase_qr_params_t phase_params[kMaxPhases]; // Q/R per phase
       uint8_t num_phases;
       float boost_accel_threshold;                // |A| to detect boost
       float coast_accel_threshold;                // |A| to detect coast
   };
   ```

2. **Initial profiles:**
   - **Model rocket:** IDLE, ARMED, BOOST, COAST, DESCENT, LANDED
   - **HAB:** IDLE, ARMED, ASCENT, BURST, DESCENT, LANDED (different Q — slow ascent, no motor)
   - **Freeform:** Single phase, default Q/R (no regime switching)

3. **Selection:** Via CLI command or compile-time default. Profile is const data — no runtime allocation.

**[GATE]:**
- Switch vehicle profile: Q/R presets change accordingly
- CLI shows active profile name
- Each profile's phase definitions match the state machine's phase detection logic
- Model rocket profile validated with bench soak (IDLE phase)

**[DIAG]:** Phase definitions don't trigger = threshold mismatch between profile and state machine. All profiles identical = profiles not differentiated enough for vehicle type.

---

## Stage 9: Data Logging — *TBD after Stage 8*

**Purpose:** Flight data storage. See SAD.md Sections 8, 9.

| Step | Title | Brief Description |
|------|-------|------------------|
| IVP-58 | LittleFS Integration | Mount filesystem on remaining flash |
| IVP-59 | Logger Service | Buffered writes at `⚠️ VALIDATE 50Hz` |
| IVP-60 | Pre-Launch Ring Buffer | PSRAM ring buffer for pre-launch capture |
| IVP-61 | Log Format | MAVLink binary format, readable by Mission Planner/QGC |
| IVP-62 | USB Data Download | Download flight logs via CLI |

---

## Stage 10: System Integration — *TBD after Stage 9*

**Purpose:** Full system verification and flight readiness.

| Step | Title | Brief Description |
|------|-------|------------------|
| IVP-68 | Full System Bench Test | All subsystems running `⚠️ VALIDATE 30 minutes`, no crashes |
| IVP-69 | Simulated Flight Profile | Replay recorded accel/baro through state machine |
| IVP-70 | Power Budget Validation | Battery runtime validation under flight load |
| IVP-71 | Environmental Stress | Temperature range, vibration (if available) |
| IVP-72 | Flight Test | Bungee-launched glider: full data capture + telemetry |

> **Milestone:** IVP-72 — **Flight Ready**.

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
| Mag calibration code change | IVP-34 — IVP-38, IVP-17 | Mag cal + accel cal regression |
| Fusion algorithm change | IVP-39 — IVP-48 | Filter correctness |
| Adaptive estimation change | IVP-54 — IVP-57 | Q/R scheduling, confidence gate |
| Watchdog policy change | IVP-49, IVP-30 | Recovery behavior + mechanism |
| Flight Director change | IVP-50 — IVP-53 | State machine correctness |
| **Major refactor** | **All Stage 1-3** | Full regression |
| **Before any release** | IVP-01, IVP-27, IVP-28, IVP-30, IVP-49, IVP-68 | Release qualification (build, USB, flash, watchdog, recovery, bench test) |

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
