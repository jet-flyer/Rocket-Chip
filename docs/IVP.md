# RocketChip Integration and Verification Plan (IVP)

**Status:** ACTIVE — Living document
**Last Updated:** 2026-03-26
**Target Platform:** RP2350 (Adafruit Feather HSTX w/ 8MB PSRAM)
**Architecture:** Bare-metal Pico SDK, dual-core AMP (see `docs/decisions/SEQLOCK_DESIGN.md`)

---

## 1. Purpose and Scope

This document defines the step-by-step integration order for RocketChip firmware, from LED blink through full ESKF sensor fusion and telemetry. Every step has a concrete verification gate — a pass/fail test that must succeed before proceeding.

**This is a living document:**
- **Stages 1-4** (Foundation, Sensors, Dual-Core, GPS): Fully detailed
- **Phase M** (Magnetometer Calibration): Fully detailed — added out-of-sequence to correct a missed Phase 3 dependency (see note below)
- **Stage 5** (Sensor Fusion): Fully detailed
- **Stage 6** (Data Logging): Core items (IVP-49–54) HW verified. IVP-55–56 deferred (stretch goals).
- **Stage 7** (Radio & Telemetry): Core items (IVP-57–61) HW verified. IVP-62 deferred (`ivp62-wip` branch). IVP-63–65 deferred (stretch goals).
- **Stage 8** (Flight Director): COMPLETE — 10 IVPs (IVP-66–75), 552 host tests, bench sim 9/9
- **Stage 9** (Active Object Architecture): Fully detailed — 7 IVPs (IVP-76–82), QF+QV migration + SPIN verification
- **Stages 10-12** (Adaptive Estimation, Ground Station, Integration): Placeholders expanded as earlier stages complete

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
| **6** | **Data Logging** | **Phase 5** | **IVP-49 — IVP-56** | **Core complete** | **(pulled forward)** |
| **7** | **Radio & Telemetry** | **Phase 5** | **IVP-57 — IVP-65** | **Core complete** | |
| 8 | Flight Director | Phase 6 | IVP-66 — IVP-75 | Full | **Crowdfunding Demo Ready** |
| **9** | **Active Object Architecture** | **Phase 6** | **IVP-76 — IVP-82** | **Full** | |
| 10 | Adaptive Estimation & Safety | Phase 6 | IVP-83 — IVP-86 | Placeholder | |
| **11** | **Ground Station** | **Phase 7** | **IVP-87 — IVP-92** | **Placeholder** | |
| 12 | System Integration | Phase 9 | IVP-93 — IVP-97 | Placeholder | **Flight Ready** |

> **Stage 6 pull-forward rationale:** Data Logging was originally Stage 9 but is a dependency for the telemetry encoder — the encoder reads from data structures (FusedState, TelemetryState, SensorSnapshot) defined by the logging architecture. Pulling logging forward establishes the canonical data model that all downstream consumers (telemetry encoder, flight director, GCS) read from. IVP numbers were renumbered sequentially. See council reviews: `docs/decisions/Telem+logging/council_data_logging.md` and `council_telemetry_protocol.md`.

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

**Implement:** `src/fusion/mahony_ahrs.h/.cpp` — independent attitude estimator running alongside ESKF. Lightweight PI controller on orientation error. Uses same IMU data from seqlock but maintains its own quaternion. Provides the independent cross-check required by the confidence gate (IVP-84).

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

## Stage 6: Data Logging

**Purpose:** Define canonical onboard data model, implement PCM-based flight logging to PSRAM and flash. Establishes data structures that all downstream consumers (telemetry encoder, flight director, GCS) read from. Pulled forward from original Stage 9 because the telemetry encoder (Stage 7) depends on these definitions.

**Dependencies:** Stage 5 (FusionTask outputs FusedState)

**Pre-stage check:** Verify ICM-20948 DLPF register configuration matches intended bandwidth and output rate. Read back ACCEL_CONFIG, GYRO_CONFIG, SMPLRT_DIV registers. Confirm oversampling/DLPF bandwidth appropriate for logging fidelity. Correct before proceeding if mismatched.

| Step | Title | Brief Description |
|------|-------|------------------|
| IVP-49 | Data Model & ICD | FusedState, TelemetryState, SensorSnapshot struct definitions ✓ |
| IVP-50 | Timestamp Architecture | MET + UTC epoch anchor from GPS/RTC ✓ |
| IVP-51 | PCM Frame Format | Sync word, header, frame type byte, decommutation table ✓ |
| IVP-52 | PSRAM Ring Buffer | Configurable rate logging (1–200 Hz, 50 Hz default) ✓ |
| IVP-53 | Flash Storage & Flight Table | Post-landing flush, flight log table, pre-erase pool ✓ |
| IVP-54 | USB Log Download | CLI flight listing + raw download + Python CSV decoder ✓ |
| IVP-55 | Raw Sensor Logging | *(deferred)* SensorSnapshot in log frames, Research Mode |
| IVP-56 | Economy Tier & HAB Flush | *(deferred)* 1–2 Hz reduced logging, periodic flash flush |

**[GATE]:** Flight log captured to PSRAM at 50 Hz during simulated flight, flushed to flash after landing, downloaded via USB, decoded to CSV with correct timestamps and plausible values. Pre-launch data (5+ sec before trigger) preserved.

---

### IVP-49: Data Model & ICD

**Prerequisites:** Stage 5 complete (FusionTask outputs FusedState)

**Implement:** Define three canonical data structures in `include/rocketchip/`:

1. **FusedState** — float32, ESKF-internal. Written by FusionTask. Contains full 24-state ESKF output (quaternion, position, velocity, biases, covariance diagnostics). This is the internal representation — never serialized directly to wire or flash.

2. **TelemetryState** — fixed-point wire-ready. Converted from FusedState by a dedicated conversion function. Consumed by both the logger (Stage 6) and telemetry encoder (Stage 7). Fixed-point scaling documented as code comments — the struct IS the ICD. Fields per telemetry council consensus: attitude quaternion (int16×4 Q15), position lat/lon/alt (int32×3), velocity NED (int16×3 cm/s), baro alt (int32 mm), GPS groundspeed (uint16 cm/s), GPS fix+sats (uint8 packed), flight state (uint8), health bitfield (uint8), battery mV (uint16), MET ms (uint32).

3. **SensorSnapshot** — raw pre-calibration readings. Written by SensorTask. Contains raw accel/gyro/mag (int16×3 each), raw baro pressure/temp (int32), GPS fields, microsecond MET. All pre-calibration — can reconstruct calibrated from raw + coefficients but never reverse.

**[GATE]:**
- All three structs compile and have `static_assert` on expected sizes
- `fused_to_telemetry()` conversion roundtrips correctly (float32 → fixed-point → back within quantization error)
- TelemetryState packed size matches telemetry council reference payload (42 bytes)
- SensorSnapshot captures pre-calibration values verified against raw register reads

**[DIAG]:** Size mismatch = padding. Use `__attribute__((packed))` or verify alignment. Conversion loss = check fixed-point scaling factors match the documented ranges.

---

### IVP-50: Timestamp Architecture — *preliminary*

**Prerequisites:** IVP-49

**Implement:** 4-byte MET (Mission Elapsed Time) from `time_us_64()` in every frame header. 49.7-day range at millisecond resolution — sufficient for all mission profiles including extended HAB flights. UTC epoch anchor from GPS time (primary) or PCF8523 RTC on Adalogger FeatherWing (secondary). Fallback to boot-relative MET when neither is available. Both MET and UTC anchor recorded in flight log metadata for post-flight reconstruction.

**[GATE]:**
- MET timestamp monotonically increasing in every logged frame
- GPS-anchored UTC epoch written to log metadata when GPS fix acquired
- Post-flight tool can convert MET → wall-clock time using stored anchor
- MET rollover behavior documented (49.7 days at ms resolution)

**[DIAG]:** MET jumps = `time_us_64()` overflow handling wrong. UTC anchor missing = GPS never acquired fix during session. RTC fallback not working = PCF8523 not on I2C bus or not initialized.

---

### IVP-51: PCM Frame Format — *preliminary*

**Prerequisites:** IVP-49, IVP-50

**Implement:** Fixed-size PCM frames: 16-bit sync word + frame header (4B MET, 1B frame type, 1B length) + payload. Frame type byte identifies tier: Economy=0, Standard=1, Research=2. Decommutation table as `const struct` from Mission Profile — maps byte offsets to field names, types, and scaling. Encode/decode roundtrip must be verified.

**[GATE]:**
- Encode a TelemetryState into a PCM frame and decode back — all fields match within quantization
- Sync word detection in a byte stream with injected corruption — decoder resynchronizes within 2 frames
- Frame type byte correctly selects the right decommutation table
- Frame size matches expected: `⚠️ VALIDATE` sync(2) + header(6) + payload(42 for Standard) = 50 bytes

**[DIAG]:** Decode fails = endianness or offset error in decommutation table. Sync never found = wrong sync word value or byte-alignment issue in stream. Wrong frame type = tier selection not wired to Mission Profile.

---

### IVP-52: PSRAM Ring Buffer — *preliminary*

**Prerequisites:** IVP-51

**Implement:** Write TelemetryState PCM frames to PSRAM ring buffer at configurable rate. Rate is a continuous slider (`log_rate_hz`, 1–200 Hz) with preset labels: Economy (1–2 Hz), Standard (50 Hz), Research (200 Hz). Decimation from fusion output rate uses averaging (not simple skip) — average groups of N samples for cleaner data. `⚠️ VALIDATE ~10` extra MAC ops per frame for averaging decimation. Pre-launch continuous capture from boot — ring buffer wraps, preserving most recent data.

**[GATE]:**
- 50 Hz Standard tier: continuous write to PSRAM verified for `⚠️ VALIDATE 10+` minutes without corruption
- Wraparound: oldest data overwritten, newest preserved, no frame corruption at wrap boundary
- Rate slider: changing `log_rate_hz` at runtime adjusts decimation counter immediately
- Pre-launch capture: at least 5 seconds of pre-launch data preserved after simulated launch trigger
- PSRAM write throughput: 50 Hz × `⚠️ VALIDATE 50B` frame = 2.5 KB/s — well within PSRAM bandwidth

**[DIAG]:** Frame corruption at wrap = pointer arithmetic error at buffer boundary. Rate wrong = decimation counter not matching fusion output rate. PSRAM access fails = QSPI init issue or conflict with flash operations (use `flash_safe_execute()` for any flash access).

---

### IVP-53: Flash Storage & Flight Table — *preliminary*

**Prerequisites:** IVP-52

**Implement:** Post-landing PSRAM-to-flash flush. Raw flash sectors (sequential write, no filesystem — per council consensus, LittleFS deferred until file management needed). Flight log table in Tier 1 storage: start_sector, end_sector, MET_start, duration_s, frame_size, rate_hz per flight. Pre-erase sector pool during pre-launch idle for future HAB periodic flush.

Flash flush requires `flash_safe_execute()` — both cores paused during sector erase (~45 ms per 4 KB sector). `⚠️ VALIDATE` 2 MB flush = 512 sectors × 45 ms ≈ 23 seconds of periodic Core 1 pauses. Acceptable post-landing.

**[GATE]:**
- Simulated flight: PSRAM buffer flushed to flash after landing trigger
- Flight log table updated with correct metadata (sector range, duration, rate)
- Flash data matches PSRAM data byte-for-byte (verified by readback)
- Multiple flights: flight table accumulates entries, no overlap in sector allocation
- Pre-erase pool: sectors erased during idle, reducing flush time

**[DIAG]:** Flash write fails = `flash_safe_execute()` timeout or sector not erased. Data mismatch = PSRAM address calculation error during flush. Table corrupt = Tier 1 storage write issue. Core 1 crash during flush = forgot `multicore_lockout` (use `flash_safe_execute()` which handles this).

**[LL]:** Entry 4 (flash ops break USB), Entry 12 (USB CDC init order)

---

### IVP-54: USB Log Download — *preliminary*

**Prerequisites:** IVP-53

**Implement:** CLI command to list flights from log table and download raw data over USB serial. Python host script (`scripts/decode_flight_log.py`) decodes PCM frames to CSV: time, altitude, accel_xyz, gyro_xyz, lat, lon, velocity_ned, flight_state. CLI: `'f'` for flight list, `'d' + flight_id` for download.

**[GATE]:**
- CLI `'f'` lists all recorded flights with ID, duration, sample count, date
- CLI `'d1'` downloads flight 1 as raw binary over USB serial
- Python script decodes binary → CSV with correct column headers and plausible values
- Downloaded data matches PSRAM/flash content (no corruption from USB transfer)
- Transfer rate: `⚠️ VALIDATE` 115200 baud ≈ 11.5 KB/s, 2 MB flight ≈ 3 minutes

**[DIAG]:** Download stalls = USB CDC buffer full, need flow control or pacing. CSV values wrong = decommutation table mismatch between firmware and Python script. Missing flights = flight table not persisted across power cycle.

---

### IVP-55: Raw Sensor Logging — *(placeholder — deferred)*

**Prerequisites:** IVP-52

**Implement:** Toggle to include SensorSnapshot alongside TelemetryState in log frames. Independent of log rate — raw sensor data at native sensor rate (up to 1 kHz IMU). Per-sensor bitmask in advanced config. Research Mode gated — requires explicit user acknowledgment.

---

### IVP-56: Economy Tier & HAB Flush — *(placeholder — deferred)*

**Prerequisites:** IVP-53

**Implement:** 1–2 Hz decimation with reduced field set (position + altitude + state only). Periodic flash flush every `⚠️ VALIDATE 60s` for long-duration missions where PSRAM volatility is a risk. Required for HAB Mission Profile where power loss before landing is expected. Pre-erase sector pool (from IVP-53) enables program-only flush during flight (~100 µs per 256B page, no multicore lockout stutter).

---

## Stage 7: Radio & Telemetry

**Purpose:** Encode logged data for radio transmission, manage radio link, implement receive-side decoding and protocol translation. Same firmware on all boards — TX/RX is a compile-time configuration (`ROCKETCHIP_RADIO_RX`), selected by Mission Profile. Not a runtime toggle.

**Dependencies:** Stage 6 (TelemetryState struct, PCM frame format), RFM95W hardware

| Step | Title | Brief Description |
|------|-------|------------------|
| IVP-57 | RFM95W Radio Driver | SPI bus init, LoRa TX/RX, polling ✓ |
| IVP-58 | Telemetry Encoder | CCSDS/MAVLink strategy interface, Mission Profile selects encoder ✓ |
| IVP-59 | Telemetry Service | Configurable TX rate, APID prioritization, duty cycle management ✓ |
| IVP-60 | RX Mode + CCSDS Decode | Compile-time RX mode — CCSDS decode, CSV output, NeoPixel link quality ✓ |
| IVP-61 | QGC Validation | End-to-end TX→RX→QGC display verification ✓ |
| IVP-62 | Bidirectional Commands | *(deferred — `ivp62-wip` branch)* Command dispatch complete, QGC USB CDC blocker |
| IVP-63 | FSK Continuous Bitstream | *(deferred)* |
| IVP-64 | Radio Mode Profiles | *(deferred)* |
| IVP-65 | Native MAVLink TX | *(deferred)* |

**Gate:** TX RocketChip transmits, RX RocketChip receives and translates, QGC/Mission Planner displays live attitude + map + flight state. Stable 10+ min at SF7/BW125.

---

### IVP-57: RFM95W LoRa Radio Driver ✓ VERIFIED 2026-02-25

**Prerequisites:** Stage 5 complete (sensor fusion running), SPI0 hardware available

**Implement:** SPI bus driver (`src/drivers/spi_bus.h/.cpp`, GPIO-controlled CS), RFM95W LoRa driver (`src/drivers/rfm95w.h/.cpp`). Optional peripheral — absent FeatherWing detected at boot via RegVersion read (expected 0x12). LoRa parameters: 915 MHz, SF7, BW 125 kHz, CR 4/5. Polling IRQ pattern (`rfm95w_poll_irq()`) instead of hardware interrupt for initial driver. Ground station: standalone Pico SDK build targeting Adafruit Fruit Jam (#6200, RP2350B) with RFM95W breakout (#3072), SPI1 (GPIO 28/30/31).

Council-reviewed (unanimous, 6 amendments incorporated): 64-bit frequency calculation, GPIO CS not hardware CS, FIFO pointer reset before TX, 100ms TX timeout, used-registers-only constants, `poll_irq()` for deferred ISR swap.

**[GATE] — All 8 gates verified 2026-02-25 (commit d2f3cbe):**
1. **Init:** `RegVersion = 0x12` confirmed, radio transitions to standby
2. **Absent hardware:** FeatherWing not stacked → graceful skip at boot, no failures
3. **TX:** packet transmitted successfully, no timeout
4. **RX:** packet received on ground station with correct payload
5. **RSSI:** −54 to −58 dBm at desk range, SNR 8–10 dB
6. **Loopback:** 100% delivery over 100 packets, <1% CRC errors
7. **Range:** link holds through interior walls at 5 dBm
8. **Integration soak:** 120K IMU reads, 0 errors at 10 Hz TX rate; all sensors nominal with FeatherWing stacked

**[DIAG]:** `RegVersion` not 0x12 = SPI wiring or CS jumper issue on FeatherWing. TX timeout = check RST line, radio may need longer settle after init. RX misses = frequency or spreading factor mismatch between flight and ground station. IMU errors during TX = SPI/I2C bus isolation issue (SPI0 and I2C1 are independent peripherals on RP2350, should not interfere).

**[LL]:** Entry 25 (use debug probe for iterative flashing — picotool bus corruption risk)

---

### IVP-58: Telemetry Encoder — *preliminary*

**Prerequisites:** IVP-57 (radio driver), IVP-49 (TelemetryState struct defined)

**Implement:** `TelemetryEncoder` strategy interface with two concrete implementations, selected by Mission Profile at boot. Both encoders read from the same `TelemetryState` struct defined in Stage 6.

1. **CcsdsEncoder:** CCSDS Space Packet (pruned stack). 6-byte primary header (big-endian per CCSDS 133.0-B-2 §4.1.1, explicit byte-swap from RP2350 little-endian) + 4-byte secondary header (MET timestamp) + 42-byte nav payload + optional 2-byte CRC = 52–54 bytes. APID field enables future multiplexing (nav at 5 Hz, diagnostics at 0.5 Hz on same link). Primary encoder for `Rocket_Pro` profiles.

2. **MavlinkEncoder:** MAVLink v2 three-message set (HEARTBEAT + ATTITUDE_QUATERNION + GLOBAL_POSITION_INT = 105 bytes). Default encoder for `Rocket_Edu` profiles. Enables direct QGC/Mission Planner display without translation layer.

Both always compiled in (~4.5 KB total). Strategy pattern — no `#ifdef`, no recompilation. `packet_type` parameter in interface.

**[GATE]:**
- CCSDS packet encodes and decodes to original values with no field corruption
- MAVLink messages parse correctly in QGC/Mission Planner
- Strategy pattern selects encoder at boot based on Mission Profile
- Encoder adds < `⚠️ VALIDATE 50µs` overhead per telemetry tick
- Big-endian wire format verified: CCSDS packets decode correctly on little-endian host

**[DIAG]:** Endianness mismatch = byte-swap logic incorrect in CCSDS header packing. Packet size exceeds LoRa payload limit (255 bytes max for SX1276) = field count too large; split into multiple APID packets or reduce nav payload. MAVLink CRC_EXTRA mismatch = message definition version difference between firmware and GCS.

---

### IVP-59: Telemetry Service — *preliminary*

**Prerequisites:** IVP-58 (encoder), IVP-57 (radio driver)

**Implement:** Downlink service running on Core 0. Configurable TX rate: 2 Hz default, burst 5–10 Hz during powered flight (state machine provides phase detection when available from Stage 8). APID prioritization for CCSDS mode (nav packets higher priority than diagnostic). Duty cycle management — at SF7/BW125, keep TX duty below `⚠️ VALIDATE 50%` to leave margin for uplink window.

**[GATE]:**
- 2 Hz packets received continuously on ground station over 5 minutes
- Burst mode: rate increases to 5 Hz when triggered (manual trigger until Stage 8 state machine)
- No queue overflow under sustained load
- IMU and baro sample rates on Core 1 unaffected by radio TX
- Duty cycle: verify TX time-on-air per packet × rate ≤ FCC Part 15 limit for 915 MHz ISM

**[DIAG]:** Queue overflow = rate too high or packet assembly too slow. Sensor rate degradation = radio TX blocking Core 0 longer than expected; profile TX call duration. Duty cycle exceeded = packet size or rate too high for current LoRa parameters.

---

### IVP-60: RX Mode + CCSDS Decode

**Prerequisites:** IVP-59 (telemetry service TX), IVP-58 (CCSDS encoder)

**Architecture (revised 2026-03-07):** RX mode is a **compile-time configuration**, not a runtime toggle. `ROCKETCHIP_RADIO_RX=1` in CMakeLists.txt selects RX mode (ground station build). Default (undefined) is TX mode (flight downlink). Same `main.cpp`, same main loop — the `if constexpr (kRadioModeRx)` branches compile out the unused path. Sensors, ESKF, logging all proceed normally if hardware is present (inert if absent). CLI stays active in RX mode. Will be set by Mission Profile in Stage 8.

**Implement:**
1. `ccsds_decode_nav()` — reverse of `CcsdsEncoder::encode_nav()`. Validates primary header (version, type, APID), verifies CRC-16-CCITT, extracts TelemetryState + sequence counter + MET.
2. `telemetry_service_start_rx()` / `telemetry_service_stop_rx()` — enter/exit RX continuous mode.
3. `telemetry_service_rx_tick()` — polls `rfm95w_available()`, calls `rfm95w_recv()`, decodes CCSDS, prints CSV line with RSSI/SNR.
4. `kRadioModeRx` constexpr in `config.h` — compile-time mode selection. Ground station CMakeLists.txt sets `ROCKETCHIP_RADIO_RX=1`. `t` key shows RX stats when in RX build.
5. NeoPixel RX overlay: solid green (receiving), 2Hz yellow blink (>1s gap), fast red blink (>5s gap).

**RX CSV output:** `RX,seq,rssi,snr,lat,lon,alt_m,vel_n,vel_e,vel_d,baro_alt_m,q_w,q_x,q_y,q_z,state,met_ms`

**[GATE]:**
- TX board: flight build (default). RX board: ground station build (`ROCKETCHIP_RADIO_RX=1`)
- RX serial shows CSV telemetry lines with valid RSSI/SNR
- CLI `t` on RX board shows packet count, RSSI, CRC errors
- NeoPixel: solid green while receiving, yellow/red blink on gap
- 5-min soak: count packets, no USB stalls
- Sensor rates on TX board unaffected

**[DIAG]:** No packets received = BW mismatch or DIO0 mapping wrong. CRC errors = frequency drift or interference. CSV stalls = USB CDC buffer full (rate-limit output or increase buffer).

---

### IVP-61: QGC Validation — *preliminary*

**Prerequisites:** IVP-60 (translation layer), IVP-59 (telemetry service)

**Implement:** End-to-end user acceptance test. TX RocketChip sends telemetry, RX RocketChip receives and translates to MAVLink, QGroundControl or Mission Planner displays attitude + map + flight state. This is the "live telemetry to QGroundControl" demo gate.

**[GATE]:**
- QGC displays real-time attitude indicator matching physical board orientation
- QGC map shows GPS position (when GPS available)
- Flight state visible in QGC HUD or status text
- Telemetry updates visible within `⚠️ VALIDATE 200ms` of packet reception
- Stable 10+ min operation at SF7/BW125

**[DIAG]:** Attitude wrong = quaternion field order or convention mismatch (NED vs ENU). Map position wrong = lat/lon scaling or sign error. Latency > 200ms = GCS rendering bottleneck, not radio link.

---

### IVP-62: Bidirectional Commands — *preliminary*

**Prerequisites:** IVP-61 (GCS link verified)

**Implement:** Uplink command channel from GCS to flight computer. Commands: calibrate, arm/disarm, parameter get/set. Each command requires an acknowledge packet in response. Partial — reception and ACK/NACK implemented here; full command execution depends on Stage 8 state machine for arm/disarm. Command authentication TBD (simple sequence number or HMAC for production).

**[GATE]:**
- Calibrate command received and acknowledged; ACK sent back to GCS
- Arm command received; ACK sent (execution deferred to Stage 8)
- Parameter set received; value updated and verified via subsequent parameter get
- Unrecognized command: NACK sent, no state change

**[DIAG]:** ACK not received = uplink packet lost or GCS not listening on RX. Parameter changes don't persist = not saved to flash after update.

---

### IVP-63: FSK Continuous Bitstream — *(placeholder — deferred)*

**Prerequisites:** IVP-57

**Implement:** SX1276 FSK continuous mode for IRIG-heritage PCM telemetry transport. Ground-side software sync detection, bit-slip correction, and frame alignment. Research Mode gated.

---

### IVP-64: Radio Mode Profiles — *(placeholder — deferred)*

**Prerequisites:** IVP-59

**Implement:** Mission Profile-driven radio configuration: SF7/BW125 default, SF9-12 HAB long range, SF6/BW500 short-range high-rate, FSK packet high-rate. Same hardware, register config changes only.

---

### IVP-65: Native MAVLink TX — *(placeholder — deferred)*

**Prerequisites:** IVP-58

**Implement:** Transmit MAVLink natively over LoRa for drone profiles or direct QGC connection without translation layer. Mission Profile option — bypasses CCSDS encoding for simplicity on profiles that don't need it.

---

## Stage 8: Flight Director

**Purpose:** Flight state machine using UML statecharts (QEP), event-driven condition evaluation, and action execution. See `docs/flight_director/STATE_MACHINE_FORMALISM_RESEARCH.md` for formalism rationale, `docs/decisions/flight_director/council_state_machine_formalism.md` for council decisions, and `docs/flight_director/QP_APPLICATION_GUIDE.md` for QEP integration details.

**Key decisions (resolved):**
- **SAD Open Question #4 resolved:** UML statecharts — not pure Mealy or Moore. Entry/exit actions for state-invariant behavior, transition actions for source-dependent irreversible operations (safety architecture). See council review.
- **Apogee is an event, not a state.** EVT_APOGEE is a detection event (velocity zero-crossing). Drogue deploy is a transition action on COAST→DESCENT.
- **QEP for Stage 8, QF+QV as target architecture.** QEP dispatches from existing superloop. Active Object migration in Stage 9.
- **STARS autocoder gated on toolchain validation** (IVP-67). If setup is clean, STARS is primary; if problematic, hand-authored QEP with STARS deferred.
- **Phase topology:** 7 top-level states (IDLE, ARMED, BOOST, COAST, DESCENT, LANDED, ABORT). DESCENT is a superstate with DROGUE and MAIN sub-phases. `state_flight` superstate provides common abort handling for BOOST/COAST/DESCENT.
- **Transition-gated irreversible actions (safety).** Pyro firing is a transition action, not an entry action. Any path into a state cannot inadvertently fire a pyro. This is a safety architecture decision — see research doc Section 5.

**Prerequisites:**
- IVP-66 (Watchdog Recovery Policy) must be completed first.
- Prior art carried forward from deprecated `docs/flight_director/RESEARCH.md` (condition evaluator, pre-arm checks, command handler, abort matrix) and `docs/flight_director/FLIGHT_DIRECTOR_DESIGN.md` (action executor, FlightState struct).

**Pending doc work (council Action Items #1, #2):** ~~SAD.md Open Question #4 must be closed~~ **DONE** (2026-03-14, moved to Resolved). The deprecated `FLIGHT_DIRECTOR_DESIGN.md` should be rewritten from the council findings and new research once Stage 8 implementation validates the design against hardware (Stage 9 prerequisite). The deprecated doc is retained as reference until then.

| Step | Title | Brief Description |
|------|-------|------------------|
| IVP-66 | Watchdog Recovery Policy | Scratch register persistence, reboot counting, state-aware recovery path |
| IVP-67 | Toolchain Validation | QP/C + STARS + QM + SPIN compilation gate against Pico SDK |
| IVP-68 | QEP Integration + Phase Skeleton | Dispatch engine, phase stubs, HSM hierarchy, CLI event injection |
| IVP-69 | Command Handler + Pre-Arm | CMD_ARM validation, two-tier pre-arm checks, rejection behavior |
| IVP-70 | Core Guard Functions | 5-6 guards for rocket profile, sustain timing, compare_field helper |
| IVP-71 | Multi-Method Detection Logic | AND/OR/TIMEOUT combinators, edge detection, validity policy |
| IVP-72 | Action Executor | LED, logging, telemetry actions on transitions and state entry/exit |
| IVP-73 | Bench Flight Simulation (SITL) | Full IDLE→LANDED sequence via CLI-injected simulated stimuli |
| IVP-74 | Mission Configuration | Baked-in const profile (FEMA approach), CLI preview |
| IVP-75 | Active Object Migration Planning | QF+QV compile gate, AO boundary design, migration plan for Stage 9 |

> **Milestone:** At IVP-74 completion, the system has calibrated sensors, GPS, sensor fusion, data logging, radio link, and a working flight state machine — **Crowdfunding Demo Ready**.

---

### IVP-66: Watchdog Recovery Policy

**Prerequisites:** IVP-30 (hardware watchdog mechanism), Stage 5 complete

**Rationale:** IVP-30 implemented the watchdog *mechanism* (dual-core heartbeat, 5s timeout, kick pattern). The mechanism is correct for ground/IDLE — a full reboot recovers cleanly. But a full reboot mid-flight (ARMED through DESCENT) is catastrophic: ESKF state lost, pyro timers reset, position/velocity knowledge gone. The state machine (IVP-68) cannot be designed without first defining what happens when the system wakes up after an unexpected reset.

**Bug fix (included):** Replace `watchdog_enable_caused_reboot()` with `watchdog_caused_reboot()` in `init_hardware()`. The `_enable_` variant checks a scratch register magic value that persists across picotool flashes and soft resets, causing false watchdog warnings on clean boots. The base `watchdog_caused_reboot()` checks `rom_get_last_boot_type() == BOOT_TYPE_NORMAL` on RP2350, correctly distinguishing true WDT timeouts from reflash reboots.

**Design principle: Subsystem restart, not full reboot.** A hardware watchdog reset is unavoidable (it's a chip-level reset), but the *software response* should restart only the failed subsystem when possible. If ESKF diverges, restart ESKF — don't reboot the whole system. If Core 1 sensor loop hangs, attempt Core 1 restart before triggering a full WDT reset. The watchdog is the last resort when targeted recovery fails, not the first response to any anomaly.

**Implement:**

1. **Scratch register persistence.** Before enabling the watchdog, write diagnostic context to `watchdog_hw->scratch[5..7]` (scratch[4] is reserved by the SDK for reboot magic). On each main loop iteration, update scratch registers with: current flight state enum (scratch[5] low byte), last tick function ID (scratch[5] high byte), monotonic reboot counter (scratch[6]), and a CRC or magic to validate the data (scratch[7]). These survive a WDT reset but not a power-on reset.

2. **Reboot counter with safe-mode lockout.** On boot, read scratch[6] reboot counter. If counter > `⚠️ VALIDATE 3` within a detection window (use scratch[7] timestamp or magic to detect rapid reboots vs. widely-spaced ones), enter safe mode: NeoPixel solid red, beacon-only operation, no ESKF, no pyro. Log reboot count to flash if storage is available. Counter resets on clean power-on (scratch registers cleared by POR).

3. **Ground-side launch abort policy.** Any watchdog reset while in IDLE or ARMED state sets a persistent `LAUNCH_ABORT` flag. This flag requires explicit operator acknowledgement (CLI command or physical reset sequence) before the system can transition to ARMED. Rationale: a WDT reset on the ground indicates a software fault that must be investigated before flight. Even a single ground-side WDT event should abort launch preparation and require a restart of pre-flight procedure. The flag persists across soft resets (stored in scratch registers) and clears only on POR or explicit operator command.

4. **ESKF failure backoff.** Add a consecutive-failure counter for ESKF init→diverge cycles. After `⚠️ VALIDATE 5` consecutive failures (init succeeds but `healthy()` returns false within N seconds), disable ESKF until manual CLI reset (`'e'` key or similar). This prevents the init→diverge→re-init churn observed during ESKF development without requiring a full system reboot. This is the model for subsystem-level recovery: detect failure, disable the failing subsystem, continue operating in degraded mode.

5. **Recovery boot path.** In `init_hardware()` / `init_application()`, after reading scratch registers: if this is a WDT reboot (not POR), log the event, print the diagnostic context from scratch registers, and set a `g_recoveryBoot` flag. The state machine (IVP-68) will use this flag to determine post-reboot behavior per flight state:
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

---

### IVP-67: Toolchain Validation (QP/C + STARS + QM + SPIN)

**Prerequisites:** IVP-66

**Rationale:** Gates the STARS adoption decision. QEP is the minimum requirement for IVP-68. STARS is the preferred authoring tool but adoption depends on clean setup. This IVP validates the entire toolchain in a single session before committing to it.

**Implement:**

1. **QP/C compilation.** Clone `github.com/QuantumLeaps/qpc`. Vendor QEP files into `lib/qep/` (same pattern as ArduPilot sparse-checkout). Write `qp_port.h` for RP2350 bare-metal Pico SDK (critical sections via `save_and_disable_interrupts()`, `Q_NASSERT` to use project's own assert). Compile against ARM Cortex-M33 target.

2. **Trivial QEP test.** 3-state HSM (IDLE→ACTIVE→DONE) with entry/exit actions and one transition action. Link against Pico SDK. Flash to hardware via debug probe. Verify state transitions via serial output.

3. **QM installation.** Install QM (Quantum Modeler) on development machine. Open, create trivial 3-state model matching the test above.

4. **STARS autocoder.** Clone `github.com/JPLOpenSource/STARS`. Run STARS JAR against the QM model. Verify it generates: C++ (QEP state handlers), Promela (SPIN model), Python (interactive simulator).

5. **SPIN verification.** Install SPIN. Write one trivial LTL safety property (`[] !(state == IDLE && action_fired)`). Verify against generated Promela model.

6. **Generated code integration.** Compile STARS-generated C++ against Pico SDK. Link and flash. Verify behavior matches hand-written test from step 2.

7. **Python simulator.** Run STARS-generated Python simulator. Send events, observe state transitions.

**[GATE]:**
- QEP compiles and links against Pico SDK ARM Cortex-M33 target
- Trivial HSM runs on RP2350 hardware with correct state transitions
- QM installs and opens on development machine
- STARS generates C++, Promela, and Python from QM model
- Generated C++ compiles and links against Pico SDK
- SPIN verifies trivial safety property against generated Promela
- Python simulator responds to simulated events

**Gate decision:** All checks pass in one session → STARS is primary authoring tool for IVP-68+. Java version conflicts, SPIN issues, or generated code incompatibilities → fall back to hand-authored QEP, revisit STARS later. Document decision in `AGENT_WHITEBOARD.md`.

**[DIAG]:** QEP won't compile = missing port header or C/C++ linkage mismatch (QEP is C99, project is C++20 — use `extern "C"`). STARS JAR fails = Java version requirement. Generated code won't link = STARS output assumptions don't match Pico SDK build. SPIN install fails = platform issue, try pre-built binary.

---

### IVP-68: QEP Integration + Phase Skeleton

**Prerequisites:** IVP-67 (QEP compiles), IVP-57 (radio link operational)

**Implement:** Flight Director as a QEP hierarchical state machine called from the existing Core 0 superloop at 100Hz. UML statechart formalism — entry/exit actions (Moore) + transition actions (Mealy) with guaranteed execution ordering.

1. **HSM hierarchy:**
   ```
   state_top                      (root — handles unhandled events)
   ├── phase_idle                 (ground, pre-arm)
   ├── phase_armed                (ground, armed, awaiting launch)
   ├── state_flight               (superstate — common abort handling)
   │   ├── phase_boost            (powered ascent)
   │   ├── phase_coast            (unpowered ascent, apogee detection active)
   │   └── state_descent          (superstate — recovery phase)
   │       ├── phase_drogue       (drogue descent, awaiting main deploy altitude)
   │       └── phase_main         (main chute descent)
   ├── phase_landed               (post-flight, data preservation)
   └── phase_abort                (safe state, profile-defined actions)
   ```

2. **State handler stubs.** Each phase is a QEP state handler function (see QP_APPLICATION_GUIDE.md Section 3.1). Stub handlers process Q_ENTRY_SIG, Q_EXIT_SIG, SIG_TICK, and defer unhandled events to parent via Q_SUPER(). Entry/exit actions initially just log transitions.

3. **Signal enum.** SIG_TICK, SIG_ARMED, SIG_LAUNCH, SIG_BURNOUT, SIG_APOGEE, SIG_MAIN_DEPLOY, SIG_LANDING, SIG_ABORT, SIG_DISARM, SIG_RESET. Starts after Q_USER_SIG.

4. **Superloop integration.** `run_flight_director()` dispatches SIG_TICK and pending command signals via `QHSM_DISPATCH()`. Identical to current tick-function pattern — QEP replaces the hand-rolled switch-case.

5. **CLI event injection.** Debug commands to inject any signal manually: `'a'` = SIG_ARMED, `'l'` = SIG_LAUNCH, etc. Enables bench testing of full state sequence without sensors.

6. **FlightState struct.** Current phase enum, flight markers (launch_time, apogee_time, landing_time), status flags. Updated by entry actions.

**[GATE]:**
- All 7 top-level phases + 2 DESCENT sub-phases reachable via CLI injection
- All transitions log: `[FD] IDLE -> ARMED`, `[FD] COAST -> DESCENT.DROGUE` format
- `state_flight` superstate handles SIG_ABORT from any flight sub-phase
- Entry/exit actions execute in correct UML order (exit source → transition action → enter target)
- `g_recoveryBoot` flag routes correctly per IVP-66 policy
- QEP dispatch <5µs per event (measure with `time_us_32()`)
- CLI `'s'` status shows current phase, flight markers, FlightState contents

**[DIAG]:** Wrong transition order = LCA calculation bug (unlikely if using stock QEP). Event not handled = missing case in handler or wrong Q_SUPER() parent. Dispatch too slow = event struct too large or handler doing too much work.

---

### IVP-69: Command Handler + Pre-Arm

**Prerequisites:** IVP-68 (phase skeleton running)

**Implement:** Command validation layer between external inputs (CLI, radio, button) and the state machine. Commands are requests that can be rejected; only validated commands generate events.

1. **Commands:** CMD_ARM, CMD_DISARM, CMD_ABORT, CMD_RESET. Each produces the corresponding SIG_* event only after validation passes.

2. **Pre-arm checks (two-tier model from deprecated FLIGHT_DIRECTOR_DESIGN.md):**
   - **Tier 1 — Platform safety (7 immutable checks):** IMU healthy, baro healthy, ESKF initialized, flash storage available, battery voltage > threshold, no LAUNCH_ABORT flag, watchdog enabled.
   - **Tier 2 — Profile-specific (up to 5 per profile):** GPS lock (if GPS-equipped), mag calibrated, pyro continuity (if pyro-equipped), radio link active (if telemetry-equipped), altitude < max ground altitude.
   - Any Tier 1 failure = hard reject with reason. Any Tier 2 failure = configurable (warn or reject per profile).

3. **Rejection behavior:** Failed CMD_ARM prints all failing checks to CLI and telemetry. No partial arming — all checks must pass.

4. **CMD_ABORT:** Always accepted from any state. No validation (abort is unconditional safety action).

**[GATE]:**
- CMD_ARM with all checks passing → SIG_ARMED dispatched, state transitions to ARMED
- CMD_ARM with IMU unhealthy → rejected, reason printed, state stays IDLE
- CMD_ARM with LAUNCH_ABORT flag → rejected until flag cleared
- CMD_DISARM from ARMED → returns to IDLE
- CMD_ABORT from any flight phase → transitions to ABORT via `state_flight` superstate
- CMD_RESET from LANDED or ABORT → returns to IDLE, clears flight markers
- CLI shows pre-arm check results: `[PRE-ARM] 7/7 platform OK, 4/5 profile OK (GPS: NO LOCK)`

**[DIAG]:** Arming despite failed check = validation bypassed. Abort not working = SIG_ABORT not handled in current state's handler chain. Pre-arm always fails = check reading stale sensor state.

---

### IVP-70: Core Guard Functions

**Prerequisites:** IVP-68 (phase skeleton), IVP-48 (ESKF provides acceleration/altitude/velocity)

**Implement:** Condition evaluator — compiled guard functions that read FusedState (via seqlock, read-only from Core 0) and produce detection events for the state machine.

1. **Guard function table (from deprecated RESEARCH.md):** Each guard is a function with signature `bool guard_xxx(const FusedState& fs, const GuardConfig& cfg)`. Guards indexed by ID for profile-driven selection.

2. **Core guards for rocket profile:**
   - `guard_accel_body_z()` — launch detection: body-Z accel > threshold for N consecutive samples
   - `guard_accel_magnitude()` — burnout detection: |A| drops below threshold (drag-only)
   - `guard_velocity_zero_cross()` — apogee detection: vertical velocity sign change
   - `guard_baro_peak()` — apogee backup: baro altitude derivative goes negative
   - `guard_altitude_agl()` — main deploy: altitude below threshold
   - `guard_stationary()` — landing detection: low velocity + low altitude change for T seconds

3. **Sustain timing.** Each guard has a configurable sustain duration — condition must hold for N consecutive evaluations before triggering. Prevents single-sample false positives.

4. **`compare_field` helper.** Generic comparator: `compare_field(value, op, threshold)` where op ∈ {GT, LT, GE, LE, EQ, ABS_GT}. Guards compose from this primitive.

**[GATE]:**
- Launch detection: hand-shake/toss triggers SIG_LAUNCH after sustain period
- Apogee detection (velocity): simulated via CLI data injection, fires SIG_APOGEE
- Apogee detection (baro backup): baro peak detected within `⚠️ VALIDATE 2s` of actual peak
- Landing detection: no false positives during 5-minute bench soak
- Sustain timing: guard with 10-sample sustain doesn't trigger on 9-sample spike
- Guard functions execute in < `⚠️ VALIDATE 50µs` total per tick at 100Hz

**[DIAG]:** False launch = vibration/handling triggers threshold; raise threshold or increase sustain. Missed apogee = velocity noise prevents clean sign change; tune baro backup. Guards too slow = too many guards active or FusedState read contention.

---

### IVP-71: Multi-Method Detection Logic

**Prerequisites:** IVP-70 (core guards working)

**Implement:** Combinators for guard functions and edge detection logic. Multiple guards can be combined for robust event detection.

1. **Combinators:**
   - **AND** — conservative: all guards must pass (e.g., velocity zero-cross AND baro peak for apogee)
   - **OR** — aggressive: any guard triggers (e.g., velocity OR baro for apogee backup)
   - **PRIMARY_PLUS_TIMEOUT** — primary guard with timer fallback (e.g., velocity zero-cross primary, 15s coast timeout fallback)

2. **Edge detection.** Guards evaluate continuously at 100Hz but events fire only on rising edge (condition becomes true, not while true). Prevents repeated event generation.

3. **Validity policy.** Per-guard validity window: guard is only evaluated during specific flight phases. `guard_velocity_zero_cross` is only valid during COAST (not during BOOST where velocity is still positive but noisy). Phase-validity prevents false positives from guards evaluated in wrong context.

4. **Missed apogee fallback.** PRIMARY_PLUS_TIMEOUT combinator: if primary apogee guard (velocity zero-cross) doesn't fire within `⚠️ VALIDATE 15s` of COAST entry, timeout fires SIG_APOGEE anyway. Safety net for noisy sensors.

**[GATE]:**
- AND combinator: both velocity AND baro must agree before SIG_APOGEE
- OR combinator: either velocity OR baro triggers SIG_APOGEE
- PRIMARY_PLUS_TIMEOUT: normal case fires on primary; with primary disabled, timeout fires after 15s
- Edge detection: guard staying true doesn't re-fire event
- Phase-validity: apogee guard in BOOST phase does not trigger
- Missed apogee fallback fires correctly after coast timeout

**[DIAG]:** AND never triggers = one guard consistently false; check individual guards. OR too sensitive = false positives from weakest guard. Timeout fires prematurely = timer started too early or duration too short. Edge detection broken = flag not reset after phase transition.

---

### IVP-72: Action Executor

**Prerequisites:** IVP-71, IVP-57 (radio for telemetry status), IVP-52 (logging for event recording)

**Implement:** Maps state entry/exit actions and transition actions to hardware outputs. UML statechart action semantics — entry actions are state-invariant (execute on any entry), transition actions are path-specific (execute between exit and entry).

1. **Entry actions (Moore — execute on any entry to state):**
   - LED patterns: ARMED=solid amber, BOOST=solid red, COAST=solid yellow, DESCENT=red flash, LANDED=slow green blink, ABORT=fast red flash
   - Telemetry flags: phase enum in telemetry packet
   - Logging: flight event markers (EVT_LAUNCH, EVT_BURNOUT, EVT_APOGEE, etc.)
   - Phase-specific config: update ESKF Q/R hint for Phase-Scheduled Q/R (Stage 10, IVP-83)

2. **Transition actions (Mealy — path-specific, execute between exit and entry):**
   - **COAST→DESCENT.DROGUE:** `fire_drogue_pyro()` — **IRREVERSIBLE, transition-gated (safety architecture)**
   - **DESCENT.DROGUE→DESCENT.MAIN:** `fire_main_pyro()` — **IRREVERSIBLE, transition-gated**
   - **state_flight→ABORT:** source-phase-specific abort actions (safe pyros during BOOST, continue logging during DESCENT)
   - Pyro actions gated by confidence flag (wired at Stage 10 IVP-85)

3. **Exit actions (cleanup):**
   - `state_flight` exit: safe all outputs, stop active timers
   - Phase-specific: flush log buffer, mark phase end timestamp

**[GATE]:**
- ARMED entry: NeoPixel solid amber, telemetry ARMED flag set
- BOOST entry: NeoPixel solid red, log marker EVT_LAUNCH written
- COAST→DESCENT.DROGUE transition: drogue pyro action logged (no actual pyro wiring yet — action executor logs intent)
- LANDED entry: NeoPixel slow green blink, buzzer recovery pattern
- Actions execute within 1 tick of state transition
- Transition actions execute BETWEEN exit and entry (verify ordering via log timestamps)
- Abort from BOOST vs abort from COAST produce different logged actions

**[DIAG]:** NeoPixel not changing = entry action not wired or LED Engine not processing. Telemetry flag stale = radio tick not reading current FlightState. Transition action in wrong order = QEP dispatch bug (unlikely with stock QEP). Pyro intent logged from wrong path = transition action on wrong transition.

---

### IVP-73: Bench Flight Simulation (SITL)

**Prerequisites:** IVP-72 (action executor working), all guard functions and combinators working

**Rationale:** End-to-end integration test of the complete Flight Director pipeline before mission configuration. Validates the full IDLE→ARMED→BOOST→COAST→DESCENT.DROGUE→DESCENT.MAIN→LANDED sequence using CLI-injected stimuli, without requiring actual flight hardware (accelerometer toss, pyro channels, etc.).

**Implement:**

1. **Simulated flight sequence.** Python script (`scripts/bench_flight_sim.py`) sends CLI commands over serial to walk the state machine through a complete flight:
   - Send CMD_ARM → verify ARMED
   - Inject SIG_LAUNCH (or simulate accel spike via data injection) → verify BOOST
   - Inject SIG_BURNOUT → verify COAST
   - Wait for apogee timeout or inject SIG_APOGEE → verify DESCENT.DROGUE + drogue action logged
   - Inject SIG_MAIN_DEPLOY → verify DESCENT.MAIN + main action logged
   - Inject SIG_LANDING → verify LANDED
   - Send CMD_RESET → verify return to IDLE

2. **Abort paths.** Test abort from each flight phase:
   - BOOST abort → verify source-specific abort actions
   - COAST abort → verify different abort actions
   - DESCENT abort → verify descent-specific abort actions

3. **Error injection.** Test recovery and edge cases:
   - CMD_ARM with failed pre-arm → stays IDLE
   - Double CMD_ARM → ignored (already armed)
   - CMD_ABORT from IDLE → rejected (not in flight)
   - WDT recovery boot flag → correct post-reboot behavior per IVP-66

4. **Flight timeline validation.** Script logs all state transitions with timestamps. Post-test analysis verifies: correct sequence, correct timing (sustain periods, timeouts), correct actions per transition.

**[GATE]:**
- Complete IDLE→LANDED sequence succeeds via scripted stimuli
- All 7 phases + 2 sub-phases visited in correct order
- Abort tested from BOOST, COAST, and DESCENT — each produces correct phase-specific actions
- Error injection: all invalid commands correctly rejected
- Flight timeline matches expected sequence and timing
- No unexpected state transitions during entire test
- Script runs unattended and produces pass/fail summary

**[DIAG]:** Sequence stalls = guard not triggering on injected stimulus; check injection mechanism. Wrong phase order = transition table error. Abort actions wrong = source-phase detection in abort handler incorrect. Script hangs = serial timeout; check USB CDC connection.

---

### IVP-74: Mission Configuration

**Prerequisites:** IVP-73 (bench flight sim validates the engine works)

**Implement:** Mission profile as a `const` data structure (FEMA principle — minimize failure modes). One hardcoded rocket profile for Stage 8, designed for future flash/SD serialization.

1. **Profile structure (from deprecated RESEARCH.md):**
   ```cpp
   struct MissionProfile {
       const char* name;                    // "model_rocket"
       uint8_t num_phases;
       PhaseDefinition phases[kMaxPhases];  // guards, actions, timeouts per phase
       PreArmCheck profile_checks[5];       // Tier 2 checks
       AbortAction abort_actions[kMaxPhases]; // per-phase abort behavior
   };
   ```
   No pointers in serializable fields — use indices. Designed for future `flash_safe_execute()` storage.

2. **Initial profiles (const, compiled-in):**
   - **Model rocket:** IDLE→ARMED→BOOST→COAST→DESCENT(DROGUE/MAIN)→LANDED. Full dual-deploy with guards from IVP-70/71.
   - **HAB (high-altitude balloon):** IDLE→ARMED→ASCENT→FLOAT→DESCENT→LANDED. Different Q/R, no pyro, altitude-based phase detection.
   - **Freeform:** Single flight phase, default thresholds, logging only. For non-rocket applications.

3. **Profile selection.** CLI command to select active profile. Selection stored as index — persists across power cycle via flash. Default: model rocket.

4. **CLI preview.** `'m'` key shows active profile: name, phase count, guard config per phase, pre-arm checks, abort behavior. Allows verification before arming.

**[GATE]:**
- Three profiles compile and are selectable via CLI
- CLI shows active profile name and key parameters per phase
- Profile selection persists across power cycle
- Bench flight sim (IVP-73 script) passes with rocket profile
- Freeform profile: single phase, no phase detection, logging only
- Profile struct has no pointers (serialization-ready)

**[DIAG]:** Load fails after flash write = struct version mismatch; bump version and re-init. Wrong thresholds = profile index incorrect. HAB profile triggers rocket guards = profile not correctly switching guard table.

---

### IVP-75: Active Object Migration Planning

**Prerequisites:** IVP-74 (Flight Director complete and working)

**Rationale:** The council confirmed QF+QV as the target architecture (Stage 9). This IVP validates that the upgrade path is viable and documents the migration plan, without migrating anything. State handler code written in IVP-68 through IVP-74 is identical to what runs inside Active Objects — zero throwaway code.

**Implement:**

1. **QF+QV compile gate.** Extend `lib/qep/` to include QF and QV source files from QP/C. Write BSP (Board Support Package): `QF_onStartup()` (configure tick interrupt), `QV_onIdle()` (WFI for power savings), critical section macros using Pico SDK primitives. Compile and link. Does NOT replace the superloop yet.

2. **Active Object boundary design.** Document which modules become Active Objects and their event interfaces:
   - `AO_FlightDirector` — wraps existing QEP state machine, receives SIG_TICK and command events
   - `AO_LedEngine` — owns NeoPixel exclusively, receives EVT_LED_PATTERN events
   - `AO_Logger` — owns flash writes, receives EVT_LOG events
   - `AO_Telemetry` — owns radio TX, receives EVT_TELEM events
   - `AO_ErrorHandler` — monitors health, publishes EVT_ERROR/EVT_WARNING

3. **Event catalog.** Define typed event structs for inter-AO communication. Published events (pub-sub) vs direct events. Event pool sizing (static arrays, no heap).

4. **Dual-core constraint documentation.** Confirm: QF+QV stays entirely on Core 0. Core 1 sensor loop unchanged. Seqlock boundary unchanged. No QP code crosses the core boundary.

5. **Migration plan document.** `docs/flight_director/ACTIVE_OBJECT_MIGRATION.md` — step-by-step plan for Stage 9 with dependencies, risks, and rollback strategy.

**[GATE]:**
- QF+QV compiles and links against Pico SDK (does not need to run — compile gate only)
- Trivial 2-Active-Object demo links (can run on hardware if time permits)
- Migration plan document written and reviewed
- Event catalog documented with types and pub-sub topology
- No regressions — existing Flight Director still works from superloop

**[DIAG]:** QF won't compile = port macros missing or C/C++ linkage. Link fails = QV idle callback not implemented. Demo crashes = BSP tick configuration wrong.

---

## Stage 9: Active Object Architecture

**Purpose:** Migrate subsystems from the superloop to QF+QV Active Objects. Each module becomes an independent actor with a private event queue, communicating exclusively via typed events. Eliminates scattered state mutations (the "why is the LED solid green" problem) and provides formal inter-module interfaces. See `docs/flight_director/QP_APPLICATION_GUIDE.md` Section 4 for migration details.

**Prerequisites:** IVP-75 (QF+QV compiles, migration plan documented). Stage 8 Flight Director fully working from superloop.

**Constraints:**
- **QF+QV stays entirely on Core 0.** Core 1 remains the deterministic sensor sampling loop. Seqlock boundary unchanged. No QP code crosses the core boundary.
- **State handler code is identical.** QEP handlers from Stage 8 move inside Active Objects unchanged. The change is how events arrive (QF queue vs manual dispatch), not what the handlers do.
- **No new features.** This is a pure architecture refactor. Feature behavior before and after must be identical — verified by re-running IVP-73 bench flight sim.

**Deferred evaluation:** QS (QP/Spy) binary tracing is evaluated during this stage as a potential replacement or supplement for current flight event logging. QS logs every state entry, exit, transition, and event dispatch to a ring buffer with ~10x less overhead than printf-style logging. Adoption decision made during or after IVP-76 based on integration effort and tracing utility. See council Decision 8.

| Step | Title | Brief Description |
|------|-------|------------------|
| IVP-76 | QF+QV BSP Integration | Board support package, tick interrupt, QV idle, stack-allocated events, QS evaluation |
| IVP-77 | LED Engine Active Object | AO_LedEngine owns NeoPixel exclusively, consolidates 19-case overlay switch |
| IVP-78 | Flight Director Active Object | Wrap Stage 8 QEP HSM in QF Active Object, replaces diagnostic printf with events |
| IVP-79 | Logger Active Object | AO_Logger owns flash writes, flash_safe_execute coordination |
| IVP-80 | Telemetry Active Object | AO_Telemetry owns radio TX scheduling, CCSDS/MAVLink encoding |
| IVP-81 | Superloop → QV Transition | Remove superloop `_due` flags, QV cooperative scheduler replaces `while(true)` |
| IVP-82 | SPIN Formal Verification | Promela model of AO event topology, deadlock/liveness verification |

---

### IVP-76: QF+QV BSP Integration

**Prerequisites:** IVP-75 (QF+QV compiles)

**Implement:** Board Support Package for QF+QV on RP2350 bare-metal Pico SDK. This makes QF+QV operational on hardware without migrating any existing modules.

1. **BSP implementation (`bsp.cpp`):**
   - `QF_onStartup()` — configure SysTick or alarm-based tick interrupt for QF time events
   - `QV_onIdle()` — call `__wfi()` for power savings when all event queues empty (replaces manual WFI in current superloop)
   - `Q_onAssert()` — route to existing `ROCKETCHIP_ASSERT` / panic handler
   - Critical section macros using `save_and_disable_interrupts()` / `restore_interrupts()`

2. **Trivial 2-Active-Object demo.** `AO_Blinker` (toggles LED on timer event) + `AO_Counter` (counts events, prints to serial). Validates: event delivery, time events, priority scheduling, QV idle. Runs alongside existing superloop (QV does not replace it yet).

3. **Stack-allocated events (`QF_MAX_EPOOL = 0`).** All events are local variables on the caller's stack. QV cooperative scheduling guarantees run-to-completion — the event is fully processed before the caller's stack frame exits. No event pools, no reference counting, no pool exhaustion risk. If deferred events are needed later, upgrade to pool allocation at that point.

4. **QS (QP/Spy) evaluation gate.** Evaluate QS binary tracing for flight event logging. Build with `Q_SPY` defined, measure code size and SRAM overhead, assess tracing utility vs current `[FD]` printf logging. Decision: adopt, defer, or reject. See council Decision 8 and `docs/flight_director/TOOLCHAIN_EVALUATION.md`.

**QS Decision (2026-03-27): DEFERRED.** QS source (`qs.c`, `qs_rx.c`) is not vendored — only the dummy header (`qs_dummy.h`). QS requires a dedicated output channel (UART or memory buffer) and we have no spare UART (GPS uses the one available). Overhead assessment requires vendoring additional QP/C files. **Not blocking:** IVP-82 (SPIN formal verification) covers the AO interaction verification that QS tracing would have assisted with, using exhaustive model checking rather than runtime tracing. If AO debugging becomes difficult during IVP-77–81, vendor QS source and route output to PSRAM ring buffer for post-hoc dump.

**[GATE]:**
- QF initializes without crash on RP2350 hardware
- 2-AO demo: LED blinks at correct rate, counter prints to serial
- QV idle callback executes between events (verify via GPIO toggle or timing measurement)
- No interference with existing superloop modules (sensor sampling, ESKF, CLI all still work)
- No heap allocation (verify with `mallinfo()` or BSS size check)
- QS evaluation documented (adopt/defer/reject with rationale)

**[VERIFIED 2026-03-27]:** All gates pass. QF_init clean, AO_Blinker 1Hz LED, AO_Counter avg=100,000µs (10Hz exact), ESKF running (20K predicts), 128K IMU reads / 4 errors (boot-only), CLI responsive, logging active. BSS delta ~200B (queue buffers + AO structs). Council plan: `harmonic-watching-wall.md`.

**[DIAG]:** Crash at QF_init = BSP tick not configured. Events not delivered = priority inversion or stack-allocated event used after scope exit. Interferes with superloop = tick interrupt conflicting with existing timer.

---

### IVP-77: LED Engine Active Object

**Prerequisites:** IVP-76 (QF+QV operational on hardware)

**Rationale:** The LED Engine is the most obvious Active Object beneficiary. Currently, NeoPixel state is set from multiple call sites across multiple files (`neo_set_if_changed()` scattered throughout) via the `g_calNeoPixelOverride` atomic overlay pattern. The `neo_apply_override()` function (IVP-72) has grown to a 19-case switch covering calibration, RX, and flight phase overlays — this consolidates into a proper pattern state machine. As an Active Object, only `AO_LedEngine` touches the NeoPixel — all other modules send typed pattern-change events.

**Implement:**

1. **AO_LedEngine state machine.** QEP HSM managing NeoPixel patterns:
   - States: IDLE_PATTERN, ARMED_PATTERN, FLIGHT_PATTERN, LANDED_PATTERN, ERROR_PATTERN, CUSTOM
   - Each state handles its own animation (solid, blink, breathe, chase) via SIG_TICK
   - Transitions on EVT_LED_PATTERN events from other Active Objects

2. **Event interface:**
   ```cpp
   struct LedPatternEvt : QEvt {
       LedPattern pattern;  // enum: SOLID_AMBER, RED_FLASH, SLOW_GREEN_BLINK, etc.
   };
   ```
   Other modules publish `EVT_LED_PATTERN` — they never call NeoPixel functions directly.

3. **Migration.** Replace all direct `neo_set_if_changed()` / `g_statusLed->fill()` / `g_statusLed->show()` calls with `QF_PUBLISH(&led_evt)`. The NeoPixel hardware pointer moves inside `AO_LedEngine` — no other module holds a reference.

**[GATE]:**
- NeoPixel behavior identical to pre-migration (ARMED=amber, BOOST=red, LANDED=green blink, etc.)
- No direct NeoPixel calls outside AO_LedEngine (grep verification)
- LED state traceable: CLI can query AO_LedEngine's current pattern state
- Animation timing correct (blink rates, transition smoothness)
- Re-run IVP-73 bench flight sim — LED patterns match expected sequence

**[DIAG]:** LED stuck = event not published or AO_LedEngine priority too low. Wrong pattern = event payload incorrect. Animation jittery = tick rate mismatch or QV scheduling delay.

---

### IVP-78: Flight Director Active Object

**Prerequisites:** IVP-77 (LED Engine migrated, proving AO pattern works)

**Implement:** Wrap the existing QEP Flight Director state machine (from Stage 8) in a QF Active Object. State handler code is unchanged — the migration is purely structural.

**Audit deferral (2026-03-26):** The standards audit found 10 diagnostic `printf` calls in `flight_director.cpp` and `go_nogo_checks.cpp` (Tier 3 findings). These are `[FD]` transition logs currently parsed by `bench_flight_sim.py`. When the FD becomes an AO, replace these with published `EVT_FLIGHT_STATE` events — the bench sim subscribes to events instead of parsing serial output. This eliminates naked `printf` from the Flight Director entirely.

1. **AO_FlightDirector.** Receives events from QF queue instead of manual `QHSM_DISPATCH()` in `run_flight_director()`. SIG_TICK delivered via QF time event (periodic timer). Command events (SIG_ARMED, SIG_ABORT) published by CLI/radio handlers.

2. **Remove `run_flight_director()` from superloop.** The superloop no longer calls the Flight Director directly — QV dispatches it based on priority and queue status.

3. **Inter-AO communication.** Flight Director publishes:
   - `EVT_LED_PATTERN` → consumed by AO_LedEngine
   - `EVT_FLIGHT_STATE` → consumed by AO_Logger, AO_Telemetry
   - `EVT_PYRO_CMD` → future: consumed by pyro safety layer

**[GATE]:**
- Full bench flight sim (IVP-73) passes with AO_FlightDirector — identical behavior to superloop version
- State transitions logged correctly
- LED patterns change via published events (not direct calls)
- No regression in dispatch timing (<5µs per event)
- Superloop `director_due` flag removed

**[DIAG]:** Events not arriving = queue too small or subscription missing. Timing different = QV priority incorrect. Regression = state handler accidentally modified during migration.

---

### IVP-79: Logger Active Object

**Prerequisites:** IVP-78 (Flight Director as AO, proving event-driven pattern)

**Implement:** Migrate the flight data logger to an Active Object.

1. **AO_Logger.** Owns flash write scheduling. Receives EVT_LOG events (flight markers, sensor snapshots). Manages write buffer, page flushes, `flash_safe_execute()` coordination. No other module calls flash write functions directly.

2. **Flash safety.** `flash_safe_execute()` + `i2c_bus_reset()` (LL Entry 31) must be called from AO_Logger's context. Verify multicore_lockout still works correctly when called from an AO event handler vs superloop tick.

**[GATE]:**
- Flight data logged correctly through AO_Logger (compare log output to pre-migration)
- No direct flash write calls outside AO_Logger (grep verification)
- Flash write coordination: `flash_safe_execute()` + `i2c_bus_reset()` still called correctly
- Re-run IVP-73 bench flight sim — logging pipeline works through AO

**[DIAG]:** Log data missing = EVT_LOG not published or AO_Logger priority too low. Flash corruption = `flash_safe_execute()` not called from correct context.

---

### IVP-80: Telemetry Active Object

**Prerequisites:** IVP-79 (Logger as AO)

**Implement:** Migrate the telemetry encoder and radio TX scheduling to an Active Object.

1. **AO_Telemetry.** Owns radio TX scheduling. Receives EVT_TELEM events (state updates, sensor data). Manages packet encoding (CCSDS or MAVLink per Mission Profile), transmission timing, and downlink priority. No other module calls radio TX functions directly.

2. **AO_ErrorHandler deferred.** Health monitoring (watchdog feed, error counting) remains in the superloop for now. Defer to whiteboard — scope is too broad for this IVP and the watchdog kick must remain deterministic (not event-driven).

**[GATE]:**
- Telemetry packets transmitted correctly through AO_Telemetry
- No direct radio TX calls outside AO_Telemetry (grep verification)
- TX timing matches pre-migration (2Hz CCSDS, no gaps)
- Re-run IVP-73 bench flight sim — full pipeline works through Active Objects

**[DIAG]:** Telemetry gaps = AO_Telemetry queue overflow or priority too low. Encoding errors = event payload mismatch.

---

### IVP-81: Superloop → QV Transition

**Prerequisites:** IVP-80 (all major modules are Active Objects)

**Implement:** Remove the remaining superloop infrastructure. QV's cooperative scheduler fully replaces `while(true)`.

1. **Replace superloop with QF_run().** The `while(true)` loop in `main()` becomes:
   ```cpp
   return QF_run();  // never returns — QV cooperative scheduler
   ```
   QV checks each Active Object's queue in priority order, dispatches one event at a time (run-to-completion), and calls `QV_onIdle()` (WFI) when all queues are empty.

2. **Remove `_due` flags and tick dividers.** Replace with QF time events:
   - `fusion_due` → QF periodic timer on AO_FlightDirector (or fusion module)
   - `logger_due` → QF periodic timer on AO_Logger
   - `telem_due` → QF periodic timer on AO_Telemetry
   - `ui_due` → QF periodic timer on CLI handler (if migrated) or kept as polled

3. **Core 1 boundary unchanged.** Core 1's `core1_sensor_loop()` is still a pure bare-metal loop. The seqlock read in the condition evaluator (AO_FlightDirector) is unchanged — it's a read-only Core 0 operation.

4. **Regression verification.** Full system bench test: sensor sampling rates, ESKF timing, LED behavior, logging, telemetry — all must match pre-migration performance.

**[GATE]:**
- `main()` calls `QF_run()` instead of `while(true)` loop
- No `_due` flags remain in `main.cpp`
- All Active Objects process events at correct rates (verified via timing instrumentation)
- Core 1 sensor sampling rate unchanged (999-1000 Hz)
- ESKF predict timing unchanged (~111µs codegen, ~486µs hybrid Bierman)
- Power consumption: QV_onIdle() WFI equivalent to or better than previous manual WFI
- Full IVP-73 bench flight sim passes
- 10-minute soak: zero sensor errors, zero unexpected state transitions

**[DIAG]:** Timing regression = priority ordering wrong or tick rate mismatch. Core 1 affected = QF code accidentally crossed core boundary. WFI not working = QV_onIdle() not calling `__wfi()`. Event starvation = stack-allocated event scope too narrow.

---

### IVP-82: SPIN Formal Verification

**Prerequisites:** IVP-81 (superloop removed, all AOs running under QV)

**Rationale:** With 4-5 Active Objects communicating via typed events, the interaction state space is large enough that unit tests alone cannot guarantee absence of deadlock or event loss. SPIN exhaustively verifies properties across all possible event interleavings. This was identified in the IVP-67 toolchain evaluation as the right time to adopt SPIN. See `docs/flight_director/TOOLCHAIN_EVALUATION.md` SPIN section.

**Implement:**

1. **Promela model of AO event topology.** Model each Active Object as a Promela process with a bounded channel (event queue). Model the event catalog: SIG_SENSOR_DATA, SIG_PHASE_CHANGE, EVT_LED_PATTERN, EVT_LOG, EVT_TELEM, SIG_CLI_COMMAND. Model pub-sub routing.

2. **Safety properties (LTL assertions):**
   - No deadlock: all AO processes always eventually consume events
   - No unbounded queue growth: channel lengths stay within configured bounds
   - Pyro safety: `[](phase == IDLE -> !pyro_fired)` — pyro never fires in IDLE
   - Liveness: `<>(phase == LANDED)` — LANDED is always eventually reachable from any flight phase
   - No simultaneous drogue + main fire in the same tick

3. **Verification output.** Run SPIN exhaustive verification, document state space size, counterexamples (if any), and any model simplifications. Add Promela source to `tools/spin/` or `verification/`.

**[GATE]:**
- Promela model compiles and runs in SPIN without syntax errors
- Exhaustive verification completes (state space bounded)
- All safety properties pass (or counterexamples are analyzed and model/code fixed)
- Model documented: assumptions, simplifications, what is and isn't modeled

**[DIAG]:** State space explosion = model too detailed (abstract sensor data, focus on event routing). Deadlock found = event subscription missing or circular dependency. Property violation = real bug or model inaccuracy.

---

## Stage 10: Adaptive Estimation & Safety

**Purpose:** Phase-aware ESKF tuning and confidence-gated safety. Integrates with the state machine (IVP-68) for flight phase detection and with the Flight Director (Stage 8) for safety-gated actions.

**Background:** Extended research (2026-02-24) demonstrated that MMAE/IMM is the wrong tool for RocketChip's flight regime switching. Real aerospace navigation (X-43A, SpaceX Grasshopper, ArduPilot EKF3, PX4 ECL) universally uses single kinematic filters with deterministic regime adaptation — not multi-model banks. Simple phase-scheduled Q/R captures the same benefit at near-zero computational cost. See `docs/decisions/ESKF/ESKF_RESEARCH_SUMMARY.md` for full rationale and benchmark data.

| Step | Title | Brief Description |
|------|-------|------------------|
| IVP-83 | Phase-Scheduled Q/R + Innovation Adaptation | Per-phase noise models tied to state machine, innovation ratio fine-tuning, optional Bierman measurement updates |
| IVP-84 | Confidence Gate | ESKF health + AHRS cross-check → binary confidence flag for Flight Director |
| IVP-85 | Confidence-Gated Actions | Irreversible actions (pyro) held when confidence flag low |
| IVP-86 | Vehicle Parameter Profiles | Mission-specific Q/R presets per vehicle type (rocket, HAB, drone) |

---

### IVP-83: Phase-Scheduled Q/R + Innovation Adaptation

**Prerequisites:** IVP-68 (state machine provides flight phase detection), IVP-48 (ESKF tuned)

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

### IVP-84: Confidence Gate

**Prerequisites:** IVP-83, IVP-45 (Mahony AHRS)

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

### IVP-85: Confidence-Gated Actions

**Prerequisites:** IVP-84 (confidence gate), IVP-72 (action executor)

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

### IVP-86: Vehicle Parameter Profiles

**Prerequisites:** IVP-83 (phase-scheduled Q/R framework)

**Implement:** Mission-specific Q/R presets per vehicle type. The Q/R scheduling (IVP-83) selects values per flight phase — this step provides the actual phase definitions and noise parameters per vehicle type.

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

## Stage 11: Ground Station

**Purpose:** Dedicated ground station platform on Raspberry Pi or PC. Everything beyond the RX RocketChip board. Independent of vehicle firmware once Stage 7 packet format is stable. Can be developed in parallel with Stages 8–10.

| Step | Title | Brief Description |
|------|-------|------------------|
| IVP-87 | Yamcs Instance & XTCE | *(placeholder)* XTCE dictionary for CCSDS packets, Yamcs on RPi |
| IVP-88 | OpenMCT Integration | *(placeholder)* OpenMCT via openmct-yamcs plugin, custom layouts |
| IVP-89 | Ground Station Pi Image | *(placeholder)* Pre-built RPi OS image, turnkey operation |
| IVP-90 | Post-Flight Analysis Tools | *(placeholder)* PCM to CCSDS replay, flight timeline, multi-flight overlay |
| IVP-91 | Web Dashboard | *(placeholder)* Browser-based telemetry for field use |
| IVP-92 | Mobile / Remote Access | *(placeholder)* WiFi AP on ground station Pi for phone/tablet |

---

## Stage 12: System Integration

**Purpose:** Full system verification and flight readiness.

| Step | Title | Brief Description |
|------|-------|------------------|
| IVP-93 | Full System Bench Test | All subsystems running `⚠️ VALIDATE 30 minutes`, no crashes |
| IVP-94 | Simulated Flight Profile | Replay recorded accel/baro through state machine |
| IVP-95 | Power Budget Validation | Battery runtime validation under flight load |
| IVP-96 | Environmental Stress | Temperature range, vibration (if available) |
| IVP-97 | Flight Test | Bungee-launched glider: full data capture + telemetry |

> **Milestone:** IVP-97 — **Flight Ready**.

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
| Data logging change | IVP-49 — IVP-54 | Data model, frames, buffer, flash |
| Radio/telemetry change | IVP-57 — IVP-62 | Encoder, service, translation |
| Adaptive estimation change | IVP-83 — IVP-86 | Q/R scheduling, confidence gate |
| Watchdog policy change | IVP-66, IVP-30 | Recovery behavior + mechanism |
| Flight Director change | IVP-67 — IVP-75 | State machine, guards, actions, mission config |
| Active Object change | IVP-76 — IVP-82, IVP-73 | AO migration, bench flight sim regression |
| **Major refactor** | **All Stage 1-3** | Full regression |
| **Before any release** | IVP-01, IVP-27, IVP-28, IVP-30, IVP-66, IVP-73, IVP-93 | Release qualification (build, USB, flash, watchdog, recovery, bench flight sim, full bench test) |

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
