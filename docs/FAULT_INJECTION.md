# Fault Injection Guide

**Status:** Active — Stage 16B (IVP-129)
**Hardware:** Raspberry Pi Debug Probe (CMSIS-DAP) + OpenOCD 0.12.0+dev
**Target:** RP2350 Cortex-M33, QP/C QV cooperative scheduler

---

## Overview

Fault injection validates that safety paths (PIO backup timers, watchdog, health monitor, ESKF divergence recovery) actually fire under failure conditions. All injection is done via the debug probe — no CLI/radio attack surface.

**Single flight binary per role (R-25-exec, 2026-05-13, Approach A):** the project ships one flight binary per role (`cmake -B build_flight ..` or `cmake -B build_station_flight -DROCKETCHIP_JOB_STATION=1 ..`). Both contain the `fault_force_*` hooks in `src/safety/fault_inject.cpp` + `src/safety/station_fault_inject.cpp`. Every hook checks `rc::test_mode_active()` at entry and no-ops on production boots. To use this guide's GDB scripts, **first arm test mode via probe** (write `rc::kTestModeMagic` to `rc::g_test_mode_arm_magic`, reset; `scripts/_rc_test_common.py::arm_test_mode_via_probe()` is the canonical helper). The three-condition AND gate (SRAM-magic AND state==IDLE AND boot-time-window) prevents accidental in-flight activation. See `docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md` for the design + council provenance.

---

## Probe Setup

```bash
# Start OpenOCD (kill stale instances first)
taskkill //F //IM openocd.exe 2>/dev/null; sleep 2
/c/Users/pow-w/.pico-sdk/openocd/0.12.0+dev/openocd \
  -s /c/Users/pow-w/.pico-sdk/openocd/0.12.0+dev/scripts \
  -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "adapter speed 5000" &
```

See `docs/agents/DEBUG_PROBE_NOTES.md` for full setup, known issues, and GDB version requirements (use Pico SDK GDB, not Chocolatey).

---

## GDB Injection Methods

### Method 1: `call` — Invoke a C/C++ Function

```
(gdb) monitor halt
(gdb) call fault_force_eskf_unhealthy()
(gdb) continue
```

**How it works:** GDB pushes a fake stack frame, sets PC to the function entry, resumes until the function returns (`bx lr`), then re-halts. Side effects (global writes, flag changes) persist after `continue`.

**Safe for:** Any function that returns in bounded time — flag setters, register writes, state changes.

**NOT safe for:** Functions that block or loop forever (e.g., `while(1){}`). GDB hangs waiting for the return. Use Method 2 for those.

### Method 2: `set` — Direct Memory / Variable Write

```
(gdb) monitor halt
(gdb) set variable g_eskfInitialized = false
(gdb) continue
```

**How it works:** GDB writes directly to the target's memory via SWD. No function call, no stack frame, no PC change. Immediate.

**Safe for:** Everything, including simulating hangs (set a flag that a loop checks) without actually hanging GDB.

**Use for Core 0 hang simulation:** Set a "stall requested" flag that the idle loop checks, rather than calling a function that never returns.

### Method 3: Register Write — Hardware Manipulation

```
(gdb) monitor halt
(gdb) set *0x50400000 = (*0x50400000) & ~(1 << 2)
(gdb) continue
```

**How it works:** Direct write to a memory-mapped hardware register. Bypasses all software abstractions.

**Use for:** PIO state machine halt, peripheral disable, GPIO forcing.

---

## Recovery After Injection

Every injection test must leave the device in a recoverable state. The standard recovery sequence:

```
(gdb) monitor reset halt
(gdb) load
(gdb) monitor resume
```

This reflashes and reboots cleanly. **Use `monitor resume`, not `monitor reset run`** — the latter doesn't reliably resume both cores on RP2350 (see DEBUG_PROBE_NOTES.md).

If the device is in a watchdog reset loop, `monitor halt` may need to be issued quickly after reset. OpenOCD's `reset halt` handles this.

---

## Existing Injectable Entry Points

These functions exist in the firmware and can be called via GDB `call`:

| Function | File | What It Does |
|----------|------|-------------|
| `ESKF::inject_error_state(dx)` | `eskf.h:549` | Apply error-state vector to ESKF (corrupts position, velocity, biases) |
| `health_monitor_set_phase(phase)` | `health_monitor.h:115` | Override flight phase for latch behavior |
| `health_monitor_clear_latches()` | `health_monitor.h:119` | Manual fault latch clear |
| `pio_backup_timer_arm(d, m)` | `pio_backup_timer.h:38` | Start PIO countdown (drogue_s, main_s) |
| `pio_backup_timer_cancel(id)` | `pio_backup_timer.h:42` | Stop one PIO SM |
| `pio_backup_timer_disarm()` | `pio_backup_timer.h:46` | Stop both PIO SMs |
| `watchdog_recovery_clear(r)` | `watchdog_recovery.h:104` | Zero reboot counter |

### Test-Mode-Gated Hooks (src/safety/fault_inject.{cpp,h})

Compiled into the single flight binary post-R-25-exec (2026-05-13). Every entry checks `rc::test_mode_active()` and no-ops on production boots; arm test mode via probe before invoking. See `docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md`.

| Function | File | What It Does |
|----------|------|-------------|
| `fault_force_eskf_unhealthy()` | `safety/fault_inject.h` | Set `g_eskfInitialized = false` → triggers CR-1 recovery |
| `fault_force_core0_stall()` | `safety/fault_inject.h` | Set stall flag → idle bridge spins, skips all work |
| `fault_force_core0_stall_clear()` | `safety/fault_inject.h` | Clear stall flag → idle bridge resumes (NOT gated — recovery action) |
| `fault_force_watchdog_stall(n)` | `safety/fault_inject.h` | Skip watchdog kick for N idle iterations |
| `fault_force_health_fail(which)` | `safety/fault_inject.h` | Placeholder — use GDB `set` on `g_health.primary` byte directly (see note below) |

**Health byte injection note:** `g_health` is `static` in `health_monitor.cpp`. GDB can still resolve file-static symbols via DWARF debug info if the binary is built with `-g`. Verify during IVP-130: `(gdb) print g_health.primary`. If GDB resolves the symbol, use `set g_health.primary = 0x55` (all subsystems FAULT). If not, either de-static the variable or add a `health_monitor_get_primary_ptr()` accessor. Try the symbol first — it should work with debug builds.
| `fault_force_ao_queue_flood(ao, n)` | `safety/fault_inject.h` | Publish N dummy SIG_SENSOR_DATA events (floods all subscribers) |
| `fault_force_pio_sm_halt()` | `safety/fault_inject.h` | Clear PIO2 CTRL SM enable bits → backup timers stop |

### GDB Scripts (scripts/fault_injection/)

| Script | Scenario |
|--------|----------|
| `inject_eskf_divergence.gdb` | Force ESKF unhealthy → verify CR-1 recovery |
| `inject_core0_stall.gdb` | Stall idle bridge → verify watchdog reboot |
| `inject_watchdog_stall.gdb` | Skip watchdog kick → verify PIO watchdog fires |
| `inject_queue_flood.gdb` | Flood AO queues → verify QP assertion or graceful handling |
| `inject_pio_sm_halt.gdb` | Disable PIO2 SMs → verify upper-layer detection (or document gap) |

**Stage O / OPT‑IVP‑01 (outside `fault_injection/` — not destructive):**

| Script | Scenario |
|--------|----------|
| `scripts/opt_ivp01_row10_dualcore_watch.gdb` | Passive dual-core sanity + **`SCB` fault registers** (`reset` → `load` → long free‑run → single halt). Intended to **disambiguate** periodic Tier‑1 `soak_gdb.gdb` halts vs real MPU/stack faults; see **`docs/MULTICORE_RULES.md` §MPU**. |

---

## PIO Backup Timer — Hardware Details

**PIO block:** PIO2 (`0x50400000` base on RP2350)
**State machines:** Two dynamically claimed SMs (`g_drogue_sm`, `g_main_sm`)
**Clock:** 1 MHz (divider 150 from 150 MHz system clock)
**Countdown:** `timeout_s * 1_000_000` cycles loaded via `pio_sm_put_blocking()`
**Output:** GPIO 12 (drogue), GPIO 13 (main) — active HIGH on timeout

To halt a PIO SM from GDB without calling C code:
```
set *0x50400000 = (*0x50400000) & ~(1 << SM_NUMBER)
```

To read which SMs are enabled:
```
print/x *0x50400000
```
Bits [3:0] are SM3..SM0 enable flags.

---

## UART1 as Alternative Channel

UART1 is available on the RP2350 Feather (GPIO 4/5 unallocated, UART0 used by GPS). For tests where the probe is needed for state inspection simultaneously, UART1 could carry fault-injection commands from a separate host script. Not currently implemented — noted for future use.

---

## Test Procedure Template

Each fault injection test follows this pattern:

1. **Flash bench binary** via probe (`load` + `monitor resume`)
2. **Wait for steady state** — ESKF initialized, sensors reading, health OK
3. **Capture pre-state** — `info threads`, `print g_eskfInitialized`, relevant globals
4. **Inject fault** — `call`, `set`, or register write per the scenario
5. **Observe** — `continue`, wait for expected behavior (timeout, state change, pyro fire)
6. **Capture post-state** — halt, inspect globals, check pyro edge log, read health byte
7. **Verify fault response** — compare actual behavior to expected (per IVP-130 test matrix)
8. **Verify recovery** — confirm system returned to clean operational state:
   - ESKF divergence: `g_eskfInitialized` returns to true (CR-1 reinit)
   - Core 0 stall: after clearing flag, sensor reads resume at ~1kHz within 1s
   - Watchdog stall: after reboot, recovery counter incremented (`g_recovery.eskf_fail_count`)
   - Queue flood: system either asserts (expected) or recovers (document which)
   - PIO SM halt: document gap if no upper-layer monitor detects halted timer
9. **Recover** — `monitor reset halt` + `load` + `monitor resume`

---

## Queue Flood: SIG_SENSOR_DATA Subscribers

`fault_force_ao_queue_flood()` publishes `SIG_SENSOR_DATA` which fans to all subscribers. Document subscriber list and queue depths here during IVP-130 HW testing to predict the failure sequence. Expected: smallest-queue AO overflows first.

| AO | Queue Depth | Subscribes to SIG_SENSOR_DATA? | Expected Behavior |
|----|-------------|-------------------------------|-------------------|
| AO_FlightDirector | 32 | TBD (verify during IVP-130) | |
| AO_Telemetry | 8 | TBD | Likely overflows first (smallest queue) |
| AO_Logger | 32 | TBD | |
| *(fill during IVP-130 HW run)* | | |

---

## Probe-Only Combined-Session Runbook (T2b pattern)

**When to use:** Manual audit-suite regression for `fault_force_*` entries when the Python helper scripts (`scripts/enhanced_fault_injection.py`, `scripts/warm_reboot_audit.py`) trip on the recurring Windows USB CDC re-enum race documented below. Probe-only avoids the race entirely.

**Why this exists:** Python helpers open the vehicle/station serial port (`COM7` / `COM9`) within ~2–7s of a probe-mediated chip reset. On Windows the USB CDC enumeration is asynchronous from `monitor reset run`'s completion, and the port handle is intermittently held by `usbser.sys` for longer than the script's retry budget. Symptoms: `PermissionError(13)`. Switching ports, restarting the script, or waiting more than ~10s usually clears it; sometimes a full OpenOCD restart is required. The race is environmental, not a firmware defect — labeled-soft pivot to this runbook per `standards/HW_GATE_DISCIPLINE.md` Rule 4 is the documented response.

**Procedure** (one GDB session, no Python serial port needed for triggering — separate `python scripts/_rc_test_common.py` or `serial_reader` for post-hoc log capture):

```
/c/Users/pow-w/.pico-sdk/toolchain/14_2_Rel1/bin/arm-none-eabi-gdb.exe build_flight/rocketchip.elf -batch \
  -ex "target extended-remote localhost:3333" \
  -ex "set confirm off" \
  -ex "monitor halt" \
  -ex "set var rc::g_test_mode_arm_magic = 0x7E57BABE" \
  -ex "monitor reset run" \
  -ex "shell sleep 6" \
  -ex "monitor halt" \
  -ex "print rc::g_test_mode_enabled" \
  -ex "print (int)rc::flight_phase_observable_get()" \
  -ex "call (void)fault_force_eskf_unhealthy()" \
  -ex "print g_eskfInitialized" \
  -ex "monitor resume" \
  -ex "detach"
```

Expected output:
- `rc::g_test_mode_enabled = true` (3-condition AND gate satisfied: magic-at-boot + phase==kIdle + within 30s boot window)
- `flight_phase_observable_get() = 0` (kIdle)
- `g_eskfInitialized = true` before call, **`false`** after (state mutation = positive-control signal)

**One-scenario-per-session discipline:** Stringing multiple `fault_force_*` calls into one session is unsafe — the first fault may cascade into MemManage / HardFault before the second call runs. Reset the chip between scenarios:
```
monitor reset halt
monitor resume
detach
```

**Timing budget:** test_mode arms for 30s post-boot (`kTestModeArmWindowMs`). The `shell sleep 6` + halt + `call` sequence completes well inside the window. Adding multiple inter-call sleeps risks timing out; one scenario per arm cycle.

**T2b coverage decomposition** (R-25-exec audit invariant):
- **Negative path** (test mode OFF → gate refuses, `[FAULT] ... gated` emitted): T2a verified directly on `fault_force_eskf_unhealthy` (2026-05-14).
- **Positive path** (test mode ON → gate accepts, state mutation observed): T2b verified on `fault_force_eskf_unhealthy` via the procedure above (2026-05-15).
- **Remaining entries**: 8/8 vehicle gated entries + 3/3 station gated entries all call `fi_test_mode_gate()` / `fis_test_mode_gate()` at line 1 of body. Grep-coverage at `src/safety/fault_inject.cpp` + `src/safety/station_fault_inject.cpp` is the mechanical audit invariant; one positive-path test exercises the shared gate helper that all entries route through.
- **Exempt entries**: `fault_force_core0_stall_clear()` + `fault_force_station_gps_restore()` are recovery actions, intentionally not gated per documented design at the call sites.

---

## References

- `docs/agents/DEBUG_PROBE_NOTES.md` — OpenOCD startup, GDB commands, known issues
- `docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md` — current bench-tier model (single binary, test-mode gate, R-25-exec)
- `safety/test_mode.h` — three-condition AND gate, probe-arming flow
- `standards/HW_GATE_DISCIPLINE.md` Rule 4 — labeled-soft gate pivot when script automation fails
- RP2350 datasheet Section 3.7 — PIO register map
- LESSONS_LEARNED.md Entry 25 — Use probe for flashing (picotool corrupts I2C)
- LESSONS_LEARNED.md Entry 33 — PIO GPIO init causes I2C interference
