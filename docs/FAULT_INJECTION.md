# Fault Injection Guide

**Status:** Active — Stage 16B (IVP-129)
**Hardware:** Raspberry Pi Debug Probe (CMSIS-DAP) + OpenOCD 0.12.0+dev
**Target:** RP2350 Cortex-M33, QP/C QV cooperative scheduler

---

## Overview

Fault injection validates that safety paths (PIO backup timers, watchdog, health monitor, ESKF divergence recovery) actually fire under failure conditions. All injection is done via the debug probe — no fault-injection code in the flight binary.

Bench binary (`cmake ..`, default) includes dev hooks in `src/dev/`. Flight binary (`cmake -DBUILD_FOR_FLIGHT=ON`) excludes them. The GDB scripts in this guide work with the **bench binary only**.

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

See `.claude/DEBUG_PROBE_NOTES.md` for full setup, known issues, and GDB version requirements (use Pico SDK GDB, not Chocolatey).

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
(gdb) set *0x50300000 = (*0x50300000) & ~(1 << 2)
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

### Bench-Only Hooks (src/dev/)

These are compiled into the bench binary only. See `docs/audits/DEV_CODE_AUDIT.md`.

| Function | File | What It Does |
|----------|------|-------------|
| `fault_force_eskf_unhealthy()` | `fault_inject.h` | Set `g_eskfInitialized = false` → triggers CR-1 recovery |
| `fault_force_core0_stall()` | `fault_inject.h` | Set stall flag → idle bridge spins, skips all work |
| `fault_force_core0_stall_clear()` | `fault_inject.h` | Clear stall flag → idle bridge resumes |
| `fault_force_watchdog_stall(n)` | `fault_inject.h` | Skip watchdog kick for N idle iterations |
| `fault_force_health_fail(which)` | `fault_inject.h` | Placeholder — use GDB `set` on `g_health.primary` byte directly (see note below) |

**Health byte injection note:** `g_health` is `static` in `health_monitor.cpp`. GDB can still resolve file-static symbols via DWARF debug info if the binary is built with `-g`. Verify during IVP-130: `(gdb) print g_health.primary`. If GDB resolves the symbol, use `set g_health.primary = 0x55` (all subsystems FAULT). If not, either de-static the variable or add a `health_monitor_get_primary_ptr()` accessor. Try the symbol first — it should work with debug builds.
| `fault_force_ao_queue_flood(ao, n)` | `fault_inject.h` | Publish N dummy SIG_SENSOR_DATA events (floods all subscribers) |
| `fault_force_pio_sm_halt()` | `fault_inject.h` | Clear PIO2 CTRL SM enable bits → backup timers stop |

### GDB Scripts (scripts/fault_injection/)

| Script | Scenario |
|--------|----------|
| `inject_eskf_divergence.gdb` | Force ESKF unhealthy → verify CR-1 recovery |
| `inject_core0_stall.gdb` | Stall idle bridge → verify watchdog reboot |
| `inject_watchdog_stall.gdb` | Skip watchdog kick → verify PIO watchdog fires |
| `inject_queue_flood.gdb` | Flood AO queues → verify QP assertion or graceful handling |
| `inject_pio_sm_halt.gdb` | Disable PIO2 SMs → verify upper-layer detection (or document gap) |

---

## PIO Backup Timer — Hardware Details

**PIO block:** PIO2 (`0x50300000` base on RP2350)
**State machines:** Two dynamically claimed SMs (`g_drogue_sm`, `g_main_sm`)
**Clock:** 1 MHz (divider 150 from 150 MHz system clock)
**Countdown:** `timeout_s * 1_000_000` cycles loaded via `pio_sm_put_blocking()`
**Output:** GPIO 12 (drogue), GPIO 13 (main) — active HIGH on timeout

To halt a PIO SM from GDB without calling C code:
```
set *0x50300000 = (*0x50300000) & ~(1 << SM_NUMBER)
```

To read which SMs are enabled:
```
print/x *0x50300000
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

## References

- `.claude/DEBUG_PROBE_NOTES.md` — OpenOCD startup, GDB commands, known issues
- `docs/audits/DEV_CODE_AUDIT.md` — What's in `src/dev/` and why
- RP2350 datasheet Section 3.7 — PIO register map
- LESSONS_LEARNED.md Entry 25 — Use probe for flashing (picotool corrupts I2C)
- LESSONS_LEARNED.md Entry 33 — PIO GPIO init causes I2C interference
