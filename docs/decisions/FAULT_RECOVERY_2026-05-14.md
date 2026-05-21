# In-Flight Fault Recovery Architecture — Phase-Aware Dispatch

**Status:** Active design decision.
**Established:** 2026-05-14 / 2026-05-15 (rework landed in commits `ed7c569` + `8baa18a`, council rounds 1+2+3 unanimous).
**Authoring captured retroactively:** 2026-05-20 (decision doc written to capture rationale that had been living in inline comments + the plan file `C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn.md`).
**Applies to:** `src/safety/fault_protection.cpp` (`memmanage_fault_handler`, `Q_onError`), `src/safety/anomalous_boot.h/.cpp`, `src/safety/crash_record.{h,cpp}` (`flight_in_progress_set/clear`), `src/flight_director/flight_director.cpp` (`enter_phase`, `flight_phase_observable_set`, `flight_phase_observable_get`), `src/active_objects/ao_health_monitor.cpp`.

---

## Problem the rework addressed

Pre-rework, the fault handler unconditionally reset the chip via `SCB->AIRCR.SYSRESETREQ` (per `FAULT_HANDLER_DESIGN.md`). This was correct on the pad (capture-and-reset preserves crash record across boot, operator sees prior-fault latch via `[HEALTH] prior-boot hardfault`), but it was the **wrong** action in flight:

- An in-flight reset interrupts every PIO timer (watchdog SM, backup-timer SMs), drops Core 1 sensor sampling for the boot-init delay, and triggers re-init of every peripheral while the vehicle is airborne. The boot-init delay alone is on the order of seconds for things like the GPS UART negotiation — a multi-second blackout during boost or descent is worse than the fault.
- The pre-rework SIO_FIFO_IRQ wedge (R-19) and related symptoms had been silently masked because the AIRCR reset path was never being exercised in flight by accident.

The rework introduces **phase-aware fault dispatch**: same handler, different action depending on whether the fault occurred on the pad (`kIdle`) or in any flight phase.

---

## B.1 — Zero in-flight reset

`memmanage_fault_handler` and `Q_onError` both check the flight phase via the fault-handler-observable phase accessor (B.3, see below) and dispatch differently:

- **`kIdle`:** capture crash record, brief visible-signal delay so the operator sees a status LED change, then `SCB->AIRCR.SYSRESETREQ`. Same as pre-rework. Boot path latches `kHealthCriticalPriorHardfault`; operator clears via CLI `r`.
- **Any flight phase (`kArmed` through `kMainDescent`, `kAbort`, `kFault`):** capture crash record, transition the FlightPhase observable to `kFault`, busy-loop with `__WFE()`. **Do NOT issue AIRCR.** PIO timers continue autonomously (watchdog SM, backup-timer SMs are independent of the ARM core), telemetry beacon continues as long as the radio path was set up before fault.

The trade is that ARM-side recovery is dead — no more sensor sampling, no more FlightDirector ticks, no more CLI. PIO-driven beacon + landing detection by the operator is the recovery path. See "B.5 — PIO beacon" follow-up in `AGENT_WHITEBOARD.md` for the future-session item that adds a PIO-independent beacon to close the in-flight ARM-dead coverage gap.

## B.2 — No silent reset on the pad

The `kIdle` path was already correct (capture + reset + boot-latch), but the rework adds a brief visible-signal delay before the reset fires. Operator sees the status LED change (red blink pattern from `kLedPhaseFault` if FD has been ticked, or the bootloader visual from the reset itself) before the reset. Without the delay, a fault on the pad could reset fast enough to look like a normal power cycle, leaving the operator unaware that anything went wrong.

## B.3 — Phase-aware dispatch via checksummed byte pair

The fault handler cannot read the FlightDirector's `me->state.current_phase` directly — that lives in QF state behind active-object boundaries, and traversing those from a HardFault context is unsafe (fault-on-fault risk per `FAULT_HANDLER_DESIGN.md` "Why the handler MUST be a single inline sequence").

Solution: a single `volatile uint32_t g_phase_observable_pair` that the FlightDirector writes from `enter_phase()`. Packing:

- Low byte = phase value (`uint8_t`).
- Bits 8-15 = bitwise complement of the phase value (`~phase`).
- Upper 16 bits unused (always 0 from initial write).

Initial value `0xFFFFFFFFU` decodes to phase=`0xFF`, complement=`0xFF`, validation `~0xFF == 0xFF` → fails (because `~0xFF == 0x00`). Validation failure routes the fault handler to `kFault` dispatch (safe-by-default — "we don't know what phase we're in, assume the worst").

After a normal `enter_phase(kIdle)` boot, the pair decodes cleanly and the handler dispatches to the `kIdle` path. If a memory-corruption fault flipped a bit in the low byte without flipping the corresponding high-byte bit, validation fails and the handler routes to `kFault` — safe-by-default again.

Implemented as `flight_phase_observable_set()` / `flight_phase_observable_get()` in `flight_director.cpp`. The accessor is the only safe way for the fault handler to read flight phase.

## B.4 — Anomalous-boot confidence gate

After a reset (any cause), `anomalous_boot_evaluate()` reads:

1. `POWMAN_CHIP_RESET` cause bits — distinguishes power-on / warm-reset / brownout / watchdog-RSM / hazard-DP / glitch-detect / SWcore-PD reset classes.
2. `.uninitialized_data` `flight_in_progress` sentinel — set on `enter_phase(kArmed)`, cleared on `enter_phase(kLanded)`. Survives `NVIC_SystemReset` (preserved-SRAM); cleared by genuine power loss.
3. AON timer prior-uptime signal — **currently stubbed to 0**. Wiring requires adding `pico_aon_timer` to `target_link_libraries` + explicit timer-start at boot. See AGENT_WHITEBOARD.md for the deferral rationale.

Returns one of `BOOT_NORMAL`, `BOOT_POWER_ON`, `BOOT_PROBABLY_MID_FLIGHT`, `BOOT_BROWNOUT`.

The verdict is consumed at:

- `main.cpp:381` — `BOOT_PROBABLY_MID_FLIGHT` **suppresses auto-zero-baro**. Auto-zero on a mid-flight reboot would lock in a baro reference at altitude, breaking apogee detection. Operator must manually clear with CLI `r` to reset to IDLE.
- Brownout cause routes to an independent `kHealthCriticalPriorBrownout` health-monitor latch regardless of mid-flight verdict — brownouts are always operator-visible.

## B.6 — ARM-state position (i): keep 500ms-persistence auto-DISARM

No code change required. The pre-rework 500ms-persistence auto-DISARM already matches ArduPilot/PX4 lived-experience for no-uplink vehicles. The position was deliberately re-validated by council (round 1) rather than changed.

## B.7 — Fault-handler reentrance guard

A static `s_in_fault_handler` flag in `memmanage_fault_handler` is set on first entry. Second entry to the handler (would indicate a fault inside the handler itself) hits the guard and halts via `__WFE()` immediately. Prevents the lockup canary scenario described in `FAULT_HANDLER_DESIGN.md` "lived experience" section.

## B.8 — Explicit `FlightPhase::kFault` value

Pre-rework, the FlightPhase enum had no `kFault` — fault dispatch was implicit, and the LED pattern was shared with `kAbort` (red blink). The rework adds:

- `FlightPhase::kFault` enum value (distinct from `kAbort`).
- `kLedPhaseFault` magenta-blink LED pattern (distinct from `kAbort` red).
- `state_fault` is NOT a real QHsm state — `enter_phase(kFault)` is invoked from the fault handler (or from `Q_onError`'s in-flight path) to write the observable pair, set the LED, and update the phase enum. The FlightDirector itself stays parked on its last QHsm state; no `Q_TRAN` is invoked from the fault context.

## kAbort invariant — flight-in-progress sentinel

`flight_in_progress_set()` is called from `enter_phase(kArmed)`. `flight_in_progress_clear()` is called ONLY from `enter_phase(kLanded)`. It is **NOT** cleared on `enter_phase(kAbort)` or `enter_phase(kFault)`.

Rationale: an aborted flight is still a flight that requires operator inspection before the next arming sequence. Clearing the sentinel on `kAbort` would let a subsequent reset look like `BOOT_NORMAL` to the anomalous-boot gate, which would re-enable auto-zero-baro and could mask a real prior incident. The operator clears the latch by transitioning through `kLanded` (or by manual CLI `r` in `kIdle`, which is the no-flight reset path).

This invariant is load-bearing for the BOOT_PROBABLY_MID_FLIGHT verdict on the next boot — without it, the verdict misfires.

---

## What is NOT in this rework

- **PIO beacon + SPI last-gasp beacon (B.5)** — deferred to combined future session. See AGENT_WHITEBOARD.md "PIO beacon + SPI last-gasp" row + council round 3 transcript.
- **AON-timer prior-uptime signal wiring** — stubbed to 0. Deferred per AGENT_WHITEBOARD.md.
- **Mid-flight-reboot survivability beyond detection** — rejected unanimously per NASA Initialization-Safe-Mode trap framing + LL Entry 36.
- **Multi-MCU Gemini/Titan-with-Core architecture** — stays as hardware-tier item; out of single-MCU scope.

---

## References

- Plan + council transcript: `C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn.md`.
- Council round 3 (NASA/JPL + Cubesat) transcript: `C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn-agent-a355e8caee0717e0b.md`.
- Companion: `docs/decisions/FAULT_HANDLER_DESIGN.md` (capture-then-reset pattern for the `kIdle` path; this doc adds the in-flight phase dispatch on top).
- Companion: `docs/decisions/HEALTH_CONTRACT.md` (`kHealthCriticalPriorHardfault`, `kHealthCriticalPriorBrownout` latches consumed by GoNoGo).
- Companion: `docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md` (R-25-exec test-mode gate that interacts with the `fault_force_*` entries used to verify this rework).
- AGENT_WHITEBOARD.md "In-flight fault recovery architecture rework — LANDED 2026-05-14/15" row tracks open follow-ups (B.5 combined session, AON-timer wiring, audit-suite regression for R-22/R-25-exec).
