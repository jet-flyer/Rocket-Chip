# Fault Handler Design — Capture-Then-Reset Pattern

**Status:** Active design decision.
**Established:** 2026-05-12 (R-3 of the 2026-05-07 master standards audit).
**Applies to:** `src/safety/fault_protection.cpp::memmanage_fault_handler`, `src/safety/crash_record.{h,cpp}`.

---

## The recovery pattern

`memmanage_fault_handler` runs when the MPU stack guard fires (or any
MemManage / BusFault / UsageFault escalates to it). It:

1. Disables interrupts.
2. Captures CFSR, HFSR, stacked PC + LR via inline asm + direct loads.
3. Writes the captured state into `g_crash_record` (preserved-SRAM,
   `.uninitialized_data` section, NOLOAD — survives `NVIC_SystemReset`).
4. Triggers `SCB->AIRCR.SYSRESETREQ` via inline asm.
5. Spins in WFE if the reset doesn't fire (defense — should not reach).

On the next boot, `health_monitor_init()` calls
`crash_record_consume_prior()`, latches `kHealthCriticalPriorHardfault`,
and emits a `[HEALTH] prior-boot hardfault` log line with the captured
context. From there, the project's existing safe-mode / FAULT-health
pivot owns the recovery (GO/NO-GO blocks ARM, LED + audio reflect
fault, telemetry exposes the state).

This replaces the pre-R-3 halt-forever LED-blink loop, which was a
holdover from the deprecated watchdog-reset era.

---

## Why the handler MUST be a single inline sequence (no function calls)

This is the load-bearing constraint that drives the handler's structure
and the function-size deviation:

> A C-level call inside a HardFault handler can fault-on-fault, which
> escalates to processor lockup (PC = `0xfffffffe` lockup canary,
> requires chip reset).

Three sources converge on this:

1. **Memfault Cortex-M HardFault debugging guide** — recommends
   capture-then-reset with no function calls.
2. **ARMv8-M Architecture Reference Manual** — *"a fault inside a
   HardFault handler causes lockup."*
3. **Lived experience** — surfaced 2026-05-12 during R-3's 3-boot
   reseat verification: triggering `fault_force_hardfault()` a
   second time within a single power-on session caused the lockup
   (OpenOCD reported *"clearing lockup after double fault"*). Root
   cause was the previous version of the handler calling
   `crash_record_capture()` as a regular C function, which pushed a
   stack frame that — with the failing stack barely above the MPU
   guard — crossed the guard boundary and re-entered the handler.

The fix is to inline the capture-and-reset operations entirely:
direct memory writes, no function calls, no stack frame setup.
Disassembly of the handler must show **zero `push` instructions**
beyond what the ARM exception entry already lays down.

---

## Function-size deviation — accepted

The inline-everything constraint puts the handler at **~38 LSLOC of
actual code** plus the few short single-line comments that name what
each block does. The project's clang-tidy configuration sets
`readability-function-size.LineThreshold = 60` (JSF AV Rule 1) which
flags the whole function.

**This is an accepted deviation, not a bug.** The handler cannot be
decomposed into helper functions without re-introducing the very
stack-push that this design exists to eliminate. Any decomposition
defeats the load-bearing constraint.

The deviation is logged in `standards/ACCEPTED_STANDARDS_DEVIATIONS.md`
under FH-1.

---

## Other recovery primitives we evaluated (and why we rejected them)

**Internal MCU watchdog reset** — the project explicitly retired this
during the safe-mode pivot. Per IEC 61508, an internal watchdog has
limited fault-coverage value because it's part of the same silicon
that can fail. Don't reintroduce.

**Watchdog-triggered chip-level reset** — same as above. The PIO
watchdog (`src/safety/pio_watchdog.cpp`) provides fault *detection*
on a separate silicon block (PIO2 state machine) which is closer to
the IEC 61508 ideal, but it doesn't trigger reset; it sets an IRQ
flag observable by the ARM, which then transitions to safe-mode
in-place.

**Capture-then-safe-mode-in-place (no reset)** — the right primitive
for in-flight faults, since reset destroys the ESKF datum and makes
the rest of the flight useless. The current handler always resets
because the MPU stack guard is a structural-integrity fault from
which recovery without restart is unsafe. A future evaluation
(tracked on `AGENT_WHITEBOARD.md`) will look at the in-flight reset
vs in-place safe-mode tradeoff per fault category.

---

## Known limitation — Core 1 SIO_FIFO_IRQ wedge after AIRCR reset

`SCB->AIRCR.SYSRESETREQ` on RP2350 is a processor-only reset (per
RP2350 datasheet §7.3.1 — AIRCR is not listed as a chip-level reset
source). SIO peripheral state, including FIFO IRQ pending bits,
survives the reset. After reset, when Core 0 boots and re-launches
Core 1 via `multicore_launch_core1()`, Core 1's NVIC retains the
pre-reset SIO FIFO IRQ pending bit, which fires before Core 1's
user code can install a handler — vectoring to `isr_invalid` and
wedging Core 1.

This is a chip-level limitation, not an R-3 logic bug. R-3 ships
partially validated (single-cycle bench evidence is clean; 3-boot
reliability is blocked on this wedge). The wedge is documented as
an architectural item for a dedicated future session covering:
PIO+safe-mode-in-place as primary fault recovery, persistent state
in `.uninitialized_data`, multi-MCU co-processor patterns
(per IEC 61508 HFT≥1).

See `AGENT_WHITEBOARD.md` for the deeper architectural notes.

---

## References

- Memfault, *"How to debug a HardFault on an ARM Cortex-M MCU"* —
  https://interrupt.memfault.com/blog/cortex-m-hardfault-debug
- ARMv8-M Architecture Reference Manual — fault escalation + lockup
  semantics.
- RP2350 Datasheet §7 (Resets) — AIRCR is processor-only on RP2350.
- IEC 61508 — single-channel fault-coverage framing.
- `AGENT_WHITEBOARD.md` — deeper architectural notes for the
  dedicated future session.
- `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` FH-1 — function-size
  deviation entry.
