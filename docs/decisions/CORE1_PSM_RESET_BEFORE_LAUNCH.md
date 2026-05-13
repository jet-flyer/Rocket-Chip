# Core 1 PSM Reset Before `multicore_launch_core1()`

**Status:** Active (R-19, audit 2026-05-07 L2)
**Decision date:** 2026-05-13
**Component:** `src/main.cpp` `init_hardware()`
**Supersedes:** none

---

## Decision

`init_hardware()` calls `multicore_reset_core1()` unconditionally before every
`multicore_launch_core1(core1_entry)`. The reset is unconditional even on
truly cold power-on, when it is technically unnecessary.

## Problem

`AIRCR.SYSRESETREQ` on RP2350 is **per-processor**, not chip-level (RP2350
datasheet §7.3.1). When Core 0 triggers `SYSRESETREQ` — e.g. from
`memmanage_fault_handler()` → `crash_record_capture()` → `NVIC_SystemReset()`
— **Core 0 reboots, Core 1 does not.**

Core 1 was previously running through `core1_entry()` → `multicore_lockout_victim_init()`
→ `core1_sensor_loop()`. The lockout-victim init installs an exclusive
handler for `SIO_FIFO_IRQ` on Core 1's NVIC. **On RP2350, `SIO_FIFO_IRQ` is
a single shared IRQ number** (different from RP2040, where each core had its
own). When Core 0 later writes the lockout request into the FIFO during any
runtime `flash_safe_execute()`, the IRQ fires on Core 1.

After Core 0's `SYSRESETREQ`, Core 1's NVIC pending bit for `SIO_FIFO_IRQ`
**survives the reset**. Core 0's reboot eventually reaches `init_hardware()`,
which calls `multicore_launch_core1(core1_entry)`. That function:

- pushes a trampoline + entry pointer + new stack onto the launch FIFO,
- signals Core 1 via the bootrom-mediated launch handshake,
- **does not reset Core 1's NVIC**,
- **does not toggle `PSM_FRCE_OFF_PROC1`**.

When Core 1 wakes from its previous loop and the bootrom hands control back
to `core1_entry()`, the pending `SIO_FIFO_IRQ` fires *before* our first line
of code (i.e. before `multicore_lockout_victim_init()` reinstalls a handler).
Vectoring to the absent handler resolves to `isr_invalid`. Core 1 wedges.

## Attempted-but-rejected fixes

The full failure-mode catalog (with reasons) lives in `AGENT_WHITEBOARD.md`
"In-flight fault recovery architecture" entry. Summary:

| Attempt | Why it failed |
|---|---|
| Drain SIO from handler | The handler runs on Core 0; it drains Core 0's inbound FIFO, not Core 1's NVIC pending bit. |
| Drain SIO + clear NVIC pending in `core1_entry` | Too late — the pending IRQ has already fired between bootrom handoff and our first instruction. |
| Install a no-op handler on Core 1 before bootrom handoff | Conflicts with `multicore_lockout_victim_init()`'s exclusive-handler assertion. |

## Why `multicore_reset_core1()` works

`multicore_reset_core1()` toggles `PSM_FRCE_OFF_PROC1` in the Power-on State
Machine (PSM_BASE + PSM_FRCE_OFF_OFFSET). The PSM is *not* the ARM AIRCR —
it is the chip-level power/reset state machine controlling each processor's
power domain. Toggling `PSM_FRCE_OFF_PROC1` forces Core 1 into hard reset,
which clears:

- Core 1's NVIC pending state (including any persisted `SIO_FIFO_IRQ` bit)
- Core 1's CPU state (registers, MSP, MSPLIM, BASEPRI, PRIMASK)
- Core 1's MPU configuration

After the reset clears, Core 1 re-enters the bootrom in a known clean state.
The subsequent `multicore_launch_core1()` then has a guaranteed clean Core 1
to hand the trampoline + entry pointer to.

`multicore_reset_core1()` also explicitly clears
`lockout_victim_initialized[1]` so that the next
`multicore_lockout_victim_init()` from Core 1 doesn't no-op out.

The SDK's own implementation (`pico-sdk/src/rp2_common/pico_multicore/multicore.c`)
contains this comment:

> Note that there is no automatic way for us to clear these flags when a
> particular core is reset, and yet we DO ideally want to clear them, as the
> `multicore_lockout_victim_init()` checks the flag for the current core and
> is a no-op if set.

This explicit clear is the second reason `multicore_reset_core1()` must
precede `multicore_launch_core1()` in any "not a truly cold boot" context.

## Why unconditional (approach A, not approach B)

The surgical alternative (approach B) is to check the crash-record magic
before Core 1 launch and only call `multicore_reset_core1()` after detected
AIRCR-style reboots. We chose unconditional reset instead because:

1. **Cost is ~1 ms** even on a truly cold boot — `multicore_reset_core1()`
   does a PSM force-off + wait + force-off-clear + FIFO drain. Negligible.
2. **Defense-in-depth.** The wedge is the failure-mode `multicore_reset_core1()`
   prevents; if any new code path triggers a non-cold reset of Core 0 that
   bypasses crash-record capture, the surgical check would silently miss it.
3. **Aligns with SDK upstream guidance** (pico-feedback #366,
   deepwiki SDK multicore docs): "in any context where Core 1 may have been
   running" is the documented condition; we satisfy it by always assuming
   yes.
4. **One fewer code path to audit.** No conditional logic, no crash-record
   timing concern, no question of "what if crash record didn't get set."

The trade-off is a slight cold-boot timing increase that doesn't pay for
itself for users who care about boot speed below ~1 ms granularity. For
RocketChip, we already do `sleep_ms(100)` and similar settling waits in init,
so 1 ms is not load-bearing.

## Long-term direction

SDK 2.3.0 is expected to move `multicore_lockout` to **doorbell**-based
signalling instead of `SIO_FIFO_IRQ` (pico-sdk issue #2222). Doorbells are
a dedicated hardware mechanism whose pending state is per-core and resets
with the core, so the entire wedge category disappears. When that SDK
version ships, this decision can be revisited — the `multicore_reset_core1()`
call may become unnecessary, but should not become harmful.

## References

- RP2350 datasheet §7.3.1 (per-processor AIRCR.SYSRESETREQ semantics)
- RP2350 datasheet §5.4 (PSM_FRCE_OFF_PROC1 register)
- Pico SDK `src/rp2_common/pico_multicore/multicore.c` `multicore_reset_core1()`
- pico-feedback issue #366 (Pico-probe Requires Explicit multicore_reset_core1()
  for Core1 FIFO/ISR Use):
  https://github.com/raspberrypi/pico-feedback/issues/366
- pico-sdk issue #2222 (Make RP2350 use a doorbell rather than FIFO IRQ for
  multicore lockout): https://github.com/raspberrypi/pico-sdk/issues/2222
- Forum thread on multicore + flash interaction:
  https://forums.raspberrypi.com/viewtopic.php?t=388734
- `AGENT_WHITEBOARD.md` "In-flight fault recovery architecture" entry
  (failure-mode catalog and the surfaced limitation that motivated R-19)
- `LESSONS_LEARNED.md` Entry 31 (flash_safe_execute corrupting I2C, same
  family of multicore + flash + IRQ interaction)

## Verification (pending)

⏸ Verification gates required before this decision doc's status can move
from Active-pending to Active-verified:

1. **Build parity** — 4-tier clean (vehicle dev/flight + station bench/flight).
   ✅ Done at commit time.
2. **3-boot reseat (HW_GATE_DISCIPLINE Rule 2)** — Boot 1 → power off →
   STEMMA-QT reseat → Boot 2 → power off → second-cable reseat → Boot 3.
   All 3 boots must show identical positive-control signals (banner
   `flight-<HEAD>`, Hardware 14/14 OK).
3. **AIRCR-triggered reset wedge fix** — drive `SYSRESETREQ` from GDB while
   Core 1 is in `core1_sensor_loop()`; confirm post-reset:
   (a) Core 1 re-enters `core1_entry()` cleanly (no `isr_invalid` wedge),
   (b) sensor reads resume (positive-control: `Hardware 14/14 OK` and
       increasing IMU read counter in Hardware Status), and
   (c) crash record consumed and reports `MemManage` reason from prior
       boot (proves the AIRCR path itself still works).
4. **L2 audit-suite regression** — re-run Phases 3/4/7 against the
   R-19-enabled binary. (Level 2 per HW_GATE_DISCIPLINE Rule 6.)

This section updates to "verified" with citations only after all 4 gates
have actually run.
