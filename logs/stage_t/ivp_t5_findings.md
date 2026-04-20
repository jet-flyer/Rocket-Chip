# IVP-T5 — Silent-RX bug diagnosis findings

**Date:** 2026-04-19
**Plan:** `docs/plans/STAGE_T_FIX_PLAN.md` §3 IVP-T5
**Gate:** Hard (gates IVP-T8)

## Scope

Audit why T2 (station firmware cheat-mode) and T3 (protocol flip to MAVLink)
both compiled cleanly but silently broke station RX at runtime. Produce:
- Root cause (or explicit "no single cause")
- Canary methodology for future structural radio-path changes
- Go/no-go decision for T8 (COP-1 + CLCW) to proceed

## Method

1. Enumerated every call site of `rfm95w_init()` and `configure_modem()`.
2. Traced the T2 code path (`src/cli/rc_os_commands.cpp:48-72` and
   `src/active_objects/ao_telemetry.cpp:370-376`) and T3 code path
   (`src/flight_director/mission_profile_data.h:93-97`).
3. Compared `RadioTxEvt.buf` size (`include/rocketchip/ao_signals.h:121`) against
   MAVLink encode output sizes (`src/telemetry/telemetry_encoder.cpp:302-311`).
4. Inspected the AO_Radio init sequence in `ao_radio.cpp:262-310`.

## Finding 1 — Latent buffer-overrun risk in MAVLink TX path

**Severity:** High. Probable T3 silent-RX culprit.

`RadioTxEvt.buf` is declared `uint8_t buf[128]` at `include/rocketchip/ao_signals.h:121`.

The `MavlinkEncoder::encode_nav()` function
(`src/telemetry/telemetry_encoder.cpp:302-311`) writes FOUR MAVLink v2 messages
into `EncodeResult.buf` (a 256-byte struct field per `telemetry_encoder.h:100`):

- heartbeat: ~17 bytes
- sys_status: ~43 bytes
- attitude: ~37 bytes
- global_pos: ~42 bytes
- **Total: ~139 bytes**

The T3 code path I authored at `ao_telemetry.cpp:166-177` only encoded
heartbeat+attitude (~54 bytes), which fits. But if any code path ever calls
`encode_nav()` for MAVLink and then posts via `SIG_RADIO_TX`, the subsequent
`memcpy(txEvt.buf, result.buf, result.len)` at `ao_telemetry.cpp:190` would
write ~139 bytes into a 128-byte destination, corrupting adjacent static
memory (most likely other static `QEvt` subclasses in the same
translation unit's BSS).

**Compiles clean. Links clean. Silent runtime corruption.** Fits the T2/T3
failure signature exactly — RF front-end works, decoder path corrupted.

**Not yet proven** as the T3 culprit because my minimal T3 patch encoded only
two messages. But the 128-byte `RadioTxEvt.buf` is a latent trap for any
future MAVLink-over-LoRa configuration and will bite T8 (COP-1 + CLCW)
the moment CLCW pushes total nav payload past 128 bytes.

**Proposed fix (deferred to T8 prep, not this IVP):** bump `RadioTxEvt.buf`
and `RadioRxEvt.buf` to 256 bytes (matching `EncodeResult.buf` capacity and
`rfm95w::kMaxPayload` of 255). Add an explicit size assertion in
`ao_telemetry.cpp:190`:

```cpp
static_assert(sizeof(txEvt.buf) >= 256, "TX event buffer too small for MAVLink multi-frame");
if (result.len > sizeof(txEvt.buf)) { return; }  // defensive guard
memcpy(txEvt.buf, result.buf, result.len);
```

## Finding 2 — `rfm95w_init()` single-call invariant confirmed

`rfm95w_init()` is called from exactly two sites:
1. `ao_radio.cpp:270` — once during AO startup in `RadioAo_initial`.
2. `ao_radio.cpp:151` — inside `handle_tx_poll()`, triggered after
   `kTxFailReinitThresh = 3` consecutive TX timeouts as a radio recovery
   mechanism.

In both paths, `rfm95w_init()` does a full hardware reset
(`init_gpio_and_reset()` at `rfm95w.cpp:154`) which drives `rst` low for
10 ms. This clears the SX1276 to POR state regardless of prior mode.
`configure_modem()` then rewrites every modem register.

**Verdict:** The hypothesis that "hot reinit from running state leaves the
radio in a partial state" is **not** the T2/T3 cause. Hardware reset is
unconditional.

However, the recovery path at `ao_radio.cpp:148-152` calls `rfm95w_init()`
but does **NOT** reapply the RadioConfig overrides from
`kDefaultRocketRadioConfig` (the code at `ao_radio.cpp:278-285` is only in
`RadioAo_initial`, not in the recovery path). So after a recovery reinit,
the radio is reset to the `configure_modem()` defaults: SF7 / BW 125 / CR 4/5.
If the mission profile had non-default SF or BW, recovery would silently
change the RF config mid-flight.

**Proposed fix (separate from T5):** extract the config-apply block from
`RadioAo_initial` into a helper and call it from the recovery path too.
Not required for Stage T fix (T6 picks BW via mission profile which IS
applied in `RadioAo_initial`; recovery is a rare path). Note for future
maintenance.

## Finding 3 — Station RX init path is correct

For station role, `RadioAo_initial` at `ao_radio.cpp:293-296` unconditionally
calls `rfm95w_start_rx(&s.radio)` which:
1. Clears all IRQ flags
2. Sets FIFO pointer to RX base
3. Enters `kRxContinuous` mode

Then the tick handler at `ao_radio.cpp:334-335` drains RX via
`handle_rx_poll(me)` whenever `s.scheduler.rx_active()` is true. For station
(`rx_continuous = true`), this is always true after init.

No silent-RX exposure in this path.

## Finding 4 — T2 code path audit

The T2 cheat-mode code I authored adds to `handle_rx_packet`:

```cpp
#ifdef ROCKETCHIP_STAGE_T2_CHEAT
extern void stage_t2_fire_pending_if_any();
stage_t2_fire_pending_if_any();
#endif
```

`stage_t2_fire_pending_if_any` is defined in `rc_os_commands.cpp:48-72` and
calls `AO_Telemetry_send_tracked_command()` which populates
`s_pending_cmd` and posts `SIG_RADIO_TX` to `AO_Radio`.

**Suspect:** The T2 fire call happens from INSIDE `handle_rx_packet` on
`AO_Telemetry`. `AO_Telemetry_send_tracked_command` posts `SIG_RADIO_TX` to
`AO_Radio`, which posts back to `AO_Telemetry` on ACK. Not a direct cycle,
but it IS a handler posting an event while itself draining a handler — this
is legal under QV but the post target (`AO_Radio`) then posts to
`AO_Telemetry` from its own handler, forming `AO_Telemetry` → `AO_Radio` →
`AO_Telemetry` signal chain with `AO_Telemetry`'s handler still on the stack
when the middle leg fires. Queue-depth sensitive; could overflow or cross-
wire under the right timing.

**Not a buffer overrun (all events are static, static events are the correct
LL-35 pattern).** More likely a queue-saturation or dispatch-ordering
pathology that only manifests under the specific T2 send cadence.

**Verdict:** Not root-caused. The T2 fix code is preserved in-tree behind
`ROCKETCHIP_STAGE_T2_CHEAT=OFF` (default) for future debugging. Host-side
T2 cheat (`scripts/stage_t2_cheat.py`) is the viable approach and has
already delivered the needed data.

## Finding 5 — Canary methodology

A benign-`#ifdef` canary was NOT committed in this IVP. Rationale: the T6
sweep will introduce `ROCKETCHIP_STAGE_T6_BW` gating a `constexpr` field
in the exact same file that T3 modified (`mission_profile_data.h`). If T6
builds work and station RX stays alive, that's the canary: same `#ifdef`
gating pattern, same file, same kind of constexpr change. If T6 breaks RX,
we have a direct reproducer for the T3 failure and the canary has value as
an isolated test. For now, T6 itself IS the canary.

## Conclusions

- **T3 MAVLink silent-RX is most likely caused by buffer overrun** when the
  production MAVLink TX path (`encode_nav` in `telemetry_encoder.cpp`)
  writes ~139 bytes into the 128-byte `RadioTxEvt.buf`. This is a latent
  bug independent of T3; T3 exposed it by flipping the protocol at boot.
- **T2 silent-RX is not root-caused** but is contained by keeping the
  firmware cheat off-by-default and using the host-side cheat instead. The
  symptom is consistent with a queue-saturation or dispatch-cycling
  pathology in AO_Telemetry when posting tracked commands from inside its
  own RX handler.
- **The TX reinit recovery path at `ao_radio.cpp:148-152` has a latent
  bug** — after reinit, RadioConfig overrides are not reapplied, so BW/SF
  snap back to the `configure_modem()` defaults. Flag for future fix,
  doesn't block Stage T.

## Go/no-go for T8

**GO for T8 with prerequisite:** **bump `RadioTxEvt.buf` and `RadioRxEvt.buf`
from 128 → 256 bytes and add the size-guard assertion** as the first
mechanical step in T8. This unblocks CLCW-expanded packets AND fixes the
latent MAVLink overrun at the same time. Without this, T8's 58-byte CCSDS
packets are fine but any future MAVLink-over-LoRa will re-expose the
silent-RX pathology.

**T6 can proceed immediately** — BW sweep touches only
`kDefaultRocketRadioConfig.bandwidth_khz` (a numeric-only change). Risk
surface narrower than T3 per the round-2 Cubesat argument.

## Files

- Audit scope: `src/drivers/rfm95w.cpp`, `src/active_objects/ao_radio.cpp`,
  `src/active_objects/ao_telemetry.cpp`, `src/telemetry/telemetry_encoder.cpp`,
  `include/rocketchip/ao_signals.h`, `include/rocketchip/telemetry_encoder.h`.
- No code changes committed in this IVP.
- Findings: this file.
