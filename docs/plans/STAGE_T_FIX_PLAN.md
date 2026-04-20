# Stage T Fix Plan — B-500 + Retry Compression + COP-1

**Status:** Ready to execute. IVPs T1-T4 (diagnostic) complete and committed; T5-T10 (fix) designed below.
**Author:** Claude + council round 2 (NASA/JPL, ArduPilot, Rocketeer, Cubesat) — 2026-04-19
**Scope:** Vehicle + station firmware + one new sweep script. Target **telemetry rate (`nav_rate_hz` field in `RadioConfig`)** stays at 10 Hz on the air.
**Non-scope:** Not MAVLink-over-air (T9 is informational only). Not FDD. Not SF6/SF5. Not a telemetry-rate reduction.
**Mirror copy:** `docs/plans/STAGE_T_RADIO_DIAGNOSTICS.md` was the diagnostic plan. This fix plan will get a companion copy in `docs/plans/STAGE_T_FIX_PLAN.md` at execution time.

> **Terminology note.** The `RadioConfig.nav_rate_hz` field (`include/rocketchip/radio_config.h:32`) is the authoritative identifier and controls the **radio telemetry-packet TX rate**, not an ESKF or internal navigation rate. `ao_radio.cpp:290` derives the TX interval as `1000 / nav_rate_hz` ms. The legacy "nav" name is preserved for code references; prose in this plan uses "telemetry rate" for clarity.

---

## 1. Context

T1-T4 diagnostic data (see `logs/stage_t/FIX_COUNCIL_REPORT.md`) proved station→vehicle command link is **collision-dominant**, not sensitivity- or interference-limited:

- First-try ACK: **6.1%** pooled (N=114, 3-4 ft bench)
- Every vehicle RX gets ACKed; zero CRC errors. Failure mode is the vehicle never hearing the command
- T2 (perfect sync cheat) scored 0% — WORSE than random — because station TX airtime (~140 ms at SF7/BW125 CCSDS) exceeds 100 ms vehicle RX window at 5 Hz telemetry rate. **Sync alone cannot close this.**
- T4 ambient: noise floor -114 dBm, no interferer, link budget is not the problem

Round 1 council picked option A (drop `nav_rate_hz` 5 → 2). User rejected: "10 Hz is target, 2 Hz unacceptable — boost phase is ~3 s for medium HPR, 5 samples at 2 Hz vs 30 at 10 Hz on the station dashboard." Round 2 re-ranked with **10 Hz telemetry rate** as hard constraint and ~2 km LOS as range floor.

### Round 2 consensus

| Rank | Fix | Mechanism | Impact |
|------|-----|-----------|--------|
| Primary | **B-500** | SF7/BW500 kHz/CR4-5, keep `nav_rate_hz = 10` | Station TX airtime 140 → ~35 ms; fits 100 ms period with ~15 ms jitter margin; link budget ~39 dB margin at 2 km LOS |
| Secondary | **F** | Retry 3000 → 500 ms | Mean successful-command latency target < 2 s |
| Tertiary | **E** | CCSDS COP-1 + CLCW | Closes retransmit loop against remaining drops; user explicitly endorsed |

Rejected: A (user vetoed), C (A+B bundling violates no-combination rule), D (FDD overkill), SF6 (implicit header breaks variable-length CCSDS/MAVLink), SF5 (SX1276 does not support).

**Decoupled log/telemetry rate:** flash log can run higher than over-the-air telemetry rate independently — orthogonal to Stage T, not in this plan. 10 Hz stays on the air as target.

---

## 2. Architectural constraints (non-negotiable)

- **LL Entry 32 — no blocking calls in AO handlers.** COP-1 retransmit state machine (T8) MUST be timer-driven via posted events. No `sleep_ms`, no spin-wait, no nested post-from-inside-a-draining-handler.
- **LL Entry 35 — static or pool-allocated events for `QACTIVE_POST`.** Pattern in use at `ao_telemetry.cpp:600-605` and `:637-643`. Any new COP-1 event follows the same pattern.
- **LL Entry 36 — hard vs soft gate distinction.** Each IVP annotates pass criteria. Soft gates are informational; hard gates block advance.
- **"Compiled clean ≠ RX works."** T2 and T3 both compiled cleanly and silently broke RX. Every radio-path IVP (T5-T10) includes a **post-flash RX-alive canary**: boot vehicle + station, issue CLI `t`, confirm `rx_count` (`ao_radio.cpp:192`) advances within 60 s and `rx_crc_errors` stays at 0. Canary failure = abort, do not proceed, do not measure.
- **No combination moves.** Each IVP is one independent config change + one verification run + one commit. Bisectable.
- **HW verification strategy (Stage T specific):** T5-T9 are bench iteration where data collection itself IS the verification — each IVP runs ack_stress or an equivalent measurement, so there's no separate "HW verify" step. The RX-alive canary above is the only per-IVP hardware check. The formal HW verification gate applies only at **T10 exit** (comprehensive soak, 6-criterion check). Per-IVP commits may be done without a separate HW verify ritual; the measurement run IS the verification.

---

## 3. IVP Breakdown

| IVP | Purpose | Primary file(s) | Gate | Est. runtime |
|-----|---------|-----------------|------|--------------|
| T5 | Diagnose T2/T3 silent-RX regression | `rfm95w.cpp:145-174`, `ao_radio.cpp` | **Hard** (gates T8) | 1-2 h bench |
| T6 | RF BW sweep — pick winner | `mission_profile_data.h:91-103` | **Hard** (gates T7/T10) | ~20 min bench |
| T7 | Retry timer 3000 → 500 ms at winner | `ao_telemetry.cpp:649-668` | **Hard** | ~15 min bench |
| T8 | CCSDS COP-1 + CLCW | `telemetry_encoder.h`, `ao_telemetry.cpp:127-150` | **Hard** | 2-3 h impl + 30 min soak |
| T9 | MAVLink retest at winner BW | `mission_profile_data.h:93` flag | **Soft** (informational) | ~15 min bench |
| T10 | Stage T exit soak | — (integration) | **Hard** (stage exit) | 30+ min soak |

### IVP-T5 — Diagnose silent-RX reinit bug

**Hypothesis (NASA/JPL + ArduPilot round 2):** `rfm95w_init()` at `rfm95w.cpp:145-174` enters Sleep unconditionally at line 164 without checking current mode. `configure_modem()` at `:112-132` rewrites RegModemConfig1/2 but may not fully reset DIO/FIFO/IRQ state. If called after a running flash+reboot (as in T2/T3), radio ends up in a state where RF front-end works but decoder silently fails. Cross-check: ArduPilot's SX1276 driver explicitly full-rewrites every config register on every init.

**Tasks (primarily audit + one canary patch):**
1. Trace every call site of `rfm95w_init()`. Confirm single-call invariant or document re-entry path.
2. Audit register-write ordering in `configure_modem()` against SX1276 datasheet 4.1.6 — especially FIFO base pointers (`rfm95w.cpp:118-119`) vs mode transition (`:170`).
3. Identify what the T2 and T3 patches changed on the radio hot path that broke RX. Look for:
   - Second `rfm95w_init` call introduced
   - Partial reconfig bypass
   - Modem register written with stale cached value vs fresh constexpr
4. Write a **benign canary patch**: add `-DROCKETCHIP_T5_CANARY` gating an unused `constexpr kCanaryMarker`. Flash, boot, confirm RX works. Proves the canary methodology before T6-T10 rely on it.
5. Produce `logs/stage_t/ivp_t5_findings.md` — root cause (or explicit "no single cause, operational rule: power-cycle between config changes").

**Pass (HARD):** Written root-cause note identifying the exact file:line causing the T2/T3 regression, plus canary confirms RX survives benign `#ifdef` changes. Either fix lands in T5 (if trivial and orthogonal) or workaround rule documented for T6-T10.

**Diagnostics if surprising:** If the bug is in `ao_radio.cpp` event handling, re-scope T5 to produce the fix. If it's a hardware/connector issue, stop and escalate — no BW sweep will help.

**Parallelizable with T6** only if T5 is read-only audit. Any T5 code change must land and soak before T6 flash.

### IVP-T6 — RF bandwidth sweep

**Hypothesis:** Airtime ∝ 1/BW at constant SF. BW125 → ~140 ms; BW250 → ~70 ms; BW500 → ~35 ms. BW500 fits two TX cycles inside 100 ms RX window. Sensitivity cost ~6 dB vs BW125 — link budget at 2 km LOS still closes with ~39 dB margin (from T4).

**Code change:** ONE file. `src/flight_director/mission_profile_data.h:91-103`. Add `#ifdef ROCKETCHIP_STAGE_T6_BW ... #else ... #endif` wrap around `bandwidth_khz` and `nav_rate_hz` initializers (the telemetry rate field) so sweep is a build-flag flip, not source edit per run.

Driver already supports BW500: `rfm95w.h:85-87` (`kBw125=0x07, kBw250=0x08, kBw500=0x09`), `rfm95w.cpp:364-371` (`rfm95w_set_bandwidth` no bounds check), `ao_radio.cpp:276-285` (BW setter chain applied at init).

**Sweep matrix (N=30 per config, 10 s inter-command, 3-4 ft):**

| Config | SF | BW (kHz) | `nav_rate_hz` (telemetry rate) | Purpose |
|--------|---:|---------:|-------------------------------:|---------|
| C0 | 7 | 125 | 10 | Control — reproduces baseline failure |
| C1 | 7 | 250 | 10 | Halfway point |
| C2 | 7 | 500 | 10 | **Primary candidate** |
| C3 | 7 | 500 | 10 | Repeat — confirms C2 isn't a lucky run |

**Per-config procedure:**
1. Flip build flag, rebuild `rocketchip.elf` vehicle + station.
2. Flash BOTH vehicle and station (BW mismatch = zero RX).
3. Post-flash canary: `t` on both, confirm `rx_count` advances within 60 s, `rx_crc_errors == 0`.
4. Run `scripts/ack_stress_test.py --count 30 --interval 10.0 --csv logs/stage_t/ivp_t6_{config}.csv`.
5. Capture: first-try ACK %, after-retries %, mean latency, station `rx_count` delta.

**New script:** `scripts/stage_t6_sweep.py` — iterates 4 configs, orchestrates rebuild + flash + canary + ack_stress + log append. Thin wrapper; don't extend `stage_t_run.py` in place (different semantics).

**Pass (HARD, two conditions):**
- First-try ACK ≥ 60% on C2 (BW500) AND jitter margin (100 ms window − measured TX airtime − measured clock drift over 30 s) ≥ 10 ms.
- C0 baseline reproduces 0-15% first-try (within T1 noise). If C0 suddenly scores 70%, something else changed — sweep is suspect.

**Mandatory follow-up: re-run T4 ambient at winner BW.** Use `scripts/stage_t4_ambient.py`. Pass: noise floor ≥ 9 dB above sensitivity at operational BW (sensitivity ~6 dB worse at BW500, so threshold tighter in absolute terms — intentional).

**Diagnostics if surprising:**
- C2 < 60% → council reconvenes. Don't proceed to T7 with a weak winner.
- C1 beats C2 → adopt BW250. Still fits nav rate constraint (~70 ms in 100 ms with ~30 ms margin), better sensitivity.

### IVP-T7 — Retry timer 3000 → 500 ms

**Hypothesis:** With collisions resolved by T6, the 3 s retry timer is now the dominant latency contributor. Cutting to 500 ms brings mean successful-command latency below 2 s.

**Code change:** ONE file. `src/active_objects/ao_telemetry.cpp`.
- Extract hardcoded 3000 at line 654 into named constant near top of file: `constexpr uint32_t kAckRetryTimeoutMs = 500U;`
- Line 654: `if (elapsed >= 3000)` → `if (elapsed >= kAckRetryTimeoutMs)`.
- LEAVE `retries_left = 3` at line 613 unchanged. Worst-case 4 × 500 ms = 2 s total — still inside < 2 s mean since most ACK on try 1-2.

**Pass (HARD):** Re-run ack_stress at T6 winner config, N=30:
- First-try ACK ≥ 85% (exit criterion target)
- Mean successful-command latency < 2 s
- Fail-after-3-retries ≤ 1%

**Diagnostics if surprising:** First-try drops from T6 result → AO queue saturation from faster retries. Check `ao_radio.cpp` TX queue depth, posted event count vs drained. Do NOT raise the retry timer to mask it.

### IVP-T8 — CCSDS COP-1 + CLCW

**Prerequisite:** T5 resolved. COP-1 introduces a state machine in `AO_Telemetry` — exactly the kind of change that broke RX silently in T2/T3. Without T5's root cause, we're flying blind.

**CLCW placement decision (council left to implementation):**

| Option | Placement | Cost | Verdict |
|--------|-----------|------|---------|
| (a) | Extend CCSDS primary header | No spare bits (`telemetry_encoder.cpp:47-51,67-85`); breaks existing decoders | REJECT |
| (b) | Carve MET secondary header | Lose ~9 bits from 32-bit MET | REJECT (silent degradation) |
| (c) | Append 4-byte CLCW to nav payload | Payload 42 → 46 B, total 54 → 58 B, decoder version bump | **ADOPT** |

**Code changes:**
1. `include/rocketchip/telemetry_encoder.h:70-75` — bump total 54 → 58 B, add `kClcwOffset` and `kClcwLen = 4` constants, increment packet-format version.
2. `src/telemetry/telemetry_encoder.cpp` — write CLCW (station's last-ACKed seq + FARM-B counter + retransmit flag) into last 4 bytes of nav payload. Station-side decoder reads CLCW, feeds to new `cop1_state_t` in `AO_Telemetry`.
3. `src/active_objects/ao_telemetry.cpp:127-150` — current `s_pending_ack` (`kApidCmdAck = 0x003`) STAYS as fast-ACK path. COP-1 is slow-path guarantee layer, not replacement.
4. New COP-1 state machine in `AO_Telemetry`:
   - MUST use `QACTIVE_POST` with static events (LL-35); mirror pattern at `:600-605`.
   - MUST NOT block (LL-32). Retransmit tick is posted timer event.
   - Station: keeps sliding window of unacked commands, retransmits based on CLCW.
   - Vehicle: updates CLCW on every nav TX with `last_cmd_seq_ok`.

**Post-flash canary:** Same as T5-T7 (`rx_count` advances, `rx_crc_errors == 0`). Plus new: send known-good command, confirm BOTH fast-ACK AND CLCW window advances on station.

**Pass (HARD):**
- First-try ACK ≥ 85% (unchanged from T7 — COP-1 is safety net, not primary accelerator)
- Every command in 100-command soak lands within 3 retries (≤ 1% dropped)
- LL-32/LL-35 audit signed off (explicit self-review checklist — no blocking calls in handlers, no stack-local events posted)

**Diagnostics if surprising:** T8 breaks RX like T2/T3 → revert immediately, consult T5 findings. LL-32/35 audit finds violations → fix in place before measuring (co-requisite, not follow-up).

### IVP-T9 — MAVLink retest at winner BW (informational)

**Task:** Re-enable existing `ROCKETCHIP_STAGE_T3_MAVLINK` flag (already at `mission_profile_data.h:93`, left in tree from T3). Rebuild at T6 winner BW. Run ack_stress N=30.

**Pass (SOFT — informational):** Confirms CCSDS first-try % ≥ MAVLink first-try % at winner BW. Expected: CCSDS within noise of MAVLink or better (CCSDS packet is smaller). If MAVLink significantly beats CCSDS, flag for post-Stage-T design review — do NOT switch mid-stage.

**Unblocked by T5.** T5 is hypothesized to be the reason T3 appeared to break RX; T9 is the real data point once T5 workaround is in place.

Station-side MAVLink-to-QGC USB output already wired at `ao_telemetry.cpp:302-340` (`dispatch_nav_output()`) — no station-side changes needed for this test.

### IVP-T10 — Stage T exit soak (formal HW verification gate)

**This is the one formal HW verification gate for Stage T.** T5-T9 are bench iteration (measurement = verification). T10 is the comprehensive integration soak that certifies the whole fix package before declaring Stage T done.

**Final config:** SF7 / BW500 / CR4-5 / `nav_rate_hz = 10` (10 Hz telemetry) / retry 500 ms / 3 retries / CCSDS + COP-1.

**Procedure:**
1. Flash final firmware to vehicle (via probe, SWD) and station (via picotool with `--ser BEC71B8EDC6AEBD1`).
2. Verify both banners show the expected build hash/config — station dashboard shows `Pkts:` counter advancing, vehicle `t` shows TX/RX healthy.
3. **Idle canary (60 s before stress):** station in dashboard mode, vehicle TX'ing normally, confirm `rx_count` advances continuously AND `rx_crc_errors == 0` for 60 s. If counter stalls even briefly, halt — silent-RX regression from T8 landing.
4. **Primary soak:** `scripts/ack_stress_test.py --count 100 --interval 10.0 --csv logs/stage_t/t10_bench.csv` at 3-4 ft bench.
5. **Optional outdoor soak:** if 20 m LOS available, rerun `--count 50 --csv logs/stage_t/t10_outdoor.csv`.
6. **Ambient sweep:** `scripts/stage_t4_ambient.py --duration 60 --csv logs/stage_t/t10_ambient.csv` with vehicle powered off. Confirm noise floor at BW500.
7. **Write exit certification:** `logs/stage_t/ivp_t10_exit_soak.md` with the six-criterion check table filled in.

**Exit criteria (ALL required, HARD gate — this is the gate):**
| # | Criterion | Target | Rationale |
|---|-----------|--------|-----------|
| 1 | First-try ACK | ≥ 85% pooled N ≥ 100 at 3-4 ft | Round-2 council target; up from T1's 6.1% |
| 2 | Fail-after-3-retries | ≤ 1% | Robustness floor; T1 baseline was 1.8% |
| 3 | Mean successful-command latency | < 2 s | Operator responsiveness (F fix target) |
| 4 | Station `rx_count` continuity | No stalls ≥ 30 s across full soak | Silent-RX regression detector (T2/T3 canary) |
| 5 | CRC errors | 0 | Link quality floor (T1 baseline was 0) |
| 6 | T4 ambient re-run at BW500 | Noise floor ≥ 9 dB above sensitivity | Interference check at wider channel |

If any misses, Stage T does NOT exit. Bisect via per-IVP commits, reopen the relevant IVP.

**Post-T10 stage-wrapup (not a gate, just discipline):**
- Update `docs/IVP.md` with BOTH diagnostic (T1-T4) AND fix (T5-T10) sections. Protected file — ask user permission before editing.
- Update `CHANGELOG.md`, `docs/PROJECT_STATUS.md`, `AGENT_WHITEBOARD.md` per session-end checklist.
- Commit `docs/plans/STAGE_T_FIX_PLAN.md` as canonical in-repo companion to the `.claude/plans/` working file.

---

## 4. Files changed summary

**Firmware:**
- `src/flight_director/mission_profile_data.h` — `kDefaultRocketRadioConfig` `bandwidth_khz` (125 → 500), `nav_rate_hz` telemetry rate (5 → 10), plus `ROCKETCHIP_STAGE_T6_BW/_NAV_HZ` `#ifdef` block for sweep.
- `src/active_objects/ao_telemetry.cpp` — line 654 magic-number extraction (`kAckRetryTimeoutMs = 500U`), new COP-1 state machine + CLCW handling (T8).
- `include/rocketchip/telemetry_encoder.h` — nav packet size 54 → 58 B, CLCW offset/length constants, format version bump.
- `src/telemetry/telemetry_encoder.cpp` — CLCW read/write in nav payload tail.
- `src/drivers/rfm95w.cpp` — only if T5 finding demands a driver fix; otherwise untouched.

**Docs (at stage exit, NOT per-IVP):**
- `docs/IVP.md` — **both** Stage T diagnostic section (T1-T4) AND fix section (T5-T10) written in one pass at T10. **NOTHING WE'VE DONE GETS LOST** — T1-T4 data is preserved in `logs/stage_t/` already and will be documented in IVP.md as the diagnostic phase of Stage T. Protected file — requires explicit user permission to edit.
- `CHANGELOG.md` — per-IVP entries during execution.
- `docs/PROJECT_STATUS.md` — Stage T exit entry at T10.
- `AGENT_WHITEBOARD.md` — Stage T status line updates per-IVP.
- `logs/stage_t/ivp_t5_findings.md` — T5 root cause note.
- `logs/stage_t/ivp_t6_sweep.csv` — sweep data.
- `logs/stage_t/ivp_t10_exit_soak.md` — exit certification.

**Tests / scripts:**
- `scripts/stage_t6_sweep.py` (new) — BW sweep harness.
- `scripts/ack_stress_test.py`, `stage_t_run.py`, `stage_t_summarize.py`, `stage_t4_ambient.py` — reused as-is.

---

## 5. Verification (end-to-end)

1. **Host unit tests** — CCSDS encoder covers new 58 B packet format and CLCW round-trip; `AO_Telemetry` COP-1 state transitions (seq advance, retransmit trigger, window full).
2. **Builds:** all 4 (vehicle bench/flight, station bench/flight) clean at every IVP.
3. **Bench sim:** unchanged per IVP (unless T8 changes expected firmware logs — update regex if needed).
4. **`t` CLI canary** — confirmed after every flash.
5. **LL-32/LL-35 audit checklist** — signed off during T8 review.
6. **T10 exit soak** — ack_stress N ≥ 100 meets all six exit criteria.

---

## 6. Council verdict reference

- Round 1 (`logs/stage_t/fix_council_transcript.md`): picked A (2 Hz nav). User rejected.
- Round 2 (`logs/stage_t/fix_council_round2_transcript.md`): consensus B-500 primary / F secondary / E tertiary. Rejected A, C, D, SF6, SF5.

---

## 7. Risks and mitigations

| Risk | Mitigation |
|------|-----------|
| BW500 has ~6 dB worse sensitivity; operational range shrinks | T6 mandates T4 ambient re-run at BW500. Link budget at 2 km still has ~39 dB margin per round 2 analysis; degrades to ~19 dB at 4-5 km degraded-conditions. |
| T5 audit finds no single root cause | Fall back to "full power-on reset between config changes" operational rule. T6-T10 carry forward unchanged. |
| T8 COP-1 introduces new silent-RX regression | Post-flash canary catches immediately; revert and consult T5 findings. |
| Faster retry (T7) saturates AO_Radio event queue | Checked via station `rx_count` continuity. If observed, fix queue depth or backpressure — do NOT raise retry timer. |
| Station and vehicle BW mismatch at flash → zero RX | Sweep script flashes BOTH; canary fails loud if mismatched. |
| T9 MAVLink retest invalidates CCSDS-default assumption | T9 is soft-gate; documented as post-Stage-T design review trigger, does not block exit. |
| BW500 catches new interferer not present on BW125 (4× wider channel) | T6 mandatory T4 ambient re-run at winner BW. |
| IVP.md never gets updated with Stage T content | T10 explicitly gates on IVP.md stage section written. Diagnostic data in `logs/stage_t/` is preserved regardless. |

---

## 8. Out of scope

- Telemetry-rate reduction (user vetoed; `nav_rate_hz = 10` stays).
- FDD / separate station→vehicle frequency (overkill per round 2).
- SF6 (breaks variable-length protocol), SF5 (unsupported).
- MAVLink-over-air as primary protocol (T9 informational only; station-side MAVLink-to-QGC parsing already wired, unchanged).
- Flash-log rate decoupling from telemetry rate — orthogonal to Stage T; flash log can run faster independently if desired, not part of this plan.
- Driver refactors beyond what T5 root cause demands.
- Any combination/bundling of IVPs. Sequential only.

---

## 9. Critical files for implementation

- `src/flight_director/mission_profile_data.h`
- `src/active_objects/ao_telemetry.cpp`
- `src/drivers/rfm95w.cpp`
- `include/rocketchip/telemetry_encoder.h`
- `src/telemetry/telemetry_encoder.cpp`
- `scripts/stage_t6_sweep.py` (new)

---

*Plan file: `C:\Users\pow-w\.claude\plans\shimmering-twirling-thimble.md`*
*Companion docs copy (at execution): `docs/plans/STAGE_T_FIX_PLAN.md`*
