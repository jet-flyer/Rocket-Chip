# Stage T Fix Plan — B-500 + Retry Compression + COP-1

**Status:** Ready to execute. IVPs T1-T4 (diagnostic) complete and committed; T5 (partial — buffer overrun fix landed, silent-RX remains) committed; **T5.5 added 2026-04-19** after first T6 sweep attempt confirmed that compile-flag-gated `RadioConfig` changes reliably break station RX (T2/T3/T6-C1 all failed the same way); T6 reworked to use T5.5's runtime-push mechanism instead of per-config rebuilds; T7-T10 unchanged.
**Author:** Claude + council round 2 (NASA/JPL, ArduPilot, Rocketeer, Cubesat) — 2026-04-19. T5.5 amendment added 2026-04-19 at user direction (runtime radio config push is both a Stage T workaround AND an operational feature for ground-commanded link tuning).
**Scope:** Vehicle + station firmware. Target **telemetry rate (`nav_rate_hz` field in `RadioConfig`)** stays at 10 Hz on the air.
**Non-scope:** Not MAVLink-over-air (T9 is informational only). Not FDD. Not SF6/SF5. Not a telemetry-rate reduction. Not in-flight config changes (T5.5 is IDLE/DISARMED only). Not adaptive power or auto-protocol switching (deferred to future stage).
**Mirror copies:** `docs/plans/STAGE_T_RADIO_DIAGNOSTICS.md` is the completed diagnostic plan. `docs/plans/STAGE_T_FIX_PLAN.md` is the canonical in-repo companion to this working file — keep both synced.

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
| T5 | Diagnose T2/T3 silent-RX regression | `rfm95w.cpp:145-174`, `ao_radio.cpp` | **Hard** (gates T8) | Done (partial — buffer fix landed, root cause unresolved) |
| T5.5 | **Runtime radio config push** (scan feature split out to its own future IVP) | `ao_telemetry.cpp`, `ao_radio.cpp`, `rc_os_commands.cpp`, `rc_os_dashboard.cpp`, `ws2812_status.*`, `telemetry_encoder.*` | **Hard** (gates T6 rework) | 3-4 sessions impl + 30 min soak |
| T6 | RF BW sweep — pick winner (via T5.5 push, NOT compile-flag rebuilds) | (none — runtime-only) | **Hard** (gates T7/T10) | ~30 min bench |
| T7 | Retry timer 3000 → 500 ms at winner | `ao_telemetry.cpp:649-668` | **Hard** | ~15 min bench |
| T8 | CCSDS COP-1 + CLCW | `telemetry_encoder.h`, `ao_telemetry.cpp:127-150` | **Hard** | 2-3 h impl + 30 min soak |
| T9 | MAVLink retest at winner BW | runtime protocol-switch (via T5.5 if extended) or `mission_profile_data.h:93` flag | **Soft** (informational) | ~15 min bench |
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

**T5 outcome (2026-04-19, post-bench):** Audit committed as `logs/stage_t/ivp_t5_findings.md`. Real buffer-overrun bug found and fixed (`RadioTxEvt.buf`/`RadioRxEvt.buf` 128→256 B). But the silent-RX pathology **persists** — first attempt at T6 sweep confirmed it: `ROCKETCHIP_STAGE_T6_BW` flag (a single-value constexpr change) broke station RX the same way T2/T3 did. **Compile-flag-gated radio config changes are not viable for Stage T.** User proposed a runtime config-push feature (T5.5 below) that both unblocks the sweep AND lands a real operational feature.

### IVP-T5.5 — Runtime radio config push + channel-find + WS2812 sweep polish

**Purpose.** Station commands vehicle to change LoRa config (`bw_khz`, `nav_rate_hz`, `sf`, `cr`, `power_dbm`) at runtime via a new MAV_CMD, using the existing tracked-command ACK path. Channel-find CLI command lets station discover an out-of-sync vehicle's current config. Dashboard shows actual-vs-expected config, silence-based mismatch warning, and a KITT/Cylon sweep animation on the RSSI LED bar while searching.

This is both a **Stage T enabler** (unblocks T6 BW sweep without per-config firmware rebuilds, bypasses the T5-class silent-RX bug) AND a **permanent operational feature** (ground-operator radio tuning, headless field use).

**Preliminary council smell-test (2026-04-19)** — NASA/JPL, ArduPilot, Cubesat, Rocketeer panel reviewed the four base techniques at premise-level (without seeing the implementation plan details). Verdict: premise sound, revisions mechanical-not-structural. Corrections folded into the design below:
- **A** (set/ACK/swap): ⚠ revise — apply must hook to **TxDone IRQ**, not fixed timer (MER-class bug — last bytes of ACK transmit on new config, ground never decodes). Revert must be **symmetric** — vehicle self-reverts on silence, not just station. Flight-state validation at **apply**, not just at receive (race during ARM transition).
- **B** (silence detector): ⚠ revise — threshold is **packet-count-based**, not wall-clock (ExpressLRS/GomSpace/SatNOGS practice; time-based is a UX lie at low rates).
- **C** (channel-find): ✓ accept — keep passive (RX-only, no TX during scan), bounded linear sweep of whitelist.
- **D** (config echo in downlink): ⚠ revise — add explicit **version/extension marker** (MAVLink v2 extension-field pattern or CCSDS secondary-header flag). Without it, old-station meeting new-vehicle produces phantom mismatches.
- **Additions from panel:** explicit `QUERY_RADIO_CONFIG` command (synchronous probe after scan), and "config just changed" flag in first nav packet on new config (UX confirmation of transition).

Systems flagged for study: ExpressLRS failsafe + binding, MAVLink v2 extension fields, DSN bit-rate change procedure, GomSpace NanoCom link-monitor, CCSDS secondary-header flag pattern. Transcripts to save to `logs/stage_t/t5.5_smell_test_transcript.md` on commit.

**Correctness council review (2026-04-19 second session)** — same panel, fed the full amended plan. Verdict: **CONDITIONAL GO**. Four load-bearing edits required before implementation starts (folded below); the rest are annotations. No panelist dissents. File:line refs spot-checked and confirmed accurate. SX1276 setter audit still needed as implementer task.

**Load-bearing finding from correctness council:** The T5 report flagged reinit-recovery path at `ao_radio.cpp:148-152` as a latent bug. Council elevated to **T5.5 prerequisite**. If `kTxFailReinitThresh` fires during/after a config push, radio snaps back to compile-time defaults (from `configure_modem()` at `rfm95w.cpp:112-132`) while `RuntimeRadioConfig` still says NEW → silent divergence with no visible indication. **Extract the apply block at `ao_radio.cpp:277-291` into a helper called from both `RadioAo_initial` AND the reinit recovery path** before any runtime config-push code lands. See "§T5.5 prerequisites" below.

**Correctness council verdict table (summary; edits folded inline below):**
| # | Topic | Verdict | Required edit |
|---|---|---|---|
| 1 | Implementation plan correctness | ⚠ revise | Reinit-recovery fix + whitelist + setter audit |
| 2 | Second-command-during-revert race | ⚠ revise | Reject with visible message, don't queue |
| 3 | Power-change-order | ⚠ revise | Power LAST, ±6 dB max per command |
| 4 | NVM persisted-config fallback | ✗ defer | Boot-restore of compile-time default suffices |
| 5 | Packet-count thresholds | ⚠ revise | Formula `max(6, ceil(1.5 × nav_hz))` |
| 6 | Auto-reconfigure mode | ✗ defer | Not in T5.5; future IVP |
| 7 | WS2812 MODE_SWEEP | ✓ accept | Keep — safety-class visibility, ~70 LOC |
| 8 | Commit gate order | ⚠ revise | 7 gates, smallest-increment first |
| 9 | File:line references | ✓ accept | Spot-checks confirmed accurate |

**§T5.5 prerequisites (must land before any SET_RADIO_CONFIG code):**
1. **Reinit-recovery reapplies RuntimeRadioConfig.** Extract `ao_radio.cpp:277-291` into `ao_radio_apply_runtime_config(RadioAoState& s)` helper. Call from (a) `RadioAo_initial` (existing), (b) the reinit recovery branch at `ao_radio.cpp:148-152`. Fixes the latent bug where `kTxFailReinitThresh` recovery silently snaps radio back to compile-time defaults. Hard gate: instrument `kTxFailReinitThresh` trigger + verify RuntimeRadioConfig survives via GDB before writing any T5.5 code.
2. **Whitelist validation in dispatcher.** `ao_telemetry.cpp` MAV_CMD_USER_2 handler must compare the incoming config tuple against the `radio_config_table.h` whitelist. Reject unknown combos with denied-ACK rather than blindly applying. Prevents arbitrary BW/SF/CR combos (e.g., SF12/BW500 which is nonsensical) from bricking the link.
3. **Station command state machine refuses concurrent operator input.** During SET → ACK → apply → LOS-watchdog window, station CLI rejects further `r`/`c`/`q` commands with "config change in progress — wait N packets." Prevents the second-command-during-revert race. Single state variable `s_radio_cmd_in_progress` on station CLI dispatch.
4. **Packet-count threshold formula.** Both station LOS watchdog and vehicle symmetric-revert thresholds scale with nav_rate_hz: `threshold_packets = max(6, ceil(1.5 × nav_hz))`. At nav_hz=2 → 6 packets. At nav_hz=5 → 8. At nav_hz=10 → 15. Station uses this value; vehicle uses `max(15, ceil(3 × nav_hz))` so vehicle window is always longer than station's (vehicle is final fallback).

**Scope constraints (user-set + smell-test-corrected):**
1. Vehicle accepts `CMD_SET_RADIO_CONFIG` only in **IDLE or DISARMED** phase. **Validate flight state BOTH at receive AND at apply** (closes the ARM-during-pending-apply race from smell-test finding A.3).
2. **Protocol — revised per smell-test A.1/A.2:**
   - Station sends on OLD channel → vehicle ACKs on OLD → vehicle **hooks apply to TxDone IRQ** (waits for ACK to physically leave antenna per SX1276 `RegIrqFlags.TxDone`, not a fixed wall-clock timer). TxDone-watchdog safety timer ~200 ms as backstop if IRQ never fires.
   - After TxDone: vehicle reconfigures radio (setters only, no reinit). Updates `RuntimeRadioConfig` atomically.
   - Station sees ACK on OLD, arms LOS watchdog based on **expected-packet count** (not wall-clock — per smell-test B). Default: 6 missed expected packets at new rate → revert.
   - Station switches its radio to NEW via identical setter path.
   - If station receives first nav packet on NEW within watchdog → LOS-confirmed, commit.
   - **Symmetric revert (per smell-test A.2):** vehicle also caches OLD config and self-reverts if it doesn't receive a valid packet on NEW within its own bounded window (12-20 expected-packets count, tuned higher than station's so station gives up first).
3. **First-nav-on-new-config flag (smell-test addition):** after vehicle reconfigures, the first nav packet emitted on the new config sets a "config just changed" flag bit. Station uses this as explicit UX confirmation that the transition physically happened — distinguishes "I received a packet, and it was from the intentional change" from "I received a packet, maybe my own old config is still valid somehow."
4. **Silence-based mismatch detector (revised per smell-test B):** station tracks **consecutive missed expected packets** relative to the current `nav_rate_hz`. Threshold: 8 missed → dashboard warning row; 20 missed → link-lost state. Scales automatically across 2/5/10 Hz rates. Existing `handle_link_quality` at `ao_radio.cpp:315-332` retains its time-based kLinkLostMs/kLinkGapMs for backward compatibility but the new mismatch flag uses packet count.
5. **Manual channel-find (`c` key) — DEFERRED per user 2026-04-19 to post-Stage-T IVP.** Channel-find is a recovery-from-mismatch feature. It's NOT needed to unblock T6 (we can reset to compile-time default if station-vehicle desync happens — symmetric revert + boot-restore suffices for bench testing). Manual push + symmetric revert handles the expected T6 testing cases. The T5.5 SX1276 research stands — CAD-accelerated scan design is sound — but the feature waits until after Stage T exit. Tracked as a future IVP in the whiteboard "Medium" section.
6. **Scan list / whitelist canonical location — revised per smell-test C + correctness council edit #2:** config whitelist lives in a new `include/rocketchip/radio_config_table.h` header as a const array of valid `{bw, nav_hz, sf, cr, power}` tuples. Used by the SET dispatcher (rejects unknown combos with denied-ACK per correctness council) and, in a future IVP, by the deferred channel-find scanner. Markdown doc (`docs/RADIO_TELEMETRY_STATUS.md`) **references the header**, not vice versa. Code is authoritative; doc is derived. Prevents the "doc drifts from constants" problem.
7. **Vehicle nav-packet embeds current config (D) — revised per smell-test D:** append 4 bytes (`bw_khz` uint16, `sf_nav_packed` uint8, `cr_power_packed` uint8) to nav payload, grows 54 → 58 B. **Version/extension marker required** — use a new APID (`kApidNavWithConfig = 0x101`) distinct from the existing `kApidNav = 0x100`. Old station decodes APID 0x101 as "ignore, unknown" (graceful). New station decodes either. CCSDS primary-header APID is already the versioning field, no new bit needed.
8. **QUERY_RADIO_CONFIG command (smell-test addition):** new MAV_CMD_USER_3 (next ID after USER_2 for SET) that solicits a synchronous current-config reply from the vehicle. Used after a channel-find scan hit for explicit confirmation. Handler path reuses the same ACK plumbing as SET. Cheap to add, high UX value.
9. **WS2812 sweep polish (`MODE_SWEEP`):** KITT-style single-LED back-and-forth animation on station's RSSI bar during channel-find and during LOS watchdog window. Color-coded-by-BW is a future theme (not this IVP).
10. **Auto-reconfigure mode** (for correctness-council review, NOT in initial protocol spec): `cmd_radio_auto r <config>` — sanity-check link first (if healthy skip sweep) → sweep if silent → send SET_RADIO_CONFIG on found channel → station switches → LOS-verify. Pre-check-then-sweep behavior. Split out for the correctness council to chew on separately.

**Not in scope for T5.5 (broken out into future IVPs):**
- **Manual channel-find / scan feature (`c` key).** Broken out per user direction 2026-04-19 into its own future IVP (post-Stage-T). Rationale: substantial implementation work with its own design surface; the SX1276 CAD research turned up enough capability to warrant dedicated design time. T5.5's symmetric revert + boot-restore fallback handles expected bench mismatches without needing scan. CAD-accelerated scan design (≤10 configs, ~1-2 s worst-case via CAD) already specified here for future use.
- **Auto-reconfigure mode** (sanity-check + scan + SET + verify in one press). Depends on channel-find. Rolls forward with scan IVP.
- **NVM-persisted known-good config fallback.** Correctness council declined — boot-restore of compile-time `kDefaultRocketRadioConfig` is adequate.
- In-flight config changes (never accepted during ARMED/BOOST/etc. — enforced by gate at receive AND apply).
- Adaptive power / auto-protocol switching tied to range (future stage).
- Bidirectional config negotiation (station always commands, vehicle always obeys).
- Multi-station / contested-spectrum scenarios.
- WS2812 color-coded-by-BW (future theme, separate IVP).

**Second-command race, power-change-order, and packet-count-threshold formula are IN scope for T5.5** — folded into the design above per correctness council edits.

**Code touchpoints (from Phase 1 exploration):**

*Vehicle command dispatch:*
- `src/active_objects/ao_telemetry.cpp:221-246` — add new switch cases for `MAV_CMD_USER_2 = 31011` (SET_RADIO_CONFIG) and `MAV_CMD_USER_3 = 31012` (QUERY_RADIO_CONFIG). Handler extracts 5 config fields from `command.param1..param5` (MAVLink COMMAND_LONG has 7 float params; use `lroundf()` for integer packing per smell-test A.7 — document cast convention in handler + unit tests for boundary values 125/250/500/7/8/9/10/11/12).
- Flight-state gate: before dispatching, check vehicle's current flight phase via new `bool AO_FlightDirector_is_ground_state()` helper added to `ao_flight_director.h`. Check applies BOTH at receive (dispatch time) AND at apply (after TxDone) to close the ARM-during-pending-apply race.
- ACK: reuse existing `s_pending_ack` path at `ao_telemetry.cpp:127-150`.
- **TxDone-keyed apply (per smell-test A.1):** pending-config buffer stored in AO state. After queuing ACK, AO_Radio monitors `RegIrqFlags.TxDone` on its next tick(s); when TxDone fires for the ACK, AO posts `SIG_RADIO_CONFIG_APPLY` internally. **Backstop safety timer (~200 ms)** via QTimeEvt in case TxDone IRQ never fires — log and abort the config change. Watchdog guard, not the primary trigger.

*Vehicle radio reconfigure:*
- New `SIG_RADIO_CONFIG_APPLY` signal in `rocketchip/ao_signals.h`. Event carries the new config values.
- New `RuntimeRadioConfig` mutable holder in `RadioAoState` (`ao_radio.h`) — defaults to `kDefaultRocketRadioConfig` on boot; mutated by the apply handler.
- **Cached OLD config (per smell-test A.2):** before mutating `RuntimeRadioConfig`, snapshot the current config into `RadioAoState.prev_config` for self-revert use.
- `AO_Radio` handles `SIG_RADIO_CONFIG_APPLY`: **re-validates flight state** (reject if no longer IDLE/DISARMED — per smell-test A.3). Transitions radio to Standby, calls existing setters (`rfm95w_set_bandwidth`, `rfm95w_set_spreading_factor`, `rfm95w_set_coding_rate`, `rfm95w_set_tx_power` — all runtime-callable per Phase 1 findings), calls `s.scheduler.set_rate(new_nav_hz)` at `radio_scheduler.h:68`, returns to kRxWindow or kTxActive as appropriate. Sets "config just changed" flag bit for the next nav packet (per smell-test UX addition).
- **No reinit of the radio.** That path is what has been breaking T2/T3/T6-C1. Setters only.
- **Symmetric vehicle-side revert (per smell-test A.2):** vehicle maintains its own "packets since config apply" counter when on new config. If count reaches ~12-20 expected packets without a valid RX from station (CRC ok, source-ID match), vehicle reverts to `prev_config`. Uses same setter-only apply path. Logs the revert. Station's revert window is shorter (~6 packets) — station gives up first visually; vehicle is the final fallback. If both end up on OLD after vehicle's revert, station's ongoing scan/manual-find picks up the re-established link.

*Vehicle nav-packet config echo (per smell-test D revision):*
- New CCSDS APID `kApidNavWithConfig = 0x101` in `include/rocketchip/telemetry_encoder.h`, distinct from existing `kApidNav = 0x100`. Old stations decoding 0x101 packets treat as unknown and skip — graceful cross-version.
- `src/telemetry/telemetry_encoder.cpp` `CcsdsEncoder::encode_nav()` emits APID 0x101 when `RuntimeRadioConfig` diverges from `kDefaultRocketRadioConfig` OR always (TBD — simplest: always emit 0x101 for new firmware, treats 0x100 as "legacy station-decode compatibility"). Appends 4 bytes to payload: `bw_khz` u16, `sf_nav_packed` u8 (SF in upper 4 bits, nav_rate_hz in lower 4), `cr_power_packed` u8 (CR in upper 4 bits, power_dbm in lower 4). Also sets the "config just changed" flag bit (smell-test UX addition) in reserved space for the first packet after apply.
- `include/rocketchip/telemetry_encoder.h:69-82` — bump packet-length constant for 0x101 to 58 B, add `kConfigEchoOffset`, `kConfigEchoLen = 4`, `kApidNavWithConfig`. Keep 0x100 constants at 54 B for backward compat.
- Decoder `ccsds_decode_nav` — dispatch on APID. For 0x101, also read config echo and surface to `TelemetryState`. For 0x100, existing path unchanged.

*Station CLI:*
- `src/cli/rc_os_commands.cpp` — new `cmd_radio_config` handler. Bind to `r` (free per Phase 1). Takes args or prompts operator for bw/nav/sf/cr/power. Calls `AO_Telemetry_send_tracked_command(MAV_CMD_USER_2, p1, p2, p3, p4, p5)` — need to extend tracked-command signature to take multiple params (currently single `p1`).
- Channel-find CLI handler (`c` key) **DEFERRED** to future IVP per user direction 2026-04-19.
- New `cmd_query_config` handler. Bind to `q` if free, else next available letter. Sends MAV_CMD_USER_3 QUERY_RADIO_CONFIG and displays the reply (synchronous probe, per smell-test addition).
- **Station command state machine (per correctness council edit #3):** single `s_radio_cmd_in_progress` flag set on SET send, cleared on LOS-committed or LOS-timeout-reverted. While set, station CLI rejects further `r`/`q` with "config change in progress — wait N packets." Prevents the second-command-during-revert race.
- **Power-change-order discipline (per correctness council edit #3 item 3):** when SET command contains multiple param changes, station and vehicle apply power_dbm LAST in the setter sequence — `rfm95w_set_bandwidth`, `rfm95w_set_spreading_factor`, `rfm95w_set_coding_rate`, `scheduler.set_rate()`, *then* `rfm95w_set_tx_power`. Additional restriction: any single SET command limited to ±6 dB power change from current. Larger changes rejected with denied-ACK — forces operator to make multi-step changes explicitly visible.
- Station-side `SIG_RADIO_CONFIG_APPLY` for its own radio reconfigure on ACK receipt to SET.
- LOS watchdog: QTimeEvt armed on ACK sent, disarmed on first successful packet on new config with "config just changed" flag. **Packet-count-based**, not wall-clock — station tracks "expected packets at new rate" and reverts after ~6 missed. Per smell-test B revision.

*Dashboard:*
- `src/cli/rc_os_dashboard.cpp:200-240` — insert two new rows between the "Pkts:" and "Seq:" lines:
  - "Radio: BW<bw> <nav>Hz SF<sf> CR<cr>" — current station config, shown alongside vehicle's echoed config (e.g., "Radio: BW500 10Hz SF7 CR5  |  Vehicle: BW500 10Hz SF7 CR5"). Yellow highlight if they differ.
  - "[status banner]" — conditional, specific per smell-test Rocketeer feedback ("be loud, be specific"):
    - Empty when link healthy and configs match.
    - "Missed 8 packets — possible config mismatch" when consecutive-missed-packet count exceeds warning threshold (packet-count-based per smell-test B, not wall-clock).
    - "Searching: BW<x> <y>Hz (3/10)…" during channel-find, showing current scan position.
    - "Config push unconfirmed — reverting in 3 packets" with live countdown during LOS watchdog window.
    - "Config committed: BW<x> <y>Hz" briefly after successful transition (then fades).

*WS2812 MODE_SWEEP:*
- `src/drivers/ws2812_status.{h,cpp}` — add `WS2812_MODE_SWEEP`. State: current LED position + direction. Update tick function identical shape to `update_blink()` at `ws2812_status.cpp` per Phase 1. Approx ~40 lines of driver code.
- `src/active_objects/ao_radio.cpp` `handle_rssi_bar()` at ~line 338 — when a shared "searching" flag is set, emit MODE_SWEEP instead of MODE_RSSI_BAR.

*New signals:*
- `SIG_RADIO_CONFIG_APPLY` (vehicle: deferred radio reconfigure on ACK TxDone; station: reconfigure on ACK receipt)
- `SIG_RADIO_CONFIG_REVERT` (both: triggered when LOS watchdog hits its packet-count threshold, or backstop safety timer fires)
- `SIG_RADIO_QUERY_CONFIG` (station: synchronous current-config probe)
- `SIG_RADIO_CHANNEL_FIND` — DEFERRED (part of post-stage channel-find IVP)
- All static events per LL-35.

**Protocol documentation:**
- `docs/RADIO_TELEMETRY_STATUS.md` — new section: "Runtime radio config push protocol" with:
  - Command format (MAV_CMD_USER_2 SET + MAV_CMD_USER_3 QUERY parameter layouts, with cast convention for integer-in-float packing)
  - CCSDS APID scheme (0x100 = legacy nav, 0x101 = nav-with-config-echo)
  - State machine diagram (ASCII) for station and vehicle during reconfigure: command → ACK queued → TxDone → apply → revert path
  - LOS watchdog sequence: packet-count threshold, station shorter window than vehicle (asymmetric on purpose — station gives up first visually, vehicle is final fallback)
  - Symmetric revert: vehicle caches prev_config, self-reverts on silence
  - Reference to scan list in `include/rocketchip/radio_config_table.h` (code authoritative, doc derived per smell-test C revision)
  - Silence-mismatch detector: packet-count-based, scales with nav_rate_hz
  - "Future: auto-reconfigure mode" subsection (placeholder for correctness-council review)
- Mirror relevant protocol spec in the T5.5 section of `docs/plans/STAGE_T_FIX_PLAN.md` when committing.

**Pass (HARD) — five conditions:**
1. **SET round-trip works:** operator presses `r`, enters desired config. Station TX on old → vehicle ACK received → TxDone fires → vehicle applies → first nav packet on new config received within the computed packet-count watchdog window (`max(6, ceil(1.5 × nav_hz))`) with "config just changed" flag set → dashboard shows new config committed. Vehicle RX still works (post-switch DISARM command ACKs at same rate as pre-switch).
2. **Symmetric revert works:** deliberately induce station-side failure (e.g., operator cancels before switching own radio). Vehicle reverts to prev_config after its own longer window (`max(15, ceil(3 × nav_hz))` packets). Boot-restore to compile-time default also verified as final fallback.
3. **Flight-state gate works at BOTH phases:** (a) command sent during ARMED → rejected with denied-ACK at receive. (b) command sent during IDLE, but ARM command arrives during pending-apply → apply handler re-validates and aborts config change, system stays on old config.
4. **Whitelist + concurrent-command guards:** (a) SET with a config tuple not in `radio_config_table.h` → rejected with denied-ACK. (b) SET sent while `s_radio_cmd_in_progress` is true → rejected with "config change in progress" without touching the pending operation. (c) SET with >6 dB power change → rejected with denied-ACK explaining the step limit.
5. **QUERY works:** operator presses `q`. Vehicle responds with current config payload. Station dashboard shows explicit authoritative current config.
6. **Reinit-recovery preserves RuntimeRadioConfig (T5.5 prerequisite gate):** force `kTxFailReinitThresh` via fault inject, verify GDB shows `RuntimeRadioConfig` survives with its most recent SET-applied values (not snapped back to `kDefaultRocketRadioConfig`).

**Canary (LL Entry 36 compliance):** post-flash on the T5.5 firmware, station `rx_count` advances within 60 s at default config. Then send a SET_RADIO_CONFIG with the same config as current (no-op equivalent) — confirms the round-trip plumbing works before attempting a real BW change.

**Diagnostics if surprising:**
- Station→vehicle command accepted but vehicle never applies: check `SIG_RADIO_CONFIG_APPLY` TxDone hook + backstop timer, verify no blocking call during apply.
- Vehicle applies but station never sees NEW-channel packet: check station's own radio-reconfigure path. Mirror problem.
- Station reverts constantly: LOS watchdog packet-count threshold may be too tight; increase from ~6 to ~10 expected packets. Document change in RADIO_TELEMETRY_STATUS.md.
- Flight-state gate accepts command when it shouldn't: verify `AO_FlightDirector_is_ground_state()` semantics AND that apply-time revalidation is wired.
- Nav-packet echo mismatched with vehicle's actual config: config-apply happened but `RuntimeRadioConfig` not updated atomically. Check ordering.
- Float→int cast mangles config values (125 becomes 124): verify `lroundf()` usage + unit-test boundary values 125/250/500/7/8/9/10/11/12.
- APID-0x101 packet ignored by old station firmware: expected (graceful cross-version). Confirm new station handles both 0x100 and 0x101.
- Vehicle reverts unnecessarily: symmetric-revert window too short. Widen to ~20 expected packets. Ensure station's revert window stays shorter (station gives up first visually, vehicle is final fallback).

**Council review sequence (two sessions):**
- **Smell-test (done 2026-04-19):** premise-level review of techniques A-D. Verdict: sound, revisions folded above. Transcript saved to `logs/stage_t/t5.5_smell_test_transcript.md`.
- **Correctness council (pending):** full plan review with specifics. Topics flagged by smell-test for this session: (1) second-command-during-revert race, (2) power-change-order discipline (power last-applied), (3) NVM-persisted known-good config fallback (power-glitch survival), (4) packet-count threshold tuning per nav rate, (5) auto-reconfigure mode design, (6) WS2812 MODE_SWEEP polish.

### IVP-T6 — RF bandwidth sweep (reworked to use T5.5 runtime push)

**Hypothesis unchanged:** Airtime ∝ 1/BW at constant SF. BW125 → ~140 ms; BW250 → ~70 ms; BW500 → ~35 ms. BW500 fits two TX cycles inside 100 ms RX window. Sensitivity cost ~6 dB vs BW125 — link budget at 2 km LOS still closes with ~39 dB margin (from T4).

**Key change from original T6:** NO compile-flag builds, NO per-config firmware rebuilds, NO reflashing between configs. Single firmware (with T5.5 active) stays on both boards for the entire sweep. Station uses `cmd_radio_config` (T5.5) to command vehicle into each config; station's own radio switches in tandem via T5.5's ACK-triggered reconfigure. Single flash, multiple configs.

**Sweep matrix (N=30 per config, 10 s inter-command, 3-4 ft — same as original):**

| Config | SF | BW (kHz) | `nav_rate_hz` | Purpose |
|--------|---:|---------:|--------------:|---------|
| C0 | 7 | 125 | 10 | Control — default config, 10 Hz rate (compare against T1 baseline 5 Hz) |
| C1 | 7 | 250 | 10 | Halfway point |
| C2 | 7 | 500 | 10 | **Primary candidate** |
| C3 | 7 | 500 | 10 | Repeat — confirms C2 isn't a lucky run |

**Per-config procedure (T5.5-based):**
1. Starting state: boards on known-good config (C0 — default).
2. Canary: `t` on both, confirm `rx_count` advances, `rx_crc_errors == 0`.
3. Run `scripts/ack_stress_test.py --count 30 --interval 10.0 --csv logs/stage_t/ivp_t6_c0.csv`.
4. For C1: press `r` on station, enter BW=250 nav=10. Verify T5.5 round-trip (ACK + LOS-confirm + dashboard shows new config).
5. Run ack_stress for C1. Repeat steps 4-5 for C2 and C3.
6. Final: command back to default, verify station and vehicle both revert cleanly.

**New script:** `scripts/stage_t6_sweep.py` — now a thin harness around ack_stress that drives `r` config changes via serial command injection on the station CLI (no reflashing). ~150 lines.

**Pass (HARD, two conditions):**
- First-try ACK ≥ 60% on C2 (BW500) AND jitter margin (100 ms window − measured TX airtime − measured clock drift over 30 s) ≥ 10 ms.
- C0 reproduces a baseline within T1 noise (if running at 10 Hz instead of T1's 5 Hz, compare packet-period ratio directly — NOT an exact 6.1% match).

**Mandatory follow-up: re-run T4 ambient at winner BW.** Use `scripts/stage_t4_ambient.py`. Pass: noise floor ≥ 9 dB above sensitivity at operational BW.

**Diagnostics if surprising:**
- C2 < 60% → council reconvenes. Don't proceed to T7 with a weak winner.
- C1 beats C2 → adopt BW250 (still fits, better sensitivity).
- T5.5 reconfigure-round-trip fails for a particular BW: T5.5 had a pre-existing bug; fix there, not here.

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
