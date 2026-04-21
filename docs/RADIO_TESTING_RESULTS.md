# Radio Testing Results

**Purpose:** Single aggregation point for every over-the-air radio
characterization test run on the RocketChip prototype. Tracks
first-try ACK, eventual ACK, CRC rate, ambient RSSI, and link-budget
margins across firmware revisions and radio configurations.

**Last Updated:** 2026-04-20

Raw per-run CSVs + vehicle/station logs live in `logs/stage_t/`. This
document summarizes them and preserves the narrative of what each run
was measuring.

---

## Bench fixture

- **Vehicle:** Adafruit Feather RP2350 HSTX + RFM95W FeatherWing.
  SWD probe attached for GDB flashing + state injection.
- **Station:** Adafruit Fruit Jam + RFM95W (SPI GP10/11/12/13; RST=6,
  IRQ=5). No probe — flashed via picotool programmatic BOOTSEL.
- **Geometry:** Bench 3-4 ft separation (`<1 ft` for T1 Run 1 control).
  Metal PC chassis partially in path. Antenna pointing varied per run.
- **Link characterization:** RSSI ~-40 to -50 dBm at 3-4 ft, SNR ~9 dB,
  noise floor -114 dBm (T4).

---

## Baseline (pre-Stage-T)

Stage 12A had an observed ~6% first-try ACK rate on station→vehicle
commands at SF7/BW125/CR4-5, 5 Hz nav. This was the trigger for Stage T
(renamed from Stage M 2026-04-18). Root cause hypothesis going in: TX
airtime (~140 ms) exceeds vehicle RX-window (~100 ms at 5 Hz), causing
near-deterministic collision.

---

## Stage T — Diagnostic Phase (T1–T4, complete 2026-04-19)

### T1 — Baseline repeatability + failure classification

| Metric | Value |
|---|---|
| Date | 2026-04-18 |
| Gate | Hard PASS |
| Config | SF7/BW125/CR4-5/20 dBm/nav 5 Hz, CCSDS |
| Runs | 5 × N=28-30 commands, 10 s interval |

**Pooled 3-4 ft geometry (Runs 2-5, N=114):**

| Outcome | Count | Rate |
|---|---:|---:|
| Total ACK | 39 | 34.2% |
| First-try ACK | 7 | **6.1%** |
| Retry 1 | 5 | 4.4% |
| Retry 2 | 13 | 11.4% |
| Retry 3 | 14 | 12.3% |
| Failed after 3 retries | 2 | 1.8% |
| No ACK received | 73 | 64.0% |

Confirmed Stage 12A's ~6% observation is repeatable (not a one-off).
First-try rate is the pass-critical metric for the fix phase — Stage T
target is ≥ 85%.

Artifacts: `logs/stage_t/t1_run[1-5].csv`, `t1_summary.md`.

### T2 — Cheat-mode sync ceiling

| Metric | Value |
|---|---|
| Date | 2026-04-19 |
| Gate | Hard PASS (pivot: host-side cheat, not firmware) |
| Intent | Perfect station→vehicle sync upper-bound |
| Result | First-try = **0%** at SF7/BW125 |

Host script tailed vehicle serial for `[STAGE_T] state TX->RXW` transitions,
fired `X` on the station `--delay-ms` after each. **Scored worse than
random** because station TX airtime (~140 ms at SF7/BW125 CCSDS) exceeds
the 100 ms vehicle RX window at 5 Hz nav — sync can't close it.

**Decisive finding: window geometry, not sensitivity or sync.** Any fix
must widen the RX window (lower nav rate) OR shrink TX airtime (higher BW
or smaller protocol overhead). Stage T chose the BW route.

Artifacts: `logs/stage_t/t2_cheat.csv`, `t2_summary.md`.

### T3 — MAVLink A/B (BLOCKED)

| Metric | Value |
|---|---|
| Date | 2026-04-19 |
| Gate | Soft — BLOCKED |

Attempted to rebuild both boards with
`ROCKETCHIP_STAGE_T3_MAVLINK=ON`. Station booted but received ZERO
packets at the decoder layer. Same failure mode as the T2 firmware-cheat
variant. **Compile-flag-gated RadioConfig changes reliably break station
RX.** Root cause not isolated; pattern tracked via T5 findings.

**This blocker motivated T5.5** — runtime config push via MAV_CMD_USER_2
bypasses the compile-flag breakage path entirely.

Artifacts: `logs/stage_t/t3_summary.md`.

### T4 — Ambient RSSI baseline

| Metric | Value |
|---|---|
| Date | 2026-04-19 |
| Gate | Hard PASS (reduced scope — CAD feasibility deferred) |
| Noise floor | **-114 dBm** at SF7/BW125 |
| Link margin at 3-4 ft bench | ~70 dB |

Vehicle powered off for 60 s of station-side `t` polling. No interferer
detected on-band. Link is not sensitivity-limited at bench range.

Original T4 Part 1 (SX1276 CAD confusion matrix) deferred — CAD-mode
driver changes too risky after T2/T3 firmware breakage pattern.

Artifacts: `logs/stage_t/t4_ambient.csv`, `t4_summary.md`.

### Council verdicts (both rounds)

Two council sessions (ArduPilot, NASA/JPL, Rocketeer, Cubesat + Professor/
Student auxiliaries):
- **Round 1:** picked Option A (drop nav to 2 Hz). User rejected — 10 Hz is
  target for boost-phase dashboard resolution.
- **Round 2:** consensus **B-500 primary** (SF7/BW500/CR4-5, keep nav 10 Hz),
  F secondary (retry 3000 → 500 ms), E tertiary (CCSDS COP-1).

Transcripts: `logs/stage_t/fix_council_transcript.md`,
`fix_council_round2_transcript.md`, `FIX_COUNCIL_REPORT.md`.

---

## Stage T — T5 / T5.5 (firmware prerequisite)

T5 diagnosed the compile-flag silent-RX bug → buffer-overrun fix landed
(`RadioTxEvt.buf` 128→256 B, IVP-T5). Silent-RX pathology on compile-flag
changes persists for unknown root cause; **T5.5 sidesteps it entirely**
via runtime SET_RADIO_CONFIG (MAV_CMD_USER_2).

T5.5 sub-IVPs 2a–2g landed 2026-04-19 to 2026-04-20:
- Reinit-recovery preserves runtime config (prereq)
- TxDone-keyed apply + backstop timer (2b)
- Station's own radio switches on accepted SET ACK (2c)
- Symmetric revert (vehicle cache + watchdog) (2d)
- Debounced flash persistence with CRC-16 validation (sub-persist)
- QUERY_RADIO_CONFIG with ACK config-echo tail (2e)
- APID 0x004 nav-with-config + dashboard row + `[CHANGED]` marker (2f)
- Station auto-revert with role-aware shorter threshold (2f part A)
- WS2812 KITT sweep during LOS-watchdog (2g)

**T5.5 is the prerequisite for T6** — single firmware stays on both
boards for the entire BW sweep; station CLI's `r` command cycles the
whitelist. Tracking: `logs/stage_t/t5.5_revalidation_list.md`.

---

## Stage T — Fix Phase (T6 onward)

### T6 — LoRa bandwidth sweep (in progress 2026-04-20)

**Procedure:** `scripts/stage_t6_sweep.py` — single firmware on both
boards, station's `r` key advances vehicle through runtime configs via
SET_RADIO_CONFIG (T5.5). ACK-stress N=30 @ 10 s per config.

**Pass criteria:**
- **First-try ACK ≥ 60 % at C2 (BW500/10Hz).**
- Jitter margin (100 ms window − measured TX airtime − 30 s clock drift)
  ≥ 10 ms at C2.
- C0 (control) reproduces T1 baseline within noise.
- Mandatory follow-up: T4 ambient re-run at winner BW.

| Config | BW (kHz) | nav (Hz) | first_try | eventual | mean_lat |
|---|---:|---:|---:|---:|---:|
| C0  | 125 | 5  | _pending_ | _pending_ | _pending_ |
| C0P | 125 | 10 | _pending_ | _pending_ | _pending_ |
| C1  | 250 | 10 | _pending_ | _pending_ | _pending_ |
| C2  | 500 | 10 | _pending_ | _pending_ | _pending_ |

Results artifact: `logs/stage_t/t6_summary.csv`.

### T7 — Retry timer 3000 → 500 ms (not started)

Follows T6 winner. Target: first-try ≥ 85 %, mean successful-command
latency < 2 s.

### T8 — CCSDS COP-1 + CLCW (not started)

Retransmit safety net via CLCW in nav payload tail.

### T9 — MAVLink retest at winner BW (informational)

Soft gate — validates CCSDS is not regressing vs MAVLink at winner BW.

### T10 — Stage T exit soak (not started)

Comprehensive 100-command soak at final config. Six exit criteria all
required: first-try ≥ 85 %, fail ≤ 1 %, latency < 2 s, `rx_count`
continuity, CRC 0, T4 ambient re-run clean.

---

## Appendix — column conventions

- `first_try`: ACK landed before the next outgoing command (i.e.,
  within the inter-command interval minus ~0.5 s).
- `eventual`: ACK landed at all (possibly after station-side retries).
- `denied`: vehicle's dispatcher rejected the command with
  `CmdAckResult::kDenied` (wrong flight state, whitelist miss, etc.).
- `no_ack`: no ACK arrived within the trailing 15 s drain window after
  the last send.
- `mean_lat_ms`: mean latency across `eventual` ACKs only.

---

## See also

- `docs/RADIO_TELEMETRY_STATUS.md` — architecture + protocol reference
- `docs/plans/STAGE_T_RADIO_DIAGNOSTICS.md` — diagnostic plan (T1-T4)
- `docs/plans/STAGE_T_FIX_PLAN.md` — fix plan (T5-T10)
- `.claude/plans/shimmering-twirling-thimble.md` — T5.5 working plan
- `logs/stage_t/` — all raw CSVs, logs, summary files, transcripts
