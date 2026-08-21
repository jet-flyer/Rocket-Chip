# L2-P5 disposition findings

**Not walk IDs.** Frozen packs stay frozen (`WN-*` / `GWF-*` / `CW-*`). This file is for
issues that showed up **while disposing** and should not stay open, but are not
themselves a walk-row close-out.

IDs: `DF-NNN`, append-only. Owner labels here (or points at the WN that already owns it).

---

## DF-001 — IVP-55 raw sensor logging was skipped, not strongly deferred

**Surfaced:** W-5 / **WN-045** (`SensorSnapshot` public header, zero `src/` consumers).
**Not a substitute for WN-045.** That WN still owns keep/fold of the header.

### What IVP-55 was

Stage 6 stretch: toggle `SensorSnapshot` into log frames at native sensor rate
(up to 1 kHz IMU), Research Mode, per-sensor bitmask.
(`docs/IVP.md` table + placeholder body; `docs/ADVANCED_SETTINGS.md` row
“Raw sensor logging (IVP-55)” Status = Deferred.)

Stage 6 **did** ship IVP-49–54: FusedState → TelemetryState → PCM/PSRAM → flash
→ USB download. `SensorSnapshot` was the third IVP-49 ICD struct; only
`FusedState` and `TelemetryState` got a live path.

### What it did *not* become

- Not implemented in `src/`.
- Stage 17 diagnostic log tier (IVP-135a: Mahony, biases, raw+cal accel, RSSI, …)
  is a **different** planned payload. It does not mention `SensorSnapshot` and
  did not absorb IVP-55.
- Whiteboard PCM re-eval (**WN-059**) is log-*shape* vs Starcom, not “wire raw
  snapshots.”
- `docs/RADIO_TELEMETRY_STATUS.md` (2026-03-30) still says onboard log is
  “full ESKF state + raw sensors” — that line is **stale** vs what IVP-49–54
  actually log.

### Why this is a DF, not “already scheduled”

The IVP/advanced-settings row is a **placeholder deferral** (“stretch goal”)
with no named successor, no date, and no written reason that still holds after
TelemetryState+PCM became the log. A deferral that sits from Stage 6 through
L2-P5 without a stronger home is **skip-shaped**, not an active workstream.

**WN-045** still closes the *header in `include/rocketchip/`*. This DF closes
the *product* question: is IVP-55 still a real future, or do we stop implying
a wired raw snapshot (header, IVP table, advanced-settings, radio-status
prose)?

### Owner choice (2026-08-20)

**Park, don’t destroy.** Layout moved to
`docs/audits/l2p5_manual_walk/parked/sensor_snapshot.h` (recoverable for a
future IVP-55). Removed from `include/rocketchip/` and from
`test/test_data_model.cpp`. IVP.md / ADVANCED_SETTINGS / RADIO_TELEMETRY_STATUS
still name IVP-55 — protected; tidy when those files are named (rem WB **R-1**).

Erase rem WB **R-1** when WN-045 is labeled parked in the disposition log.

---

## How this file relates to W-2 / agent packs

Owner chunk still only **labels WN-\***. If a WN’s proposition is the same
as a Grok/Claude row, **dispose that WN now** and cite the agent IDs as extra
trust. Do not wait for chunk 2.

Agent-only P (no WN) still waits for Grok then Claude. Those chunks **re-scan**
disposed WNs first so they don’t re-fix a P the owner already closed.

### WN that already *is* the W-2 question (address in owner pass)

| WN | P | Agent overlap |
|----|---|---------------|
| **WN-002** | `g_imu` shared handle; header does not state single-writer-after-handoff (or pause exception). Correctness not established from the header alone. | **GWF-006** (handoff/exclusivity not on the header). **WN-001** is the *comment* on the same banner — separate label (trim vs rewrite). |

### Same *leaf*, different P — do **not** smuggle into the WN

| WN | WN’s P | Agent P (chunk 2–3 unless owner pulls) |
|----|--------|------------------------------------------|
| **WN-264** | Do pause *files* earn a module? | Fail-open / no nest: GWF-383/386, CW-B34-03/02 |
| **WN-042** | Is seqlock still the right *design*? (early-impl re-eval) | Dual-home of six atomics: GWF-007, CW-X4-09 |
| **WN-045** | Does `SensorSnapshot` exist in prod? | Units/brief lies: GWF-069/070; no producer: CW-X4-10 (same P as WN-045 — cite on WN-045) |

### No WN (stay in agent remediations; re-scan after owner labels)

UART `volatile` / `gps_uart_reinit` NVIC (GWF-135, CW-B09-04/03);
cal session both cores (GWF-222, CW-B18-01/B36-02);
`g_bestGpsValid` second writer (Grok station_fault_inject);
`rc_log` ring Core1 vs drain (Grok on `rc_log.cpp`).
