# L2-P5 Remediation Whiteboard — temporary; must be EMPTY when disposition closes

Companion to `L2P5_DISPOSITION_PLAN.md`. Working memory for the disposition
pass. **Not** a record of closed WNs (frozen findings stay frozen). **Not**
`AGENT_WHITEBOARD.md` (that is project-wide and has no end-of-life).

> Treat like an IRL whiteboard: **erase resolved rows**, don’t mark them done.
> A row’s continued presence means it still needs attention.

---

## Lifecycle

**This file must be empty when L2-P5 disposition closes** (Plan-3 /
`AUDIT_COVERAGE_CATCHUP_*`, before deleting the worktree). Non-empty at close
means a row never landed or never moved to a durable home.

When empty, **delete the file** (history is in git). Same close rule as the
walk WB (`L2P5_WALK_WHITEBOARD.md`, drained 2026-08-17).

---

## What goes here

Only things that would otherwise get lost **or** force an edit of a frozen pack:

- Pulled-forward WNs (e.g. W-2 overlap in an otherwise-DEFER cluster)
- DF items mid-flight (`L2P5_DISPOSITION_FINDINGS.md` is the durable log;
  this board is the live queue)
- Follow-ons revealed while remediating a WN (C.4a vs coincident)
- “Don’t forget” process (protected-file names still needed, IVP/doc tidy)

## What does NOT go here

| Item | Home |
|------|------|
| Frozen observation text | `L2P5_WALK_FINDINGS.md` / GWF / CW (never edit) |
| Per-WN REMEDIATE/ACCEPT/DEFER labels | `L2P5_DISPOSITION_LOG.md` when that sitting runs |
| Durable product skip (IVP-55, etc.) | `L2P5_DISPOSITION_FINDINGS.md` |
| Project-wide deferrals still true after L2-P5 | `AGENT_WHITEBOARD.md` |

**Rule of thumb: if it has a home, it goes home. If the home is frozen, land
the *action* here until the action is done, then erase.**

## Row format

```
### R-N — <one-line title>
**Surfaced:** <date> · <how>
**What:** <item + citations>
**Disposition target:** <where it lands when resolved>
**Blocking?** <no — disposition continues / yes — stalls>
```

---

## Rows

### R-1 — Parked `SensorSnapshot` (prod header removed)

**Surfaced:** 2026-08-20 · DF-001 / WN-045; owner: out of prod, not destroyed

**What:** Layout parked at
`docs/audits/l2p5_manual_walk/parked/sensor_snapshot.h`. Removed from
`include/rocketchip/` and from `test/test_data_model.cpp` sizeof assert.
IVP.md / ADVANCED_SETTINGS.md / RADIO_TELEMETRY_STATUS.md still mention
IVP-55 / “raw sensors” — those files are protected.

**Disposition target:** Erase when WN-045 is labeled in the log as parked
(not a live prod ICD). Protected-doc tidy when owner names those files.
**Blocking?** No

### R-2 — WN-001 / WN-002 `g_imu` header contract

**Surfaced:** 2026-08-20 · W-2 overlap; owner: fully address now (early-impl
cluster, pulled forward)

**What:** Rewrite `shared_state.h` `g_imu` comment to the observed model
(Core 0 boot init → `g_startSensorPhase` → Core 1 1 kHz + recovery re-init;
Core 0 CLI/cal still `icm20948_read`).

**Follow-on (do not bury in frozen WN text):** after handoff, Core 0 still
calls `icm20948_read(&g_imu)` from `cal_hooks.cpp` `cal_read_accel` and
`cli/rc_os_commands.cpp` `print_direct_sensors` **without** `core1_i2c_pause`.
Mag cal already uses the seqlock. That I2C race is the “correctness not
established” half of WN-002 — header alone cannot close it.

**Disposition target:** Erase this row when the header comment has landed.
Follow-on becomes **R-3** (or fold into R-3 now).
**Blocking?** No — header can land without the race patch.

### R-3 — Core 0 post-handoff `icm20948_read` vs Core 1 (WN-002 follow-on)

**Surfaced:** 2026-08-20 · writing the WN-002 contract

**What:** `cal_read_accel` and `print_direct_sensors` use `g_imu` on Core 0
after sensor phase start. Pause primitive exists for flash; not used here.
Options later: pause around those reads, or read seqlock like mag, or document
as accepted bench-only race.

**Disposition target:** Owner pick; then code or ACCEPT with safety one-liner
on the log. Not an agent-chunk UART item.
**Blocking?** No for continuing other buckets; yes for calling WN-002 *fully*
closed.
