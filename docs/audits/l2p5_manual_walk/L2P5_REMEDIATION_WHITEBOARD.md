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

### R-8 — `config.h` filename is free

**Surfaced:** 2026-08-22 · sitting 7. Owner: original jobs already live in
dedicated files; delete the grab-bag; name is open for a *real* compile-time
config later (not this pass).

**What:** `include/rocketchip/config.h` removed. Live pieces retargeted:
`board.h` pins, `job.h` role (`job::kRadioModeRx` at call sites), `version.h`
semver (`kFirmwareVersion`, no `kVersionString` alias), `rc_debug.h` `DBG_*`.
Dropped with the file (walk: unused): `RC_ASSERT`, `ROCKETCHIP_TIER_*` /
`ROCKETCHIP_FEATURE_*`, `rocketchip::i2c` / `timing`. Do not recreate as an
umbrella “just in case.”

**Protected docs still name the old file** (need owner to name them to tidy):
`docs/SCAFFOLDING.md`, `docs/SAD.md` (tree + §13.1 TIER_*),
`standards/DEBUG_OUTPUT.md`, `docs/audits/VERSION_STRING_AUDIT.md` (alias note).

**Disposition target:** Erase when those named docs are updated, or when a
deliberate new `config.h` is introduced for actual compile-time flags.
**Blocking?** No

### R-9 — Version bump process (WN-010 / WN-067) — not invented this sitting

**Surfaced:** 2026-08-22 · sitting 7. Owner recalled the “more robust, rot-resistant”
version conversation.

**What that conversation already was:**
- IVP-127b / `VERSION_STRING_AUDIT.md`: one header (`version.h`), no literals
  elsewhere. That landed. It did **not** stop rot.
- LL Entry 2 / CODING_STANDARDS Debugging: monotonic `kBuildIterationTag`
  every debug rebuild; `__DATE__ __TIME__` is not enough. Tag still `"16B-init"`.
- WN-010/067: numbers frozen since 2026-04-15 (`0.16.0` / `0.5.0` /
  `"16B-init"`). Live discriminant is CMake `kGitHash`. `SESSION_CHECKLIST.md`
  has **no** version-bump item. A banner that says SSOT does not make agents bump.

**Not this sitting:** do not invent git-describe / auto-semver / a hook without
an owner-picked scheme. Do not silently bump `0.16.0`.

**When scheduled:** pick (1) wrap-only manual bump of `kFirmwareVersion` when
you name it, and/or (2) a checklist item  / hook that fails if
`kBuildIterationTag` is unchanged across a firmware sitting, and/or (3) drop
the manual tag and treat `kGitHash` as the only machine identity. Until then
the header wording must not claim a process that does not exist.

**Disposition target:** Owner-scheduled sitting or wrap. Erase when a written
process exists (checklist named, or hook, or explicit “git hash only”).
**Blocking?** No for rest of sitting 7.

### R-10 — Test remaining dispositions in groups of 2–4

**Surfaced:** 2026-08-22 · after `config.h` `a97d46c`. Owner: do not let a
bucket’s edits pile up untested.

**What:** Rest of sitting 7 and later Phase 3 sittings: pick **2–4 WNs**,
explain, edit, host ctest + firmware build, HW gate if the paths need it,
commit that group. Then the next group. Do not finish a 10–15 WN bucket in
the tree and gate once. Comment-only groups still get a host build; they
skip 3-boot only when the change is pure-software.

**Disposition target:** Erase when L2-P5 disposition closes (process for
this pass).
**Blocking?** No

### R-11 — HAB lockout-skip (`EMERG_DEPLOY`) not implemented

**Surfaced:** 2026-08-22 · sitting 8 / WN-195. Owner: no first rocket flight
yet; a speculative HAB bypass of deploy lockouts is a critical failure
point — take the code out, keep the feature logged.

**What was removed:** `MissionProfile::emergency_deploy_anytime`, generator
field `EMERG_DEPLOY`, wizard emit, `.cfg` keys. The combinator never read
the flag; docs claimed it skipped lockouts. Generator now **rejects** the
key if someone puts it back.

**When HAB is scheduled:** re-add as a profile bit **and** wire
`guard_combinator` lockouts (tests: rocket still locked; HAB skip explicit).
Do not restore a stored-but-unread flag.

**Disposition target:** Erase when HAB lockout-skip is implemented and
tested, or when HAB is dropped as a product.
**Blocking?** No

### R-7 — Sitting 13 comment bins (from sitting 5 Doxygen apply)

**Surfaced:** 2026-08-22 · Phase 3 sitting 5. Owner: drop Doxygen; short `//`
contracts; do not add a “don’t use Doxygen” line to `CODING_STANDARDS.md`.
Inventory: `L2P5_DOXYGEN_INVENTORY_2026-08-22.md` (90 tagged; **75 walk-missed**).

**What sitting 5 did (apply the same bins in sitting 13):**

1. **Markup:** `@file` / `@brief` / `@param` / `@return` / `/**` / `///` → `//`.
   Filename is not a comment. `@param` that only names the argument dies with
   the signature. Generated `eskf_codegen.h` stays generated (do not hand-edit).
2. **Contract:** one or two facts the signature does not carry — units, who
   owns a buffer, what `false` means, which core may call, a surprising
   precondition. Field units on structs (`// m/s^2`) stay. Restating the
   function name does not.
3. **Three bins for the data in the comment** (WN-085 is the house rule):
   - **Live contract** — keep as `//`.
   - **True, wrong home** — move to the named SSOT (`NOTIFY_CONTRACT.md`,
     `HEALTH_CONTRACT.md`, `flash_layout.h`, fusion design notes) and leave
     a pointer. Do not invent a new protected design doc.
   - **False / restatement / process** — delete. IVP/Stage/council/session
     essays, machine-local plan paths, `@brief Initialize the GPS`.
4. **Do not** add a prohibition to `CODING_STANDARDS.md`. The `.cpp` 15–25%
   density band already exists; header carve-out stays mechanical, not an
   essay license.
5. **Mechanical traps from this sitting:** one-line `/** @brief … */`,
   indented class-method `/**` blocks, `/// @param` left behind after `///`
   → `//`. Re-grep `@file|@brief|@param|@return|/\*\*|///` on `src/`+`include/`
   before calling sitting 13 closed.

**Sitting 13 (118 archaeology WNs):** same three bins, file by file. Do not
re-litigate Doxygen. Do not polish a header sittings 7–12 will still rewrite
if those sittings have not run yet — that is why 13 is last. WN-234 is an
*invariant* (MAVLink ARM no-op): strip-stale-promise vs wire ARM, not a
comment trim.

**Disposition target:** Sitting 13 applies this. Erase when that sitting
lands (or when L2-P5 closes if the bins are copied into Plan-3).
**Blocking?** No

### R-6 — Thin-file / hopeful-future nameplates (seeded WN-035)

**Surfaced:** 2026-08-21 · WN-035 (`notify_backend.h`) — not a public-vs-private
question. A file touched by 1–2 same-folder TUs does not earn *separateness*;
stale banners and “this already exists” asserts hide there.

**What:** Speculative plugin slot from IVP-115 (AP_Notify-shaped backends,
vtable refused). Two decls, one caller (`ao_notify` tick), LED `.cpp` real,
audio `.cpp` a stub (WN-228 — do not couple). Fold decls into
`src/notify/notify_resolver.h` (already the notify module header) and delete
`include/rocketchip/notify_backend.h`. Functions stay.

**Pattern (cite from later earn-rent WNs):** If a file is only a nameplate for
symbols that already have a home next door, or a “just in case / someday”
slot with no second live consumer, fold the nameplate. Do not keep a file so
a future OLED/audio/capability can pretend it already has an interface.
False “this exists” is worse than an honest missing file. Same family:
WN-032 (`job_capabilities.h`), job packs (WN-034), audio stub TU (WN-228).

**`notify_resolver.h` is not a test-only file.** `resolve_led_pattern` is
what the LED backend runs on the tick; `decode_health_faults` is what
AO_Notify uses. Host tests include that header to hit those *same*
functions without QP/hardware. Tests as a second consumer of production
code are the point. A header that exists *only* so tests compile, with no
firmware caller, would be the bad case — not this one.

**Disposition target:** This row stays as the pattern citation. Erase when
L2-P5 disposition closes (pattern should be in the log close-out / Plan-3
if still needed). WN-035 code fold is the first application.
**Blocking?** No

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

### R-4 — Codegen A/B (2026-08-21) — do not silent-regen

**Surfaced:** 2026-08-21 · Phase 2 generated-files labels. Owner: labels only this
phase; A/B already ran so park the result here for the codegen audit.

**What:** `python scripts/generate_profile.py profiles/rocket.cfg --output <temp>`
(no overwrite of prod). `rocket.cfg` sha256 prefix still `e1c22265fc444258`.
Committed `src/flight_director/mission_profile_data.h` still has the two Stage-T
post-gen edits (`#ifdef ROCKETCHIP_STAGE_T3_MAVLINK`, IVP-T6 comment). Regenerating
in place would drop the MAVLink switch. **New vs the main-WB codegen-audit row:**
HAB `generate_profile.py profiles/hab.cfg` **exits with missing required fields**
(`BARO_LAND_RATE_MPS`, `BARO_LAND_HOLD_MS`, `DESCENT_MAX_MS`, `DROGUE_TIMER_S`,
`MAIN_TIMER_S`); unknown cfg keys `DROGUE_TIMER_ACTION` / `MAIN_TIMER_ACTION` /
`SAFE_MODE_ACTION` ignored. `test/test_hab_profile_data.h` not compared.
`generate_fpft.py` not re-run (in-place write + date stamp).

**Disposition target:** Main WB **Codegen audit** row. Absorb Stage-T edits into
`generate_profile.py` (or a checked-in post-step) before any regen. Fix HAB
cfg/generator/fixture together. Erase this row when that audit has a green A/B
for rocket + hab (and records fpft if in scope).
**Blocking?** No — WN-196 stays DEFER. Do not regen this pass.

### R-5 — WN-267 PIO backup-timer flesh-out (not ACCEPT)

**Surfaced:** 2026-08-21 · Phase 2 early-impl labels. Owner: not keep-forever;
further flesh-out to a proper PIO backup-timer system is planned WB work.

**What:** Current dual countdown is a recent deliberate IVP-130 feature (WN-267).
Do not treat as amateur leftover. Do not ACCEPT as done. Quality / lifecycle /
LL-42 residuals wait on that sitting.

**Disposition target:** Main WB **Early-impl** table row *PIO backup pyro timers*.
Erase when that eval/flesh-out has a keep-with-why or a named rework plan.
**Blocking?** No
