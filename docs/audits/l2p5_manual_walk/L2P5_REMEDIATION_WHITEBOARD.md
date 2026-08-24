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

### R-12 — PIO WDT still needs a rework

**Surfaced:** 2026-08-22 · sitting 9 group 1. Owner: PIO watchdog is not a
Go/No-Go station and is not proven by a green preflight.

**What:** Do not promote `kHealthPioOk` into Tier 1. Dedicated sitting:
role vs ARM, CLI display, and whether a PIO WDT fault is pad-blocking.
Until then USER_GUIDE says a GO verdict is not “PIO WDT proven.”

**Disposition target:** Erase when that rework lands.
**Blocking?** No

### R-13 — Go/No-Go is station pad control; vehicle Estes is wire-arm

**Surfaced:** 2026-08-22 · sitting 9. Owner: GNG is for **station** ARM of
a radio vehicle. Vehicle-only (Estes-type) arms the igniter with a
physical wire — nothing software-ARM-able.

**Firmware today:** `command_handler` still runs `go_nogo_evaluate` on
vehicle CLI ARM (bring-up). Product pad flow is station `a` / ACK.
Do not treat vehicle USB ARM as the Estes procedure. Later sitting may
stop offering software ARM on vehicle-only images.

**Disposition target:** Erase when vehicle-only ARM matches the product
(no GNG software ARM) or station-only ARM is the only path.
**Blocking?** No for sitting 9 groups 2–4.

### R-14 — Pyro edge logger WIP, not armed at boot

**Surfaced:** 2026-08-22 · sitting 9 group 4 / WN-274. Owner: option 1
(honest WIP) **and** not active in flight boot until finished.

**What:** `pyro_edge_logger_init()` removed from `init_pio_safety()`.
Banner/USER_GUIDE/debug `y`: bench GPIO capture on PIO timer pins, 64-slot
fill-and-stop, not forensic. Re-arm only when pyro HW is on those pins
**and** a PCM/flight-log consumer exists.

**Disposition target:** Erase when that sitting lands, or when the files
are deleted.
**Blocking?** No

### R-15 — PCM ring `frame_count` uint32 wrap (GWF-291)

**Surfaced:** 2026-08-23 · log-ring overlay sitting. Owner: still worth
tracking, especially future high-rate logging; CCSDS/Starcom logging may
replace this ring.

**What:** `ring_stored_count` is `min(frame_count, max_frames)`. After
`frame_count` wraps at 2^32 the ring is still full but stored_count looks
empty. At 50 Hz that is ~2.7 years; high-rate logging makes it closer.

**Disposition target:** Starcom / CCSDS logging replace, or a saturating
counter if this ring survives. Comment on the field until then.
**Blocking?** No

### R-17 — Tiny 2350 pin-map allowlist (GWF-038 / GWF-039)

**Surfaced:** 2026-08-24 · overlay one-off. Not this sitting.

**What:** `board_tiny_2350_common.h` still binds I2C SCL and PSRAM CS both
to GPIO 21, and `board_led_set` ignores `kLedActiveHigh`. Live hazard is
already gated (`TINY_2350_BRINGUP_OK` + Plus `kPsramAvailable = false`).
Same sitting as leftover `CW-B02-03` map nits. Needs Pimoroni schematic
and a legal QMI CS1 (0 / 8 / 19 / 47) — do not invent a pin here.

**Disposition target:** Tiny allowlist sitting. Erase when the map is
allowlisted or the pack is dropped.
**Blocking?** No

### R-16 — Re-run SPIN after overlay remediates

**Surfaced:** 2026-08-24 · owner. bench_sim ran; SPIN did not.

**What:** Master SPIN gate (`tools/spin/`, last `SPIN_OK_31` / 6 models)
on `grok/l2p5-disposition` after FD bitmask/pyro, flash pause, health
seqlock, fault capture, MPU read-back. Prefer `rocketchip_fd.pml` +
`rocketchip_flash_protocol.pml`. Model lie vs firmware → `.pml` in the
same sitting (R-13). Same row: `AGENT_WHITEBOARD.md`.

**Disposition target:** Erase when the gate is green or a model fix lands.
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
