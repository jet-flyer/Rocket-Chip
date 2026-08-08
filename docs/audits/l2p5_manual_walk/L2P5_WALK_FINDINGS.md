**Last edited:** 2026-08-08 · Grok · rf_link_health WN-275 fix; sensor_core1 WN-276–281

# L2-P5 Walk Findings

Owner-initiated observations from the manual file walk (may be drafted with an agent after discussion).
Same path strings as `L2P5_WALK_ITINERARY.md` (coverage checkboxes); add path sections as files are walked.
Tangents → `L2P5_WALK_WHITEBOARD.md`. Not PASS/FAIL, remediation, or disposition.

| Rule | Detail |
|------|--------|
| **IDs** | Global `WN-NNN`, fixed prefix, never renumber. Only noteworthy entries get IDs. `— nothing of note.` is unnumbered and does not advance the counter. **Next ID** = `max(WN-*)+1` in body (line below is convenience; body wins). Section IDs may look out of order after revisits — feature, not bug. |
| **Who** | `[Agent]` = who wrote the text (name only). Every entry is owner-initiated; agents do not open paths or invent `nothing of note` on their own. |
| **Form** | `**WN-NNN** — [Agent] · \`kind\` · **title**` then observation. Kind = context (`comment`, `invariant`, `ownership`, `cross-file`), not priority/fix-timing. Lead with **locus** (path ~lines) + short **quote** so the claim stays brief and undiluted; taxonomy/follow-ons after. |
| **Path placement** | Each `#### \`path\`` is the itinerary path for the **claim’s locus** (where the quoted code/comment lives). Before appending a WN: open/confirm that path section — do **not** append under the last `####` in the file. Cross-file notes: section = declaring file (or Project-wide). Mis-nested WNs are process bugs; fix on notice. |
| **Append-only** | Frozen once written; never edit prior text (no “Related: WN-N+1” back-links on older entries — new WNs cite older ones only). Later find on same path → new global `WN`. Owner-directed one-time fixes only. |
| **Split vs merge** | One `WN` per claim that could disposition differently later. **Same disposition → merge** (multi-locus list; keep full context in the body). Nuance can wait for disposition later — do not split only to preserve sub-bullets. Especially on **central / dense** leaves (fusion, FD, logging, …) prefer fewer richer WNs over ~10 thin ones. See walk WB **W-13**. Same sitting ≠ forced merge when dispositions truly differ. |
| **Content** | Observations only; no design close-outs. Stated claim on the declaring file; enforcement/evidence under the implementing path when walked (new `WN`, don't edit old). |
| **Itinerary sync** | Adding a later path while an earlier itinerary path is checked off but missing here → flag owner for `— nothing of note.` (do not invent it). |
| **Project-wide** | Findings that are not a single itinerary path live under **Project-wide** (kept **above** per-file tiers so later file appends do not bury them). Still use global `WN-NNN`. |

**Next ID:** WN-282

---

## Project-wide

*(Not tied to one itinerary path. Append new `WN`s here; do not renumber.)*

**WN-004** — [Grok] · `ownership` · **License / SPDX / third-party attribution hygiene**  
Early-repo practice (per-file `SPDX-License-Identifier: GPL-3.0-or-later` + short Copyright,
root `LICENSE`, `THIRD_PARTY_LICENSES.md`) is still the right shape — not obsolete. Spot-check
2026-08-02 shows it is **partially stale** and worth a dedicated later pass (not mid-walk file
churn):

- **SPDX on authored sources:** ~181/184 `src/**`+`include/**` headers carry `GPL-3.0-or-later`
  and `Copyright (c) 2025-2026 Rocket Chip Project`. Missing SPDX: generated-ish
  `mission_profile_data.h`, `wmm_tables.{h,cpp}` (policy for generated files not restated here).
- **Third-party list lag:** `THIRD_PARTY_LICENSES.md` **Last updated: 2026-02-22**. In-tree
  since then / not listed as shipped deps: e.g. `EXTERNAL/etl-20.47.1`, `lib/qep` (QP/C),
  `lib/mavlink` (and any other vendored trees). WMM entry still points at
  `src/fusion/wmm_declination.cpp` — confirm path vs current `wmm_tables.*` / generator story.
- **Still proper approach:** per-file SPDX + root LICENSE + third-party inventory (optional later:
  REUSE/CI missing-SPDX gate). Not a reason to drop per-file headers.

**WN-054** — [Grok] · `ownership` · **Comment-density policy: header exemption needs re-eval**  
Locus: `standards/CODING_STANDARDS.md` Comment Density (§ ~2026-05-13 / `158b0df`) —
target 15–25% **`.cpp` only**; **headers excluded** with “Doxygen-heavy … 60–85% …
appropriate API documentation.”

**Provenance:** band floor cites Polyspace / OSS studies; **header carve-out is project
assumption**, not a primary standard. “Doxygen” was baked in without requiring that
comments actually *be* structured API docs — owner walk note: Doxygen was not an
established team convention first heard mid L2-P5.

**Effect:** little/no density enforcement on `.h`, while many public headers are modules
(not thin API flyers). Walk evidence: high comment mass is often **process archaeology /
restatement** (**W-6**, **WN-048**, **WN-053**, board/job banners), not useful Doxygen.
Real Doxygen-style API blocks (e.g. parts of `mavlink_rx` / encoder) look like the
**exception**, not the rule.

**Later:** re-evaluate policy (narrow/drop blanket `.h` exemption; upper-bound smells;
measure header-as-module files; require real contract comments only). No mid-walk
CODING_STANDARDS edit. Related: **W-6**, **W-7**.

**WN-081** — [Grok] · `ownership` · **Doxygen-style API comments: re-validate, then apply consistently or drop**  
Locus: **Project-wide** — pattern of `/** @file / @brief / @param / @return */` on
public headers (example trigger this sitting: full API blocks on
`drivers/gps_pa1010d.h`; also `i2c_bus.h`, telemetry/encoder surfaces, many
`include/rocketchip/*.h`). CODING_STANDARDS density carve-out assumes
“Doxygen-heavy” headers (**WN-054** / **W-7**) without a project decision that
Doxygen is *the* required style.

**Claim:** Owner direction mid-walk: treat Doxygen markup as a **policy choice to
re-evaluate**, not an ambient habit. Steps for later (standards / walk close-out —
not mid-walk mass edit):

1. **Still valid?** Decide whether structured Doxygen (or equivalent extractable
   API docs) remains a project goal — tooling actually used (or planned), value
   vs plain short comments, host/IDE cost. If not, stop writing `@brief`/`@param`
   for ceremony and retire the density exemption’s Doxygen premise.
2. **Iff still valid:** define **where** it is required (public driver/API headers
   only? all `include/rocketchip`? never `.cpp`?) and a **minimum shape**
   (identity + returns/preconditions; not process essays — **W-6**), then apply
   **consistently** so half-Doxygen / half-none isn’t the permanent state.
3. **If not valid:** prefer short non-Doxygen contracts; strip or leave rot to a
   cleanup pass; update CODING_STANDARDS so agents stop defaulting to `@brief`
   walls.

**Not** a finding that every existing `@brief` is wrong, and **not** a demand to
annotate every leaf now. Related: **WN-054**, **W-7**, **W-6**; per-file density
notes remain evidence only.

**WN-100** — [Grok] · `ownership` · **Regulatory / legal-config hazards: warn at risk lines + project audit**  
Locus: **Project-wide** (seed: `rfm95w.cpp` ~L38–39 `// ISM band frequency (US, FCC
Part 15)` + `kDefaultFreqHz = 915e6`; also TX power defaults, band-specific
paths, any RF/GPS export-sensitive knobs). Owner walk 2026-08-05.

**Claim:** Code that can **break law if misconfigured or deliberately bypassed**
needs more than a casual region name: either **accurate, scoped regulatory
statements** (where/when legal, authority, power limits) with a pointer to a
**project legal/RF compliance doc**, or explicit “not a compliance guarantee —
operator responsibility” + doc SSOT. Prefer **line-adjacent warnings** on
frequency/power/band knobs that can put the device out of compliance. **Project
audit:** sweep for such surfaces (radios primary; others as found). **Starcom
caveat:** when Starcom lands RF/PHY/config APIs, apply the **same safeguards**
(warnings + doc SSOT + no silent “looks legal” defaults without policy). Main WB
row tracks Starcom half. RFM seed: `kDefaultFreqHz` / Part 15 one-liner. Not legal
advice from the walk — process/architecture flag only.

**WN-086** — [Grok] · `ownership` · **Bespoke drivers: re-evaluate quality, residuals, and third-party/PA options**  
Locus: **Project-wide** — all first-party peripheral drivers (not only
`gps_pa1010d`), e.g. I²C GPS, UART GPS, IMU, baro, radio, bus wrappers, NeoPixel,
etc. Trigger sitting: PA1010D cold-boot / blind-PMTK / bus interaction history;
same class as Early-impl / rework-eval (I²C backend, flaky recover).

**Claim:** For each bespoke driver, later evaluate as a set (not mid-walk rewrites):

1. **How well is ours implemented?** Correctness, robustness, dual-core/bus
   discipline, error handling, host-testability — against current code + LL /
   residual plans (e.g. Fruit Jam GPS cold-boot, I²C contention).
2. **Ongoing issues?** Known flakiness, workarounds, blind config, recover
   coupling — if residuals are structural, treat as rework candidates rather
   than endless local patches.
3. **Working third-party / prior art?** Known solid drivers or patterns (vendor
   app notes, Adafruit/SparkFun/ArduPilot/Pico examples, Flipper PIO I²C, lwGPS
   already vendored, etc.) that could **replace**, **back**, or **improve** ours —
   with license + standards (adopted-code) gates. Do not reinvent if a proven
   option fits; do not pull Arduino stacks blindly either.

**Outcome per driver:** keep-and-document, improve-in-place from PA, swap backend
under project API, or schedule rework (align with Early-impl group where
overlap). **Not** a mandate to replace PA1010D this week; pairs with **WN-078**–
**080** (bus layer), I²C rework-eval row, CODING_STANDARDS prior-art rule.

**WN-085** — [Grok] · `comment` · **Triage / “why this path differs”: brief + commit/CHANGELOG, not essays**  
Locus: **Project-wide** policy (trigger: `gps_pa1010d.cpp` ~L81–84 Grok-triage
PMTK capture notes; same class as R-5/council blocks elsewhere in that file and
other early drivers). Owner walk direction 2026-08-05.

**Claim:** When code exists to **check for differences in execution** (early-init
before USB, alternate error path, instrumentation for a known regression), a
**brief** comment that that is *why* the special path exists is good. Full
dev-session narrative (who stepped in, which Unit/Tier, council option letter,
“keeps Tier 5 out of this commit”) does **not** belong in source. For further
info, **point to a commit hash and/or CHANGELOG entry** (and LL if applicable) —
same discipline as not hosting IVP/council tracking in headers (**WN-076**,
**W-6**). Apply this when evaluating comment / Doxygen policy (**WN-054**,
**WN-081**) and when cleaning process archaeology. Per-file evidence seed:
**WN-083**, **WN-084**.

**WN-163** — [Grok] · `ownership` · **Central features: design docs vs code + math still good**  
Locus: **Project-wide** — systems that define **central product behavior** (calibration
routines, ESKF/fusion, FD, etc.). Surfaced while walking `calibration_manager` (2026-08-06).

**Claim:** For each such feature, later pass should (a) check **design/IVP/plan docs still
align with what the code does**, (b) **spot-check math** (fits, gates, models) still
intended and correct, not only comments. Not mid-walk rewrites; disposition/audit queue.
Seeds: cal manager (**WN-155–162**), fusion ESKF/ZUPT/mag blocks.

---

## Tier 1 — Foundations

### include/rocketchip/

#### `include/rocketchip/shared_state.h`

**WN-001** — [Grok] · `comment` · **`g_imu` banner**  
Quote: `// IMU device handle (initialized on Core 0, used on Core 1)`. Reads as a closed
lifetime contract. Observed model is phase-based: Core 0 boot-fills under I2C ownership; after
`g_startSensorPhase` release, Core 1 does steady-state use and may re-`icm20948_init` on recovery.
Comment omits handoff, re-init, and post-handoff exclusivity.

**WN-002** — [Grok] · `invariant` · **`g_imu` shared handle**  
Handle is shared RAM across cores. Header does not state single-writer-after-handoff (or the
pause exception). Correctness of that invariant is not established from this file alone.

#### `include/rocketchip/rc_log.h`

**WN-003** — [Grok] · `comment` · **Comment mass is narrative, not a tight API contract**  
~74% of non-blank lines are comments. File-top banner is mostly “why/how we built R-5” and
restated policy, not a short “what this does.” Treat as one later pass over the whole comment
surface (not isolated one-liners).

- **Redundant with docs:** stdio ban chain → `docs/decisions/STDIO_REPLACEMENT_PLAN.md`;
  usage / hot-path / ISR / truncation → `docs/STDIO_TO_RC_LOG_MIGRATION_GUIDE.md`; format
  surface → `docs/audits/STDIO_FORMAT_SPEC_INVENTORY_2026-05-15.md`; unit/migration story →
  CHANGELOG R-5 + `docs/plans/R5_STDIO_REMOVAL.md`.
- **History in the header:** Unit B surface lock, Units C–J migration framing, “what was decided”
  tone — CHANGELOG/plan territory, not permanent API prose (migration closed).
- **Stale mechanism:** banner still describes printf → ETL `format_to` translation; Unit B /
  implementation path is hand-rolled subset + `etl::to_string` (see CHANGELOG Unit B / `rc_log`
  land notes).
- **Dead local cite:** `C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn.md` (not in repo);
  live plan under `docs/plans/`.
- **Keep-shaped residual (for the later pass):** short contract still useful if kept thin —
  128 B + truncate, non-blocking drop-on-full, drain from Core 0, pointer out to plan/migration/
  inventory/LL 39 — not a second full essay.

#### `include/rocketchip/config.h`

**WN-005** — [Grok] · `comment` · **Standards naming restated in file banner**  
Quote: `Per CODING_STANDARDS.md: Constants use k prefix… Global variables use g_ prefix…`.
Restates project-wide naming already owned by `standards/CODING_STANDARDS.md` (and naming
gates). No local API contract — pure policy echo in a config header.

*(RC_ASSERT block — same locus, separate disposition; cross-linked. Order follows comment
block top→bottom. One-time edit of premature single WN-006 into this group.)*

**WN-006** — [Grok] · `comment` · **`RC_ASSERT` banner over-cites standards**  
Section stamp `P10-5 / LOC-3.1 / JSF AV Env 15` over-claims. P10-5 is density + side-effect-free
+ recovery (project: aspirational); “LOC-3.1” is imprecise vs in-tree P10-5≡JPL-16 mapping;
JSF Env 15 is a broad parent only. Does not justify this DEBUG/no-op macro shape as compliance.
Related: WN-007, WN-008.

**WN-007** — [Grok] · `ownership` · **`RC_ASSERT` unused (0 call sites)**  
Macro defined in `config.h` only; no `RC_ASSERT(...)` callers in tree. Dead surface after
refactors — easy to keep “for standards” while nothing uses it. Related: WN-006, WN-008.

**WN-008** — [Grok] · `ownership` · **`RC_ASSERT` in prod header vs test rework**  
Quote: `Side-effect safety: expression is NOT evaluated in release builds.` Flight header,
DEBUG active / else no-op. Given R-25 single-binary + `test_mode` partitioning (closed) and
what still needs rework for prod vs gated paths: revisit whether this helper belongs in
production code at all, or remove / explicit test-dev policy — not a standards-stamped
dead surface that disappears in release. Related: WN-006, WN-007.

**WN-009** — [Grok] · `comment` · **Version “SSOT” wording over-promises**  
Quote (L52–53): `// Version constants — single source of truth in version.h`. Absolute SSOT
claim is too strong (alias here; git hash / board / job identity elsewhere; see also
`version.h` banner). Better as *intention*: canonical home other code should pull version
*constants* from — not “the only identity truth in the tree.” Related: WN-010, WN-011.

**WN-010** — [Grok] · `ownership` · **Version numbers stale; harden multi-agent tracking**  
`kFirmwareVersion`/`kRcOsVersion` still **0.16.0** / **0.5.0** since IVP-127b (2026-04-15);
`kBuildIterationTag` still `"16B-init"`. Major work since with no semver bump; live
discriminant is mostly `kGitHash`. Evaluate: refresh now + more robust process aligned with
multi-agent workflow (e.g. session checklist / wrap-up line and/or commit hook) so version
fields don’t silently freeze again. Related: WN-009, WN-011.

**WN-012** — [Grok] · `ownership` · **Product tier / feature defines — needed? proper mechanism?**  
L61–71: hand-toggled `ROCKETCHIP_TIER_CORE` / `MAIN` / `TITAN` (`MAIN` on) and
`ROCKETCHIP_FEATURE_*` (USB_CDC, NEOPIXEL, I2C, SPI, RADIO — all `1`). **Unused** outside
this header — no `#if` consumers. Comment-toggle / always-on flags are unenforced and not a
real config surface. Tiers are largely conceptual; features are hardware-ish but not wired;
board/job already select hardware/role via CMake + `board.h` / `job.h`. **Determine:** drop
for a unified codebase until product/HW options actually branch, **or** if needed, replace
with a proper mechanism (CMake/SKU/capabilities/wizard — not hand-edit defines in `config.h`).

**WN-013** — [Grok] · `ownership` · **Partial job re-export + essay in `config.h`**  
L73–78: four-line comment restates `job.h`/CMake (role vs MissionProfile) then only
`using job::kRadioModeRx` and `using job::kDefaultMavlinkOutput`. Incomplete job surface
(`kRole`, capabilities live elsewhere; callers often include `job.h` anyway). Out of place
as a radio-ish pair in global config — either a deliberate full convenience re-export of
the job API, or drop the usings and keep role flags with `job.h` / radio/role call sites.
Comment should not re-host job vocabulary. Related: WN-012, WN-015.

**WN-014** — [Grok] · `ownership` · **Pin aliases — remove to proper home?**  
L80–110 `rocketchip::pins::*` only re-export `board::*` (true pin SSOT is `board.h` /
`board_*.h`, not MissionProfile and not this file). Dual call surface (`board::` vs
`rocketchip::pins::`). **Determine:** drop aliases and use `board::` only, or keep a thin
facade with a clear single owner — do not treat `config.h` as HW pin config. Related: WN-015.

**WN-015** — [Grok] · `ownership` · **Does `config.h` need to exist? (whole-file)**  
*Applies to the entire file; section is the path under walk — deeper dive at disposition.*
Grab-bag of RC_ASSERT, version alias, dead tier/feature flags, partial job usings, pin
aliases, unused i2c/timing tables, DBG macros — several pieces already owned elsewhere
(`version.h`, `job.h`, `board.h`, `rc_log`, DEBUG_OUTPUT). **Determine:** dissolve into
proper homes vs keep a minimal “compat umbrella” include. Item-level WNs (005–014, …) are
evidence; this is the file-role question. Related: WN-012–014, WN-016–018 and other
`config.h` WNs.

**WN-016** — [Grok] · `ownership` · **DBG helpers live in grab-bag `config.h`**  
L140–199 debug section (toggle, `dbg_*`, `DBG_*`) pulled into anything that includes
`config.h`. Not a dedicated debug/test unit; gating is only `if constexpr (kDebugEnabled)`
inside always-present templates (and default CMake forces `DEBUG=1`). **Determine:** move to
a dedicated debug header (or test-gated module) with clear include/gate policy — not
production kitchen-sink. Related: WN-008, WN-015, WN-017, WN-018. **Also:** `standards/DEBUG_OUTPUT.md`
still shows an older `#ifdef` macro-only sketch and points at `config.h` — already stale vs
template + rename layout; **evaluate that doc for refresh after code placement/API fixes**
so the standard doesn’t keep teaching the wrong shape.

**WN-017** — [Grok] · `comment` · **R-5 DBG repoint essay (L156–170)**  
Multi-paragraph migration history (Unit B step 4, stdio leak, ArduPilot amendment, rc_log
128-byte contract). CHANGELOG / R-5 docs territory. **Related to L172–194:** yes — that blob
is the only long “explanation” above the live `dbg_print` / `dbg_error` / `dbg_state`
implementations; the functions themselves have almost no local “what this does” docs beyond
the essay. Prefer short API notes or a doc pointer, not a commit narrative. Related: WN-003,
WN-016.

**WN-018** — [Grok] · `ownership` · **`DBG_*` macros only rename `dbg_*`**  
L196–199: “Compatibility macros — map old names to C++20 template functions” — pure rename
surface (`DBG_PRINT` → `dbg_print`, etc.). Call sites still use `DBG_*`. Band-aid dual API;
resolve to one naming surface (migrate callers or drop templates and keep macros), don’t
keep forever-rename layers. Related: WN-016, WN-017.

#### `include/rocketchip/version.h`

*(Brief note while walking `config.h` version section — claim lives in this header.)*

**WN-011** — [Grok] · `comment` · **Phantom `version_string()` in banner**  
Quote: `All print sites must use version_string() or the constants below.` No project
`version_string()` API exists; callers use the constants. Over-promising helper name.
Related: WN-009, WN-010.

**WN-067** — [Grok] · `ownership` · **Version SSOT is aspirational; tracking still weak (ties WN-010)**  
Locus: whole `version.h` + banner “Single source of truth… All print sites must use…”
Treated as the version home (`VERSION_STRING_AUDIT.md` IVP-127b) but still mostly
**manual** constants (`kVersionMajor/Minor/Patch`, `kFirmwareVersion`, `kRcOsVersion`,
`kBuildIterationTag`) plus partial CMake inject (`GIT_HASH`). Feels optional in practice
if agents can ship without bumping it — same failure mode as **WN-010** (stale numbers;
need robust multi-agent / hook / checklist method). **Later:** enforce propagation
(hook or checklist), not just a header labeled SSOT. Related: **WN-009**, **WN-010**,
**WN-011**.

**WN-068** — [Grok] · `ownership` · **HW-/build-SKU identity mixed into version header**  
Locus: ~L33–39 `PICO_BOARD` / `kBoardName`; also job-role `#ifdef ROCKETCHIP_JOB_*`
(~L23–31) and R-25-exec / IVP soak comments. Board string is **Pico-SDK build
identity**, not firmware semver; overlaps board-pack story (`board::kBoardName` used
by CLI/log paths). Pulls HAL/SKU concerns into the “version SSOT” file and works
against **hardware-agnostic** domain code (compatible ARM MCU class — not literally
every ISA). **Later:** keep version numbers here; board/job identity via board/job
packs or build metadata helpers, not this header. Related: **WN-063**, board family
**WN-019**–**WN-022**.

#### `include/rocketchip/linker_symbols.h`

**WN-069** — [Grok] · `ownership` · **Superfluous band-aid header for reserved linker symbols**  
Locus: whole file (~22 lines) — two `extern` refs to linker `__StackBottom` /
`__StackOneBottom` plus clang-tidy `// NOLINT` suppressions; banner frames this as
the intentional home for “one unavoidable suppression” (TP-2 / L2-P5 council
2026-06-24; `ACCEPTED_STANDARDS_DEVIATIONS`).

**Fan-in:** `main.cpp` (MPU setup), `fault_protection.h`, `fault_inject.cpp`.

**Claim:** Public `include/rocketchip/` header whose job is mainly to **quarantine
a toolchain/lint band-aid** (reserved `__` names for MPU stack guard). That is a
workaround shape, not a durable domain API — file feels **superfluous** as a
standalone public unit. **Later:** handle more properly (fold into safety/MPU
boundary, different stack-guard approach, or narrower private header) so the
project isn’t carrying a dedicated public SSOT for NOLINT + linker externs.
No mid-walk redesign. Related: TP-2 deviation log.

**WN-070** — [Grok] · `invariant` · **In-source NOLINT on stack symbols (disallowed policy)**  
Locus: ~L16–19 two `// NOLINTNEXTLINE(bugprone-reserved-identifier,readability-identifier-naming)`
on `__StackBottom` / `__StackOneBottom`. Easy to miss inside the comment block.

**Claim:** Same class as **WN-043** (NOLINT on seqlock `sizeof` assert): project policy
is **no in-source NOLINT suppressions** — fix or accept via deviation log /
tool config, don’t plant NOLINT that can copy-paste into other files. This header
was explicitly built to *centralize* those suppressions (band-aid from **WN-069**);
that pattern must not propagate under the newer prohibition. **Later:** remove
NOLINTs here when TP-2 / MPU boundary is reworked; do not treat “one documented
NOLINT file” as a template for more. Related: **WN-069**, **WN-043**, TP-2.

#### `include/rocketchip/board.h`

**WN-019** — [Grok] · `comment` · **File banner mixes contract with history**  
L4–19 `@file` block: useful bits (compile-time selector, `board::` home, how to add a board,
pointer to BOARD_COMPARISON) sit with stage/history framing (“Stage J: Fruit Jam HAL”,
same-binary narrative) that belongs in CHANGELOG/docs, not permanent header prose. Prefer a
short “what this does” + add-board steps; drop or rehome stage-era narrative. Related: WN-020.

**WN-020** — [Grok] · `ownership` · **Silent `else` defaults to Feather HSTX**  
L39–41: unknown / unmatched board macros fall through to `board_feather_rp2350.h`. Wrong or
missing board identity can still build and flash with Feather pins. Prefer fail-closed:
`#error` (or equivalent) on unsupported board, **or** an explicit empty/uninit board pack that
cannot be mistaken for a flight map — not a quiet HSTX default. Related: WN-019, WN-021.

**WN-021** — [Grok] · `comment` · **Tiny 2350 / Pico 2 still scaffolding (false completeness)**  
L30–38 comments: “scaffolding, gated by TINY_2350_BRINGUP_OK / PICO2_BRINGUP_OK (IVP-143).”
**Still true (2026-08):** packs exist but are **not** HW-verified flight boards —
`board_tiny_2350_plus.h` / `board_pico2.h` (and common) still `#error` unless those defines
are set; pin maps datasheet-sourced / TODO radio pins on Pico 2; base `board_tiny_2350.h`
still absent. Selector lists them as peer `#elif`s next to Feather/Fruit Jam → looks
supported. **Resolve:** keep explicit “unsupported until bring-up” (comments + fail-closed
aligned with **WN-020**), or complete bring-up and drop scaffolding language — don’t leave
half-boards looking first-class. Related: WN-019, WN-020, WN-022.

**WN-022** — [Grok] · `comment` · **Board-pack file header format — family-wide consistency**  
*Board HAL section-wide (not only Feather):* brief top-of-file HW summary is good (e.g.
Feather L4–11: MCU, flash/PSRAM, bus roles, pointer to BOARD_COMPARISON). Prefer also a
**product number** and **store/product URL** when they fit in a line or two (Feather already
has `#6130`; others may lack a link). Banner should also list **SKU built-ins** explicitly
even when “obvious” (onboard GPS chip + part name, onboard LoRa, PSRAM, LED/WS2812, etc.)
and distinguish **on-board soldered** vs **SKU-bundled plug-in** (e.g. kit includes GPS
module to attach ≠ onboard GPS). **While walking every `board_*.h` / common:** confirm
each pack has a clearly laid-out, consistent shape (banner fields + section order:
I2C/SPI/radio/NeoPixel/LED/reset/PSRAM/UART/capabilities/identity/helpers) so community
ports match. Note gaps/drift per file; disposition may be a shared template or checklist,
not one-off edits. Related: WN-019, WN-021. See **WN-024** (built-in vs expansion).

**WN-023** — [Grok] · `ownership` · **Optional board hooks → no-op on every other pack**  
Example: `board_release_peripheral_reset()` — real work only on Fruit Jam (GPIO 22); Feather
and other packs carry empty bodies so shared init can call unconditionally (`main.cpp`).
Uniform call site is fine; **N−1 no-ops per single-board feature does not scale**. While
walking board packs, inventory similar “one board only / empty elsewhere” hooks (and any
like them). **Later disposition:** pick a better pattern (e.g. capability + `if constexpr`,
shared default + override) so FJ-specific (etc.) logic stays in that board’s file without
no-op sprawl. Needs design thought — not a drive-by delete. Related: WN-022.

**WN-024** — [Grok] · `ownership` · **UART GPS block misplaced on board packs**  
Locus: `board_feather_rp2350.h` ~L63–67 (same pattern on other packs). Quote:
`// --- UART GPS ---` / `// Feather has UART0 on GPIO 0/1 — available for GPS FeatherWing`
/ `kUartGpsAvailable = true` / `kUartGpsTxPin`/`RxPin`. **Primary:** this block is expansion
(wing / hand-solder / future booster), not carrier SKU wiring like PSRAM CS — belongs with
GPS/expansion config, not “board has GPS.” Board may still document free UART pins only.
**Taxonomy for banner/API (WN-022):** onboard soldered chip (name it) vs kit plug-in vs
optional expansion — don’t blur into one `kUartGpsAvailable`. Callers: `gps_uart.cpp`,
`main.cpp`. Related: WN-014, WN-022.

#### `include/rocketchip/board_fruit_jam.h`

**WN-025** — [Grok] · `comment` · **M1/N1/M2/M3 tags look like bug tickets**  
Locus: `board_fruit_jam.h` L12–18 (`[M1]`/`[N1]` banner), also `[M2]` ~L54 LED,
`[M3]` ~L85 UART GPS — same cohort. Quote (banner): `[M1] GPIO 5 conflict… Button3` /
`[N1] SPI1 bus sharing… ESP32-C6 CS=GPIO 46`. **Provenance:** landed together in Stage J
BSP commit `7aeea79` (2026-03-07); message says *“Council amendments M1-M4, S1-S4, N1-N2
implemented.”* So tags are **council amendment IDs**, not open bug IDs — but **in-file they
are opaque** (no legend) and still read as ticket soup. **N1** is specifically radio vs
**onboard ESP32-C6** SPI1 CS arbitration (shared bus with WiFi chip), not a random leftover;
user memory of dropping ESP WiFi work is consistent with N1 remaining as a **coexistence
constraint** after WiFi effort was abandoned. Real pin facts OK as plain comments or
BOARD_COMPARISON. **Later:** rehome council-ID noise; per-board scratch / HW-labeled PRs if
needed — not mainline header archaeology. Related: WN-022.

**WN-026** — [Grok] · `comment` · **Onboard extras dumped at EOF; need implement status**  
Locus: `board_fruit_jam.h` L104–108. Quote: `// --- Fruit Jam extras ---` then comment-only
ESP32-C6 / SD / buttons / Audio I2S pin lists (no `board::` symbols). **Primary:** onboard
SKU pin facts belong in the **top HW blurb** (WN-022) or real constants — not an appended
orphan block. Stage J originally said “not used … future”; that status was lost.  
**Family-wide:** packs should state clearly **onboard but not wired/implemented yet** or
**ESP (etc.) WIP / abandoned** so pin notes or capability flags don’t read as “works 100%.”
Also: `kSdCardAvailable`/`kDvmAvailable` true without pin APIs is the same false-confidence
class. Related: WN-022, WN-024, WN-025.

#### `include/rocketchip/board_pico2.h`

**WN-027** — [Grok] · `ownership` · **Board WIP gate: premise OK, wording/policy incomplete**  
Locus: `board_pico2.h` ~L20–22 (same: `TINY_2350_BRINGUP_OK` in tiny packs). Quote:
`#ifndef PICO2_BRINGUP_OK` / `#error "…not yet verified… Define PICO2_BRINGUP_OK after
hardware bring-up."` **Premise solid:** fail-closed so an unverified pin map can’t silently
build as flight-ready. **Problem:** name/message reads as **PM status** (“bring-up done?”)
and a pinky-swear define — goes stale (force-define without verification, or never remove).
**Prefer:** keep a **build-policy** gate if it comes from a **documented procedure** (CMake
allowlist / flag framed as “this target is WIP,” not “I finished the IVP”); align with
selector fail-closed (**WN-020**/**WN-021**). **Also define WIP labeling:** clear **WIP** in
**filename and top banner** (not “experimental” — that evokes future advanced-feature work
on the roadmap). Use this as a **prototype policy** for HW-specific / new-board files and
later **officially supported vs unsupported boards**. Related: WN-021, WN-022, WN-026.

#### `include/rocketchip/board_tiny_2350_common.h` / `board_tiny_2350_plus.h`

**WN-028** — [Grok] · `ownership` · **Tiny packs: more WIP, weak WIP labeling, oversplit**  
Locus: `board_tiny_2350_common.h` L7–17 (+ TODO pin guesses throughout); `board_tiny_2350_plus.h`
L11–15 / L29–31. Quote (common): `share ~95% of their pin map` / `scaffolding only` /
`datasheet-sourced best-guesses`. **More preliminary than Pico 2** (many TODO(Tiny_2350)
verify notes) yet **no WIP in filename**; plus has `TINY_2350_BRINGUP_OK` (see **WN-027**),
common alone has no gate. Banner cites base `board_tiny_2350.h` which **does not exist**.
**Structure:** Plus only sets `kPsramAvailable` + `kBoardName` — effectively identical map
aside from PSRAM; does not warrant a multi-file split — prefer **one Tiny pack** with
in-file / compile-time PSRAM presence (like SKU variants), not common+plus. Pin pass can
resolve guesses later. Related: WN-021, WN-027.

#### board HAL — multi-file rollup (all packs walked)

**WN-029** — [Grok] · `ownership` · **UART GPS + LoRa pin blocks on every pack (rollup)**  
Cites **WN-024** (primary: expansion vs board; taxonomy). **Inventory — every pack carries
both patterns:**

| Pack | UART GPS block | Radio/LoRa CS·RST·IRQ |
|------|----------------|------------------------|
| `board_feather_rp2350.h` | yes (~L63–67) | yes (~L32–35) “FeatherWing” |
| `board_fruit_jam.h` | yes (~L84–89) unavailable | yes (~L42–45) “breakout/adapter” |
| `board_pico2.h` | yes (~L68–73) | yes (~L42–46) “breakout” |
| `board_tiny_2350_common.h` | yes (~L71–76) | yes (~L44–48) “breakout” |

**None of the current boards ship onboard LoRa** — radio pins are expansion/adapter defaults,
same class as UART GPS. Disposition with **WN-024**: expansion/booster layer vs board free-pin
docs; banner built-ins (**WN-022**) only for true onboard radio/GPS.

#### `include/rocketchip/job.h`

**WN-030** — [Grok] · `comment` · **Name “job” is vague; scaffold map thin**  
Locus: `job.h` ~L4–11 banner; also `mission_profile.h` ~L11–13. Quote (MP): `Distinct from
job.h (device role: vehicle vs station)`. “Job” alone does not advertise *device role*; the
clearer phrase already lives in the MissionProfile comment, not in SCAFFOLDING’s one-liner
(`Device role selector`) or a SAD three-axis map (board / role / mission profile). Placement
in `include/rocketchip/` is fine (selector + packs, parallel to board). Later: rename
consideration (`device_role` / keep “job” with louder banners) + scaffold/SAD “open this for
role, not pins or MissionProfile” — note only, not mid-walk rename.

**WN-031** — [Grok] · `ownership` · **Mutually exclusive DeviceRole may be wrong long-term**  
Locus: `job.h` ~L32–47 (`enum class DeviceRole` + `#if` one-of `job_relay` / `job_station` /
default vehicle). Compile-time exclusive roles fit today’s single-binary-per-role builds.
Premise friction: a **relay function can live on a vehicle** (and on Titan-class multi-node /
same-MCU sketches, roles may co-reside). Exclusive `kVehicle | kStation | kRelay` is a
product/build axis, not necessarily a permanent architecture law. **Lower priority** —
revisit when multi-role or co-resident link-layer designs are real; do not redesign mid-walk.
Related: capability predicates in `job_capabilities.h` assume one `kRole`.

#### `include/rocketchip/job_capabilities.h`

**WN-032** — [Grok] · `ownership` · **Does this need its own header?**  
Locus: whole file — three `kRole*` `constexpr bool`s (~7 functional LOC) after
`#include "rocketchip/job.h"`, rest comment. Born IVP-142c (`dadcbf1`) as a new
file (not split from `job.h`); stated intent was capability-masking home, not a
file-size gate (60-line rule is per-function `.cpp`). Question for later: fold
into `job.h` (or packs) vs keep a separate unit only if the capability surface
grows.

#### job packs — `job_vehicle.h` / `job_station.h` / `job_relay.h`

*(Family claim; same three-line surface on every pack. Opened while walking relay; applies
to all three itinerary leaves.)*

**WN-033** — [Grok] · `comment` · **Job-pack banners: rot risk; prefer pointer over restatement**  
Locus: file banners on **all three** packs (`job_vehicle.h`, `job_station.h`,
`job_relay.h`) — e.g. relay ~L3–10 (RX→CRC→re-TX, “no AO_Telemetry/ESKF/FD”,
`Council 3 [C3-R2]`); vehicle/station restate TX/RX role + MissionProfile distinction.
Local sketches read fine today; risk is **stale restatement** of design that lives
elsewhere (council / AO_Radio / stage plan / future Starcom) while packs and call sites
move. **Suggestion only (no disposition mid-walk):** short identity line + pointer to one
durable design/decision home, rather than re-iterating what that doc says the role is.
Same policy question as other thin selector packs (board banners, etc.).

**WN-034** — [Grok] · `ownership` · **Job-pack constexpr surface may not earn three files**  
Locus: each of `job_vehicle.h` / `job_station.h` / `job_relay.h` — same three symbols:
`kRole`, `kRadioModeRx`, `kDefaultMavlinkOutput` (plus comments).

1. **`kDefaultMavlinkOutput`** — `false` on **all three** packs; re-exported in `config.h`
   (**WN-013**); **no `src/` consumer** at walk time — may already be obsolete for every
   job. MAVLink-on-USB is station presentation, not a vehicle TX or relay-pipe decision.
   Relay especially is a transparent link-layer pipe (banner + IVP-98); it should not be
   “choosing MAVLink or not.”

2. **`kRadioModeRx`** — vehicle `false`, station/relay `true`. **Similar issue:** compile-time
   exclusive RX-vs-not on the *job pack* feels like the wrong home, and the exclusivity is
   **outright too narrow** — all three roles can need RX and/or TX (especially after CCSDS /
   Starcom rework; half-duplex vehicle already RX-capable in places). Today this flag is a
   real branch in `main` / CLI / station idle / telemetry paths, but it is acting as a
   coarse “station-shaped build” stand-in more than a true radio-direction model.

3. **Parallel-pack boilerplate** — same class as **WN-023** (uniform surface on packs that
   do not own the concept: dead/dummy constexprs, or a one-bit role caricature). Stronger
   reading: three-job land (IVP-95) made every pack look the same even where symbols are
   meaningless or wrong long-term.

4. **If those flags do not belong here — what are the individual job files doing?**  
   Remaining non-comment content is essentially `kRole = …`. That may still justify a
   thin pack (or collapse into `job.h` `#if` arms / a single table). **Open question for
   later**, not a mid-walk close-out: three files vs selector-only vs capabilities-only.

**Suggestion only; no hard decision now.** Prefer to **revisit only after Starcom library
work** (CCSDS / link-layer surface will move radio direction, relay pipe, and any
return of MAVLink defaults). Related: WN-013, WN-023, WN-031, WN-032.

#### `include/rocketchip/notify_backend.h`

**WN-035** — [Grok] · `ownership` · **Does this need its own public header?**  
Locus: whole file — two free-function decls (`notify_backend_led_update` /
`notify_backend_audio_update`); rest banner (P10-9 / no vtable, compile-time backends).
Implementations in `src/notify/*`; sole production caller `ao_notify` tick. **Same class
of question as WN-032** (thin public unit whose functional surface is tiny — “does a
separate file earn rent?”). **Do not couple dispositions:** outcomes may differ entirely
(fold vs keep vs relocate); this note only records that the *sparseness / separate-file*
question applies here too.

#### `include/rocketchip/notify_intents.h`

**WN-036** — [Grok] · `comment` · **High comment density on an otherwise useful header**  
Locus: whole file (~138 lines). Real surface is solid (per-category intent enums +
`NotifyState` + beacon/vehicle_lost flags) — multi-hop contract for notify. Comment mass
is heavy (banner, priority rationale, per-enum LED color notes, Stage L/T narrative on
beacon/vehicle_lost). Headers are exempt from the `.cpp` 15–25% density band, but density
still reads high vs the type surface; later trim/pointer pass possible without questioning
the file’s right to exist.

#### `include/rocketchip/radio_config.h`

**WN-037** — [Grok] · `comment` · **“V2 (not V1)” undefined**  
Locus: banner ~L11 `// Advanced overrides available in V2 (not V1).` Rest of top block is
fine (sibling to MissionProfile, generator derives SF/BW/CR). **V1/V2 never named** here
(schema? product tier? generator format?). Vague — name the versioned artifact or drop.

**WN-038** — [Grok] · `ownership` · **`kDefaultRadioConfig` assumes one radio family**  
Locus: ~L39–48 default struct (TX, CCSDS, 2 Hz, 20 dBm, SF7, 125 kHz, CR5). Safe as a
default only if this header is **tightly bound to the current LoRa/SX1276-class radio**
(RFM95-shaped knobs). File itself is generic `RadioConfig` + includes only
`telemetry_encoder.h` — **no radio/SKU name or capability gate** in this unit.
`radio_config_table.h` is more explicitly SX1276; consumers include storage, profile data,
telemetry encoder, tests. **If multi-radio or Starcom-era PHY shows up, these numbers are
not “neutral” defaults** — couple defaults to a named radio/profile path, or stop shipping
a single global default here. Starcom impact: use include/consumer graph when that work
lands (types likely shared; defaults/PHY fields may move).

#### `include/rocketchip/radio_config_table.h`

**WN-039** — [Grok] · `comment` · **Over-authoritative council banner**  
Locus: top block ~L3–20. Tone claims **canonical** whitelist, “Code is authoritative;
docs/… references THIS header (not vice versa)”, and “Design rules (correctness-council
edit #2)” as if council edit is permanent law in-header. Process/provenance belongs in
decision/IVP notes; banner should state what the table *is* without over-binding authority
or council archaeology.

**WN-040** — [Grok] · `ownership` · **SX1276-specific file under generic radio_config name**  
Locus: whole file — `RadioConfigEntry` / `kRadioConfigTable` / `radio_config_in_whitelist` /
`radio_config_sx1276_legal` (explicit SX1276 datasheet ranges, PA_BOOST, SF/CR/BW).
Filename + “RadioConfig whitelist” read **universal**; body is **this firmware’s LoRa/SX1276
presets + legality**. Fine **if labeled and classified as HW-specific** (name, banner, path
story) — not as the abstract radio-config SSOT for every future PHY.

**Include / consumer check (W-5):** direct includes — `ao_telemetry.cpp` (SET path +
sx1276_legal), `radio_config_storage.cpp` (read validation), `rc_os_debug.cpp` (digit
presets), `rc_os_commands.cpp` (cycle table). No other radio family consumers. So today it
behaves as **HW-specific policy for the current SX1276 path**, not a multi-radio abstraction
— but the public name doesn’t say that.

**Later (if this survives Starcom):** clean up labeling and consider a **standard pattern**
for per-radio config tables (one table/legal helper per PHY or SKU), not one global
“the” radio config table. Related: **WN-038** (defaults on generic `radio_config.h`).

#### `include/rocketchip/radio_scheduler.h`

**WN-041** — [Grok] · `ownership` · **Prime Starcom supersession candidate**  
Locus: whole file — half-duplex TX-priority `RadioScheduler` SM (`RadioPhase`,
vehicle TX-slot / station-relay RX continuous). Nothing else sticks out on walk.
**Include check:** only `ao_radio.{h,cpp}` owns/uses it. Prime candidate for this
unit to be **completely superseded by Starcom** link-layer work rather than evolved
in place — re-check consumers when that lands. Related: **WN-034** / **WN-040**
(radio/job surface also deferred past Starcom).

#### `include/rocketchip/sensor_seqlock.h`

**WN-042** — [Grok] · `ownership` · **Re-evaluate whether Stage 3 seqlock design is still the right path**  
Locus: **whole file** (struct + inline `seqlock_write`/`seqlock_read` + `g_sensorSeqlock`)
and the design it cites — not a single comment line. Banner/struct note “per
`SEQLOCK_DESIGN.md` (council-approved)” points at the **2026-02-06** Stage 3 council
(unanimous; IVP-24 single-buffer, poll sequence, cal on Core 1, static SRAM, etc.).
Header was extracted later (Stage 13 Phase 0A, 2026-04-04) from `main.cpp`; fields and
consumers have grown since (MCU temp, station idle GPS, health, notify, …).

**Holistic later pass:** confirm the **initial design is still the right path** for the
system as built and for near-term work — rates, struct mass, DMB/`memcpy` protocol,
single combined buffer, reader set, station vs vehicle use — against current code and
`SEQLOCK_DESIGN.md`, not just that comments still say “council-approved.” Observation
only; no mid-walk redesign.

**WN-043** — [Grok] · `invariant` · **NOLINT on sizeof static_assert (disallowed)**  
Locus: ~L100–101  
`static_assert(sizeof(shared_sensor_data_t) == 156, // NOLINT(readability-magic-numbers)`.  
**Critical / policy:** in-source `NOLINT` suppressions are **not allowed** anymore — fix the
underlying check (e.g. named `constexpr` expected size, or other non-suppress approach), do
not silence magic-number tidy. Size gate itself is valuable (layout contract with
`SEQLOCK_DESIGN.md`); only the NOLINT is the defect.

**WN-044** — [Grok] · `comment` · **Tombstone for removed `g_calNeoPixelOverride`**  
Locus: ~L161–162  
`// g_calNeoPixelOverride removed. CLI uses AO_LedEngine_post_override(),` /  
`// FD uses AO_LedEngine_post_pattern(). No cross-core atomic needed.`  
Not needed in the live header if the removal is already in CHANGELOG/git; don’t keep
“what was deleted” notes here — drop or leave only a pointer if something still misleads.

#### `include/rocketchip/sensor_snapshot.h`

**WN-045** — [Grok] · `ownership` · **Does `SensorSnapshot` still need to exist?**  
Locus: whole file — packed 40-byte `SensorSnapshot` (raw accel/gyro/mag/baro/GPS + MET).

**Existence first:** include/consumer check — **only** `test/test_data_model.cpp` (size
assert); **no `src/` producer or consumer.** Public ICD stub without a live path.

**Why it was made:** Stage 6 / IVP-49 three-struct data model (`FusedState` /
`TelemetryState` / `SensorSnapshot`) per logging council; **IVP-55 raw sensor logging
deferred** — intended home for this type. Trace those ends on disposition: IVP/docs/
tests/SCAFFOLDING that still treat the triple as shipped, host size-only tests, any
advanced-settings “raw logging” rows — tidy so nothing implies a wired raw snapshot if
the header goes.

**If it survives scrutiny (owner: unlikely):** then address secondary issues — (1)
**HW-specific without labeling** (comments fix ICM-20948 / AK09916 / DPS310 ADC models;
filename/banner read universal); (2) **sparseness** of a public `include/rocketchip/`
unit with zero firmware fan-in. Do not treat survival as the default path.

#### telemetry public headers — `telemetry_encoder.h` / `telemetry_state.h` / `mavlink_rx.h`

**WN-046** — [Grok] · `ownership` · **Telemetry trio likely Starcom-affected / replaceable**  
Locus: family — `include/rocketchip/telemetry_encoder.h`, `telemetry_state.h`,
`mavlink_rx.h` (CCSDS/MAVLink encode, wire `TelemetryState`, USB MAVLink RX GCS).
Same class as radio scheduler / link-layer notes: **probably affected or replaced by
Starcom work** rather than evolved in isolation. Revisit consumers and keep/replace when
that lands; no mid-walk redesign. Related: **WN-041**, **WN-040**.

#### `include/rocketchip/telemetry_state.h`

**WN-051** — [Grok] · `ownership` · **DEPRECATED aliases still live with zero consumers**  
Locus: ~L66–68  
`// Legacy aliases — remove after all consumers migrated (IVP-107)` +  
`kHealthEskfHealthy` / `kHealthZuptActive` marked DEPRECATED but still **real
`constexpr`s** (not commented out). Grep: **no `src/`/`test/` users** — live flag is
`kFlagsZuptActive` (`data_convert`, tests). Migration note is stale; symbols pollute the
public API and can confuse health-byte vs flags-byte layouts. **If file survives
Starcom (**WN-046**):** delete the two aliases (and the legacy line); don’t leave
DEPRECATED live code. Related: **W-6**, **WN-044** (tombstone class).

#### `include/rocketchip/ao_signals.h`

**WN-052** — [Grok] · `ownership` · **Defer deep redesign until QP/QF work**  
Locus: whole file — system-wide QP/C `RcSignal` catalog, event structs, `evt_cast`
(CAST-2). Same class as Starcom-gated radio/telem surfaces: **signal map and event
shapes will move with upcoming QP/QF (QP/C++?) work**. Prefer not to invest mid-walk
in renumbering/catalog cleanup beyond observations; re-open after that workstream.

**WN-053** — [Grok] · `comment` · **Comment mass + momentary/process archaeology**  
Locus: whole file (~200 lines, heavy comment share) — council date/rename (banner),
“Historically N held …”, LEGACY/IVP/Stage L tags on enumerators, multi-line slot
history (~L76–90). Real catalog needs some naming; **council land-dates, removed-slot
narratives, IVP step tags** are W-6-class noise. **If revisited post QP/QF:** thin to
live signal meanings + true invariants; drop momentary dev/council prose. Related:
**W-6**, **WN-048**.

#### `include/rocketchip/telemetry_encoder.h`

**WN-047** — [Grok] · `comment` · **Banner lists protocol layout (stale risk)**  
Locus: file banner ~L3–20 — enumerates CCSDS Space Packet layout (6B+4B+42B+2B = 54B),
MAVLink message set and ~byte totals. Same class as **WN-033** (role-pack banners): useful
once, **re-words or drifts vs real encoder/ICD**. **Suggestion:** short identity + pointer
to protocol SSOT (spec / decision / Starcom docs), not a second copy of wire layout here.
Body constants/APID enums can stay code; banner should not re-host the full packet recipe.

**WN-048** — [Grok] · `comment` · **Very high Doxygen/comment density**  
Locus: whole `telemetry_encoder.h` — heavy `/** @brief / @param / @return */` on methods
plus banner (see **WN-047**). Those tags are **comments only** (doc tools/IDE); no
compile or runtime effect. File reads largely as documentation with a thin API skeleton
(~comment-heavy vs declarations). Headers are exempt from the `.cpp` density band, but
this ratio still sticks out. **Only relevant if the file survives Starcom (**WN-046**);**
if it does, prefer thinner contract (names + rare true invariants) and/or generated docs
from a single SSOT rather than a hand-maintained 70%-comment public header. Suggestion
only — no mid-walk rewrite.

#### `include/rocketchip/mavlink_rx.h`

**WN-049** — [Grok] · `comment` · **SAFETY CONTRACT: keep idea, re-check claims**  
Locus: banner ~L12–16 — dispatcher-only; protocol ACKs; no ARM/pyro/mode; must go
through FD (IVP-67) + HW interlocks (IVP-73). **Relevant / important to restate** for a
GCS command path. **If file survives Starcom (**WN-046**):** evaluate that nothing is
**over-promised or over-authoritative** (e.g. “does NOT execute…” vs actual
`mavlink_rx.cpp` / feed path; IVP numbers as if they are the permanent law vs current
FD/interlock names). Contract should match code; process provenance can point at docs.

**WN-050** — [Grok] · `comment` · **IVP-62 stage line in banner is superfluous**  
Locus: ~L18 `IVP-62: Bidirectional MAVLink Commands (Stage 7: Radio & Telemetry)`.
Does not state a live invariant — land/step archaeology. Drop or move to docs if the
file survives. Instance of broader **W-6** (IVP/council-in-code-comments sweep).

#### `include/rocketchip/led_patterns.h`

**WN-055** — [Grok] · `comment` · **Pattern value-range map lives only in header notes**  
Locus: banner ~L12–21 “Value ranges:” (`0` off; `1–8` cal; `9–11` RX; `12–18` beacon
overlays; `20–27` flight phase; `28–29` pre-arm/boot; `30–36` sensor; `41–46` faults).

**Current?** Yes vs live `k*` blocks in this file (ranges match constants; unused
slots 19 / 37–40 are just gaps). This is real namespace guidance for the LED code
catalog — not stage fluff.

**Problem:** Guidance of that kind should not be SoT only as code-comment prose.
No equivalent compact range map in design docs checked: `docs/decisions/NOTIFY_CONTRACT.md`
has intent→code rows (different concern); `docs/AO_ARCHITECTURE.md` has layers/Stage L
pointers; `docs/USER_GUIDE.md` LED State Reference is operator color/mode only;
`docs/hardware/STATUS_INDICATORS.md` is stale FreeRTOS-era. Implementers/operators
should not have to open a constants header to learn the code namespace.

**Later (not mid-walk):** put the range taxonomy in an accessible design home
(e.g. NOTIFY_CONTRACT or AO_ARCHITECTURE LED section — one SSOT), link thin from
this header if a one-liner remains; do not keep the only copy as file banner notes.
Related class: **WN-047** (banner re-hosts layout), **W-6** (comment as doc home).

**WN-056** — [Grok] · `comment` · **Beacon-overlay essay restates design contract in code**  
Locus: section banner ~L54–64 above `kFdLandedBeacon`…`kFaultCore1StallBeacon` —
2Hz base+white alternate; posted when `beacon_auto`; recovery-crew “preserve base
color”; `beacon_manual` → pure white `kFdBeacon` (CLI findme / GCS); `kFaultSafeMode`
(45) already blue+white so no sibling overlay code.

**Current?** Behavior matches Stage L design: same rules are written out in
`docs/decisions/NOTIFY_CONTRACT.md` (Stage L beacon flags, composition table,
preserve-color rationale, SafeMode “unchanged” row) and summarized in
`docs/AO_ARCHITECTURE.md` (Stage L beacon blurb + pointer to contract).

**Problem:** Unlike **WN-055** (range map missing from docs), this *is* design
guidance already hosted properly — the multi-line code block is a second copy.
Constants may keep short per-line color notes (`// Green + White alt 2Hz`); the
behavior/rationale essay should not live as the working explanation next to
`constexpr`s.

**Later (not mid-walk):** drop or collapse L54–64 to a one-line “see
NOTIFY_CONTRACT Stage L beacon overlay”; keep named codes + thin color tags.
Related: **WN-055**, **WN-047**, **W-6**.

**WN-057** — [Grok] · `ownership` · **`k*Neo*` compat aliases: temp became permanent**  
Locus: ~L119–168  
`// Backward-compatibility aliases (non-namespaced)` +  
`// Used throughout main.cpp, rc_os.cpp, ao_led_engine.cpp until those files`  
`// are migrated to use rc::led:: namespace directly.`

**Claim:** Comment frames a **pending migration** that is not tracked as open
work elsewhere; dual API (`rc::led::k*` vs global `kCalNeo*`/`kFdNeo*`/…) has
hardened into the live surface (Stage L even **added** more `k*Neo*` aliases).

**Trace (2026-08-04 grep):**

| Surface | `rc::led::k*` | global `k*Neo*` |
|---------|---------------|-----------------|
| `notify_backend_led.cpp` + `test_notify.cpp` | full use | none |
| `ao_led_engine.cpp` (`led_apply_pattern` switch + layers) | none | **primary** |
| `ao_rcos.cpp` (`cal_neo` + wizards) | none | **primary** (`kCalNeo*`) |
| `main.cpp` | none | **none** (comment overstates consumers) |
| `test/` (except via notify tests above) | — | no Neo aliases |

Values are equal (aliases = namespaced codes), so runtime is consistent; the
debt is **two vocabularies** + a stale “until migrated” promise.

**Docs:** No PROJECT_STATUS / open-IVP “finish rc::led migration” item found.
`docs/AO_ARCHITECTURE.md` still lists `kCalNeo*` as the module’s export face.
Trim notes (`CODE_TRIMMING_AUDIT`, POST_BIERMAN) treat live `kCalNeo*` as **KEEP**
/ RC_OS territory — i.e. accepted dual-use, not a scheduled close-out.
`ao_signals.h` `LedPatternEvt` comment still says `kCalNeo*` “from
ws2812_status.h” (path wrong; same legacy naming).

**Later (not mid-walk):** (1) decide one public name set (`rc::led::` preferred);
(2) migrate `ao_led_engine` switch + `ao_rcos` cal path; (3) delete alias block;
(4) fix/remove “until migrated” / wrong-consumer comments; (5) align AO_ARCHITECTURE
+ `LedPatternEvt` comment. Same class as **WN-051** (DEPRECATED live with weak
tracking). Related: **W-6**.

#### `include/rocketchip/pcm_frame.h`

**WN-058** — [Grok] · `comment` · **PCM layout / protocol notes must live in design docs**  
Locus: file banner ~L3–22 (55 B layout: sync `0xEB90`, MET, type, len, `TelemetryState`
payload, CRC-16; triple resync gate); Flight Log Header block ~L158–163 (“RCLG”,
version, Council C-A3); decom-table blurb ~L98–103 (ground auto-decode).

**Role (for walk posture):** **Onboard data-logging frame format** (ring/flash,
`ao_logger`, event markers) — not the RF/Starcom air path. Looser Starcom/CCSDS
adjacency exists (council three-layer story: PCM log vs CCSDS radio vs post-flight
repackage; CCSDS logging-related features may appear later) but **not** the same
tight coupling as `telemetry_encoder` / `mavlink_rx` (**WN-046**). Payload type is
`TelemetryState` (shared model).

**Design-home check:** Architecture intent is largely in
`docs/decisions/Telem+logging/` (`council_data_logging.md`, `telemetry_comparison.md`,
`revised_ivp_stages.md` IVP-51 PCM frame, three-layer PCM-onboard / CCSDS-radio).
Header still **re-hosts** the full byte recipe + resync algorithm + flight-header
contract as if it were the working guide.

**Claim (same class as **WN-047** / **WN-055**):** layout, resync rules, and
flight-header semantics are **design guidance** — SSOT should be a proper document
(keep council/IVP current; thin header to identity + pointer + true compile-time
sizes/`static_assert`s). Do not treat code comments as the only or primary place
to learn the format. No mid-walk rewrite. Related: **W-6**.

**WN-059** — [Grok] · `ownership` · **Revisit whether PCM-onboard logging is still the right shape**  
Locus: whole unit + consumers (`src/logging/pcm_frame.cpp`, `ao_logger`,
`flash_flush`, FD `LogEventId`) — Stage 6–era PCM fixed frames (sync + MET + type +
payload + CRC; event frames; 64 B flight header).

**Context since implement:** AO migration, Notify/LED stack, radio/telem evolution,
Starcom planned (link + possibly broader data-path), Stage 17 logging tier plans
still route through this file, dual telem paths (CCSDS/MAVLink air vs PCM flash).
CCSDS can grow logging-related facilities later — **related but not a reason to
freeze or rewrite this leaf mid-walk**.

**Claim:** Confirm the **approach** is still right (PCM fixed frames to flash/PSRAM
ring, raw stream vs FS, `TelemetryState` as payload, separate event frames, decom
table for ground) against current product direction — not a drive-by format tweak.
Questions for later revisit (no close-out now): still right density/rate model?
Still right split vs air protocol after Starcom? Still right event-ID surface?
Economy/Research types vs only Standard live? Any supersession by Starcom/CCSDS
log containers, or keep PCM as intentional onboard layer?

**Later:** dedicated logging architecture re-read (council docs + live logger),
after or coordinated with Starcom telem decisions — **lighter gate than pure telem
headers**, but not “ignore forever.” Related: **WN-058**, **WN-046** (payload
coupling only).

#### `include/rocketchip/fused_state.h`

— nothing of note.

Thin public snapshot struct (`FusedState`: float32 ESKF-internal fields + GPS/health/
confidence/MET). Banner and field notes stay at provenance level (e.g. `vert_vel_eskf`
not raw baro; `baro_alt_rate_mps` for IVP-120) — appropriate for a data layout header,
not a design essay. Deeper core-math / fusion rework analysis is a **later** pass
(not this leaf).

#### `include/rocketchip/flash_layout.h`

**WN-060** — [Grok] · `comment` · **Flash layout map must live in design docs, not as authoritative comments**  
Locus: file banner ~L3–16 (top-down region map: cal / flight table / flash-safe test /
log / firmware) plus region block comments on the `constexpr`s.

**Claim:** Layout is system design — where regions sit, sizes, top-down anchoring from
`PICO_FLASH_SIZE_BYTES`, non-overlap. That guidance must have a proper design-doc
home that can be kept current. Header may keep derived constants + compile-time
checks; comments must **not** be the authoritative picture (stale-risk is high —
banner already lags live code, e.g. radio-config sector). Same rule as prior leaves:
**reference docs, never host the working map in comments.** Related: **WN-055**,
**WN-058**, **W-6**. No mid-walk rewrite.

**WN-061** — [Grok] · `comment` · **Council citation in banner is unnecessary**  
Locus: ~L16 `// Council C-A4: boot validation ensures regions don't overlap firmware.`
(and related ~L78 `// Boot validation [Council C-A4]`). Process provenance, not a
live API note. Drop; `static_assert`s already enforce non-overlap. **W-6**.

**WN-062** — [Grok] · `ownership` · **Early flash-layout feature — re-evaluate later, low priority**  
Locus: whole unit (top-down dual-sector cal/table/radio-cfg, firmware reserve, log
span, compile-time checks). Same class as onboard logging (**WN-059**): landed early
in the project; may still be the right shape, but worth a later architecture pass
after multi-board / Starcom / storage evolution — **not high priority**, not mid-walk.

**WN-063** — [Grok] · `ownership` · **Layout design must stay hardware-agnostic (flash only)**  
Locus: whole file — intended design is portable regions over “some flash,” not an
RP2350 product map. Written when RP2350 was effectively the only board; still uses
Pico SDK anchors (`PICO_FLASH_SIZE_BYTES`, `FLASH_SECTOR_SIZE`, host defaults 8 MB /
4 KB). Fixed `kFlashFirmwareReserve = 512 KB` and dual-sector wear patterns may be
fine, but **all features and intended layouts/designs** in this area should remain
**hardware-agnostic** aside from the inherent need for flash memory (no silent
RP2350-/Feather-only assumptions in the contract). **Later:** confirm docs +
constants express “any board with enough flash,” not one SKU’s era. Related:
**WN-062**, board-pack family (**WN-019**–**WN-022**).

#### `include/rocketchip/prearm_fail_ticks.h`

**WN-064** — [Grok] · `comment` · **Banner hosts council/design prose that doesn’t belong**  
Locus: file banner ~L3–20 — Stage L IVP-L3 tag; full semantics of the counter
(`remaining == 0` / `state_changed` / decrement); post-flow reset on
`AO_Notify_post_prearm_fail()`; “JPL council 2026-04-18” rationale for refresh-on-
repost. Design/process narrative next to a 10-line pure helper. Code may keep a
one-line identity; council land-dates and essay semantics belong in
NOTIFY_CONTRACT / Stage L docs if needed — not here. Related: **W-6**, **WN-061**.

**WN-065** — [Grok] · `ownership` · **Does this need its own public header?**  
Locus: whole file — one `constexpr` (`kPreArmFailTicks = 99`) + one `inline`
pure function `prearm_fail_tick_next()`.

**Fan-in (grep):**
| Consumer | Use |
|----------|-----|
| `src/active_objects/ao_notify.cpp` | include + call/counter (sole firmware user) |
| `test/test_prearm_fail_ticks.cpp` | host unit tests for the pure helper |

No other `src/`/`include/` includes. Extracted for host-testability (Stage L), not
because many modules share it. Same class as thin single-purpose headers
(**WN-032** job_capabilities-style question): could live next to notify
(`ao_notify` private header, `notify_*` helper, or tests via a small notify test
hook) rather than `include/rocketchip/` public surface. **Later:** keep only if
public testability of this helper is still required; else fold. No mid-walk move.

#### `include/rocketchip/station_output_mode.h`

**WN-066** — [Grok] · `ownership` · **Re-evaluate standalone header after RCOS rework**  
Locus: whole file — thin enum + `AO_RCOS_get/set/cycle_output_mode` (Stage 12B
extract, commit `175eb9e`, "Council A3" circular-include break when mode ownership
moved Telem → RCOS).

**Claim:** Need for a **broken-out** public header may go away or change shape once
RC_OS / AO_RCOS is reworked as a proper UX layer. Main-repo whiteboard already has
an open item: **RC_OS table-driven key dispatch / UX architecture**
(`AGENT_WHITEBOARD.md` — table-driven maps, UX vs domain ownership, station/vehicle
gating; points at `docs/ROCKETCHIP_OS.md`, `docs/AO_ARCHITECTURE.md`, CODE_TRIMMING).
**Later:** after that rework, decide keep shared leaf vs fold into RCOS/Telem
surface; don't invest mid-walk. Related: **WN-065** (thin public header class).


## Tier 1 — math/

#### `math/vec3.{cpp,h}`

**WN-071** — [Grok] · `comment` · **Host-purity banner: good intent, not over-authoritative law**  
Locus: `vec3.h` ~L6–7 `// Pure C++ — no Pico SDK dependencies. Must compile on any host.`

**Check:** Currently true — header has no includes; `vec3.cpp` only `vec3.h` + `<cmath>`. Useful **intent** for a foundation math type (host tests + no board coupling).

**Claim:** Do not read as an **inherent eternal property** of vector math or as stronger than build/process enforcement. It is a **project constraint / design goal** for this unit (and similar pure math). If someone later pulls in a Pico header, the comment would lie until updated; the real gate is host-build / include discipline. Prefer wording as intent (intended pure / host-buildable) or a short pointer to build rules, not absolute law. Same class as over-strong SSOT banners (**WN-009**). Related: **W-8** (HW-agnostic guidance).

**WN-072** — [Grok] · `comment` · **Zero comments in vec3.cpp — sparse vs empty**  
Locus: whole `vec3.cpp` (~30 lines) — only SPDX/copyright; no file banner, no note on `kNormEpsilon` / near-zero `normalized()` policy.

**Claim:** Straightforward out-of-line math can stay **sparse** (header already labels the type). Owner doubt: **zero** body comments may still be too low even for simple files — at least the non-obvious policy (epsilon floor → zero vector) is behavior callers rely on and is invisible from the header. Not a demand for essay density; **later** decide a minimal floor for tiny `.cpp` (one-liner on edge policy vs accept pure code + tests). Related: **WN-054** / **W-7** (density band is about excess more than a floor).

#### `math/quat.{cpp,h}`

**`quat.cpp`**

**WN-073** — [Grok] · `invariant` · **In-source NOLINT on DCM indices (disallowed policy)**  
Locus: `quat.cpp` — four `// NOLINT(readability-magic-numbers)` at ~L77
(`to_rotation_matrix(float m[9])`), ~L95 `m[5]`, ~L97 `m[6]`, ~L98 `m[7]`.
Grep: no other NOLINT/suppress in this file.

**Claim:** Same policy as **WN-043** / **WN-070** — in-source NOLINT suppressions
are **not allowed**. Fixed 3x3 DCM slots should use named constants / clearer
structure if the linter fires, or a logged deviation — not local NOLINT. **Later:**
remove all four; do not treat math DCM code as a NOLINT exception. Related:
**WN-043**, **WN-070**.

**`quat.h`**

— nothing of note.

#### `math/mat.h`

**WN-074** — [Grok] · `ownership` · **Filename `mat.h` too vague; consider `matrix`**  
Locus: path `src/math/mat.h` (type `Mat<R,C>`, aliases `Mat3` / `Mat15` / …). House
short form from Phase 5 (`Matrix.h` → `mat.h` next to `vec3` / `quat`). Owner:
**`mat` reads as “math”** and is too vague; **`matrix`** would be clearer.

**Claim:** No technical reason the path must stay short. **File** rename
`mat.h` → `matrix.h` is a **small** cascade (~6 `#include "math/mat.h"` sites +
doc path citations). Renaming the **type** `Mat` → `Matrix` is a **large** API
churn (ESKF + tests) and is **not required** for the vagueness fix.

**Later:** filename-only rename (+ includes + docs); leave type `Mat` unless a
separate API cleanup. No mid-walk rename.

**WN-075** — [Grok] · `comment` · **Host-purity / float authority lines — same class as vec3**  
Locus: ~L6–10 “Pure C++ — no Pico SDK… Must compile on any host.” + “All float, no
double. Static storage, no heap…” (+ Sola ref). Same pattern as **WN-071** (`vec3.h`)
and `quat.h` purity blurb.

**Claim:** Useful **intent**, not inherent eternal law, and should not be the **only**
place the pure-math / host-build / float32 rules live — re-evaluate whether this
authority belongs in a shared math or build-doc home, with headers **pointing**, not
re-stating absolute law. Related: **WN-071**, **W-8**.

#### `drivers/i2c_bus.{cpp,h}`

**WN-076** — [Grok] · `comment` · **IVP / process tags in header are tracking noise**  
Locus: `i2c_bus.h` ~L125–126  
`// Bus Recovery (IVP-13a)`  
(and any similar council/IVP provenance on this unit).

**Claim:** IVP step / council land-tracking belongs in **IVP / CHANGELOG / LL**, not
as permanent section labels or essay provenance in a public driver header. Drop the
ticket tags; keep only live API identity if needed. Same class as **WN-061**,
**WN-064**, **W-6**.

**Later:** strip process tags on disposition pass; no mid-walk edit.

**WN-077** — [Grok] · `comment` · **Banner + recovery docs over-authoritative / stale risk**  
Locus: `i2c_bus.h` banner ~L3–8  
`Uses I2C1 on GPIO 2 (SDA) and GPIO 3 (SCL) - the STEMMA QT connector`  
while body is board-abstracted (`board::kI2c*`, `BOARD_I2C_INSTANCE`); also
recover/reset/imu Doxygen (~L129–158) that re-hosts algorithm/history
(“toggles SCL up to 9 times…”, “after rapid flash cycles…”) as if the comment
were the SSOT.

**Claim:** Comments must not **over-claim** a fixed board pinout or connector brand
when pins come from board packs, and must not be the **only** home for critical
bus/recover contracts (stale when packs or recovery policy change). Prefer short
API notes + pointer to board packs / LL / design docs; do not restate absolute
pin maps or process narrative. Related: **WN-071**, **WN-060**, **W-6**, **W-8**.

**Later:** banner → board-pack identity only; shrink Doxygen to return/precondition
notes; host full recover story in LL/design if still needed.

**WN-078** — [Grok] · `ownership` · **HW-/device-specific surface must leave or go universal**  
Locus: banner STEMMA/GPIO pin claims; product connector framing; public
**`i2c_bus_imu_recovery`** (~L149–159) + ICM-20948 bank/PWR_MGMT constants in
`.cpp` (~L285–326) on a **generic bus** unit; known-address table
(`kI2cAddrDps310` / `Icm20948` / …) is product-sensor inventory on the bus façade.

**Claim:** Multi-board direction (**W-8** / **WN-063** class): HW-specific
assumptions in this driver must be **removed or made universal** — pins/instance
only via board packs (already partly done in code; comments/API still leak
Feather-era STEMMA); device latch recovery belongs with the **IMU driver** (or a
named device recovery hook), not as a permanent `i2c_bus_*` product of one chip.
Bus layer keeps init/transfer/recover-generic; product addresses may stay as thin
constants or move next to drivers.

**Later:** rework-eval with Early-impl group / optional PIO-I²C backend; no
mid-walk refactor.

**WN-079** — [Grok] · `comment` · **Prior-art block: keep only used PA, at use sites**  
Locus: `i2c_bus.cpp` file banner ~L7–10  
`Prior Art: NXP UM10204…; Linux kernel i2c-algo-bit.c…; Pico SDK hardware/i2c.h…`  
Recovery body already cites used sources at the algorithm (~L200 NXP 3.1.16;
~L205–218 “Linux kernel pattern” on SCL-stuck + per-pulse SDA).

**Claim:** Prior-art comments are fine **when that art was actually used** in the
implementation. A top-of-file “we looked at / we use SDK” list is superfluous if
nothing distinctive was taken, or if the real inspiration is already (better)
noted at the lines that implement it. Pico SDK `hardware/i2c.h` is the **platform
API**, not project prior-art research — listing it as PA is noise. Prefer: no
banner PA dump; **at the recovery helpers**, short cite of NXP/Linux only where
the 9-clock / early-exit / SCL-stuck patterns live; drop unused or pure-SDK
entries entirely.

**Later:** relocate or trim; do not invent PA essays mid-walk. Related: **W-6**,
standards prior-art rule (cite what informed the design, not a bibliography).

**WN-080** — [Grok] · `ownership` · **Scan device-name `switch` embeds product HW inventory**  
Locus: `i2c_bus.cpp` ~L94–122  
`switch (addr) { case kI2cAddrDps310: … "DPS310 Barometer"; … }`  
plus supporting inventory: header `kI2cAddr*` (~L38–41), `.cpp` alt addrs
`kI2cAddrIcm20948Alt` / `Dps310Alt` (~L22–24), GPS skip
`addr != kI2cAddrPa1010d` (~L95–97). Section comment `// Identify known devices`
is vague. (Same class as hardwired product map on a generic bus unit —
extends **WN-078**.)

**Claim:** This is literally **known hardware**, not a universal bus primitive.
An exemption for “diag may name expected sensors” is possible but weak long-term:
codebase is **RC HW in mind, not only for it** — hardwired peripherals will differ
by board (onboard vs STEMMA vs none). A **more robust** home is a board-owned
inventory (or table) that `i2c_bus_scan` **reads**, rather than a switch inside
the bus driver. Project already has that layer: **board packs**
(`include/rocketchip/board_*.h` via `board.h`) for pins/instance/capability flags —
not **mission profiles** (`profiles/*.cfg` / generated mission data: flight params)
and not **job packs** (`job_vehicle` / `station` / role). **Later** options (pick in
disposition, not mid-walk): (1) board-pack table `{addr, label, probe_ok?}`;
(2) thin `board_i2c_devices.h` included only from scan; (3) scan prints raw addrs
only, labels live in ground tools. Drop vague “Identify known devices”; no
historical/process tags on these lines. Related: **WN-078**, **W-8**, board family
**WN-019**–**022**.

#### `drivers/gps_pa1010d.{cpp,h}`

— nothing of note *(header / first pass; Doxygen policy → **WN-081**).*  
*(Owner second pass on `.cpp` comments → **WN-082**–**084** below.)*

**WN-082** — [Grok] · `comment` · **Prior-art banner: same check as i2c — used only, prefer use-site**  
Locus: `gps_pa1010d.cpp` ~L9–12  
`Prior Art: CDTop PA1010D datasheet; Adafruit … chunked reads; lwGPS …`  
Use-site already cites Adafruit/Quectel/SparkFun on the filter (~L113–117) and
buffer size (~L90–95 GlobalTop/Quectel + pico-examples).

**Claim:** Same rule as **WN-079**: keep PA only if actually used in the design;
no top-of-file “we looked at / we vendor” bibliography. Datasheet + Adafruit
padding filter **were** used (body cites them) — those cites belong **at the
helpers**, not as a third copy in the file banner. “lwGPS vendored” is include
graph, not prior-art research. **Later:** trim banner PA; keep short use-site
cites. Related: **WN-079**, **W-6**.

**WN-083** — [Grok] · `comment` · **R-2 / R-5 / council dev-record blocks have no place in code**  
Locus: `gps_pa1010d.cpp` ~L35–44  
`// R-2 absorbed (R-5 Unit D part 2a, 2026-05-16, council-approved): …`  
through NASA/JPL framing and `.rodata` trade essay; also rehashed at ~L219–221
and ~L267–276 (`R-5 … council Option X` / Tier 5 scope / commit-message
byte-on-wire baseline).

**Claim:** These are **development records** (which ticket absorbed which, which
council option, which tier commit avoided which file) — permanent home is
**CHANGELOG / plans / council docs**, not multi-line banners next to
`constexpr` PMTK strings or `get_debug_status`. Code may keep *what the constants
are* (checksum-verified sentences); drop session archaeology. Related: **W-6**,
**WN-076**, **WN-085**.

**WN-084** — [Grok] · `comment` · **Other large comment islands: protocol essays + process noise**  
Locus (same file), additional blocks beyond **WN-083**:

| ~Lines | Smell |
|--------|--------|
| ~L81–84 | “Grok-triage instrumentation…” — full session narrative; **brief** “capture PMTK results for post-USB `b` status (early init before CDC)” is enough; detail → commit/CL (**WN-085**) |
| ~L90–95 | Buffer-size essay (Arduino Wire 32, 5.8 ms @ 400 kHz, 10 of 1000 IMU cycles) — timing rationale partly useful; comparative history / pico-examples ref → docs or one line + pointer |
| ~L104–118 | `read_nmea_data` Doxygen: three packet types + Adafruit filter story — protocol contract that can go stale if only here; prefer short behavior + app-note/datasheet pointer (same class as **WN-077** / **WN-060**) |
| ~L214–221 | Blind-PMTK cold-boot window essay + R-5 rehash — live **why** (send before probe / cold-boot window) is worth **one short** invariant; ticket/regression narrative → CL/LL |

**Claim:** Not every multi-line comment is wrong — keep **live invariants** (blind
config before probe; keep LF only after CR; full 255-byte read). Strip or rehome
**dev comments**, duplicated R-5 stories, and long “info that should live in docs.”
Related: **WN-083**, **WN-085**, **W-6**, **WN-081**.

#### `drivers/gps_uart.{cpp,h}`

— nothing of note.  
*(Owner-closed leaf; itinerary ticked 2026-08-05. Concurrency ring hot-spot noted
on itinerary for disposition-time check if needed — no WN filed this walk.)*

#### `drivers/gps.h`

**WN-087** — [Grok] · `comment` · **Transport-neutral intent OK; mark done vs WIP clearly**  
Locus: file banner ~L3–11  
`Shared by all GPS transport backends (I2C, UART, SPI)` +  
`Pattern: transport-neutral types + transport-specific backends` +  
`See also: gps_pa1010d.h (I2C), gps_uart.h (UART)`  
and ~L36–37 `All GPS backends (I2C, UART) produce this struct` vs enum
`gps_transport_t` (~L81–85): `NONE` / `I2C` / `UART` only — **no SPI**.

**Claim:** Intent of a **transport-neutral data contract** for all GPS backends
is fine and matches use (PA1010D + UART both fill `gps_data_t`; consumers use
this type). **Not** a problem that the design looks forward. Problem is **false
completeness**: banner lists **SPI** as a peer of I2C/UART while nothing
implements SPI GPS and the transport enum doesn’t name it. Same class as board
scaffolding looking first-class (**WN-021**): better tracking of **what is
actually implemented** vs **downstream / WIP**. Keep the intent block; make
done vs planned explicit (e.g. “I2C + UART backends live; SPI not implemented”
or drop SPI until it exists). Optional: note data shape is **NMEA-class product**
(not every possible GNSS protocol) if that avoids over-reading “all GPS.”

**Later:** one-line status in banner; no mid-walk redesign of `gps_data_t`.
Related: **WN-021**, **WN-027** (WIP labeling).

#### `drivers/icm20948.{cpp,h}`

**`icm20948.h`**

— nothing of note *(path-specific).*  
*(Doxygen density: wall of `@brief`/`@param`/`@return` on nearly every type and
API — owner estimate ~**6:1** comment:code mass on this header; evidence for
project-wide **WN-081** / density **WN-054**, not a separate path claim.)*

**`icm20948.cpp` — banner accuracy (checked, no separate WN)**  
File banner ~L7–20 (PA: ICM-20948 DS-000189, AK09916, ArduPilot bypass; bank
architecture; BYPASS_EN + AK at 0x0C; internal master eliminated). **Checked
against implementation this sitting:** code uses bank select `0x7F`, bank0/2
regs only (bank3 intentionally absent), `enable_bypass_mode` + direct
`ak09916::kI2cAddr` `0x0C`, no SLV0/I2C-master path — matches narrative.
ArduPilot cite is **pattern prior art** (used — bypass commit `29c1676`), not a
line-sync claim. Datasheet **rev numbers** not re-fetched against PDFs this
sitting (process: verify at disposition if citation rot matters). Treat banner
as **accurate to the driver**; keep form (used PA + live architecture).

**~L210–213 `enable_bypass_mode` order comment — keep (not a process dump)**  
`Must disable I2C master FIRST… then BYPASS_EN` is a **live HW sequencing
invariant** next to the only place that does it — not R-5/session archaeology.
Body matches (clear `kI2cMstEn`, then set `kBypassEn`). Leave unless shortened
for style; not **WN-083**-class.

**WN-088** — [Grok] · `invariant` · **Substantial in-source NOLINT magic-number regions**  
Locus: `icm20948.cpp` — **only** these blocks (full-file grep; no other NOLINT
in this file):

| Lines | Scope |
|-------|--------|
| ~L468–536 | `NOLINTBEGIN/END(readability-magic-numbers)` around `parse_accel_gyro_temp` **and** entire `read_mag_bypass` (buffer indices + mag LE pairs) |
| ~L581–585 | accel XH/XL/… byte pairs in `icm20948_read_accel` |
| ~L609–613 | gyro pairs in `icm20948_read_gyro` |
| ~L634–645 | AK09916 ST1/ST2/axis offsets in `icm20948_read_mag` |

**Claim:** Same **disallowed** class as **WN-043** / **WN-070** / **WN-073** —
in-source NOLINT is not the project remedy. Volume here is high (one BEGIN spans
~70 lines including logic that is not “just offsets”). Prefer named `constexpr`
byte indices / layout helpers (or shared parse helpers) so tidy is clean without
suppression. **Provenance:** clang-tidy P3 magic-number work `5ee49b9` (2026-02-09)
explicitly allowed NOLINT for “burst read buffer byte indices” alongside
constexpr extraction — that commit family is the place to re-audit for **other
files** that got the same treatment (grep `NOLINT` under `src/`: e.g.
`sensor_core1` / `calibration_data` / `rc_os_commands` DCM indices — already
partially ticketed elsewhere). **Later:** strip all four regions in this driver;
sweep sibling suppressions from that remediation era. No mid-walk mass delete.

**WN-089** — [Grok] · `ownership` · **Lazy mag re-init on hot path needs recreate/test**  
Locus: `read_mag_bypass` ~L522–528  
`// Mag lost after device reset — attempt lazy re-init once per divider cycle`  
`init_magnetometer(dev);` (~220ms comment; path does reset settle 100ms +
bypass/config sleeps — order ~100ms+ real).

**Claim:** Looks like a **one-off recovery** for “mag_initialized cleared / lost
after reset” rather than a fully characterized design. Landed with bypass
migration (`29c1676`, 2026-02-10). Calling `init_magnetometer` from the **1 kHz
read path** (even throttled by `kMagReadDivider`) can block for **hundreds of ms**
on Core 1 — high stakes if still live. Owner: **recreate the failure mode and
test** a proper fix (or prove path is dead/unreachable) before trusting or
deleting. Do not “clean comment only.” Related: **WN-086** (bespoke driver
re-eval), dual-core I²C discipline.

#### `drivers/baro_dps310.{cpp,h}`

**WN-090** — [Grok] · `comment` · **OS/rate selection table belongs in HW/sensor doc (header may keep thin pointer)**  
Locus: `baro_dps310.h` ~L23–51 — multi-line table (noise Pa / alt m / meas time /
current / max rate by OS 1×–128×), ArduPilot 16× note, flight vs ground
recommendations, duty-cycle model (`CONT_BOTH`), Stage 10 phase-scheduled OS
future, then the four `kBaroDps310*` constexprs.

**Claim:** Content is **useful** and on further look may be acceptable *near* the
tuning constants, but it is **fairly important system/sensor guidance** that should
have a proper home in a **HW- or sensor-specific guideline/doc** (if not already a
straight copy of Infineon datasheet Table 16 + project choices) so it cannot go
stale only in a header. Code may keep short rationale + pointer; full table and
“future Stage 10” narrative should not be the sole SSOT. Same class as **WN-055** /
**WN-060** (maps/contracts in comments). **Also this leaf:** heavy Doxygen on the
API surface + this table → **substantial comment:code ratio** — evidence for
**WN-081** / **WN-054**. Full Doxygen file inventory → walk WB **W-10** (not
extended here). **Later:** confirm table vs datasheet; rehome or cite doc; no
mid-walk delete of useful numbers.

**WN-091** — [Grok] · `comment` · **Prior-art banner: check used-only / use-site (same class)**  
Locus: `baro_dps310.cpp` ~L9–12  
`Prior Art: Infineon DPS310…; ruuvi.dps310.c…; Adafruit DPS310…`  
Implementation is a **ruuvi wrapper** + I2C callbacks (vendor path is real);
datasheet informs OS table in **header** (**WN-090**); Adafruit “init sequence
reference” not cited at any specific helper line this sitting.

**Claim:** Same rule as **WN-079** / **WN-082**: keep PA only if used; prefer
use-site cites; “vendored ruuvi” is dependency identity more than research essay.
**Later:** verify Adafruit actually informed init; trim or relocate.

**WN-092** — [Grok] · `ownership` · **Atmospheric / hypsometric constants in baro driver; wider universal-SSOT audit**  
Locus: `baro_dps310.cpp` ~L24–27  
`kStdAtmPressurePa`, `kHypsometricScale`, `kHypsometricExponent`  
(+ use in altitude helper ~L196). Not DPS310 silicon constants — **ISA / formula**
physics shared by any pressure→altitude path.

**Claim:** Such values should **not** live in a HW-specific driver unless there is
a datasheet-tied reason (there isn’t). Need a **more universal home** and a
**reliable SSOT** (shared math/atmos header, fusion, or constants module). This
leaf is a seed for a **wider audit**: domain constants (atmosphere, gravity,
unit scales, Earth models, …) currently parked next to one peripheral — find
and rehome. Related: **W-8** (agnostic placement), **WN-086** (driver scope).

**WN-093** — [Grok] · `invariant` · **NOLINT identifier-naming for ruuvi callbacks; duty-cycle pointer**  
Locus: ~L69–73 (forward decls) and ~L95–123 (`pico_read` / `pico_write` bodies)
`NOLINTBEGIN/END(readability-identifier-naming) — params match ruuvi…`;  
~L140 `// See baro_dps310.h for duty cycle model and tradeoff table.`

**Claim (NOLINT):** In-source NOLINT still **disallowed policy** class (**WN-043**
family) even when matching third-party typedef names — better: thin C wrappers
with project names calling ruuvi, or logged TP deviation for the callback
surface only. Only these two regions in this file (grep). **Claim (L140):**
comment correctly **depends** on the header table (**WN-090**); any rehome or
edit of that duty-cycle/OS guidance must update **at least this pointer** (and
config values if numbers change). Not a free-floating essay — a coupling note.

#### `drivers/rfm95w.{cpp,h}`

**WN-096** — [Grok] · `invariant` · **Datasheet-backed constants: good pattern; schedule deeper verify**  
Locus: both halves — header banner ~L10–11 Semtech `DS_SX1276-7-8-9_W_APP_V7` +
RadioHead; “Table 41” reg list (~L24–57); BW codes (~L87–90); airtime §4.1.1.6
(~L361); plus `.cpp` init/register use that must stay consistent with the same
map.

**Claim:** Direct datasheet references on HW drivers are **desirable** (do this
for all HW drivers). Datasheets essentially don’t change → **comment rot risk
negligible** for the cite itself. **Preliminary check this sitting (not final):**
reg addresses and common bit encodings match the standard SX1276 LoRa map;
`kLoRaMode` / mode nibbles match `.cpp` `set_mode`; version `0x12` and BW
0x07/08/09 look correct. **That is not enough.** Owner: a **deeper dive** is still
in order at disposition — line-by-line against the named PDF rev (Table 41 vs
LoRa map section, PaDac/PA_BOOST, IRQ flag bits, audit expected values, airtime
formula assumptions, RadioHead deltas). Record pass/fail then; do not treat the
walk skim as closed verification.

**WN-097** — [Grok] · `ownership` · **RFM95W / LoRa driver: defer non-critical work past Starcom**  
Locus: **whole pair** `rfm95w.{cpp,h}` (and by extension project LoRa radio path
consuming it).

**Claim:** This is a prime **Starcom-adjacent** surface — either impacted by
Starcom link/PHY work, or a template for **what Starcom may need to provide** if
HW radio drivers stay in project scope. **Non-critical** cleanup/refactor on this
chunk should **wait until after Starcom** work clarifies ownership and API
shape. **Exceptions:** true critical defects (safety, silent link failure, bus
corruption) may still be fixed sooner. Aligns with other Starcom-gated radio/telem
candidates (**WN-041**, **WN-046** class). Walk may still file comment/policy
notes; disposition of structural rework waits.

**WN-094** — [Grok] · `comment` · **Superfluous IVP / council tags on RFM95W header**  
Locus (examples): `// IVP-T11:` on Lna/ModemConfig3/InvertIQ (~L36,51,53);
section `// Boot-Time Audit (IVP-T11)` (~L95); Doxygen “IVP-132a.4 re-eval”
(~L167); “IVP-T11” on `rfm95w_read_audit` (~L183); also “Council Amendment #5”
(~L25), “Council #3/#1/#6”, “Council C3-R3”, “Stage T Batch B prelim” in API
blocks.

**Claim:** Ticket/council land tags are **process tracking**, not live API
contract — same class as **WN-076** / **W-6**. Drop tags; keep short technical
notes (e.g. LnaBoostHf, AgcAutoOn, optional peripheral, RegIrqFlags vs DIO0)
where they help. Detail stays in CHANGELOG/IVP.

**WN-095** — [Grok] · `comment` · **Heavy Doxygen on rfm95w.h — inventory seed**  
Locus: whole public API (~L147–376) dense `@brief`/`@param`/`@return` plus
multi-line usage essays (non-blocking TX block ~L210–223, airtime/timeout
Stage T prose ~L345–352).

**Claim:** Another substantial Doxygen leaf — evidence for **WN-081** / **WN-054**;
add to walk WB **W-10** inventory seeds (`rfm95w.h`). Prefer short contracts;
long GDB/usage/history → docs or LL. No mid-walk mass strip.

**WN-098** — [Grok] · `comment` · **PA banner OK-to-check; council amendment list is process dump**  
Locus: `rfm95w.cpp` ~L7–18  
`Prior Art: SX1276 datasheet; RadioHead RH_RF95; Adafruit adafruit_rfm9x` +  
`Council amendments incorporated: #1…#6` multi-line list.

**Claim:** PA entries — same check as **WN-079** class (used only; datasheet/RadioHead
clearly inform driver; Adafruit cite verify at use-site or trim). **Council #1–#6
block** is a land-record index (belongs in CHANGELOG/plan), not permanent file
prologue — drop or one-line “see Stage 7 / plan” (**W-6**, **WN-094**). Citing a
**coding rule** next to code is fine/encouraged; a numbered council shopping list
is not that.

**WN-099** — [Grok] · `comment` · **Section banner “==== … (JSF AV Rule 151)” is vague**  
Locus: ~L27–29  
`// ============================================================================` / `// File-scope constants (JSF AV Rule 151)` / `// ====…`

**Claim:** `====` is only a **visual section break** (same style as other
banners in this tree), not a language or standards construct — reads as a
“major break” without saying what. **JSF AV Rule 151** (prefer named constants
over magic numbers) as **reasoning** for *why* these are `constexpr` is good and
encouraged — but the header doesn’t say that; it just tags the rule. Prefer e.g.
“Named constants (JSF AV 151 — no magic numbers in body)” or drop the rule ID if
the names already document intent. Not a defect in the constants themselves.

**WN-101** — [Grok] · `ownership` · **Radio module packaging (FeatherWing vs breakout) needs clear abstraction**  
Locus: ~L180 “FeatherWing is not stacked”; banner/product framing RFM95W
FeatherWing #3231; driver is really **SX1276 + SPI + CS/RST/DIO pins**.

**Claim:** Same silicon can sit on **FeatherWing, breakout, or hardwired board**.
Driver API already takes pins (good), but comments/assumptions still **Wing-
centric**. Differences (shared GPIO, DIO routing, reset polarity, presence
detect) may need an explicit **board/pack compatibility layer** (board packs
already supply pins — extend for radio presence, DIO reliability, packaging
notes) rather than driver special cases. Not full “HAL rewrite” mid-walk; design
when radio ownership settles (**WN-097** Starcom). Related: board packs
**WN-019**–**022**, **WN-102**.

**WN-102** — [Grok] · `ownership` · **Fruit Jam DIO0 / RxDone: board-specific path in generic driver**  
Locus: `rfm95w_available` ~L337–344  
`GPIO DIO0 polling unreliable on some boards (Fruit Jam GPIO5 shared with
Button3…)` → uses **IRQ register** instead of DIO0.

**Claim:** Looks like a **board-specific workaround** embedded in the common
driver (and following “register is authoritative” path). Prefer board capability
/ radio-pack policy (“trust DIO0” vs “IRQ flags only”) rather than naming Fruit
Jam inside `rfm95w.cpp`. May couple with **WN-101**. Defer non-critical reshape
with **WN-097** unless link bugs force sooner.

**WN-103** — [Grok] · `comment` · **Council #6 poll_irq: relevant but temp / unfinished ISR story**  
Locus: `rfm95w_poll_irq` ~L347–350  
`// Council #6: Isolated poll function for future ISR swap. Currently polls
GPIO; future version can check a flag set by ISR.`

**Claim:** Slightly more relevant than pure ticket tags (documents **intent** of a
thin poll hook), but still reads as **temporary / unfinished** design. Either
**label explicitly as interim** + track finish (ISR or board-policy drop of GPIO
poll), or complete via board/HAL path (**WN-101**/**WN-102**). Don’t leave
“future ISR” forever without a ticket outside the source. Related: **WN-094**,
**W-6**.

*(Also ~L160 IVP-T11 one-liner — same process-tag class as **WN-094**; drop tag,
keep “caller compares kAudit*Expected” if useful.)*

#### `drivers/spi_bus.{cpp,h}`

**WN-104** — [Grok] · `ownership` · **Generic SPI bus framed as SX1276 / FeatherWing one-off**  
Locus: `spi_bus.h` banner ~L3–13 “SPI bus driver for **LoRa FeatherWing**”;
GPIO-CS story tied to **SX1276** FIFO bursts; ~L33 `5 MHz (SX1276 supports…)`;
`spi_bus_init` Doxygen ~L40–44 still names **SPI0** and **GPIO 20/22/23** while
body is board-abstracted; **~L55–63** (and write twin ~L67–68)
`SX1276 SPI protocol: CS low → send (reg & 0x7F)…` / `(reg | 0x80)`.

**Claim:** Thin SPI + GPIO-CS for multi-byte bursts is a **reusable protocol
pattern** (and PA cites RadioHead/LoRaMac/Adafruit for that). Framing the whole
unit as FeatherWing/SX1276 — and baking **device-specific reg R/W framing** into
`spi_bus_*_reg` — couples the bus layer to one peripheral. Tight protocol↔chip
coupling is real, but prefer: generic SPI init/transfer/CS helpers, with
**SX1276 framing at the radio driver** (or a named `sx1276_spi_*` thin), so
future SPI devices don’t inherit one-off bloat. Same class as **WN-078** /
**WN-101** (device/board leakage into shared bus). Related: **W-8**, **WN-097**.

**WN-105** — [Grok] · `comment` · **Council/IVP on g_spi_error_count; brief trace OK**  
Locus: ~L101–104  
`// IVP-132a.4 (ArduPilot council #4): SPI hot-path error counter…`  
(+ soak/GDB usage notes).

**Claim:** Multi-line process provenance is the usual **W-6** smell. Owner
clarification for this class of comments: a **brief** “council decided
(date and/or commit hash for trace)” **can be OK** for traceability — not a full
council essay, not a bare ticket with no pointer. Prefer e.g. one line on *what
the counter is* + optional `see <hash> / CHANGELOG` over “IVP-132a.4 (ArduPilot
council #4)” as permanent API surface. Apply when cleaning process tags
elsewhere (**WN-094**, **WN-076**, **WN-085**).

**WN-106** — [Grok] · `comment` · **PA / banner: whole file framed as SX1276 task**  
Locus: `spi_bus.cpp` ~L7–12  
`GPIO-controlled chip select for SX1276 burst FIFO compatibility` +  
`Prior Art: Pico SDK hardware/spi.h; SX1276 datasheet (burst FIFO…)`.

**Claim:** Same PA rules as **WN-079** batch: SDK is platform API not research
essay; SX1276 datasheet is real for **bit-7 R/W + held-CS burst** — but the
banner makes the **entire unit** sound like one radio job. Prefer use-site /
split naming when dispositioning with **WN-104**/**WN-108**. Batch with other
driver PA banners.

**WN-107** — [Grok] · `comment` · **IVP-132a.4 on g_spi_error_count definition**  
Locus: ~L26–29 (same counter as header ~L101–104).

**Claim:** Process tag class — apply **WN-105** (brief council + date/commit OK;
drop bare IVP essay). One place for the comment is enough (header or cpp).

**WN-108** — [Grok] · `ownership` · **RED FLAG: named like universal SPI bus, behaves as SX1276 SPI**  
Locus: whole `spi_bus.{cpp,h}` — name + board-abstracted init (pins/instance)
read as **generic bus**; implementation is **register framing + Mode0 + 5 MHz
defaults for SX1276** (`kSpiReadMask`/`kSpiWriteFlag`, Mode 0 comment “SX1276
requirement”).

**Fan-in (this sitting):** production `spi_bus_read_reg` / `write_reg` / burst
callers are **`rfm95w.cpp` only** (plus `spi_bus_init` from `main`). No second
SPI device uses this “bus.”

**Claim:** **Huge red flag** if the intent was a universal SPI layer — this is
effectively a **single-device (LoRa/SX1276) SPI helper** wearing a generic name.
Either (1) rename/move under radio (`sx1276_spi` / next to `rfm95w`) and keep
truth in the name, or (2) split true generic SPI (init, transfer, CS hold) from
device framing so the universal path is real. Do not grow one-offs here. Extends
**WN-104**; pairs **WN-097** / **WN-101**. No mid-walk rename.

**WN-109** — [Grok] · `ownership` · **Header + cpp: same red flag — HW-specific disguised as universal**  
Locus: **both** `spi_bus.h` and `spi_bus.cpp` (single claim spanning the pair).

**Claim:** Tie **WN-104** (header: FeatherWing/SX1276 framing, reg R/W protocol
docs, stale SPI0 pin Doxygen) and **WN-108** (cpp: SX1276 masks/Mode0, sole
`rfm95w` fan-in) into one picture: this is **not** “universal code with a few
HW comments” — it is **HW-/device-specific SPI for SX1276** that **looks**
universal (name `spi_bus`, board pins, init). Direction of the bug is
**specific → fake-general**, not the reverse.

**Disposition spectrum (later, not mid-walk):**
1. **Simple rename / rehome** under radio (`sx1276_spi` / next to `rfm95w`) if
   scope stays one device — may be enough if no second SPI client is planned.
2. **Further work** if a real multi-device SPI bus is needed: split generic
   transfer/CS from device framing; invent no more one-offs in a fake bus layer.

Related: **WN-104**, **WN-106**–**108**, **WN-078** (i2c device leakage),
**WN-097**. No mid-walk rename.

#### `drivers/mcu_temp.{cpp,h}`

**WN-110** — [Grok] · `ownership` · **MCU-temp vs generic ADC driver; comment mass**  
Locus: `mcu_temp.h` banner ~L3–17 (Stage 16C IVP-142a; RP2350 die sensor ADC
input 4; formula; **ADC-consumer single-owner** caveat if battery ADC later) +
API comments ~L27–50; `.cpp` is `adc_init` / `adc_select_input` / `adc_read` +
temp formula only.

**Claim:** Header **already states** multi-channel ADC sharing risk — good. Same
class of question as **spi_bus** / device-specific “bus” units: should this
exist as **`mcu_temp` only**, or as a small **ADC driver** (channel select,
serialize consumers, Vref) with die-temp as the first client? Today only use is
MCU temp, but an ADC-shaped module would **guide better PA search** (Pico SDK
examples, shared ADC HALs) than “die temperature driver.” HW/code coupling
quirks for ADC (package channel 4 vs 8 already in cpp, exclusive `adc_select`)
belong in that deep dive — not ignored. Banner also carries IVP process tag
(**W-6**).

**Also header:** large **non-Doxygen** comment:code ratio (file banner + dense
per-API notes) — density evidence; not structured Doxygen (**WN-081** / **W-10**
are Doxygen-specific; this is plain comment mass / **WN-054** class).

**Later:** re-eval shape (keep thin temp API vs adc_* + temp helper); PA under
ADC; process-tag clean. No mid-walk rewrite. Related: **WN-108**/**109** (fake-
universal vs narrow), **WN-086**.

**WN-111** — [Grok] · `ownership` · **Temp ADC channel A/B: package toggle vs board/SKU**  
Locus: `mcu_temp.cpp` ~L25–32  
`#if PICO_RP2350A` → `kTempAdcInput = 4` else `8` (RP2350B / Fruit Jam comment).

**Claim:** If this unit stays **die-temp-specific** on RP2350, package-level
channel choice may legitimately live here (feature is RP2350A/B silicon). Still
**SKU/board-coupled**: difference is package/board class, not “temp algorithm.”
Only A/B today, but a **board-pack / SKU capability** (`board::kMcuTempAdcInput`
or similar) is more robust if channels ever diverge further or another MCU
joins — same pattern as other pin/instance board constants. Deep-dive with
**WN-110** (ADC vs temp shape). Related: **W-8**, board packs.

**WN-112** — [Grok] · `comment` · **Stuck-detector essay vs short invariant**  
Locus: ~L35–41 multi-line rationale for `kStuckThresholdSamples = 60` (1 Hz,
bench 0.93°C spread, 60s floor…); used at ~L71–79 (and `mcu_temp_is_stuck`).

**Claim:** Useful engineering judgment, but a **long essay for one constant** /
simple consecutive-identity loop. Prefer short invariant (“60 identical samples
@ ~1 Hz ⇒ stuck”) + pointer to bench note/CHANGELOG if needed; full narrative
is **W-6**/doc territory. Related: **WN-110** density.

#### `drivers/ws2812_status.{cpp,h}`

**WN-113** — [Grok] · `ownership` · **Name “status” collides with notify engine; role is pattern/indication driver**  
Locus: path/name `ws2812_status.*`; header banner ~L3–12 “status LED driver” /
“ArduPilot-style **status indication patterns**” (solid/breathe/blink/…); types
`ws2812_mode_t`, `ws2812_set_*`.

**Claim:** **Notify engine** (`ao_notify`, `notify_backend_led`, intents) is
supposed to own *what* the vehicle is telling the user. This unit looks like a
**HW driver of indication patterns** (PIO/WS2812 + mode rendering) that notify
(and LedEngine) talk **to** — not a second status policy engine. The word
**status** in the filename freezes the wrong layer and invites one-way-or-the-
other confusion. Banner already says **patterns**; rename lean:
`ws2812_patterns` / `ws2812_indications` (or similar) so name matches role.
**Either** clarify stack (notify → … → this driver) in docs **or** merge
ownership later — not both competing “status” homes.

**Header body:** no other path-specific defect this sitting **if** project docs
/ architecture maps clearly say: go here to **define/change NeoPixel HW
behavior** (modes, PIO, transitions). Note: `led_patterns.h` is already named
SSOT for **pattern constants** (codes); this file is the **render/driver** —
both must stay discoverable without looking like duplicate “status” systems.
Related: **WN-055**–**057** (led_patterns), notify/Stage L docs. No mid-walk
rename.

*(Pedagogy — no WN: header vs cpp “Legos vs assembly” is clear on this pair;
walk WB **W-11**. Note only: ~L200–202 short IVP-T5.5 line is a **good** brief
ticket reference class — model for **WN-105**.)*

**WN-114** — [Grok] · `invariant` · **NOLINT magic-numbers on HSV convert**  
Locus: ~L490–537 `NOLINTBEGIN/END(readability-magic-numbers)` around
`ws2812_hsv_to_rgb` (sector thresholds 60/120/… still literal despite some
named constants).

**Claim:** Only NOLINT block in this file (grep). Same **disallowed** class as
**WN-043** / **WN-088** — prefer named sector bounds or accept a logged TP-style
deviation for standard HSV math; don’t leave a 50-line suppress. Comment already
admits sector geometry is the reason.

#### `drivers/lwgps_opts.h`

**WN-115** — [Grok] · `comment` · **Banner should explain role, origin, and vendored hook**  
Locus: whole file — currently only `@brief LwGPS configuration for RocketChip`
plus bare `#define LWGPS_CFG_*` knobs; no `.cpp` (by design).

**Claim:** Sparse opts header is **correct shape** (MaJerle LwGPS user-config
pattern: project `lwgps_opts.h` included from vendored `lib/lwgps/.../lwgps_opt.h`,
compile-time only). What’s missing is a **clear file banner**: this is **not** a
GPS driver; it is **Rocket Chip’s config for the vendored LwGPS NMEA parser** in
`lib/lwgps/`; library code lives there; our GPS drivers (`gps_pa1010d`,
`gps_uart`) feed bytes into that parser; opts are found via include path
(`src/drivers`). Point at template / docs if useful. **Later:** expand banner
only — no logic change. Related: **WN-086** (adopted code clarity).

---

## Tier 2 — Domain logic & infrastructure

### fusion/

#### `fusion/eskf_runner.{cpp,h}`

**WN-116** — [Grok] · `comment` · **File banner too long — will rot**  
Locus: `eskf_runner.h` ~L3–15 module banner (owns ESKF/Mahony/confidence/GPS session/buffer/bench;
`qv_idle_bridge` ~200Hz seqlock; `SIG_SENSOR_DATA` after predict; Council A6 read-only accessors).

**Claim:** A **brief** file role description is fine — especially for confusing bits. This banner
is **too much detail** for a header top; it will go stale on any real change to ownership, call
path, or publish behavior. Prefer short evergreen role + pointer if needed; not a mini-SAD.

**WN-117** — [Grok] · `comment` · **Council R-6 line: prefer commit/CHANGELOG over bare ticket**  
Locus: ~L30 `// Compact ESKF state for circular buffer (no P matrix — R-6 council requirement)`.

**Claim:** Brief and useful as layout intent, but like prior walk notes on process tags: a
**commit hash and/or CHANGELOG date** traces the decision better than a bare council/ticket id
that readers must archaeology. Related theme: **W-6**, **WN-085**.

**WN-118** — [Grok] · `ownership` · **GPS session stats comment smells one-shot test, not evergreen API**  
Locus: ~L43–46 banner on `gps_session_stats_t` — "GPS outdoor session stats — accumulated while
GPS is active. Printed on reconnect via 's'. Lets user verify movement gates."

**Claim:** Reads as **one specific testing session that already happened**, not a durable product
contract. Either (a) this is **evergreen** operator/diag feature and the comments should say so
without past-session voice, or (b) it is **test-only** scaffolding and does not belong in the
fusion public header long-term. If the feature is useful beyond that session, decide **where it
lives** (fusion vs diag/CLI) deliberately — not leave ambiguous "outdoor session" prose.

**WN-119** — [Grok] · `comment` · **Brake block comment is historical narrative**  
Locus: ~L122–131 runaway-restart brake section banner (ex-watchdog_recovery, LL 29/34,
`kEskfMaxFailCycles=5`, CLI re-enable, power-cycle clear).

**Claim:** Historical migration / motivation essay does **not** belong as permanent header
prose — or it needs **resolution** if the story is still open (is the brake settled? still
"ex-watchdog"?). Keep a short live contract (what disabled means, who re-enables); move LL/migration
detail to docs/CHANGELOG. Same class as long process banners (**W-6**).

**WN-120** — [Grok] · `comment` · **Many other API comments relevant but too long**  
Locus: rest of `eskf_runner.h` doc comments on public APIs (init/tick/getters/bench/reinit, etc.)
beyond the banner and the blocks already called out.

**Claim:** Content may be **relevant**, but volume is high for a header — same rot risk as
**WN-116**. Prefer short contracts; detail only where behavior is non-obvious.

**WN-121** — [Grok] · `comment` · **LL Entry 1 cite on `g_eskf` needs re-eval**  
Locus: `eskf_runner.cpp` ~L70–72 — `// Per LL Entry 1: ESKF struct is ~970 bytes — file-scope, not stack.`

**Claim:** Points at the **literal first** Lessons Learned entry (stack overflow from large
locals). Directionally still right (large objects off stack), but much has changed since
Entry 1 — size/layout/context may have moved. **Re-evaluate** the cite (current `sizeof`?
short “file-scope: too large for stack” without LL-1 archaeology?). Do not treat “LL Entry 1”
as frozen proof without checking.

**WN-122** — [Grok] · `ownership` · **GPS outdoor session state + stats (cpp) — same as header; deeper home/role dive**  
Locus: ~L87–90 `g_gpsSess`; ~L349–363 `eskf_tick_gps_stats` (“post-session review via 's'”);
GPS update call site.

**Claim:** Same theme as **WN-118** (one-shot outdoor/test voice vs evergreen). Impl is here.
**Deeper dive warranted:** may be RC_OS/CLI-diag bookkeeping more than core fusion — decide
whether it belongs in this TU. One product disposition can cover **WN-118** + this.

**WN-123** — [Grok] · `comment` · **Process/ticket comments (R-25, CR-N) — clean up / retarget**  
Locus: ~L97–100 and ~L194 R-25-exec bench notes; CR-1/2/3/4 tags (~L165, 182, 198, 525,
**569**); same ticket voice class as header (**WN-116**, **WN-117**, **WN-120**).

**Claim:** Dev/process notes need the same hygiene as the header: short live contract or
commit/CHANGELOG under current standards — not multi-line ticket essays. **CR-4** (~L569) only
means “seqlock fail is a separate early return from data-validity”; the `CR-N` labels are not
self-explanatory in-tree. Opaque tags same class as bare R-25/R-6.

**WN-124** — [Grok] · `invariant` · **INTERIM Z-up→NED negate is HW-specific, no safeguard**  
Locus: ~L117–130 `sensor_to_ned_{accel,gyro,mag}` — INTERIM Adafruit ICM-20948 Z-up; hard-coded
Z negate; comment’s “proper fix: board_rotation…”.

**Claim:** Admits interim + specific breakout; **code has no board/rotation gate or other
safeguard** — always negates Z. Written for one HW item. Other mounts can get wrong axes
silently. Needs explicit notation and/or real board_rotation path, not only INTERIM prose.

**WN-125** — [Grok] · `comment` · **Mag yaw bootstrap comment: mostly good, shorten; verify current**  
Locus: ~L147–150 (mag yaw if available; gate rejection if yaw stuck at 0; ArduPilot EKF3 cite).

**Claim:** Content useful; **too long**. Shorten. Spot-check still matches current ESKF/mag
gate behavior.

**WN-126** — [Grok] · `comment` · **Baro “~32Hz DPS310” reads as SSOT rate**  
Locus: ~L232 `// Baro altitude measurement update (~32Hz DPS310 rate, on new data)`.

**Claim:** Treats a **driver/sensor setting** (and SKU) as fusion SSOT. Real path is “on new
`baro_read_count`.” Prefer “on new baro sample”; Hz/part number optional non-normative example
only.

**WN-127** — [Grok] · `ownership` · **Mag 3D / WMM path: feature assumptions, HW-ish detail, silent degrade**  
Locus: ~L258–259 auto-enable needs cal + WMM; ~L279–307 `try_enable_mag_3axis`; ~L309+
`eskf_tick_mag` (AK09916 ~10Hz comment; 3-axis vs heading-only).

**Claim:** Partly HW-agnostic (flags + WMM), but need confidence HW/builds **without**
mag/WMM/GPS do not **break** or **silently fail** badly. Mag tick names **AK09916** + rate —
HW-specific detail in domain code (**W-8**). Re-read enable + fallback as a product path.

**WN-128** — [Grok] · `comment` · **ZUPT block should read clearly as ZUPT first**  
Locus: ~L413–416 before `eskf_tick_zupt` (on-pad IDLE/ARMED, tight R, external cites).

**Claim:** Needs clearer wording that this **is the ZUPT** path — title/first line first;
pad/phase policy and external cites secondary and shorter.

**WN-129** — [Grok] · `ownership` · **`ROCKETCHIP_HOST_TEST` ifdefs — re-check sequestration rules**  
Locus: ~L492–496 `now_ms`; ~L517–519 / 538–546 full-tick bench; ~L551–556 `SIG_SENSOR_DATA`
publish; file-top host stubs ~L22–30.

**Claim:** Host/test carve-outs inside flight fusion. `#ifdef`’d, but **newer sequestration
rules** apply — re-check allowed pattern vs stubs outside production TUs. Disposition against
current host-test policy (not auto-delete).

**WN-130** — [Grok] · `comment` · **Trailing comment: brake file split only for host tests**  
Locus: ~L687–688 — brake in `eskf_brake.cpp` so host tests link without runner SDK deps.

**Claim:** End-of-file comment is odd; structure is framed as **for host testing ease**.
Re-check: still the real reason / still needed, or state a cleaner module boundary? Related
**WN-129**.

#### `fusion/eskf.{cpp,h}`

*(Owner-directed regroup 2026-08-06: prior thin WN-131–142 collapsed into **WN-131–134**
below. All loci/context kept; sub-nuances free to split at disposition. IDs **WN-135–142**
were not left as empty stubs — next new global ID is **WN-135**. See **W-13**.)*

**WN-131** — [Grok] · `comment` · **`eskf.h` comment density / wrong home — design-doc material & ticket tags**  
Loci (`eskf.h`):
- ~L6–20 file banner: 24-state ESKF summary, Solà arXiv, PX4/ArduPilot, “all noise from
  ICM-20948 unless empirical,” every constant cited — **design-doc / reference essay**, not
  a header top; short role line OK if kept.
- File-wide: one short line per constant/API is good; many **3–4 line** blocks and some
  singles with **5–8+** comment lines — general density problem on this header.
- ~L163–164 P diagonal clamping “council review **RF-2**” (and similar bare council/C-n tags
  elsewhere in this header, e.g. C-1 near GPS) — process tags prefer commit/CHANGELOG or a
  one-line live “why,” same hygiene as **WN-117** / **WN-123**.
- ~L206–234 mag heading **constants** block: long tutorial (H≈yaw-only, tilt, 10Hz spin /
  wrap_pi, 300σ gate / mNIS death spiral) — essay length even where content is true.
- ~L251–270 **ZUPT** section + σ/R prose (unobservable v/p ~30s, stationarity = RF-5,
  ArduPilot/PX4 cites, P_v collapse / gain vanishes) — if depth needed → **ZUPT design doc**
  (or link); if rehash of external refs → short comment + cites only, don’t repeat.
- ~L368–380 `update_mag_heading` API wall (params, two-tier interference, declination,
  H approx, Solà, Joseph) — trim to contract; detail → doc.

**Claim:** One disposition family — **comment volume and home** (code vs design doc vs
commit pointer). Per-line constants OK; multi-line tutorials and ticket archaeology not.
Do not lose the listed loci when remediating; disposition may still peel ZUPT-doc vs
banner-trim vs ticket-rewrite as separate tasks later.

**WN-132** — [Grok] · `comment` · **`ESKF_USE_BIERMAN`: removed-when OK; “kept on” unclear**  
Locus: ~L30–35 — Bierman only path; Joseph removed 2026-07; define always `1` “so residual
call sites / docs can still mention it.”

**Claim:** Good to record **what was removed and when**. Unclear what always-on
`ESKF_USE_BIERMAN` still buys (dead switch? residual `#if`s? docs only?). **Address**
delete vs intentional keep; comment should be **briefer**. Distinct from density (**WN-131**):
this is a **leftover feature switch**, not essay length.

**WN-133** — [Grok] · `ownership` · **Noise/init defaults and comments are prototype-HW-centric**  
Loci (`eskf.h` constants / nearby):
- ~L6–20 banner also claims ICM-20948 as default noise SSOT (overlap with **WN-131** home;
  **substance** here is HW coupling).
- ~L78–100 IMU noise spectral densities + bias walks — ICM-20948 DS-000189 tables; ArduPilot
  contrasts; “verified correct.”
- ~L138–154 P init — “Solà + ICM-20948 ZRO” / board-level ZRO tables.
- ~L197–204 baro R — **DPS310** @ 8× OSR → 0.033 m σ.
- ~L206–234 mag R/gates — **AK09916** noise, 10Hz update assumptions (length → **WN-131**;
  SKU lock → here).
- ~L277–289 GPS R — **MT3333 / PA1010D** CEP50, HDOP scale, council C-1; ArduPilot u-blox
  contrast.

**Claim:** Comments and default numbers read as **current prototype SKUs**, not necessarily
universal ESKF. **Verify code isn’t hard-wired** beyond “defaults for the sensors we use”
(silent wrong-R if baro/mag/GPS/IMU change). Same tension as **W-8**, runner INTERIM axes
(**WN-124**). Disposition may still split IMU vs baro vs mag vs GPS later without losing
loci above.

**WN-134** — [Grok] · `invariant` · **Some defaults justified only for one mission/flight shape**  
Loci:
- ~L119–123 `kSigmaWindWalk` — ArduPilot wind; **parachute descent** high drag / low inertia
  “more like Copter” → 0.2 (not Plane 0.1).
- ~L186–195 `kMaxHealthyVelocity` = 500 m/s — **hobby rocket** burnout ~Mach 1.5, ICM
  silent-zero divergence story; 500 m/s headroom narrative.

**Claim:** Starting points from a **single phase/vehicle example** are fine if labeled, but
must be **evaluated for universality** (other phases, HAB, low-speed, etc.) — wind process
noise and velocity health guard may be wrong-sized outside that story. Phase Q/R may already
override some of this; confirm rather than assume. Sub-nuances (wind only vs vel only) can
split at disposition; both are “not universal as written.”

**WN-135** — [Grok] · `comment` · **`eskf.cpp` large inline essays/tables — density / design-doc home**  
Loci (non-exhaustive; owner noticed these — more may exist in ~1.8k LOC file):
- ~L111–118 gravity→attitude init narrative — shorten if kept.
- ~L200–219 `build_F` F_δ **block table** — OK as a design ref, too large inline with code.
- ~L264–269 `build_Qc` — length maybe OK but **needs clearer wording** (R-8/R-9 buried).
- ~L337–347 `predict` codegen history / sparse-vs-codegen essay.
- ~L401–420 `reset` / G Jacobian long form + RF-1.
- ~L549–556 negative-diag floor story (host replay 2026-07-09).
- ~L705–722 `update_baro` measurement-model wall; ~L729–734 dual-H/bierman note.
- ~L789–823 `update_mag_heading` **document inserted into code** (model steps, UNOBSERVABLE,
  two-tier interference, Joseph, LL Entry 1).
- ~L911–923, ~L987–993, ~L1081+, ~L1174+, ~L1186+, ~L1268, ~L1334, ~L1404, ~L1465–1467,
  ~L1569, ~L1652 — further large explanatory / table blocks (same class; inventory at
  disposition).
- ~L1014–1032 `update_zupt` banner (stationarity, sequential scalar, ArduPilot/PX4).

**Claim:** Same family as header **WN-131** — per-function design essays and ASCII matrices
belong in **design docs** (or short cite + link), not multi-screen comments beside flight
math. Keep contracts short; do not lose the listed loci when trimming. Comment rework should
also help **file size** pressure (**WN-139**).

**WN-136** — [Grok] · `comment` · **Opaque ticket / equation / “surfaced” refs in `eskf.cpp`**  
Loci + what they appear to mean (verify at disposition):
- ~L9–13 **“R3”** — compile-time **codegen↔`eskf.h` sigma drift guards**
  (`static_assert` + `-Wfloat-equal` suppress for constexpr identity). Label “R3” is not
  self-explanatory in-tree; comment explains *what* but not *which review* R3 was.
- ~L37 **JSF AV Rule 151** — “file-scope constants from inline literals”; **re-verify** the
  rule still applies / is the right cite for this block.
- ~L73 **“Solà (2017) Eq. 9”** on `skew` — OK paper pin if Eq. 9 is still the skew definition
  in that paper; “what’s eq 9?” should be answerable without opening arXiv (or drop to
  “skew-symmetric [v]×, Solà”).
- ~L102 / ZUPT ~L1020+ **“RF-5”** — stationarity check (same council tag as header
  stationarity constants); bare ticket.
- ~L163, ~L178 Solà §/Eq. pins on `propagate_nominal` — vague if reader can’t map without
  the paper open; same class as Eq. 9.
- ~L266–268 **R-9** / **R-8** on Q_d / clamp inhibited blocks.
- ~L421 **RF-1** on reset G.
- ~L610–611 (and ~L549–554 related) “Surfaced 2026-07-09…” UD/factorize repair — date OK;
  **update with durable refs** (commit/CHANGELOG/PR); **double-check the fix itself** still
  correct under current Bierman/UD path.

**Claim:** Process/equation labels and “surfaced on DATE” notes need **readable live meaning**
or **commit/CHANGELOG** pointers under current standards — same hygiene as **WN-123** /
**WN-131** ticket tags. Distinct from pure length (**WN-135**): even a short “RF-5” is opaque.

**WN-137** — [Grok] · `ownership` · **Module boundary: `eskf` vs codegen / verify / non-core aids**  
Loci / themes:
- ~L4 include `eskf_codegen.h`; ~L9–32 **R3 drift `static_assert`s** against codegen sigmas —
  codegen-specific coupling **in** `eskf.cpp`. Evaluate whether this (and similar) belongs
  next to **codegen / generator** (or a tiny bridge TU), not the core filter impl — even if
  related.
- ~L337–347 / `predict` uses codegen FPFT; ~L343 / dense **`predict_dense` retained as
  verification (Test 8)** — evaluate that this isn’t **host-test-only** machinery bloating
  the flight TU (or that sequestration rules are met). Related **WN-129** host-test pattern
  on runner.
- Overall feel of **`eskf.{h,cpp}` pair**: code named ESKF should **handle ESKF work**;
  codegen artifacts, long verification paths, and maybe other “related but not core” bits
  need a deliberate home.
- **ZUPT** (~L1014+ and header ZUPT constants/docs **WN-131** / **WN-140** family): owner
  note — *perhaps all ZUPT code should live together in separate files* if it keeps growing;
  not mandatory split mid-walk, but structure question for disposition.

**Claim:** **Boundary and placement** disposition — what stays in `eskf.cpp` vs
`eskf_codegen` / tests / optional `eskf_zupt` (names TBD). Keep full locus list; sub-splits
(codegen vs ZUPT vs dense verify) allowed later.

**WN-138** — [Grok] · `ownership` · **File-scope constants block: HW-specific / magic risk**  
Locus: ~L39–66 anonymous-namespace constants (`kMinInnovationVariance`, `kQuatNormTolerance`,
`kMaxGyroBias` “10 dps… **ICM-20948** ±5°/s ZRO”, block spans, P-growth epsilon, …).

**Claim:** High risk of **HW- or prototype-specific values** (and under-sourced thresholds)
living as local magic with light comments. Re-check each against datasheet/profile defaults
and **WN-133** header noise story; prefer named sourced defaults or profile hooks where the
value is really SKU-dependent. Overlaps header HW theme; **this locus is the cpp locals**.

**WN-139** — [Grok] · `ownership` · **`eskf.cpp` size (~1.8k+ LOC) — worth a structural look**  
Locus: whole `src/fusion/eskf.cpp` (~**1826** lines at walk; header ~625). Among larger
project TUs even if no hard LOC gate is violated.

**Claim:** Size alone is not automatic FAIL, but **worth evaluating** (split by concern:
predict/reset, measurements, ZUPT, GPS origin, UD/Bierman glue, …) especially after comment
trim (**WN-135**) still leaves a heavy impl. Ties to module-boundary **WN-137**. Confirm
against any project size standards at disposition; owner recollection: one of the largest
files in the tree.

#### `fusion/eskf_brake.cpp`

**WN-140** — [Grok] · `ownership` · **Tiny solo TU — does the brake need its own file?**  
Locus: whole file (~41 LOC impl) — banner ~L3–14: **Migrated 2026-04-22 from
`watchdog_recovery`**; runtime-only consecutive-fail disable; file-local statics;
USER_GUIDE safety-posture note. API decls still live on `eskf_runner.h`; runner end
comment (**WN-130**) also claims split was for **host-test link** without runner SDK deps.

**Claim:** File is **very small** — evaluate whether a dedicated translation unit still
earns rent vs living next to runner/brake call sites. The **2026-04-22 migration** cite
(and host-test link story) should drive the investigation; keep, merge, or re-home
deliberately. Not a defect in the counter logic itself this sitting.

#### `fusion/eskf_state.h`

**WN-141** — [Grok] · `comment` · **Banner: vague refs + unclear state “table”; keep short**  
Locus: ~L6–16 — Solà “S5”, 24-state layout line, ArduPilot `statesArray[24]` / NavEKF3,
extended-state inhibit blurb.

**Claim:** Mix of **vague source pins** (Solà section form, ArduPilot file cite without a
durable pointer) and an **attempted state layout listing that doesn’t read as a clear
table**. Same shortness theme as other fusion headers (**WN-131** family): useful index
file does not need a long banner — tighten layout presentation (or real table elsewhere)
and make any keep refs scannable. Rest of file (named `kIdx*` constants) not flagged this
sitting.

#### `fusion/eskf_codegen.{cpp,h}`

— **exempt from walk** (itinerary standing exemption + CG-1: auto-generated by
`scripts/generate_fpft.py`; size/comment/design lenses N/A). Spot-check only: both
files carry `AUTO-GENERATED … DO NOT EDIT` (cpp also SymPy stamp 2026-02-21). Not a
semantic pass; not `nothing of note.`

#### `fusion/confidence_gate.{cpp,h}`

**WN-142** — [Grok] · `comment` · **Safety-layer banner claims need elevated scrutiny (wording)**  
Locus: `confidence_gate.h` ~L6–19 — binary “trust my estimates”; FD consumer;  
`// This is a PLATFORM SAFETY layer — NOT configurable by Mission Profiles.`  
`// All thresholds are VALIDATE defaults for field tuning.`  
Also pyro lock when `confident=false`, hysteresis lines.

**Claim:** If this is **safety-critical** (platform safety, pyro lockout, not mission-profile
tunable), the banner and related comments need **more scrutiny than ordinary fusion prose** —
especially **wording** (what “PLATFORM SAFETY” means vs USER_GUIDE levels, VALIDATE vs frozen
flight limits, “field tuning” vs “NOT configurable”). Rest of header not flagged this sitting
beyond that safety-claim surface.

**WN-143** — [Grok] · `comment` · **`confidence_gate.cpp` has no module/role header at all**  
Locus: `confidence_gate.cpp` top — SPDX/copyright then `#include` and functions (~73 LOC);
no file-level role banner.

**Claim:** After many fusion TUs with **overlong** banners, this one is the opposite: **no
explanation header**. Ironic but real — for a safety-facing gate (**WN-142**), a **short**
role line (what it does / who calls it / hysteresis) would match project norms without
reintroducing essay density. Nothing else flagged on the cpp this sitting.

#### `fusion/innovation_monitor.{cpp,h}`

**WN-144** — [Grok] · `comment` · **Council A7 design-properties cite — update / re-verify**  
Locus: `innovation_monitor.h` ~L13–18 `// Design properties (Council A7):` one-way Q inflate,
cap, freeze during phase ramps.

**Claim:** Bare **council decision** tag needs the usual refresh (commit/CHANGELOG or drop to
live “why”). If it **did** come from that council item, fine **as long as the properties still
hold** (one-directional inflate, max cap, caller freezes on phase ramps) under current ESKF
wiring. Re-verify at disposition; don’t treat “Council A7” as self-proving.

**WN-145** — [Grok] · `comment` · **`innovation_monitor.cpp` no module/role header**  
Locus: cpp top — SPDX then include/impl (~59 LOC); no file-level role banner.

**Claim:** Same pattern as **WN-143** (`confidence_gate.cpp`) — **no notable header**. Pair is
intentionally **brief**; name/role (innovation / NIS window → Q scale) fits a specialized
helper, so shortness of the **code** is fine — only the missing one-liner role note on the
cpp is the comment gap. Header already carries the purpose blurb.

#### `fusion/mahony_ahrs.{cpp,h}`

**WN-146** — [Grok] · `comment` · **`mahony_ahrs.h` banner mostly fine; re-check refs + density**  
Locus: ~L6–21 role + Mahony 2008 + PHASE5_MAHONY_PLAN + ArduPilot/PX4/BF/INAV list; also
general multi-line constant notes (~L45–68) and method docs later in header.

**Claim:** Banner content is **mostly fine** — still **double-check** paper/plan/stack
references still resolve and match current code. Secondary: **general comment density**
on this header (not banner-only) — same tidy-toward-short-contract theme as other fusion
headers (**WN-131** family), without calling the role blurb overlong.

**WN-147** — [Grok] · `ownership` · **Council cites: important, not infallible project pillars**  
Locus: ~L41–43 `// Council-approved parameters (arXiv:… + 3-stack consensus)`; also e.g.
~L63 “Council tightened from ±25%”; pattern across walk (A6/A7/RF-*/C-n, etc.).

**Claim:** Council decisions are **important** and worth citing when real, but be careful
that a **single offhand council suggestion** does not freeze into a **major project pillar**.
The council process is a **useful auxiliary**, not bulletproof authority — re-validate
that “council-approved” params/policies still earn their weight on evidence, not on the
label alone. Applies beyond this file whenever a bare council tag is the only load-bearing
justification (**WN-144** A7, **WN-136** RF-*, etc.).

**WN-148** — [Grok] · `comment` · **`mahony_ahrs.cpp` no module/role header**  
Locus: cpp top — SPDX then includes/impl; no file-level explanation chunk.

**Claim:** Same missing-banner pattern as **WN-143** (`confidence_gate.cpp`), **WN-145**
(`innovation_monitor.cpp`). Owner note: these may have been **written together** as a batch
with no cpp banners. One short role line per TU (or a shared style decision) at disposition;
nothing else flagged on this cpp this sitting.

#### `fusion/ud_factor.{cpp,h}`

**WN-149** — [Grok] · `comment` · **`ud_factor.h` massive comment ratio**  
Locus: whole header (~55 lines) — banner L6–14, struct/API docs, large Bierman section
from ~L39 onward relative to thin declarations.

**Claim:** **Comment mass dominates** the header vs the actual surface (UD24 + a few
functions). Only note on the `.h` this sitting — tidy to short contracts; detail → design
doc (Bierman/UD path already has CHANGELOG cite). Density family **WN-131** / **WN-146**.

**WN-150** — [Grok] · `comment` · **`ud_factor.cpp` Doxygen top + large algorithm blocks; plain role line?**  
Loci: ~L3–12 `@file`/`@brief` Doxygen (Thornton/Bierman, time_critical, LL 30) — not the
same “empty top” as **WN-143**/**WN-145**/**WN-148**, but owner still notes **no plain
explanation chunk** and questions **whether cpp banners are even needed** (style decision).
Large blocks: ~L51–59 modified Cholesky steps; ~L87–96 Bierman section; ~L102–105
`bierman_compute_fg` prose.

**Claim:** Algorithm essays / multi-line formula comments should shrink or move to a UD/Bierman
note; keep short “what/why” at call sites if anything. Fold the **“are cpp file banners
required?”** question with the missing-banner cluster (**WN-148** etc.) at disposition —
one house rule, not per-file flip-flops.

**WN-151** — [Grok] · `invariant` · **NOLINT magic-numbers on `24` array dims**  
Locus: ~L32 `ud_to_dense(… float p[24][24])`, ~L50 `ud_factorize(… const float p[24][24])`
— `// NOLINT(readability-magic-numbers)`.

**Claim:** `24` is the ESKF state size (`kN` / `eskf::kStateSize` already exist in-tree).
Prefer typed/`constexpr` dims or shared alias over NOLINT-suppressed literals (same class as
**WN-114** / **WN-043** NOLINT-magic discipline).

#### `fusion/phase_qr.h`

**WN-152** — [Grok] · `comment` · **Council 2026-03-29 cite + general density**  
Loci: ~L17 `// Council review 2026-03-29: unanimous approval, 7 amendments (A1-A7).`;
file-wide comment mass (banner delta formula ~L9–15, struct field notes, phase index
list, etc.).

**Claim:** Council line needs usual treatment — durable pointer / still-valid, and
**WN-147** caution (council not infallible pillar). Also **general comment density** on
this header (shorten; keep live Q/R contract). Nothing else this sitting.

#### `fusion/wmm_tables.{cpp,h}`

— **light / generated** (itinerary: data table — light). Both files stamp
`AUTO-GENERATED by scripts/generate_wmm_table.py` / Do not edit; WMM2025 tables.
Not a semantic standards walk of the table body. Codegen inventory / hand-edit audit →
walk WB **W-14**.

### calibration/

#### `calibration/calibration_data.{cpp,h}`

**WN-153** — [Grok] · `comment` · **Section titled “Magic Numbers” — check vs house magic-number rules**  
Locus: `calibration_data.h` ~L17–22 `// Magic Numbers and Version` then
`kCalibrationMagic = 0x52434341` (“RCCA”) and `kCalibrationVersion`.

**Claim:** The banner **literally labels a section “Magic Numbers.”** Against house policy
(JSF-151 / CODING_STANDARDS “no magic numbers” + `readability-magic-numbers`), that wording
is a red flag even if the *intent* is a **flash/file signature + schema version** (named
constants, not bare literals in formulas). Confirm the values are **sourced/justified**
(fourCC convention, version changelog) and rename the section so it does not read as an
allowed magic-number dump. Disposition may keep the constants, fix only the label/docs.

**WN-154** — [Grok] · `comment` · **CRC-16 comment assumes insider knowledge; ITU-T V.41 vague**  
Locus: `calibration_data.cpp` ~L12–16 `// CRC-16/CCITT constants (ITU-T V.41)` + poly/init
names; section ~L22–24 `// CRC16 (CCITT polynomial 0x1021)`.

**Claim:** Reader is expected to already know what **CRC-16/CCITT** is (owner only knows from
when this was written). **ITU-T V.41** alone is a thin/vague pin without poly/init table or
a project logging CRC home (e.g. shared `crc16_ccitt` if one exists). Prefer a short live
line (poly 0x1021, init 0xFFFF, bit order) + durable cite, or reuse a common CRC helper so
this TU doesn’t re-own the algorithm story.

#### `calibration/calibration_manager.{cpp,h}`

**WN-155** — [Grok] · `comment` · **IVP ticket tags + large API comment blocks**  
Loci (`calibration_manager.h`):
- IVP process tags: ~L11 (IVP-17 in `@brief` list), ~L66 gyro (IVP-15), ~L85 accel level
  (IVP-16), ~L146 6-pos (IVP-17), ~L207 mag (IVP-35/36).
- Large blocks: ~L149–160 `calibration_start_6pos_position` async protocol essay;
  ~L245–255 `calibration_compute_mag_cal` two-step LM essay.

**Claim:** IVP numbers are **work-item archaeology** (**W-6** / **WN-085**) — keep only if
they still point to live IVP docs; else drop or commit/CHANGELOG. Large Doxygen blocks should
shrink to short contracts (async 6-pos call order; mag fit steps → design note if needed).

**WN-156** — [Grok] · `ownership` · **Calibration path: general caution vs HW-specific code**  
Locus: pair as a whole — calibration is a **HW-focused process** (sensor samples, rates,
field µT bounds, fit gates). Cpp already names **ICM-20948** (~L614) and **RP2350 TRNG**
(~L900); header rates/thresholds imply current sensors.

**Claim:** **General caution** (not a line-by-line SKU audit this sitting): keep domain
calibration logic **as HW-agnostic as practical** (SI units, named thresholds, board/driver
edges own SKU details). Prototype-era numbers and part-name comments tend to freeze into
“the” cal path (**W-8** / fusion **WN-133**). Re-read thresholds and comments when sensors
change; don’t bake breakout-specific assumptions into the manager API.

**WN-157** — [Grok] · `invariant` · **Cal sample counts commented as if default sensor Hz is evergreen**  
Locus: `calibration_manager.cpp` ~L19–26 e.g. `kGyroCalSamples = 200 // ~2 seconds at 100Hz`,
accel `100 // ~1s at 100Hz`, baro `50 // ~1s at 50Hz`.

**Claim:** Comments **assume default producer rates**. Clean wording so duration is either
(a) derived from sample count × actual feed rate, or (b) explicitly “nominal at N Hz feed.”
Confirm **code** does not treat those Hz as fixed evergreen (only sample counts matter if
callers feed slower/faster).

**WN-158** — [Grok] · `comment` · **Large algorithm / process comment blocks in manager cpp**  
Loci: ~L36–41 6-pos offdiag null-space essay; ~L609–614 store_6pos offdiag + ICM ±2%;
~L735–739 mag angular sep / ArduPilot; ~L780–786 LM/FP-1 extraction narrative.

**Claim:** Density / design-doc material — shorten or move; same family as fusion essays
(**WN-135**). ICM line also feeds **WN-156** HW caution.

**WN-159** — [Grok] · `comment` · **Cross-file / boot-order notes — verify or drop**  
Loci: ~L68 `// LM damping… live in lm_solver.h` (points at another file’s contents);
~L177–178 `// Storage init happens in main before USB` / load from already-init storage.

**Claim:** Talking about **what lives in another file** can go stale — prefer include/
API or omit. Boot-order comment must **still be true** (main still inits storage before
this path); re-verify at disposition.

**WN-160** — [Grok] · `comment` · **Vague “Phase” / “Stage” labels (IVP-sounding)**  
Loci: ~L476 `// Async 6-Position API (Phase D1)`; ~L1095 / ~L1111 / ~L1129 / ~L1144
`// Stage 1: Ellipsoid…` / `// Stage 2: Board rotation…` (and similar); ~L70
`// Post-fit validation bounds (from IVP.md)`.

**Claim:** “Phase D1” / numbered stages sound like **IVP or plan stages**, not
self-evident code structure. If they are **pipeline steps**, say that plainly; if IVP
phase names, point to a durable doc or drop. IVP.md bounds cite needs the same
hygiene as **WN-155** IVP tags.

**WN-161** — [Grok] · `invariant` · **Many NOLINT magic-number regions in mag fit / apply**  
Loci (NOLINTBEGIN/END clusters): ~L796–814 sphere jacobian; ~L855–898 mag residual/jacobian;
~L919–925 ellipsoid seed; ~L944–963 / ~L1008–1018 / ~L1078–1083 (param layout blocks as
present); ~L1112–1117 / ~L1145–1150 board-rotation 3×3 indices.

**Claim:** Repeated **readability-magic-numbers** suppressions on param/matrix indices.
Prefer named param indices / shared 3×3 helpers (or accept with a single logged deviation)
— same discipline as **WN-151** / **WN-114**. Inventory at disposition; do not leave
scattered NOLINT as the design.

**WN-162** — [Grok] · `ownership` · **Mag thin uses RP2350 TRNG Fisher–Yates — universality**  
Locus: ~L900–913 `mag_thin_samples` — Fisher–Yates via `get_rand_32()` / `pico/rand.h`,
comment “RP2350 hardware TRNG”.

**Claim:** If the technique is **tied to that chip’s TRNG**, evaluate portability (host
tests, other MCUs) and whether any RNG with uniform `uint32` is enough. Extends pair HW
caution **WN-156**; may need host stub / abstraction, not only a comment.

#### `calibration/calibration_storage.{cpp,h}`

**WN-164** — [Grok] · `comment` · **Header Doxygen density; sparse specialized surface may be OK**  
Locus: `calibration_storage.h` — `@file`/`@brief`/per-API Doxygen on a thin dual-sector
flash interface (~50 lines).

**Claim:** Same **Doxygen density** pattern as other headers (**WN-081** / **WN-054**).
Surface is **sparse** (init/read/write/erase) — that may be **fine for a specialized
storage façade**; disposition is style consistency, not “add more API.” Prefer short
contracts over ceremony if Doxygen policy is re-evaluated.

**WN-165** — [Grok] · `comment` · **Cpp flash-layout banner: right kind of comment; check rot**  
Locus: `calibration_storage.cpp` ~L7–12 — Sector A/B addresses at end of 8MB flash,
dual-sector + sequence number for newest valid.

**Claim:** This is the **right kind** of comment (live layout contract). Still **verify it
matches** `flash_layout.h` / current SKU flash map (impl already pulls
`kFlashCalSectorA/B` ~L26–30 — banner hardcodes `0x7FE000` / `0x7FF000`). If addresses
move, banner **rots first**. Prefer “see `flash_layout.h`” + non-normative example, or
keep numbers only if they stay generated/synced.

**WN-166** — [Grok] · `ownership` · **Early cal-storage feature — prior-art / re-eval worth a look**  
Locus: pair (dual-sector wear-level cal flash). Owner: looks fine otherwise; **very early**
feature.

**Claim:** Worth a **prior-art / design re-check** (LittleFS/other projects’ dual-sector
cal patterns, power-fail, dual-core `flash_safe_execute`) — keep-with-why or planned
rework. Not a mid-walk rewrite. Ties Early-impl index / **WN-163** design-vs-code for
central features.

#### `calibration/lm_solver.{cpp,h}`

**WN-167** — [Grok] · `comment` · **`lm_solver.h` banner/history/council/API blocks**  
Loci:
- ~L6–15 top block a bit long; ~L11–15 R-6c / FP-1 “what we did” — may be OK as role, but
  **don’t need the migration story** if already in the deviation log / CHANGELOG.
- ~L21 council consensus 2026-02-10 on LM λ constants — cite hygiene (**WN-147**).
- ~L70–74 `lm_solve` multi-line template/param essay — shorten to contract.

**Claim:** Live role (pure-function LM for mag fits, host-testable) is fine; process
history and long template signature comments should slim. Itinerary FP-1/templates flags
not re-opened as defects this sitting — owner did not flag residual fn-ptrs.

— **nothing of note.** (`lm_solver.cpp` — short LA primitives; owner-directed.)

#### `calibration/cal_hooks.{cpp,h}`

**WN-168** — [Grok] · `comment` · **`cal_hooks.h` massive banner + Stage/audit archaeology**  
Loci: ~L3–17 whole top block (role list + Stage 13 Phase 8 extraction + R-17/R-18
dead `cal_pre_hook` / `core1_i2c_pause` essay); ~L11–12 Stage/audit process tags.

**Claim:** Banner is **far too large** for four declarations. Drop or move “what we
deleted in the audit” to CHANGELOG; keep a short “rc_os cal callbacks + post-save Core1
reload.” Sparse/specialized surface (**four hooks**) may be a fine **single-purpose**
TU — evaluate keep-as-thin-bridge vs merge into rcos/cal — but not by stuffing history
into the header (**W-6**).

**WN-169** — [Grok] · `comment` · **`cal_hooks.cpp` IVP/Stage tags + large HW-ish blocks**  
Loci: ~L6 Stage 13 Phase 8; ~L39–42 full `icm20948_read` vs accel-only (data-ready /
zeros after ~200) — large + **ICM-20948-specific** behavior; ~L94–104 post-hook /
R-17/R-18 extraction essay again.

**Claim:** Trim process archaeology. The ICM full-read comment may be a **real
invariant** (keep short + datasheet/LL cite) but is **SKU-specific** — same HW caution
as **WN-156**. Re-evaluate whether this file should stay a thin specialized bridge or
absorb more / less once banners are honest.

### flight_director/

#### `flight_director/flight_director.{cpp,h}`

**WN-170** — [Grok] · `comment` · **HSM / state / signal tables belong in a design doc**  
Loci (`flight_director.h`): ~L3–36 full HSM tree + signal list banner; ~L63–74 usage
snippet block. (`flight_director.cpp`): ~L3–17 ABORT behavior table; many per-state
// Accepts: tables e.g. ~L342–346, ~L376–381, ~L414–417, ~L590–599 (ABORT particularly
large); more instances not listed exhaustively.

**Claim:** ASCII **statecharts and transition tables** are valuable but **too large inline**
— move to FD design doc; keep one-line role + pointer in code. Pattern is file-wide on
the cpp, not one-off.

**WN-171** — [Grok] · `ownership` · **Backward-compat FlightSignal alias — not needed this stage?**  
Locus: h ~L56–61 `FlightSignal` as backward-compatible alias; values unchanged note.

**Claim:** Looks like **compat shims**. At this project stage, evaluate whether
compatibility aliases are still required or can be removed (grep callers → drop).
Don’t keep forever “in case.”

**WN-172** — [Grok] · `comment` · **IVP/dev refs + safety posture wording (header)**  
Loci: h ~L88–91 `beacon_cb` IVP-121 + council 2026-05-20 + FAULT_RECOVERY doc path;
~L113–116 launch abort “level 3 safety posture”, USER_GUIDE “see doc” (good), power-cycle
clear.

**Claim:** IVP/council tags — usual hygiene (**W-6**, **WN-147**). If launch abort is
truly **safety-critical** as claimed, wording needs **elevated scrutiny** (same family as
confidence-gate **WN-142**); “see USER_GUIDE Safety State Model” is the right shape —
ensure code matches the doc.

**WN-173** — [Grok] · `comment` · **`flight_director.cpp` process tags, dated docs, vague B.x, IVP essays**  
Loci:
- ~L21–22 IVP / fault-recovery **in `#include` lines** (odd place for process tags).
- ~L60–61 FAULT_RECOVERY **dated** filename — good “see doc” idea, date-in-path **rots** /
  pauses owner; prefer stable doc id/title.
- ~L140–146 B.3 packing essay; ~L211 B.3 seed — **what is B.3 / B.6?** vague without the
  decision doc open (user also noted B.6 — confirm if present elsewhere).
- ~L262–284 IVP-121 multi-channel landing block (+ code) — large IVP-ref’d essay.
- ~L355–358 Council 2026-05-20 IDLE re-init — large + dated council.
- Tables already under **WN-170**.

**Claim:** Same density/archaeology family as header; **opaque B.x** and **dated decision
paths** need live meaning or stable links. IVP-121 logic may stay; comments should not
rehost the IVP writeup.

#### `flight_director/command_handler.{cpp,h}`

**WN-174** — [Grok] · `comment` · **`command_handler_validate` per-command rules block long**  
Locus: `command_handler.h` ~L39–47 — ARM Go/No-Go, DISARM ARMED-only, ABORT phases +
Amendment #1 note, RESET LANDED/ABORT, `go_nogo_input` only for ARM.

**Claim:** Useful contract but **long for a header** — belongs in design/CLI command
policy doc (or short bullets + “see …”). Keep one-liner on the function if anything.

**WN-175** — [Grok] · `comment` · **Opaque “R-25-exec” on test-mode ARM gate**  
Loci: cpp ~L9 `#include "safety/test_mode.h" // R-25-exec: refuse ARM if test mode armed`;
~L45–49 multi-line “R-25-exec council amendment #2 (second clearing gate)…” essay.

**Claim:** **R-25-exec** is not self-explanatory here (bench-tier deprecation / test-mode
workstream — see PROBLEM_REPORTS / `BENCH_TIER_DEPRECATION_2026-05-13.md` if kept). Prefer
live rule (“refuse ARM while test mode active”) + durable commit/CHANGELOG; drop ticket
essay. Same process-tag hygiene as **WN-123** / **WN-173**.

#### `flight_director/action_executor.{cpp,h}`

**WN-176** — [Grok] · `comment` · **Action-type / FIRE_PYRO safety tables + ActionEntry param map**  
Loci: h ~L4–20 action-type list + Council Amendment #2 (FIRE_PYRO never entry/exit —
transition only); ~L86–94 ActionEntry param interpretation table. Cpp ~L3–5 almost no
role banner.

**Claim:** Action catalog and safety rule are **design-doc material** (or short pointer);
council/safety claims need **proper treatment** if still load-bearing (**WN-142** /
**WN-172** family). Param table can be one line per type. Cpp front is the opposite
extreme (thin) — fine if h carries role.

**WN-177** — [Grok] · `ownership` · **LED phase codes split: main.cpp overlay scheme + this enum**  
Locus: h ~L42–45 `LedPhaseValue` “extends kCalNeo* / kRxNeo* overlay scheme in main.cpp”,
values from 20 to avoid collision.

**Claim:** If LED overlay policy is one system, **why half in main and half here?**
Evaluate single home (notify/LED module vs FD) so cal/rx/flight codes don’t drift across
files. Ties to sparse “is this TU enough?” (**WN-178**).

**WN-178** — [Grok] · `ownership` · **Pair is sparse — still the right breakout?**  
Locus: whole pair (~134 h / ~73 cpp) — thin dispatch over `flight_actions.h` constexpr
lists + callbacks.

**Claim:** Specialized and small may be OK, but with LED numbering half in main
(**WN-177**) and actions data in `flight_actions.h`, confirm this isn’t an awkward
**half-extraction**. Keep as clear “execute action lists” façade or fold toward one
actions module — deliberate either way.

#### `flight_director/go_nogo_checks.{cpp,h}`

**WN-179** — [Grok] · `invariant` · **Two-tier Go/No-Go model — first walk encounter; verify SSOT**  
Locus: h ~L7–14 Tier 1 platform (blocks ARM) vs Tier 2 profile (warn only); station lists;
`all_go` = Tier 1 only.

**Claim:** Owner note: **first recalled mention** of this two-tier split in the walk.
Confirm it is still the intended product model, documented outside this header (USER_GUIDE /
SAD / IVP), and that **callers + profiles** (`require_gps_lock` etc.) match. Not “delete” —
**double-check condensation and control** (**WN-182**).

**WN-180** — [Grok] · `comment` · **kGoNoGoMaxChecks bump: dated commit/council archaeology**  
Locus: h ~L24–27 “Bumped 12→16 on 2026-05-14 (commit b/3 of fault-recovery)…” PriorHF +
PriorBOR stations.

**Claim:** Live capacity constant OK; **date-tied commit/council** story → CHANGELOG; short
“room for Tier-1 prior-fault + Tier-2 stations.”

**WN-181** — [Grok] · `comment` · **cpp IVP/Stage tags + garbled etl::string reason note**  
Loci: cpp ~L5 `@brief … (IVP-69)`; ~L39–42 Stage T Batch B IVP-T14 RF Link station essay;
~L47–50 “Branches with %u%% build via etl::string…” — **garbled / hard to parse** (meant
printf `%%` vs ETL string formatting?).

**Claim:** Drop or relocate IVP/stage archaeology; rewrite the ETL reason-building note in
plain English if kept.

**WN-182** — [Grok] · `ownership` · **Go/No-Go vital path — condensed ownership + criticality list**  
Locus: pair + callers (`command_handler` ARM, main snapshot fill, mission profile Tier-2
flags). Feature is **vital** for ops safety.

**Claim:** Double-check Go/No-Go is **tracked and controlled in one place** (this module as
evaluate/print SSOT; snapshot fill and station list not scattered without map). Tangent
(owner): project may want a **criticality inventory** of safety/ops-critical files and
functions — not C++ “tier,” product risk tier. Walk WB **W-15**.

#### `flight_director/guard_evaluator.{cpp,h}`

**WN-183** — [Grok] · `comment` · **Banner: sustain/managed model — ensure not sole SSOT**  
Locus: h ~L3–18 — sustain counters, managed vs unmanaged, phase validity, Council A4.

**Claim:** Explanation is fine in spirit; **verify** the managed/unmanaged + sustain model
is also in design/FD docs so this header isn’t the **only** place that knowledge lives
(**WN-163**).

**WN-184** — [Grok] · `invariant` · **IVP tags + critical “DO NOT” only in comments on kGuardManaged**  
Loci: ~L38 / ~L56 IVP-120 on `kBaroStationary`; ~L42–48 Council A4 +  
`// DO NOT modify at runtime. This array is the contract between the evaluator and the combinator.`  
plus managed/unmanaged table.

**Claim:** Important **contract** (“do not modify at runtime”) lives in **comments that can
be ignored** — not `const`/type-level enforcement beyond `constexpr` (array is still
mutable if someone casts). Prefer stronger structure or tests that lock the table; slim
IVP tags. Safety-adjacent: treat as load-bearing (**W-15**).

**WN-185** — [Grok] · `comment` · **Odd hybrid: table-like Doxygen on guard_evaluator_tick**  
Locus: ~L82–92 — multi-line `//` bullets then `@param` Doxygen lines for the same API.

**Claim:** Looks like **half-migrated docs** (ordinary comments + Doxygen params) —
odd regardless of length. Pick one style; keep short.

— **nothing of note.** (`guard_evaluator.cpp` — owner-directed.)

#### `flight_director/guard_combinator.{cpp,h}`

**WN-186** — [Grok] · `comment` · **Header comment ratio high; pair sparse — evaluate home**  
Locus: `guard_combinator.h` (~90 lines) — long `@file` three-layer safety architecture +
industry cites + Council 2026-03-25; dense Doxygen/field notes relative to surface.
Cpp: no path-specific defect this sitting.

**Claim:** **Large comment ratio** on the header (same family as **WN-149** / density
WNs). Pair overall can feel **sparse** as a breakout (combinator + lockout gates vs
evaluator/FD) — **evaluate** keep as clear safety layer vs over-split (**WN-178** shape).
Cpp: **nothing of note** beyond that shared sparsity question.

#### `flight_director/flight_state.h`

**WN-187** — [Grok] · `comment` · **Banner a bit large + IVP; phase/fault tables → design doc**  
Loci: ~L4–14 IVP-68 Stage 8 banner; ~L24–46 phase topology table + kAbort vs kFault essay;
~L66–86 fault-observable accessor (B.1/B.2/B.3 packing) — **B.2 vague** without decision
doc open.

**Claim:** Topology and fault-pair rules are **design-doc material** if not already
elsewhere (point, don’t rehost). Slim banner; resolve B.x labels (**WN-173**).

#### `flight_director/flight_actions.h`

**WN-188** — [Grok] · `comment` · **Safety FIRE_PYRO table + NeoPixel table + FAULT essay**  
Loci: ~L4–23 Amendment #2 pyro-only-on-transition + per-phase LED table; ~L127–140 FAULT
entry block with **fault-recovery 2026-05-14** dated council essay.

**Claim:** Safety callouts need proper treatment if still law (**WN-176**); LED/phase
tables and long FAULT narrative → design doc / stable link, not dated path essays.

**WN-189** — [Grok] · `comment` · **Stage/IVP/trigger comments must match current product state**  
Locus: these leaves and similar FD files that cite **Stage 8**, IVP-N, “no physical pyro
in Stage 8,” Batch/Stage T, etc.

**Claim:** Comments that freeze a **stage or trigger** can lie after the product moves on.
When remediating or re-reading, **sync comments to current capability** (or rephrase as
historical only with CHANGELOG). Walk WB **W-16**. Applies beyond these two files
wherever Stage/IVP/“in Stage N” language appears.

#### `flight_director/mission_profile.h`

**WN-190** — [Grok] · `comment` · **Banner: clarify mission-use-case config (not board/job); design doc**  
Locus: ~L4–15 MissionProfile feeds FD; distinct from `job.h` (vehicle vs station); rocket
vs HAB vs freeform; serialization-ready.

**Claim:** Directionally right but **not as clear as it could be** — this is
**user-definable per mission / use-case** config (not per board/SKU; job is role). Prefer
design-doc home for the full story; short in-header line: “boot-locked mission/use-case
thresholds for FD.” Confirm wording matches product (profile ≠ vehicle board pack).

**WN-191** — [Grok] · `ownership` · **SI units on profile — project-wide mandatory in functional code**  
Locus: ~L38 `// All thresholds use SI units (m, m/s, m/s^2, ms).`

**Claim:** Worth restating here, but **SI in functional code is project-wide**, not a
local nicety — reinforce in standards if not already crystal (UX display may convert;
HW datasheet units only at the edge when unavoidable). Functional/flight math and
profiles stay SI.

**WN-192** — [Grok] · `comment` · **⚠️ PRELIMINARY markers — emoji compatibility + stale “prelim”**  
Loci: ~L51–54, ~L80, ~L86 guard/lockout/timer blocks marked `⚠️ PRELIMINARY` + validate
before flight.

**Claim:** (1) Confirm **⚠️ is universally OK** in source (no IDE/toolchain mojibake;
ASCII alternative if needed). (2) Secondary: **prelim should be resolved by now** for a
mature walk — either validate and drop ⚠️, or keep with explicit “still unvalidated”
disposition, not eternal yellow stickers.

**WN-193** — [Grok] · `comment` · **ACCEPTED_STANDARDS_DEVIATIONS callback may be stale / soft permission**  
Locus: ~L54 `// See ACCEPTED_STANDARDS_DEVIATIONS.md if deploying unvalidated.`

**Claim:** Odd pointer: may be **stale**, and can be read as **permission to deviate**
further. Prefer “do not fly unvalidated” + real gate/checklist, not “see deviations
file.”

**WN-194** — [Grok] · `comment` · **Safety lockout comments (Council A1)**  
Locus: ~L77–79 deploy/apogee lockouts protect chute from high-q; phase gating primary.

**Claim:** Safety-related — elevate scrutiny of wording vs live lockout behavior
(**WN-172** / **W-15**); keep short + doc pointer if deep.

**WN-195** — [Grok] · `invariant` · **`emergency_deploy_anytime` override — critical scrutiny**  
Locus: ~L93–96 HAB emergency chute skips lockouts for ABORT pyro; rocket respects gates.

**Claim:** An **override that weakens safety gates** is a critical surface: must not be
**easy to flip by bug, wrong profile, or silent default**. Scrutinize who sets it
(generator/profile), tests, and naming/docs. High bar for any path that bypasses lockouts.

#### `flight_director/mission_profile_data.h`

— **light / generated** (`scripts/generate_profile.py` from `profiles/rocket.cfg`; stamp
AUTO-GENERATED / do not edit). Not a semantic deep-walk of the data body.

**WN-196** — [Grok] · `ownership` · **Comments inside generated profile data will be lost on regen**  
Locus: body e.g. ~L54–75 `// IDLE (0)`, `// BOOST (2)`, … phase labels between struct
initializers; file top ~L1–5 is generator-owned (OK if re-emitted).

**Claim:** **Hand or “helpful” comments in the generated output** are either **wiped on
regen** or force the generator to re-emit them every time. Prefer: labels only in
`.cfg` / generator templates, or none in the output. Critical oddity for any “edit the
header” habit. Wider hand-edit / drift automation → walk WB **W-14** (expanded).

#### `flight_director/guard_functions.{cpp,h}`

**WN-197** — [Grok] · `comment` · **Large Doxygen ratio on thin pure-guard API; pair sparse**  
Locus: `guard_functions.h` (~85 lines) — `@file` IVP-70 + per-guard multi-line Doxygen
for simple bool predicates.

**Claim:** Same **high Doxygen / comment ratio** as other FD headers. Pair is **small**
(pure one-sample checks; sustain lives in evaluator) — **evaluate** keep as clear split
vs merge into evaluator (**WN-186** / **WN-178** sparsity family). Header-only density
claim; cpp separately below.

— **nothing of note.** (`guard_functions.cpp` — ~49 lines; no path-specific notes this
sitting beyond the pair-sparsity eval in **WN-197**.)

### log/ + logging/

**WN-198** — [Grok] · `ownership` · **Why `src/log/` vs `src/logging/` — one-file split looks accidental**  
Locus: repo layout — `src/log/` holds only `rc_log.cpp` (public header under
`include/rocketchip/rc_log.h`); `src/logging/` holds rings, flash, PCM, CRC, etc.
Itinerary groups both as “logging/ + log/.”

**Claim:** Separation is hard to justify with **one** TU under `log/`. Prefer **one
directory** (owner lean: **`logging/`** if merge for consistency with other modules, or
**`log/`** for brevity — pick one and move). Walk prep lumping was reasonable; dual folders
likely historical error. Not mid-walk move.

#### `log/rc_log.cpp`

**WN-199** — [Grok] · `comment` · **Long council decision + format-spec inventory in banner**  
Locus: ~L4–27 Approach A council 2026-05-15, absolute plan path, ETL reasoning, full
supported/unsupported printf-spec lists (inventory cite).

**Claim:** **Does not belong as a multi-screen file top** — decision → CHANGELOG/decision
doc; format contract → short list or inventory doc pointer. Council cite needs usual
hygiene (**WN-147**) if kept at all.

**WN-200** — [Grok] · `comment` · **parse_spec “printf” wording + libc-printf phase-out confusion**  
Locus: ~L62–64 `// Parse a printf %-spec starting at *p_inout...`

**Claim:** Comment is awkward / incomplete as a contract. Project largely **phased out
libc printf** for logging — `rc_log` **is** the printf-subset replacement, so “printf-spec”
is intentional but **wording should say rc_log’s format language**, not imply raw printf
is still the system. Clarify for readers who expect printf gone.

**WN-201** — [Grok] · `comment` · **Large design essays (float path, ring sink, drain)**  
Loci: ~L177–197 float hand-roll vs ETL/libc (council criterion); ~L428–443+ target
drop-oldest ring design; ~L527–531 / ~L542–545 ring-health / handle_percent; ~L659–712
`rc_log_drain_to_cdc` idle-path, hold-on-disconnect, TinyUSB flush essay.

**Claim:** Valuable content for a **logging design note**; too large in-file. Keep short
live invariants (drop-oldest, hold-on-disconnect, drain from idle only). Same density
family as fusion/FD essays.

#### `logging/ring_buffer.{cpp,h}`

**WN-202** — [Grok] · `comment` · **Banner: design/council + PSRAM volatile durability caveat**  
Locus: h ~L7–28 layout, crash recovery seqlock, council PSRAM uncached write,  
`IMPORTANT: PSRAM is volatile… flash flight table is durable…`

**Claim:** Design/council bulk → doc. **PSRAM volatility / only-survives-if-Vcc** is a
**safety/ops durability** caveat — keep short + accurate, and fold into criticality
review (**W-15** / logging durability story), not bury in a long banner.

**WN-203** — [Grok] · `comment` · **`kRingMagic` “magic value” vs magic-number standard**  
Locus: ~L37–38 `kRingMagic = 0x52434C47` “RCLG”.

**Claim:** File/wire **magic constants** are usually **not** the same as banned magic
*numbers* in formulas (JSF-151) — still **check** naming/docs so “magic” doesn’t look
like a standards carve-out (**WN-153** calibration magic section). Named fourCC is fine
if justified.

**WN-204** — [Grok] · `comment` · **RingHeader seqlock table + file-wide Doxygen density**  
Locus: ~L40–49 crash-recovery write protocol as comment table; general high Doxygen
ratio across API.

**Claim:** Protocol steps → design note or short bullets; density family **WN-081**.

**WN-205** — [Grok] · `comment` · **cpp IVP/Stage + confusing “Phase 1/2/3” seqlock wording**  
Loci: ~L7 IVP-52b Stage 6; ~L24–34 `// Phase 1/2/3` on seqlock write steps.

**Claim:** IVP/stage tags (**W-16**). “Phase” here means **protocol steps**, not flight
phases — same confusion class as FD “Phase D1” / Stage labels (**WN-160**). Rename to
“step” or “seqlock stage” so it doesn’t read as flight Stage N.

#### `logging/flash_flush.{cpp,h}`

**WN-206** — [Grok] · `comment` · **Council req. #1 + flush Sequence table / Doxygen density**  
Loci: h ~L11–12 `xip_cache_clean_all` council; ~L82–99 `flush_ring_to_flash` @params +
numbered Sequence 1–6 (table-like).

**Claim:** Council tag hygiene (**WN-147**); sequence steps → short list or design note.
Doxygen density on multi-arg flush APIs.

**WN-207** — [Grok] · `comment` · **JPL-25 parameter-limit cite unclear without standards context**  
Locus: cpp ~L270–271 `FlightEntryLayout` “grouped to keep save_flight_entry within the
JPL-25 parameter limit.”

**Claim:** **JPL-25** here is the house **parameter-count** rule (≤6 params; see
CODING_STANDARDS / clang-tidy ParameterThreshold), not a random ticket. Comment should
say “JPL-25 / house max-params” or just “keep param count ≤6” so readers don’t need
archaeology.

**WN-208** — [Grok] · `comment` · **Council req. #1 on xip_cache_clean_all call site**  
Locus: cpp ~L348–349 same council cache-clean requirement as header.

**Claim:** Duplicate process tag; prefer one durable cite or live “must clean XIP before
read PSRAM for flush.”

#### `logging/flight_table.{cpp,h}`

**WN-209** — [Grok] · `ownership` · **Name `flight_table` vague — prefer flight-log table?**  
Locus: module path/name `flight_table` vs role (index of flights in flash log region).

**Claim:** “Flight table” is ambiguous (phase table? markers?). **Flight log table** (or
similar) would match dual-sector **log index** role. Rename if fan-out is manageable —
evaluate at disposition (include/CMake/callers).

**WN-210** — [Grok] · `comment` · **Banner: council flash map + dual-sector design; Doxygen density**  
Locus: h ~L7–17 council req. #4 address map, dual-sector A/B, CRC-32 line; Doxygen on
APIs generally.

**Claim:** Layout table → `flash_layout` / design doc (banner can rot vs SKU);
council tag hygiene. Density family **WN-081**.

**WN-211** — [Grok] · `comment` · **CRC-32 used heavily — clarify for non-insiders**  
Locus: h ~L15, ~L54, ~L66, ~L94, CRC APIs; cpp includes `crc32.h` and compute/validate
helpers throughout.

**Claim:** Same class as **WN-154** (CRC-16 on cal): term may be familiar, but a **short
live line** (poly/family if non-obvious, “integrity over table/entry excluding field”)
helps; or point at shared `crc32` helper docs. Not a algorithm rewrite mid-walk.

#### `logging/log_decimator.{cpp,h}`

**WN-212** — [Grok] · `comment` · **IVP/Stage tags + Markley PA cite; Doxygen density**  
Loci: h ~L7–16 box-car / quat Markley 2007 + IVP-52c Stage 6; Doxygen on API; cpp ~L7
same IVP-52c Stage 6.

**Claim:** IVP/stage hygiene (**W-16**). **Prior art:** Markley 2007 antipodal-protect
quat average — double-check cite still matches implementation (component-wise + sign
flip + normalize). Doxygen density on a small specialized module.

#### `logging/data_convert.{cpp,h}`

**WN-213** — [Grok] · `ownership` · **Sparse convert TU: density/IVP if kept; math currency**  
Loci: h (~40 lines) comment ratio + IVP-49 Stage 6; quantization table ~L23–28; cpp ~L96
IVP-107 health pack. Pair is thin FusedState↔TelemetryState convert.

**Claim:** If it **stays separate**, still trim IVP/Doxygen noise. **Sparse** — evaluate
keep vs fold into encoder/PCM path (**WN-178** family). Separately: **re-verify
quantization math/techniques** (Q15, cm/s, mm alt, packing) still match ICD/telemetry
and current `TelemetryState` layout (**WN-163**).

#### `logging/pcm_frame.cpp`

**WN-214** — [Grok] · `ownership` · **PCM frame path radio-adjacent / Starcom-gated; “Gate N” wording**  
Locus: whole cpp (encode/decode/resync of PCM frames for log/telem wire); ~L63–111
decode/find_sync labeled **Gate 1/2/3** (sync, length, CRC).

**Claim:** General flag — PCM framing sits on the **radio/telemetry/log** surface;
treat as **Starcom-gated supersession candidate** with related early-impl / Early-impl
PCM row (**WN-059** on `pcm_frame.h`, main WB radio/PCM). Not “delete mid-walk.”  
Also: “Gate 1/2/3” is the same **stage/gate wording confusion** class as Go/No-Go tiers
and seqlock “Phase” steps — rename to **check** / **step** if kept (**WN-160** / **WN-205**).
Pairs with header findings **WN-058**–**059**.

#### `logging/psram_init.{cpp,h}`

**WN-215** — [Grok] · `comment` · **Banner IVP/council/map + Doxygen; council on flash-safe API**  
Loci: h ~L3–23 APS6404L / Feather HSTX / XIP map table / council #1/#3 / IVP-52a Stage 6;
Doxygen density; ~L79–81 council req. #2 hard gate on `psram_flash_safe_test`.

**Claim:** Layout + council essays → design/board pack; short live contract. IVP/stage
(**W-16**). Doxygen density.

**WN-216** — [Grok] · `ownership` · **Bespoke APS6404L / Feather PSRAM — board-coupled; PA + datasheet**  
Locus: pair — SparkFun/AudioMorphology/Arduino-Pico lineage (h ~L10–11, cpp ~L7–8);
part **APS6404L-3SQR**, pin 8 CS1, QPI cmds hardcoded for this SKU.

**Claim:** **Prior-art** lineage OK — double-check still current vs upstream. Code is
**board/part-specific**: callers and board packs must not pretend HW-agnostic PSRAM;
other boards need their own init or HAL. **Datasheet ref** in banner/PA block would help
(**W-8**, **WN-086** bespoke drivers).

**WN-217** — [Grok] · `comment` · **“Test 3” / flash-safe test permanence; Step N as good phase-wording model**  
Loci: cpp ~L241 `// Test 3 addresses: start, middle, end` (means **three** address
points in self-test, not “Test #3” suite id — clarify wording); ~L295–301 council req.
#2 flash-safe integrity test — confirm **not a temporary** diagnostic left in prod
API (`psram_flash_safe_test` is public); ~L327–356 `// Step 1`…`// Step 5` on that
procedure.

**Claim:** “Test 3” is ambiguous. Flash-safe test may stay if it’s a real boot/gate
helper — label as permanent vs CLI-only. **Positive example:** numbered **Step N** here
does **not** sound like flight Stage/Phase — model for fixing **Phase N** confusion
elsewhere (**WN-205**, **WN-160**) without a “pass” ticket; note as style precedent only.

#### `logging/radio_config_storage.{cpp,h}`

**WN-218** — [Grok] · `comment` · **Banner: IVP-T5.5 + orphan “Option C”; sparse dual-sector API**  
Locus: h ~L4–13 Stage T IVP-T5.5, “Option C (debounced)” without option A/B context,
dual-sector pattern.

**Claim:** IVP/stage (**W-16**). **“Option C” is out of context** without the decision
doc — drop or link. Thin read/write façade → sparseness/eval keep vs cal-storage pattern
(**WN-178** family); if kept, short role banner only.

**WN-219** — [Grok] · `comment` · **LL Entry 4/12 and 31 cites may be stale**  
Locus: h ~L20–21 init before stdio (LL 4/12); ~L30 `flash_safe_execute` LL Entry 31.

**Claim:** Re-check LL numbers/content still match (**WN-121** re-eval pattern). Prefer
short live boot-order/flash-safe rule + optional LL pointer if still accurate.

**WN-220** — [Grok] · `ownership` · **SX1276-legal validate — HW-coupled OK if module is clear**  
Locus: cpp ~L113–116 reject configs not `radio_config_sx1276_legal`…; user direction
2026-04-21 on advanced values.

**Claim:** HW-specific check is fine **if** the file is clearly **radio/SX1276 config
flash storage** (not generic “settings”). Dated user-direction can slim. Starcom/RF
path may supersede (**WN-214** family). Density on cpp not flagged beyond this.

#### `logging/crc16_ccitt.h`

**WN-221** — [Grok] · `comment` · **Banner IVP + poly/init OK but re-check; C++20 note fragile**  
Loci: ~L3–16 IVP-49 Stage 6 + poly/init/final-XOR/bit-order + table size; ~L25
`// Compile-time CRC-16-CCITT table generation (C++20 constexpr)`.

**Claim:** Spec block may stay (better than cal CRC vagueness **WN-154**) — still
**verify** matches CCSDS/PCM use. IVP/stage (**W-16**). “C++20 constexpr” is
**out of place / rot risk** if dialect changes — prefer “compile-time table” without
language-version pin unless enforced elsewhere.

**WN-222** — [Grok] · `comment` · **“Exception 1 (JSF AV-182)” cast note unclear**  
Locus: ~L63–64 (and same pattern on crc32) void*→uint8_t* “Exception 1 (JSF AV-182)…
confined to this low-level byte routine.”

**Claim:** Reads like a **rule exception** without pointing at an accepted-deviation
entry. If it **is** a logged exception, fix the log/wording; if it’s only “cast
contained here,” say that plainly (JSF-182 cast discipline) — not “Exception 1.”
Remediate if it papers over a real ban without ACCEPTED entry.

#### `logging/crc32.h`

**WN-223** — [Grok] · `comment` · **Same banner/IVP pattern as crc16; Doxygen keep with inventory**  
Locus: ~L3–15 IVP-53a Stage 6 + IEEE poly/init; Doxygen on `crc32` / `crc32_update`
(~L55–59, ~L72–77); same JSF AV-182 lines as **WN-222**.

**Claim:** Spec block re-check + IVP hygiene like **WN-221**. Two Doxygen API blocks
look fine — track with other Doxygen files (**W-10** inventory) for keep/drop
consistency. JSF-182 wording covered by **WN-222**.

### diag/

**WN-224** — [Grok] · `ownership` · **`src/diag/` is only diag_stats — folder layout odd**  
Locus: `src/diag/` holds solely `diag_stats.{cpp,h}` (same shape as lone `src/log/`).

**Claim:** One-module folder is hard to justify (**WN-198** log/). Prefer home under
`cli/` / `safety/` / `logging/` or a broader `diag/` only if more tools land. Evaluate
with keep/delete of the feature (**WN-225**).

#### `diag/diag_stats.{cpp,h}`

**WN-225** — [Grok] · `ownership` · **What is this / is it still needed?**  
Locus: pair — T=0 soak preconditions, full serial snapshot (AO queues, MSP, radio,
health, sensors), MSP high-water tick; always-on after R-25-exec migration from
`src/dev/`.

**Claim:** Banners explain **migration**, not a crisp product role. Further scrutiny:
still required for Stage-17 soaks / ops, or leftover bench tooling in the flight
binary? If needed, document SSOT (USER_GUIDE / soak runbook); if not, plan retire.
Criticality may be **ops** not flight (**W-15**).

**WN-226** — [Grok] · `comment` · **Huge comment ratio / R-25-exec + IVP essays on both files**  
Loci: h ~L4–18 IVP-132, R-25-exec, SWE-133, dump call paths; cpp ~L4–17 same migration
block + always-on list; high narrative density for a thin API (3 functions).

**Claim:** Process history → CHANGELOG/decision doc; short “soak snapshot + T=0 identity
check, read-only.” IVP/R-25 hygiene (**W-16**, **WN-175**).

**WN-227** — [Grok] · `comment` · **Orphan persona/council one-liners in dump body**  
Loci: cpp ~L26 `// ... kRadioCs` include line oddity; ~L50 `// Radio IRQ wiring evidence
(NASA/JPL)`; ~L59 `// SPI hot-path error counter (ArduPilot)`; ~L68 R-25-exec again.

**Claim:** **Out-of-context** council/persona tags without a table of who decided what.
Either drop or one durable design cite. L26 include comment “not sure why there” — re-eval
if needed.

### notify/

#### `notify/notify_backend_audio.cpp`

**WN-228** — [Grok] · `ownership` · **Audio backend is a no-op stub — evaluate keep vs delete**  
Locus: whole file — Stage 14 IVP-115 stub; TLV320DAC3100 + AP tone parser deferred;
RTTTL-like tone string constants `[[maybe_unused]]`; `notify_backend_audio_update` empty.

**Claim:** Owner: ArduPilot-style tones would be nice; Fruit Jam has DAC/speaker but
**I²C issues** blocked enable. Evaluate whether a **stub TU + dead tone tables** still
earn rent until audio stage, or fold constants into a design/data file and drop the
no-op. Near-future **more robust notification engine** (SM/AO) may absorb this —
align with Stage 15 audio IVP plans on main WB.

#### `notify/notify_backend_led.cpp`

**WN-229** — [Grok] · `comment` · **Banner large + IVP refs stale by own admission**  
Locus: ~L3–15 Stage 14 IVP-115/116; “In IVP-115 this is compiled but not called…
IVP-116 wires it up…”

**Claim:** Comment **admits unfinished wiring then claims later IVP finished it** —
classic **stale stage freeze** (**W-16**). Rewrite to **current** call graph (who calls
`notify_backend_led_update` now) or delete the historical story.

**WN-230** — [Grok] · `comment` · **Beacon overlay block: mostly OK, shorten + update Stage L**  
Locus: ~L103–110 Stage L beacon_manual/auto remap essay.

**Claim:** Useful live behavior; **shorten**; “Stage L” label update or drop (**W-16**).

#### `notify/notify_resolver.h`

**WN-231** — [Grok] · `comment` · **Large banner; “not public API” + host-test motivation**  
Locus: ~L3–15 Stage 14 IVP-115; priority order; “internal… Not part of public
notify_backend.h”; “testable directly from host.”

**Claim:** Don’t need long **what we’re not** essays; short “internal LED priority
resolver for backend + host tests.” Host-testability is fine one-liner, not a multi-
line “we did this for tests” justification class. Density + stage tags as usual.

### telemetry/

#### `telemetry/mavlink_rx.cpp`

**WN-232** — [Grok] · `ownership` · **Name “rx” vs bidirectional GCS role; Starcom/MAVLink future**  
Locus: file name `mavlink_rx` + banner ~L5–9 “GCS command receiver” but also “generates
protocol responses”; IVP-62 titled **Bidirectional** MAVLink Commands.

**Claim:** Naming as pure **RX** is misleading — most MAVLink links aren’t RX-only even
on half-duplex (parse in, ACK/param/mission replies out). Clarify product role (USB GCS
secondary path vs LoRa primary). **Starcom-gated** with other telem (**WN-041**,
**WN-046**): prefer **save/rework after Starcom** (or a **MAVLink translation layer** on
Starcom) rather than deep invest in this TU now.

**WN-233** — [Grok] · `comment` · **IVP/Stage 7 banner + vendored mavlink include**  
Loci: ~L11 IVP-62 Stage 7; ~L21–28 `common/mavlink.h` c_library_v2 with GCC pedantic
suppress; ~L15 **double-include** of `mavlink_rx.h` “guard test.”

**Claim:** IVP/stage hygiene (**W-16**). Official **header-only MAVLink C library** is
valid PA/vendoring (not random copy-paste of app logic) — ensure `lib/mavlink` is
properly attributed (**WN-004**). Double-include “test” does **not** belong in production
source — remove.

**WN-234** — [Grok] · `invariant` · **ARM command still no-op “IVP-67 will wire”**  
Locus: ~L210–213 `MAV_CMD_COMPONENT_ARM_DISARM` — “Pre-Flight Director: ACK but no-op.
IVP-67 wires to real ARM.”

**Claim:** Reads as **temporary unfinished wiring**. Confirm whether still true (FD exists
now) or resolved and comment/code stale. Safety-adjacent if GCS thinks it armed.

**WN-235** — [Grok] · `ownership` · **“Legacy” SET_MODE path — red flag under no-back-compat**  
Locus: ~L234–238 `// Legacy SET_MODE message (#11) — same logic as DO_SET_MODE`.

**Claim:** “Legacy” for wire protocol can mean GCS still sends it, but under project
**no backward-compat for abandoned shapes**, evaluate: still required by QGC/MP, or dead
weight that should go. Don’t keep dual paths only for nostalgia (**WN-171** FlightSignal
alias class).

#### `telemetry/telemetry_encoder.cpp`

**WN-236** — [Grok] · `ownership` · **Name is universal; body is CCSDS+MAVLink dual stack; Starcom replace**  
Locus: file name `telemetry_encoder` vs contents — CCSDS primary/secondary headers (CCSDS
133.0-B-2), nav APID encode, **and** MAVLink pack helpers (`c_library_v2`); banner ~L5
“CCSDS and MAVLink telemetry encoders.”

**Claim:** Not CCSDS-only, but name still **reads more universal** than “CCSDS nav + MAVLink
GCS helpers.” Very likely **replaced/split under Starcom** (**WN-041**, **WN-046**,
**WN-232**) — gate deep rework; consider naming that matches dual role until then.

**WN-237** — [Grok] · `comment` · **IVP/Stage/T tags + Q15 constant without plain meaning**  
Loci: ~L18 IVP-107 health; ~L138 Stage T IVP-T5.5 nav-with-config; ~L432 IVP-122 CCSDS
cmd ACK; ~L36–37 `kQ15Scale = 32767` “Q15 fixed-point.”

**Claim:** IVP/stage hygiene (**W-16**). **Q15** = 16-bit fixed-point with 15 fractional
bits (value ≈ int16 / 32767 for unit range, e.g. quaternion) — one plain phrase in comment
if kept; already used in `data_convert` (**WN-213**).

**WN-238** — [Grok] · `comment` · **TelemetryState layout table in comments**  
Locus: ~L96–102 nav payload write — bytes 0–39 / met_ms / reserved layout table.

**Claim:** Useful for ICD adjacency; belongs in **wire ICD / design doc** if it grows;
keep short live “first 40 B of TelemetryState + pad” if any.

### station/

**WN-239** — [Grok] · `ownership` · **`src/station/` only holds idle_tick pair**  
Locus: `src/station/` — solely `station_idle_tick.{cpp,h}` (same one-module-folder
pattern as `src/log/`, `src/diag/`).

**Claim:** Evaluate whether a **`station/` directory** earns rent for one thin idle
helper, or re-home under `core1/` / `cli/` / role init. Related **WN-198**, **WN-224**.

#### `station/station_idle_tick.{cpp,h}`

**WN-240** — [Grok] · `ownership` · **Pair is small — size / breakout eval**  
Locus: h ~25 lines, cpp ~100 lines — station GPS poll from idle bridge reusing
`core1_read_gps` + seqlock.

**Claim:** Specialized but **tiny** — evaluate keep as clear station-role hook vs
inline in `main`/idle bridge (**WN-178** sparsity family).

**WN-241** — [Grok] · `comment` · **Header large ratio; IVP/LL in top block; IVP-140 vs 141 drift**  
Locus: h ~L3–11 Stage 16C IVP-140 “scaffolding only… GPS in IVP-141” vs cpp body that
implements GPS poll (IVP-141).

**Claim:** High comment-to-code ratio for two decls. IVP/LL hygiene (**W-16**). Header
still claims **no-op scaffolding** while cpp does real work — **stale** relative to
current product.

**WN-242** — [Grok] · `comment` · **Cpp banner rehashes project record; file-wide density**  
Locus: cpp ~L3–29 Stage 16C IVP-141, boot GPS bind, core1_read_gps share, rate limit,
LL 32 / watchdog margin essay; remaining body comments similarly heavy.

**Claim:** Large portion restates what belongs in IVP/CHANGELOG/LL — keep short live
“station idle: ~10 Hz GPS via shared core1 reader + seqlock.” General non-Doxygen
density across file.

## Tier 3 — Integrators

### safety/

#### `safety/fault_protection.{cpp,h}`

**WN-243** — [Grok] · `comment` · **Header large ratio; IVP/R-3/plan archaeology**  
Locus: h ~L3–8 OPT-IVP-01 `@file`/`@brief`; ~L30–40 R-3 `kFaultBlink*` tombstone + plan
B.1/B.2/B.3/B.7 + HW_GATE Rule 6; ~L46–84 long Doxygen on three APIs (phase policy,
pre-2026-05-14 Q_onError watchdog story, CAST-1 paragraph). Header ~86 lines for three
decls + one constant.

**Claim:** High comment-to-code ratio. Mass is mostly process/def history (IVP extraction,
R-3 recovery narrative, plan section refs), not short live contracts. Tombstone for removed
blink constants belongs in CHANGELOG/decision docs. Related **W-6**, **W-16**, **WN-054**.

**WN-244** — [Grok] · `comment` · **Cpp rehashes header + B.1–B.7 tags + AP table; small pair**  
Locus: cpp ~L3–8 OPT-IVP banner; ~L20–33 B.7 reentrance essay; helpers ~L36–97 multi-paragraph
rationale; handler ~L99–103 plan B.1–B.3/B.7 + `FAULT_HANDLER_DESIGN` / FH-1; inline B.n
tags ~L109–157; MPU setup ~L224–253 R-3 AP encoding truth table; file ~293 lines with large
comment fraction. Pair is small in live code overall.

**Claim:** Same narratives as header plus plan labels (B.1/B.2/B.7) and a full PMSAv8 AP
table after the encoding fix already lives in the code. Keep short live contracts + pointer
to `FAULT_HANDLER_DESIGN.md` / `FAULT_RECOVERY_*`; move essays/tables out. On tiny LOC any
banner wall skews the 15–25% `.cpp` density band — evaluate absolute bulk/dup, not only %
(**WN-054**, **W-6**, **WN-240** sparsity family).

**WN-245** — [Grok] · `invariant` · **NOLINTBEGIN/END on MPU magic numbers (disallowed)**  
Locus: cpp ~L261–292  
`// NOLINTBEGIN(readability-magic-numbers) — PMSAv8 MPU register bit fields…` through  
`// NOLINTEND(readability-magic-numbers)` around RBAR/RLAR/ctrl/SHCSR packing.

**Claim:** In-source NOLINT suppressions are **not allowed** — fix with named sourced
constants/helper, or accept via deviation log / tool config. Same class as **WN-043**,
**WN-070**, **WN-073**. Finding is the NOLINT mechanism (not a magic-number hunt).

#### `safety/anomalous_boot.{cpp,h}`

**WN-246** — [Grok] · `ownership` · **Separate module for mid-flight boot gate — placement dubious**  
Locus: pair — free functions + static `g_signals` / `g_initialized`; called once early
from `main.cpp` (`anomalous_boot_init`); consumers health/CLI/baro-zero gate. Header
claims mission-critical “refuse fresh-pad boot if mid-flight” confidence gate.

**Claim:** Existence as a **standalone safety free-function module** is dubious as-is.
Something this critical should probably live in an **AO** or another more robust /
testable home (clear ownership, exercise path, integration with health/FD), not a thin
orphan snapshot next to boot. Placement/eval later — not mid-walk rehome. Related
**W-15**, **WN-240** sparsity/breakout family.

**WN-247** — [Grok] · `comment` · **Header massive banner L4–32; general density**  
Locus: h ~L3–33 wall — council 2026-05-14, B.1 zero in-flight reset, false-positive bias
essay, brownout-vs-mid-flight split, absolute path  
`C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn.md` (plan B.4), RP2350 POWMAN refs;
body still has multi-line API notes (~L46–66).

**Claim:** ~30-line design essay on a small header. Process/plan archaeology + machine-local
path (breaks for other agents/hosts). Keep short live contract (early one-shot init,
sentinel clear, verdict bias) + pointer to decision/plan SSOT. General comment density high
for remaining decls. Related **W-6**, **W-16**, **WN-054**.

**WN-248** — [Grok] · `comment` · **Cpp L48–61 AON-timer deferral block; HW-specific surface**  
Locus: cpp ~L48–61 multi-paragraph “AON timer prior-uptime read — DEFERRED to commit (b)”
(POWMAN timer survival matrix, link libs, ambiguous zero); `read_prior_uptime_ms()` is
`return 0U`. Adjacent ~L27–46 POWMAN cause-bit mask comments.

**Claim:** Large comment block for deferred/absent hardware path; live code is a stub.
HW-specific POWMAN detail belongs in datasheet/decision note or stays thin once (b)
lands — don’t keep a commit-(b) essay as permanent body prose. Related **W-6**.

#### `safety/flight_in_progress.cpp`

**WN-249** — [Grok] · `ownership` · **Flight-in-progress sentinel living alone — general caution**  
Locus: whole file — `g_flightInProgressMagic` + set/clear/was_set; decls in
`crash_record.h`; set/clear from FD phase transitions; read/clear once at boot via
`anomalous_boot`. Banner notes split from `crash_record.cpp` for host-test link.

**Claim:** Same class of caution as **WN-246**: a mission-critical **reset-survival
sentinel** as a thin standalone translation unit is easy to under-own / under-test /
miss in review. Link-isolation for host tests is a real constraint, but the *capability*
should still sit in a clear robust home (with anomalous_boot / crash / FD recovery story),
not only as an orphan magic word. Placement/eval later.

**WN-250** — [Grok] · `comment` · **Comment density on tiny sentinel TU**  
Locus: ~L3–9 why-this-file-exists banner; ~L15–21 host-test vs target section / `dsb`
rationale; body is three short functions (~L30–45).

**Claim:** High comment-to-code ratio for ~45-line file. Keep short live contract
(uninitialized-data magic, clear-on-read at boot, host plain global) + pointer if
needed; drop restatement of crash_record / test-link story. Related **W-6**, **WN-054**.

#### `safety/health_monitor.{cpp,h}`

*(Owner note, not a WN: banner already documents AO_HealthMonitor tick + AO consumers —
ties into AOs as designed; no placement-orphan finding.)*

**WN-251** — [Grok] · `comment` · **Header L35–43 + density; IVP; tables; HW coupling**  
Locus: h ~L35–43 primary-byte layout + MCU-out-of-primary IVP-142b-1 essay; ~L51–105
Secondary/Critical enums with multi-line IVP/council/R-3/brownout prose; ~L152–173
persist-ticks council essay; ~L190–203 MCU temp thresholds with datasheet °C + Stage 18
note; ~L270–272 tombstone; banner Stage 13 IVP-104. Bit layouts and threshold tables live
in comments as much as in code.

**Claim:** Large comment ratio for a contract header. IVP/stage/council archaeology
(**W-6**, **W-16**). Encoding tables belong in `HEALTH_CONTRACT.md` if they grow — short
live bit map + pointer. **Potential HW coupling:** die-temp thresholds, POWMAN brownout
narrative, RP2350 margin wording in comments/constants — keep sourced numbers, avoid
embedding board essays in the API surface. Related **WN-054**, **W-8** if HW-agnostic
policy applies at disposition.

**WN-252** — [Grok] · `ownership` · **`DBG_PRINT` health paths — recheck debug/testing policy**  
Locus: cpp `#include "rocketchip/config.h"` // DBG_PRINT (~L32); many sites e.g.
~L318–332 init prior-fault banners; ~L437–465 latch transitions; ~L513–541 MCU/critical
transitions; ~L660–688 latch clear.

**Claim:** Double-check these still follow **current** debug/testing policy
(`standards/DEBUG_OUTPUT.md`, R-5 `rc_log` path, prod vs DEBUG gate, flight-phase noise).
Not asserting wrong today — revalidate vs **WN-016**/`config.h` DBG placement and whether
health transitions should be always-on `rc_log`, gated DBG, or quieter. Related **WN-016**,
**WN-017**.

**WN-253** — [Grok] · `comment` · **Cpp density; tables/IVP; audit history essays**  
Locus: cpp banner ~L3–11 Stage 13 IVP-104 restatement; static-state blocks ~L55–82 R-3 /
fault-recovery essays; latch/MCU/critical tick bodies carry multi-line history; mirrors
header tables/IVP tags throughout.

**Claim:** Same comment-mass class as header — process/audit narrative over short live
invariants. Point at `HEALTH_CONTRACT.md` / CHANGELOG; keep tick logic readable. Related
**W-6**, **WN-251**.

**WN-254** — [Grok] · `comment` · **“Tier 2: Profile” label confusing**  
Locus: cpp ~L762 `// Tier 2: Profile -- GPS needs fresh snapshot` (Go/No-Go fill; similar
Tier 1/2 vocabulary in `go_nogo_checks.h` and `HealthState::go_nogo_ready` “tier-1”).

**Claim:** “Tier 2” here is **Go/No-Go profile checks**, not L2-P5 walk tiers / product
tiers / fusion tiers — easy to misread mid-review. Prefer “Go/No-Go profile (GPS…)” or
pointer to go_nogo SSOT naming so it doesn’t collide with other “tier” languages in-tree.

#### `safety/crash_record.{cpp,h}`

**WN-255** — [Grok] · `comment` · **Header banner PA/dev history; non-repo plan ref; density; HW**  
Locus: h ~L3–26 huge banner — R-3 audit, halt-forever → capture-then-reset, recovery
2026-05-14 commit b/3, phase-aware dispatch restatement; ~L20  
`plan parsed-soaring-popcorn.md sections B.1/B.2/B.3/B.7` (temp work-plan name, **not** an
in-repo doc path); ~L66–87 long `crash_record_capture` essay (R-1/R-4/R-20, commit (b));
~L103–119 flight-in-progress sentinel + RP2350 SRAM/BOR prose. High comment ratio overall.

**Claim:** Process/PA archaeology over short live contracts (**W-6**). **L20** points at a
transient Claude-plan title, not `docs/decisions/FAULT_*` or similar — fix to real SSOT or
drop. Rest of file: density + more **possible HW coupling** in comments (SRAM retention
voltage, POWMAN BOR, AIRCR/SCB addresses, RP2350 §6). Keep layout + magic/reason + 1-line
pointers. Related **WN-247** (same plan-name class), **WN-251**.

**WN-256** — [Grok] · `comment` · **Cpp L12–20 block; HW surface**  
Locus: cpp ~L12–20 `.uninitialized_data` / NOLOAD / magic-garbage / why `g_crash_record` is
extern for fault handler; body uses `scb_hw` CFSR/HFSR/AIRCR + ARM reset recipe ~L27–51.

**Claim:** Banner restates header design. Capture path is inherently HW/SCB — keep thin
sourced register notes; avoid re-hosting ARMv8-M tutorials. Related **W-6**, **WN-255**.

**WN-257** — [Grok] · `invariant` · **Consume clears magic to avoid re-report — latch discipline**  
Locus: cpp ~L62–65  
`// Clear the magic so a clean boot doesn't re-report…` / `g_crash_record.magic = 0` after
successful `crash_record_consume_prior`. (Related pattern ~L36–39 magic-last write for torn
write reject.)

**Claim:** Double-check design: re-report suppression relies on **code correctly clearing**
magic (and nothing re-arming a phantom record). Prefer a **latch-closed** / one-shot consumed
state that is hard to re-open by accident, rather than “hope the clear path always runs.”
Not asserting current path is wrong — revalidate vs health critical latch and multi-boot
story before treating clear-as-ack as sufficient. Related **WN-249** sentinel caution.

#### `safety/fault_inject.{cpp,h}`

**WN-258** — [Grok] · `ownership` · **Fault-inject is test code in mainline flight tree**  
Locus: pair under `src/safety/` — GDB-callable `fault_force_*`, volatiles checked from idle /
watchdog / production paths; migrated from `src/dev/` into single flight binary (R-25-exec
banners; runtime `test_mode_active()` gate). Small TU cluster.

**Claim:** This is **testing / probe injection**, not flight product logic. Should not live
in mainline `src/safety/` if it needs to exist at all — re-home to a clearly non-flight
build (dev target, test-only link, host/probe harness), or drop. Runtime gate is not a
substitute for **not shipping the surface** in the flight source tree. Related R-25-exec /
`FAULT_INJECTION.md` / `test_mode` story at disposition.

**WN-259** — [Grok] · `comment` · **Comment density / R-25-exec dev history on inject pair**  
Locus: h ~L3–15 IVP-129 + R-25-exec migration essay; cpp ~L3–13 same restatement +
~L38–45 gate-helper audit narrative. Bodies are short force hooks.

**Claim:** Same density/PA archaeology class as other safety leaves (**W-6**). Fine as a
one-off test tidbit only if placement (**WN-258**) is accepted; still slim banners to
“GDB fault inject; test_mode gate; see FAULT_INJECTION.md.”

#### `safety/station_fault_inject.{cpp,h}`

**WN-260** — [Grok] · `ownership` · **Station fault-inject is test code in mainline tree**  
Locus: pair under `src/safety/` — `fault_force_station_*`, RX/ACK drop counters, GPS loss;
migrated from `src/dev/` (R-25-exec step 6 banners); hooked from `ao_telemetry` /
`gps_uart`. Small TU.

**Claim:** Same as **WN-258**: station-side **probe test inject** should not live as
mainline safety product code if retained — non-flight home or remove. Gate + job_station
dead branches do not make it flight architecture.

**WN-261** — [Grok] · `comment` · **Station inject: density / R-25-exec history**  
Locus: h ~L3–16 IVP-132a + R-25 migration; cpp ~L3–16 same + ~L32–39 gate mirror comment;
bodies short.

**Claim:** Same comment class as **WN-259** / **W-6**. Slim if kept as test tidbit;
placement first (**WN-260**).

#### `safety/test_mode.{cpp,h}`

**WN-262** — [Grok] · `ownership` · **test_mode is test/inject infrastructure in mainline tree**  
Locus: pair under `src/safety/` — probe-arm magic, `test_mode_active()` gate for all
`fault_force_*` / test affordances; single-binary Approach A. Small but load-bearing for
bench inject.

**Claim:** Same **bucket as WN-258 / WN-260**: this is **test / fault-injection
infrastructure**, not flight product logic. Placement in mainline flight sources is
questionable if inject rehomes or is dropped; evaluate together (gate + inject surface as
one non-flight or clearly sequestered unit). Not asserting remove without a home for the
gate if inject stays.

**WN-263** — [Grok] · `comment` · **Header L3–37 huge design/dev-history block**  
Locus: h ~L3–37 wall — R-23 / F-2026-05-13-004 / R-22, council personas, Approach A
bullet essay (three-condition AND, Therac-25 dual clear, no CLI arm, PX4 SYS_FAILURE_EN
precedent); rest of header still dense (magic, window, accessors, R-25-exec step 11).
Cpp ~L3–6 points at header for “full design rationale + council decision provenance.”

**Claim:** **Ensure this history lives elsewhere** — decision already cited
(`docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md`); header should keep short live
contract (probe magic + Idle + boot window; clears on Idle-exit / refuse ARM; `test_mode_active()`
SSOT) + one pointer, not re-host the full PA/dev narrative. Same density class **W-6** /
**WN-259**.

#### `safety/core1_i2c_pause.{cpp,h}`

**WN-264** — [Grok] · `ownership` · **Standalone pause pair — needed alone or not at all?**  
Locus: pair — only `core1_i2c_pause()` / `core1_i2c_resume()` wrapping
`g_core1PauseI2C` / `g_core1I2CPaused` atomics (owned in `shared_state`); ~40 lines cpp.
Callers: flash paths / `rc_os_commands` / cal (R-17).

**Claim:** Flag whether these files **earn a dedicated module** vs fold into
`shared_state` / `sensor_core1` / flash-safe helpers, **or** whether the capability is
wrong-shaped entirely. Tiny API over two atomics is the breakout smell family
(**WN-240**, **WN-246**). Not mid-walk rehome.

**WN-265** — [Grok] · `comment` · **Header: ~3 API lines vs dozens of comment lines**  
Locus: h ~L6–46 LL-31 race essay, R-15/R-17/R-11, why-not-cal_hooks, R-17 dead
`cal_pre_hook` finding; then ~L50–63 two short function contracts.

**Claim:** Extreme comment-to-code ratio — design/audit history, not a thin contract.
Move LL-31 / R-17 narrative to LESSONS / decision / flash-safe doc; leave 1-line purpose
+ pause/resume contracts. Related **W-6**, **WN-054**.

**WN-266** — [Grok] · `comment` · **Cpp general comment density**  
Locus: cpp body — per-branch comments on sensor-phase skip, already-paused, timeout
belt-and-suspenders (LL-31/R-15), resume dual-flag clear (~L16–44). Small file; comments
restate logic and audit tags.

**Claim:** General density issue on a short TU — slim to non-obvious timeout/resume
ordering only. Related **W-6**, **WN-265**.

#### `safety/pio_backup_timer.{cpp,h}`

**WN-267** — [Grok] · `ownership` · **PIO backup timers — relatively recent deliberate feature**  
Locus: pair — PIO2 dual countdown (drogue/main), arm/cancel/disarm, autonomous of ARM
cores; itinerary notes PIO lifecycle / LL 42.

**Claim:** Owner walk note: this is a **relatively recent** intentional feature. Do **not**
disposition as amateur / early-impl leftover or “why does this exist” without that context.
Still review quality normally; age is not a free pass. Related early-impl group only if
rework-eval is chosen later — not default bucket.

**WN-268** — [Grok] · `comment` · **Header action table + general density**  
Locus: h ~L6–18 purpose + timer-action table (`0` disabled / `1` drogue / `2` main) +
bench-testing note; remaining API one-liners.

**Claim:** Comment table is useful but can drift vs profile/config SSOT — keep short or
point at profile field docs. General header density mild but same class (**W-6**): slim
to live contracts.

**WN-269** — [Grok] · `comment` · **Cpp no file-level explanation; a few body blocks**  
Locus: cpp opens with includes only (no `@file`/purpose banner, unlike many safety peers);
body has sparse blocks (claim SMs, load program, arm path, etc.).

**Claim:** If peers need a short “what/why” pointer, this TU has **none upfront** — either
add a one-liner + decision/LL pointer, or accept pure code if header is SSOT. A few mid-body
comment blocks: keep only non-obvious PIO/SM budget notes. Related **W-6**.

**WN-270** — [Grok] · `ownership` · **L173 `ROCKETCHIP_HOST_TEST` branch — test stubs only?**  
Locus: cpp ~L5 `#ifndef ROCKETCHIP_HOST_TEST` … ~L173 `#else // ROCKETCHIP_HOST_TEST`
through host stubs (`pio_backup_timer_init` always true, arm sets bools, no PIO).

**Claim:** Confirm: this is **host-test stubbing of a real flight feature**, not a
test-only definition of the backup timers themselves. Naming/placement is standard dual-
compile; double-check stubs don’t over-claim “fired”/“armed” semantics vs target. Not a
finding that the feature is test-only.

#### `safety/pio_watchdog.{cpp,h}`

**WN-271** — [Grok] · `comment` · **Header IVP / layer-stack comments up top**  
Locus: h ~L6–15 IVP-88, three-layer safety architecture (smart / heartbeat / backup
IVP-89); countdown formula note ~L21–23.

**Claim:** IVP/dev architecture restatement in header — keep short live “PIO2 heartbeat
IRQ0; feed from main” + pointer to IVP/STAGE11 doc. Related **W-6**, **W-16**.

**WN-272** — [Grok] · `comment` · **Cpp no top block again**  
Locus: cpp opens with includes only (same as **WN-269** backup timer); short body.

**Claim:** No file-level purpose banner — consistent pair with backup timer; same
disposition (one-liner + doc pointer if peers require it, or accept header-as-SSOT).

**WN-273** — [Grok] · `invariant` · **L22 “PIO2 dedicated to safety” claim — rule + enforcement?**  
Locus: cpp ~L22–23  
`// Use PIO2 — dedicated to safety (PIO0 = WS2812, PIO1 = reserved)` / `g_pio = pio2`.

**Claim:** Double-check this is an actual **project rule** (budget map / CODING or HW doc)
and is **properly enforced** if so — not only a comment here. Evidence elsewhere is
descriptive (STAGE11 budget table, NeoPixel claim notes, LL) rather than a mechanical
gate. Confirm SSOT + whether other code can still claim PIO2 SMs / programs and break
the “dedicated” assertion. Related PIO budget early-impl / whiteboard notes.

#### `safety/pyro_edge_logger.{cpp,h}`

**WN-274** — [Grok] · `ownership` · **Edge logger: unclear product role, untested, don’t over-claim**  
Locus: pair — h ~L3–7 “IVP-130… Flight-binary essential… forensic…”; cpp ISR → static
buffer → `dump_cli`.  
**Provenance:** `2469fc3` **IVP-130** PIO backup-timer shakedown (“no logic analyzer”);
`main` init; CLI `y` dump only — no PCM/flight-log/telemetry consumer.

**Claim:** Owner: **doesn’t know what product this is** and **has not been properly tested**
yet. Do not treat as a finished flight-forensic feature from path + “essential” wording, and
do not treat historical IVP-130 PASS language as present-day verification of this module.
Re-establish role (bench helper vs real post-flight path) and re-verify or demote before any
“pyro timing forensic ready” claim. Related **W-16**, **W-3**.

— nothing of note. *(for header/cpp as separate halves — only the ownership claim above)*

#### `safety/rf_link_health.h`

**WN-275** — [Grok] · `comment` · **Large comment ratio; tables / tuning essays (Starcom-gated leaf)**  
Locus: ~L3–12 Stage T banner; ~L34–77 tunables with multi-line LOS/frame-min tables and
2026-04-21 user-direction essays; Schmitt/deadman/anchor blocks restate design §§.

**Claim:** Comment mass (threshold tables, rate examples, rationale) should live in
`STAGE_T_T14_DESIGN.md` / Starcom docs — short live constants + pointer. Related **W-6**,
**WN-054**. **Starcom disposition (not a separate WN):** leaf is radio link-health —
keep/evolve/replace with Starcom; same pattern as telemetry density notes that only matter
if the file survives (**WN-048** / **WN-046** class). Precedence: standalone “Starcom
candidate” WNs (**WN-041**, **WN-046**) when that is the *primary* ownership claim; when
other walk findings exist, gate them with a Starcom qualifier rather than a second WN.

#### `core1/sensor_core1.{cpp,h}`

**WN-276** — [Grok] · `ownership` · **`src/core1/` holds only this pair**  
Locus: `src/core1/` — solely `sensor_core1.{cpp,h}` (same one-module-folder pattern as
`src/station/`, `src/log/`, `src/diag/`).

**Claim:** Flag whether a **dedicated folder** earns rent for one pair, or re-home under
`safety/` / top-level / `drivers` boundary. Related **WN-239**, **WN-264**.

**WN-277** — [Grok] · `comment` · **Header large comment ratio; sparse API**  
Locus: h ~L3–8 banner; multi-line Doxygen on few decls (`core1_entry`, best-gps, `core1_read_gps`,
ESKF externs) — thin public surface, high prose share.

**Claim:** Sparse API vs comment mass — may go with the odd solo-folder packaging
(**WN-276**). Slim contracts; move IVP/share-with-station essays to docs. **W-6**, **WN-054**.

**WN-278** — [Grok] · `comment` · **Cpp density / IVP / R- refs; L479–494 boot-wait essay**  
Locus: file-wide IVP/OPT-IVP/Stage tags; ~L479–494 R-1 vehicle vs station boot-wait essay
(timeout, crash_record, Holzmann P10, static_assert roles).

**Claim:** General density + process archaeology. Huge boot-wait block belongs in
decision/LL + short live “vehicle: 10s bound then crash_record; station: unbounded.” Related
**W-6**, **W-16**.

**WN-279** — [Grok] · `invariant` · **NOLINTBEGIN/END identity matrix indices (disallowed)**  
Locus: cpp ~L378–382  
`// NOLINTBEGIN(readability-magic-numbers) -- 3x3 identity matrix diagonal indices` …
`// NOLINTEND` on `m[0]/m[4]/m[8]`.

**Claim:** In-source NOLINT **not allowed** — named indices/`kIdentityDiag*` or deviation
log. Same class **WN-043**, **WN-245**, **WN-073**.

**WN-280** — [Grok] · `invariant` · **JSF AV Rule 1 cite on Core1SensorCycle — verify**  
Locus: cpp ~L386–388  
`// Extracted from core1_sensor_loop() for JSF AV rule 1 compliance; pure state container.`

**Claim:** Double-check **JSF AV Rule 1** is the correct rule for this extraction (and that
the extract actually satisfies it). Wrong rule numbers have proliferated before (e.g.
notify “Rule 170” saga). Confirm against JSF text + function-size gates; fix cite or drop
if ceremonial.

**WN-281** — [Grok] · `invariant` · **0 °C is a realistic MCU temp — sentinel must not be 0**  
Locus: cpp ~L438–440  
`// Initial MCU temp sentinel so seqlock readers don't see an all-zeros 0.0°C before the
first capture.` / `mcu_die_temp_c = -999.0F`.

**Claim:** Owner: **0 °C is realistically reachable** (cold pad / outdoor leave-on). Using
zero-as-unset would false-positive as a real reading — sentinel (-999) direction is right;
keep any consumer of “valid temp” keyed off the sentinel (or validity flag), not “!= 0”.
Confirm health/`mcu_temp_classify` / readers honor -999 / absent consistently.
