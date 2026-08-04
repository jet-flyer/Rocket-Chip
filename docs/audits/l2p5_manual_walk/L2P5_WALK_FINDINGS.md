**Last edited:** 2026-08-04 · Grok · math/mat.h WN-074–075

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
| **Split vs merge** | One `WN` per claim that could disposition differently later. Same sitting ≠ merge. One `WN` only if bullets support a single claim. |
| **Content** | Observations only; no design close-outs. Stated claim on the declaring file; enforcement/evidence under the implementing path when walked (new `WN`, don't edit old). |
| **Itinerary sync** | Adding a later path while an earlier itinerary path is checked off but missing here → flag owner for `— nothing of note.` (do not invent it). |
| **Project-wide** | Findings that are not a single itinerary path live under **Project-wide** (kept **above** per-file tiers so later file appends do not bury them). Still use global `WN-NNN`. |

**Next ID:** WN-076

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
