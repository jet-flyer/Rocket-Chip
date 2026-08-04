**Last edited:** 2026-08-04 · Grok · ao_signals closed WN-052–053; session end 25/184

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

**Next ID:** WN-055

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
