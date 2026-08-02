**Last edited:** 2026-08-02 · Grok · session end: config.h walked (WN-005–018); itinerary 3/184

# L2-P5 Walk Findings

Owner-initiated observations from the manual file walk (may be drafted with an agent after discussion).
Same path strings as `L2P5_WALK_ITINERARY.md` (coverage checkboxes); add path sections as files are walked.
Tangents → `L2P5_WALK_WHITEBOARD.md`. Not PASS/FAIL, remediation, or disposition.

| Rule | Detail |
|------|--------|
| **IDs** | Global `WN-NNN`, fixed prefix, never renumber. Only noteworthy entries get IDs. `— nothing of note.` is unnumbered and does not advance the counter. **Next ID** = `max(WN-*)+1` in body (line below is convenience; body wins). Section IDs may look out of order after revisits — feature, not bug. |
| **Who** | `[Agent]` = who wrote the text (name only). Every entry is owner-initiated; agents do not open paths or invent `nothing of note` on their own. |
| **Form** | `**WN-NNN** — [Agent] · \`kind\` · **title**` then observation. Kind = context (`comment`, `invariant`, `ownership`, `cross-file`), not priority/fix-timing. Anchor with symbol and/or quote (lines optional/soft). |
| **Path placement** | Each `#### \`path\`` is the itinerary path for the **claim’s locus** (where the quoted code/comment lives). Before appending a WN: open/confirm that path section — do **not** append under the last `####` in the file. Cross-file notes: section = declaring file (or Project-wide). Mis-nested WNs are process bugs; fix on notice. |
| **Append-only** | Frozen once written; never edit prior text. Later find on same path → new global `WN`. |
| **Split vs merge** | One `WN` per claim that could disposition differently later. Same sitting ≠ merge. One `WN` only if bullets support a single claim. |
| **Content** | Observations only; no design close-outs. Stated claim on the declaring file; enforcement/evidence under the implementing path when walked (new `WN`, don't edit old). |
| **Itinerary sync** | Adding a later path while an earlier itinerary path is checked off but missing here → flag owner for `— nothing of note.` (do not invent it). |
| **Project-wide** | Findings that are not a single itinerary path live under **Project-wide** (kept **above** per-file tiers so later file appends do not bury them). Still use global `WN-NNN`. |

**Next ID:** WN-019

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
