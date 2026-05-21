# 4-tier scaffolding retirement — inventory

**Date:** 2026-05-21
**Origin:** User direction 2026-05-21: "we don't need backwards compatibility and if you find anything that surfaces artifacts of the 4 builds we need to address it this session."
**Context:** R-25-exec (2026-05-13) collapsed the 4-tier build matrix (vehicle-bench / vehicle-flight / station-bench / station-flight) to 2-tier (vehicle-flight / station-flight). The collapse was deliberately conservative — leaving backward-compat scaffolding so any external workflow that knew the old names would still parse cleanly. The R-25-exec decision doc explicitly said "**Tier-collapse commit (per amendment #5 — last, not first):** `verify_build_parity.sh` 4-tier → 2-tier, CMakeLists.txt removes `NOT_CERTIFIED_FOR_FLIGHT=ON` build configurations" — that line landed for the build-parity verifier and CMakeLists *gating*, but the `CMakePresets.json` preset list and the `_rc_test_common.py` Python-side scaffolding were left in place. R-26 (2026-05-15) cleaned the *firmware-side* dead `#ifdef`. This inventory closes the remaining script + CMakePresets-side scaffolding.

## Classification key

- **DEAD** — comment-only reference (tombstone) inside an active file. Trivial to remove.
- **SCAFFOLDING** — code that still works (parses, compiles, runs) but exists only for backward compat. Removable per user direction.
- **CONSUMER** — production script that *uses* a retired-name symbol. Must be migrated to the canonical name (`*_FLIGHT` instead of `*_BENCH`) before the alias is deleted.
- **TEST FIXTURE** — synthetic test data exercising the retired banner-suffix discrimination. Removable per user direction (no firmware can ever again emit `dev-<sha>` post-R-26).
- **HISTORICAL** — frozen-on-commit document (CHANGELOG, audit report, decision doc, PROBLEM_REPORTS closed row). Do not touch.

---

## Inventory

### `CMakePresets.json` — SCAFFOLDING

| Lines | Item | Action |
|---|---|---|
| 6-13 | `vehicle-bench` preset (with `NOT_CERTIFIED_FOR_FLIGHT=ON` cache var) | DELETE preset entirely |
| 14-21 | `vehicle-flight` preset's `NOT_CERTIFIED_FOR_FLIGHT=OFF` cache var | DELETE the `cacheVariables` map (flag is no-op) |
| 24-33 | `station-bench` preset (with `NOT_CERTIFIED_FOR_FLIGHT=ON` cache var) | DELETE preset entirely |
| 34-44 | `station-flight` preset's `NOT_CERTIFIED_FOR_FLIGHT=OFF` cache var | DELETE the `cacheVariables` map (keep `PICO_BOARD` + `ROCKETCHIP_JOB_STATION`) |
| 47-50 | `buildPresets` list — 4 entries, 2 for the bench presets | DELETE the 2 bench entries |

After: 2 configure presets (`vehicle-flight`, `station-flight`), 2 build presets.

### `scripts/_rc_test_common.py` — SCAFFOLDING

| Lines | Item | Action |
|---|---|---|
| 126-132 | Build-tag suffix comment explains `dev-<sha>` vs `flight-<sha>` + the `bench` legacy. Re_RE_BUILD_TAG regex still matches `(dev\|bench\|flight)` | Comment: simplify to "always `flight-<sha>` post-R-26". Regex: tighten to `r'\bflight-([0-9a-f]{6,12})\b'`. |
| 176-184 | `Build` enum with `DEV` and `FLIGHT` values + historical comment about the `BENCH` → `DEV` rename | Remove `DEV` value. Keep `FLIGHT`, `ANY`, `UNKNOWN`. Update comment. |
| 217-228 | `matches()` permissive-on-Build.ANY logic | Simplify — Build is now always FLIGHT (or UNKNOWN at classify time if banner unparsed). |
| 233 | `TARGET_VEHICLE_DEV = Target(Role.VEHICLE, Build.DEV)` | DELETE |
| 236 | `TARGET_STATION_DEV = Target(Role.STATION, Build.DEV)` | DELETE |
| 240-243 | Backward-compat aliases `TARGET_VEHICLE_BENCH` / `TARGET_STATION_BENCH` | DELETE both aliases + their preceding 4-line "Backward-compat aliases (2026-04-27)" comment block |
| 316-318 | classify_banner: `build = Build.FLIGHT if m_build.group(1) == 'flight' else Build.DEV` | Simplify: regex won't match non-flight anymore, so `build = Build.FLIGHT` |
| 851-852 | `__all__` export list contains `TARGET_VEHICLE_BENCH` / `TARGET_STATION_BENCH` | DELETE |

### `scripts/test__rc_test_common.py` — TEST FIXTURE (synthetic) + CONSUMER

| Lines | Item | Action |
|---|---|---|
| 41-42 | imports `TARGET_VEHICLE_BENCH` + `TARGET_STATION_BENCH` | DELETE from import list |
| 51-64 | `VEHICLE_BENCH_BANNER` synthetic fixture (with `dev-b411e3a`) | DELETE the fixture entirely |
| 135-145 | `test_classify_vehicle_bench()` test function | DELETE the test function (regex no longer needs to recognize `dev-` suffix) |
| 139 | `check('build is DEV', ...)` | Falls away with the test function |
| 184-189 | `test_classify_vehicle_modes()` uses `VEHICLE_BENCH_BANNER` | Update to use only `VEHICLE_FLIGHT_BANNER` |
| 208-244 | `test_target_matches()` extensive tests using VEHICLE_BENCH / DEV-vs-FLIGHT discrimination | Simplify: VEHICLE/STATION matching only; build-suffix discrimination removed |
| 252-262 | `test_banner_frozen()` uses `VEHICLE_BENCH_BANNER` | Switch to `VEHICLE_FLIGHT_BANNER` |
| 265-273 | `test_target_frozen()` uses `TARGET_VEHICLE_BENCH` | Switch to `TARGET_VEHICLE_FLIGHT` |
| 276-281 | `test_target_str()` asserts `'vehicle-dev'` string format | Either delete the dev-suffix assertion line, or keep only the FLIGHT + ANY assertions |
| 389 | decorator-test fixture uses `TARGET_STATION_BENCH` | Switch to `TARGET_STATION_FLIGHT` |
| 394 | matching assertion | Switch to `TARGET_STATION_FLIGHT` |

Tests reduce in count by ~3 (the dev-classification ones go); remaining tests get simpler.

### `scripts/cla_collect.py` — CONSUMER

| Lines | Item | Action |
|---|---|---|
| 41 | `from _rc_test_common import (..., TARGET_VEHICLE_BENCH, ...)` | Change to `TARGET_VEHICLE_FLIGHT` |
| 451 | `@rc_test(target=TARGET_VEHICLE_BENCH)` | Change to `TARGET_VEHICLE_FLIGHT` |
| 469 | `find_target_port(TARGET_VEHICLE_BENCH, ...)` | Change to `TARGET_VEHICLE_FLIGHT` |
| 483 | `open_classified_port(port_name, target=TARGET_VEHICLE_BENCH, ...)` | Change to `TARGET_VEHICLE_FLIGHT` |

### `scripts/soak_test.py` — CONSUMER

| Lines | Item | Action |
|---|---|---|
| 29 | `from _rc_test_common import (..., TARGET_VEHICLE_BENCH, ...)` | Change to `TARGET_VEHICLE_FLIGHT` |
| 88 | `@rc_test(target=TARGET_VEHICLE_BENCH, watchdog_s=86400.0)` | Change to `TARGET_VEHICLE_FLIGHT` |
| 101 | `find_target_port(TARGET_VEHICLE_BENCH, ...)` | Change to `TARGET_VEHICLE_FLIGHT` |
| 110 | `open_classified_port(port_name, target=TARGET_VEHICLE_BENCH, ...)` | Change to `TARGET_VEHICLE_FLIGHT` |

### `CMakeLists.txt` — DEAD (tombstone comments)

| Lines | Item | Action |
|---|---|---|
| 729 | "The NOT_CERTIFIED_FOR_FLIGHT-only --undefined block is now empty" | Decision: Keep or strip? Tombstone explains what the *absent* block used to do. Recommendation: STRIP, the code is gone. |
| 763 | "...NOT_CERTIFIED_FOR_FLIGHT was retired here." | Decision: Keep or strip? Lives inside a longer commentary block that's still load-bearing (explains the single-binary decision). Recommendation: STRIP the sentence; keep the surrounding explanation. |
| 800 + 814 | "R-25-exec step 8 (2026-05-13): NOT_CERTIFIED_FOR_FLIGHT retired." | Decision: Keep or strip? These mark Stage-T flag locations. Recommendation: STRIP — the gating mechanism is clear from the surrounding flag name. |

### `src/main.cpp` and `src/fusion/eskf_runner.cpp` — DEAD (tombstone comments)

| Lines | Item | Action |
|---|---|---|
| `src/main.cpp:433` | "R-25-exec step 8 (2026-05-13): ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS..." | STRIP the tombstone |
| `src/fusion/eskf_runner.cpp:517` | Same | STRIP the tombstone |
| `src/fusion/eskf_runner.cpp:663` | Same | STRIP the tombstone |

These three are all "where the `#ifdef` used to live" tombstones from R-25-exec. The gating macro is gone, the deleted-code-explanation has aged out.

### `include/rocketchip/version.h` — SCAFFOLDING (constants)

| Lines | Item | Action |
|---|---|---|
| 19-24 | `kBuildConfig = "flight"` and `kBuildForFlight = true` (R-26 made unconditional, with explanation comment) | Keep `kBuildConfig` (3 consumers in banner-printing). Consider deleting `kBuildForFlight` (only 1 consumer in `diag_stats.cpp:39` printing the flag value — pointless when the flag has only one value). Recommendation: DELETE `kBuildForFlight` and its diag_stats consumer line. Keep `kBuildConfig` since "flight" in the banner is still meaningful as a self-identifier. |

### `src/diag/diag_stats.cpp` — CONSUMER (small)

| Lines | Item | Action |
|---|---|---|
| 39 | `printf("..." , kBuildConfig, kBuildForFlight ? 1 : 0);` | Remove `kBuildForFlight ? 1 : 0` argument + corresponding format specifier. |

### Active documentation — SCAFFOLDING (descriptive text)

| File | Lines | Item | Action |
|---|---|---|---|
| `docs/BOARD_FIRMWARE_VERIFICATION.md` | 108 | "...confirm the values match the target role. (NOT_CERTIFIED_FOR_FLIGHT..." | Update text — the flag is retired. |
| `docs/BUILD_SYSTEM_AUDIT.md` | 69 | "(NOT_CERTIFIED_FOR_FLIGHT retired 2026-05-13 per R-25-exec — single..." | Likely keep — this *is* the historical record of the audit. Read first to decide. |
| `docs/FAULT_INJECTION.md` | 13 | (matches `ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS`) | Read + decide |
| `docs/PROJECT_STATUS.md` | 104 | (matches) | Read + decide |
| `docs/audits/VERSION_STRING_AUDIT.md` | 24 | "`kBuildConfig` `"bench"` or `"flight"`" | Update to "always `"flight"` post-R-26" |
| `docs/audits/AUDIT_COVERAGE_QUICK_CLOSURES_2026-05-15.md` | (various) | Historical-record per `SESSION_CHECKLIST.md` trigger map | DO NOT TOUCH (frozen historical-record) |
| `docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md` | (various) | Historical-record decision doc | DO NOT TOUCH |
| `docs/audits/MASTER_STANDARDS_AUDIT_2026-05-07*.md`, `*_2026-05-13.md` | (various) | Historical audit reports | DO NOT TOUCH |
| `docs/plans/STAGE16C_STATION_DECOUPLING*.md` | (various) | Plan docs (frozen historical-record) | DO NOT TOUCH |
| `docs/plans/IVP-132a.4_reeval.md` | 16 | (matches `BUILD_FOR_FLIGHT`) | Plan doc — DO NOT TOUCH |
| `docs/plans/DEFERRED_CLEANUP_PLAN_2026-05-13.md` | 107 | (matches) | Plan doc — DO NOT TOUCH |
| `docs/audits/DEV_CODE_AUDIT.md` | 9, 11, 19-22, 38 | (matches `BUILD_FOR_FLIGHT`) | Historical audit — DO NOT TOUCH |
| `docs/benchmarks/UD_BENCHMARK_RESULTS.md` | 479 | "Both are dev-build only (!BUILD_FOR_FLIGHT); the Hardware..." | Read + decide — likely historical too |
| `standards/AUDIT_GUIDANCE.md` | 256 | Already has parenthetical noting retirement | Keep as-is (it's correct documentation) |
| `.claude/SESSION_CHECKLIST.md` | 69 | (matches `ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS` or similar in trigger map?) | Read + decide |

---

## Proposed commit sequencing

Following the dependency order (consumer → alias → enum → preset), to avoid a "broken import" intermediate state on any commit:

1. **Commit A — Consumer migrations.** Update `cla_collect.py` + `soak_test.py` + `test__rc_test_common.py` decorator-test fixture (line 389) to `TARGET_VEHICLE_FLIGHT` / `TARGET_STATION_FLIGHT`. Update `diag_stats.cpp:39` to stop using `kBuildForFlight`. Host ctest must pass after this commit. Pre-commit hook gate fires (touches `src/`).
2. **Commit B — Test-fixture cleanup.** Delete `VEHICLE_BENCH_BANNER` + `test_classify_vehicle_bench` + DEV-related assertions in `test__rc_test_common.py`. Simplify `test_target_matches` / `test_target_str` / `test_classify_vehicle_modes`. Run the test directly to confirm no regression. Pure-Python.
3. **Commit C — `_rc_test_common.py` alias + enum removal.** Delete `TARGET_VEHICLE_BENCH`, `TARGET_STATION_BENCH`, `TARGET_VEHICLE_DEV`, `TARGET_STATION_DEV`. Delete `Build.DEV` from the enum. Tighten `_RE_BUILD_TAG` regex to `flight-` only. Simplify `matches()`. Update `__all__`. After this commit, `Build` enum has 3 values: FLIGHT, ANY, UNKNOWN. Run test__rc_test_common.py to confirm.
4. **Commit D — `CMakePresets.json` cleanup.** Delete `vehicle-bench` + `station-bench` presets. Delete `NOT_CERTIFIED_FOR_FLIGHT` cache variables from the remaining 2. Reduce `buildPresets` to 2 entries. Verify `cmake -B build_flight --preset vehicle-flight ..` still configures.
5. **Commit E — Header/source tombstone strips.** Trim the 4 `NOT_CERTIFIED_FOR_FLIGHT` tombstone comments in `CMakeLists.txt`. Trim the 3 `ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS` tombstones in `src/main.cpp` + `src/fusion/eskf_runner.cpp`. Optionally delete `kBuildForFlight` constant from `version.h` after consumer migration in Commit A. Pre-commit hook fires (touches `src/` + `CMakeLists.txt`).
6. **Commit F — Doc reconciliation.** Update `BOARD_FIRMWARE_VERIFICATION.md`, `VERSION_STRING_AUDIT.md`, possibly `FAULT_INJECTION.md` + `PROJECT_STATUS.md` (after reading). Historical docs (audits, decisions, plans, CHANGELOG, frozen LL entries) are NOT touched per state-of-system vs historical-record discipline in `SESSION_CHECKLIST.md`.

Pre-commit hook will fire on commits A, D, E (touch `src/` or `CMakeLists.txt`). Each commit must observe the positive-control signal per HW_GATE_DISCIPLINE Rule 3. Commits B, C, F are pure-Python or pure-doc.

## Risk

- Commit A → C order matters. Reversing C and A would break imports in B / C / D.
- The retired-tier `vehicle-bench` preset will fail any external pinned recipe (an old shell script or CI config that says `cmake --preset vehicle-bench`). Per user direction "no backwards compat" — this is accepted. No `vehicle-bench` recipe exists inside the repo per the grep audit.
- The test file `test__rc_test_common.py` has detailed assertions about the regex's ability to recognize `dev-<sha>` — those go. Test count drops by ~3.

## Additional finds (added 2026-05-21 after user widened the sweep)

### `standards/` and `.claude/` — SCAFFOLDING (parenthetical retirement notes)

User direction 2026-05-21 was "address any references that surface 4-build artifacts." Re-swept `standards/` + `.claude/` + `README.md`. Two state-of-system docs carry retired-macro parenthetical notes:

| File | Line | Item | Action |
|---|---|---|---|
| `standards/AUDIT_GUIDANCE.md` | 256 | "(`NOT_CERTIFIED_FOR_FLIGHT` + `ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS` were retired by R-25-exec 2026-05-13; runtime gate `rc::test_mode_active()` replaced them.)" | STRIP the entire parenthetical. The sentence works without it. |
| `.claude/SESSION_CHECKLIST.md` | 69 | "(`NOT_CERTIFIED_FOR_FLIGHT` and `ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS` retired 2026-05-13 per R-25-exec — single flight binary per role with runtime test-mode gating via `rc::test_mode_active()`; see `docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md`.)" | KEEP the "single flight binary per role with runtime test-mode gating" framing — that part is load-bearing. STRIP the retired-macro names. Result: "(single flight binary per role with runtime test-mode gating via `rc::test_mode_active()`; see `docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md`.)" |
| `docs/BUILD_SYSTEM_AUDIT.md` | 69-71 | "(`NOT_CERTIFIED_FOR_FLIGHT` retired 2026-05-13 per R-25-exec — single flight binary with runtime test-mode gating; see `docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md`.)" | KEEP the "single flight binary with runtime test-mode gating" framing. STRIP the retired-macro name. |

These move to **Commit F** (doc reconciliation).

### `standards/CODING_STANDARDS.md` — CLEAN

Confirmed with `grep -E "BUILD_FOR_FLIGHT|NOT_CERTIFIED_FOR_FLIGHT|ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS|vehicle-bench|station-bench|kBuildConfig|kBuildForFlight|build/rocketchip-dev|4-tier"`: **no matches.** The Code Classification table is unchanged; no 4-tier artifacts present.

The "prescriptive vs descriptive framing" question (whether the three-tier Flight-Critical / Flight-Support / Ground classification stays as a gate input or becomes informational-only) is a *separate* design discussion — it's about the three runtime tiers, not the retired four build tiers. That discussion stays deferred to its own session.

## Out of scope (not retiring this session)

- `docs/audits/VERSION_STRING_AUDIT.md:24` — historical audit, do not touch.
- `docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md` — historical decision doc, do not touch.
- `docs/audits/DEV_CODE_AUDIT.md` — historical audit, do not touch.
- All other audit / decision / plan / CHANGELOG references — historical-record.
- `standards/CODING_STANDARDS.md` "Code Classification" table prescriptive-vs-descriptive design discussion — separate WB row, deferred (this is *not* a 4-tier artifact; it's a different question about how three *runtime* classifications interact with the gate system).
