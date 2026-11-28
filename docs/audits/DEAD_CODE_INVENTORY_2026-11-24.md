# Dead-Code Inventory (2026-11-24)

**Tool:** `scripts/audit/find_dead_code.py`  **Build dir analyzed:** `build`
**add_executable count:** 81  **ROCKETCHIP_SOURCES count:** 81  **src/**/*.cpp count:** 85

## Methodology

Per `standards/CODING_STANDARDS.md` "Scratch Tools and Dead-Code Discipline" (2026-05-22, four-persona council unanimous):

- **Class 1 (unbuilt orphans):** `src/**/*.cpp` files not in EITHER source list. Never compiled.
- **Class 2 (DCE candidates):** Files in `add_executable` whose per-TU object emits zero externally-visible symbols. R-5 Unit D failure mode.
- **Class 3 (stale build cache):** `.cpp.obj` files in build dir whose source no longer exists.
- **Class 4 (CMake source-list drift):** Files in `ROCKETCHIP_SOURCES` (pedantic-gated) but NOT in `add_executable` (compiled) — silent missing-build. Or vice versa: in `add_executable` but escaping the pedantic gate. The CMakeLists.txt block at the ROCKETCHIP_SOURCES definition warns about this drift explicitly.

**Caveats:**

- Class 2 false-positive: a TU consisting entirely of `static` definitions referenced internally emits zero external symbols by design. Verify by hand-reading before deleting.
- Class 2 false-negative: link-time dead-stripping is not modeled (build has no LTO; cross-TU DCE is limited to per-TU static extraction).
- Out of scope: AST dead-branch, dead CLI command, dead global variable detection.

## Findings

### Class 1: Unbuilt orphans (4)

These `.cpp` files exist in `src/` but are not in either source list. They are never compiled into either firmware binary.

- `src/drivers/baro_dps310_legacy.cpp`
  - Doc-walk: `grep -n "baro_dps310_legacy" docs/SAD.md docs/SCAFFOLDING.md docs/AO_ARCHITECTURE.md docs/MULTICORE_RULES.md standards/CODING_STANDARDS.md README.md docs/PROJECT_STATUS.md`
- `src/drivers/gps_pa1010d_legacy.cpp`
  - Doc-walk: `grep -n "gps_pa1010d_legacy" docs/SAD.md docs/SCAFFOLDING.md docs/AO_ARCHITECTURE.md docs/MULTICORE_RULES.md standards/CODING_STANDARDS.md README.md docs/PROJECT_STATUS.md`
- `src/drivers/i2c_bus_legacy.cpp`
  - Doc-walk: `grep -n "i2c_bus_legacy" docs/SAD.md docs/SCAFFOLDING.md docs/AO_ARCHITECTURE.md docs/MULTICORE_RULES.md standards/CODING_STANDARDS.md README.md docs/PROJECT_STATUS.md`
- `src/drivers/icm20948_legacy.cpp`
  - Doc-walk: `grep -n "icm20948_legacy" docs/SAD.md docs/SCAFFOLDING.md docs/AO_ARCHITECTURE.md docs/MULTICORE_RULES.md standards/CODING_STANDARDS.md README.md docs/PROJECT_STATUS.md`

## Operator next steps

For each Class 1 / 2 / 4 finding:

1. **Trace history before acting** (policy rule, added 2026-05-22 after the `BaroKF` case). For each finding, run `git log --all --oneline -- <file>` and `grep -ni <symbol> CHANGELOG.md`. If history shows a deliberate intermediate state (e.g., kept for host tests after firmware removal), the right action is usually to restore the intended state by fixing the drifted metadata, NOT to delete.
2. Run the doc-walk grep above for the finding. If matches are found, the deletion commit MUST update those docs in the same commit (per `SESSION_CHECKLIST.md` trigger-driven edit rule).
3. Read the file. Confirm the apparent deadness — look for `static` machinery referenced from paired headers, ODR-shared template instantiations, friend-class arrangements, anything that could be load-bearing without an external symbol.
4. If confirmed dead: delete + update docs + update `CMakeLists.txt`.
5. If confirmed alive but flagged (e.g., paired-header `static` arrangement, deliberately-kept host-test target): document in the commit closing the finding.

Per the policy: this report is advisory. No auto-deletion.

## Hand-walk (2026-11-24, Tiny 2350 Phase 1 stage close)

- **Class 1, all 4 `*_legacy.cpp`: keep for now.** History shows a deliberate state: `0f53a02c` (2026-09-01) replaced `i2c_bus` with `i2c_master` and kept the old drivers uncompiled as `*_legacy`; `docs/SCAFFOLDING.md` lists them as not in the flight link. `AGENT_WHITEBOARD.md` notes 3 of them no longer compile. Their dead `--wrap` reboot wrappers were removed in this stage close (LL Entry 50). Deleting the files is a separate owner decision; git history keeps them.
- **Classes 2-4:** no findings.
- **Host-test survey (`survey_test_subjects.py`):** 19 tests flagged; every included header is header-only, and all but one are included by production code. Finding: `src/cli/cli_keymap.h` is used only by `test/test_cli_keymap.cpp` (added `c93019db`, 2026-08-27). Noted for later; not deleted in this stage close.
