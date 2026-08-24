# Dead-Code Inventory (2026-08-24)

**Tool:** `scripts/audit/find_dead_code.py`  **Build dir analyzed:** `build_flight`
**add_executable count:** 72  **ROCKETCHIP_SOURCES count:** 72  **src/**/*.cpp count:** 72

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

### Class 3: Stale build cache (4)

Compiled object files in the build directory whose source no longer exists. Not a tree-dead-code finding (the source IS gone), but signals previous migrations that didn't clean the build cache.

- `build_flight/CMakeFiles/rocketchip.dir/src/fusion/eskf_brake.cpp.obj` (expected source `src/fusion/eskf_brake.cpp` missing)
- `build_flight/CMakeFiles/rocketchip.dir/src/safety/core1_i2c_pause.cpp.obj` (expected source `src/safety/core1_i2c_pause.cpp` missing)
- `build_flight/CMakeFiles/rocketchip.dir/src/safety/flight_in_progress.cpp.obj` (expected source `src/safety/flight_in_progress.cpp` missing)
- `build_flight/CMakeFiles/rocketchip.dir/src/safety/test_mode.cpp.obj` (expected source `src/safety/test_mode.cpp` missing)

Resolution: `cmake --build build_flight --target clean && cmake --build build_flight`.

## Operator next steps

For each Class 1 / 2 / 4 finding:

1. **Trace history before acting** (policy rule, added 2026-05-22 after the `BaroKF` case). For each finding, run `git log --all --oneline -- <file>` and `grep -ni <symbol> CHANGELOG.md`. If history shows a deliberate intermediate state (e.g., kept for host tests after firmware removal), the right action is usually to restore the intended state by fixing the drifted metadata, NOT to delete.
2. Run the doc-walk grep above for the finding. If matches are found, the deletion commit MUST update those docs in the same commit (per `SESSION_CHECKLIST.md` trigger-driven edit rule).
3. Read the file. Confirm the apparent deadness — look for `static` machinery referenced from paired headers, ODR-shared template instantiations, friend-class arrangements, anything that could be load-bearing without an external symbol.
4. If confirmed dead: delete + update docs + update `CMakeLists.txt`.
5. If confirmed alive but flagged (e.g., paired-header `static` arrangement, deliberately-kept host-test target): document in the commit closing the finding.

Per the policy: this report is advisory. No auto-deletion.
