# Dead-Code Inventory (2026-05-22)

**Tool:** `scripts/audit/find_dead_code.py`  **Build dir analyzed:** `build_flight`
**add_executable count:** 75  **ROCKETCHIP_SOURCES count:** 75  **src/**/*.cpp count:** 75

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

**No findings.** The tree is clean against the current build.
