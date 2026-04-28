# RocketChip Standards Audit — 2026-04-28 (Stage O / OPT-IVP-01, 02, 05)

**Audit date:** 2026-04-28  
**Scope:** Stage O safety + globals + ESKF dev profiling — `src/safety/fault_protection.*`, `include/rocketchip/shared_state.h`, `src/shared_state.cpp`, `src/main.cpp` GPS bind helpers, `src/fusion/eskf_runner.*`, `src/cli/rc_os_commands.cpp` (bench output), `docs/benchmarks/UD_BENCHMARK_RESULTS.md`.  
**Codebase:** `main` at audit time; host suite `build_host` (`-DBUILD_TESTS=ON`).

---

## Executive summary

| Check | Result |
|-------|--------|
| **Host `ctest`** | **786/786 PASS** (`ctest --test-dir build_host`) |
| **Target firmware** | **`rocketchip` builds** (`cmake --build build`) — bench/dev tier |
| **Fault/MPU consolidation** | **PASS** — single module `fault_protection`; `__StackBottom` declared in `main.cpp`; no duplicate `mpu_setup_stack_guard` bodies |
| **Global state** | **PASS** — declarations in `shared_state.h`, definitions in `shared_state.cpp`, CMake lists updated |
| **JSF / flight-support classification** | **PASS** — fault path remains assertion-only / no dynamic allocation in fault handlers (unchanged from prior review) |
| **Hardware gates (full list)** | **Recorded** in `docs/baselines/stage_o_hw_verification_2026-04-28.md` — vehicle `bench_sim` 2/2; **5 min soak** optional for small Stage O refactors; **5 min** + SPIN + MPU for major/high-risk follow-ups (see runbook) |

---

## Notes

- A **full** Stan­dards sweep (clang-tidy every `src` file) is **not** repeated here; Stage O used focused verification + existing pre-commit scope. For milestone closure, use `.claude/SESSION_CHECKLIST.md` item 18 if required.
- **`fault_inject`:** There is **no** dedicated host test named `fault_inject` — the plan refers to **dev** `fault_inject.cpp` and **GDB** scripts under `scripts/fault_injection/`. That remains the intended workflow.

**Sign-off:** Automated checks above PASS; on-target items listed in the baseline runbook with dates/status.
