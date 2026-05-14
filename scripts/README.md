# RocketChip Test & Audit Scripts

This directory contains automation for testing, validation, and standards auditing.

## Required Tools

These tools must be available in your PATH (Git Bash / MSYS2 recommended):

| Tool          | Purpose                              | Install (Windows)                          | Used By                     |
|---------------|--------------------------------------|--------------------------------------------|-----------------------------|
| clang-tidy    | Static analysis (JSF AV / JPL / P10) | LLVM installer or `choco install llvm`     | `run_clang_tidy.sh`         |
| lizard        | Cyclomatic complexity                | `pip install lizard`                       | `run_clang_tidy.sh`         |
| cppcheck      | Complementary bug-finding static analysis | `choco install cppcheck` or download from https://cppcheck.sourceforge.io | `run_cppcheck.sh` |
| Git Bash      | Shell environment for all scripts    | Git for Windows (includes bash)            | All `.sh` scripts           |

**Notes:**
- All audit scripts are designed to run from **Git Bash**, not raw PowerShell or CMD.
- For milestone audits, run from a Git Bash session with the above tools in PATH.
- See `AUDIT_GUIDANCE.md` for the master audit procedure and when each script should be executed.

## Scripts Overview

### Regular / Periodic Use
- `soak_test.py` — General system stability soak
- `i2c_soak_test.py` — I2C/sensor error monitoring
- `bench_sim.py`, `station_bench_sim.py` — Flight Director + CLI end-to-end test
- `replay_harness_host.py` — Host-side ESKF replay (placeholder; R-25-exec
  amendment #4 retired the on-MCU CSV-streamer replay path 2026-05-13)
- `cli_test.py` — Non-destructive CLI tests

### Audit Scripts (Category 1 & 2)
- `run_clang_tidy.sh` — Tiered standards audit (clang-tidy + lizard + platform guards)
- `run_cppcheck.sh` — Complementary static analysis (bugprone patterns)
- `analyze_stack_usage.sh` — Stack usage and SRAM budget checker

See `standards/AUDIT_GUIDANCE.md` for full classification and execution order.
