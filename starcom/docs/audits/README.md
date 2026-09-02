# Starcom audits

Starcom-owned. Extract-ready with the library. Not Rocket-Chip `docs/audits/` and not a firmware gate.

This folder is the house-standards remainder after IVP 23's initial camelBack pass. Blue Book claims live in `../CONFORMANCE.md`, not here.

| File | What |
|------|------|
| [`IVP23_REPORT.md`](IVP23_REPORT.md) | Living Grey / JSF 151 map + NASA identifier resolution. |
| [`IVP23_TIDY_RUN.md`](IVP23_TIDY_RUN.md) | First host clang-tidy run (core only), 2026-08-31, on the pre-camelBack fork. |
| `CLANG_TIDY_2026-08-31_core.txt` | Raw tidy log from that run. |

Tools: `starcom/.clang-tidy`, `starcom/scripts/run_clang_tidy.ps1` (and `.sh`). Must pass `--config-file starcom/.clang-tidy`. Do not call RC `scripts/audit/full_tree_clang_tidy.sh`.
