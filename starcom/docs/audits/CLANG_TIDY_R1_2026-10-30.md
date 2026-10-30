# clang-tidy R1 baseline, 2026-10-30

This is the baseline count for the Starcom code fixes (`STARCOM_AUDIT_2026-10-29.md`, Section 2, row R1). Each code batch compares its count with this one.

- Config: `starcom/.clang-tidy` at commit `f53e391c` (comments out of `Checks:`, new header filter and exclude list).
- Tool: clang-tidy 21.1.8 (`C:\Program Files\LLVM\bin\clang-tidy.exe`).
- Inputs: the 19 translation units in the MinGW compile database at `rc-audit-logs\scbuild`. The loop is the V-tidy command in Section 2.
- Count: `tidycount.py`. A warning is counted one time for each file, line, column and message. A header warning that many files show is counted one time.
- Raw log: `CLANG_TIDY_R1_2026-10-30.txt` (this folder).

Known noise: each file gives one `clang-diagnostic-unknown-warning-option` error for the GCC-only `-Wstack-usage=1024` flag (19 in total). The count does not include these errors.

## Total: 630 warnings

| Count | Check |
|---|---|
| 304 | `hicpp-uppercase-literal-suffix`, `readability-uppercase-literal-suffix` (one warning, two names) |
| 179 | `hicpp-signed-bitwise` |
| 117 | `readability-magic-numbers` |
| 9 | `misc-const-correctness` |
| 5 | `cppcoreguidelines-avoid-non-const-global-variables` |
| 5 | `cppcoreguidelines-pro-type-reinterpret-cast` |
| 2 | `misc-use-internal-linkage` |
| 2 | `cert-err33-c` |
| 1 each | `readability-identifier-naming`, `readability-function-cognitive-complexity`, `readability-function-size`, `bugprone-branch-clone`, `readability-avoid-nested-conditional-operator`, `readability-container-size-empty`, `readability-named-parameter` |

No `clang-analyzer-core` warning shows in Starcom.

Header warnings (shown now because of the new header filter): `copp.hpp` 4, `cop1.hpp` 3, `uslp.hpp` 1, `v3.hpp` 1.

Compared with the audit (Section 1, pinned 21.1.8 run): suffixes 297 then 304, magic numbers 115 then 117. The header warnings, which the old filter did not report, are the cause: 7 suffix and 2 magic-number warnings are in `copp.hpp`, `cop1.hpp`, `uslp.hpp` and `v3.hpp`. The raw log has 640 warning lines, because 10 header warnings repeat in each file that includes the header. Signed-bitwise (179) and const-correctness (9) are the same.

Second count: Goddard, 630, the same for each check (2026-10-30).

For each code batch, count again with this same loop (with the 19 known errors), so the numbers compare.
