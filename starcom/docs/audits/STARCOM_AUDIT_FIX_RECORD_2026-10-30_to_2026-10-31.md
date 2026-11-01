# Starcom audit fix record, 2026-10-30 to 2026-10-31

This file records what each step of the Starcom code-fix plan did. The findings, the triage, and the dispositions are in [`STARCOM_AUDIT_2026-10-29.md`](STARCOM_AUDIT_2026-10-29.md) (Section 1 and Section 2). This file does not repeat them. The facts come from `git log`, the commit messages, the audit file, and the recount records [`CLANG_TIDY_R1_2026-10-30.md`](CLANG_TIDY_R1_2026-10-30.md), [`CLANG_TIDY_R2_2026-10-31.md`](CLANG_TIDY_R2_2026-10-31.md) and [`CLANG_TIDY_R3_2026-10-31.md`](CLANG_TIDY_R3_2026-10-31.md). "Not recorded" means that no such source gives the fact.

## Summary

- Dates: audit 2026-10-29 (`cc1a2368`). Plan 2026-10-30. Last step (R3) 2026-10-31 (`3b110368`).
- Section 1 FAIL rows: 42 at the audit (31 JSF, 3 Power of 10, 5 JPL LOC, D.8, and the 2 Tier 1 gate rows; the R2 table lists the same 42 rows; the R3 table merges the 2 Tier 1 rows). 16 at R2 (`1660f2af`). 7 at R3: JSF 6, 9, 13, 151, 209, 215 and LOC-17. Each of the 7 fails only in `src/starcom_adapt/` (JSF 6 fails because the others fail). Only D.8 closed because of a regrade alone. JSF 130 was regraded by a full read, and the 7 purpose comments, 1 repeat cut and 20 cite fixes from that read are in the same commit (R3).
- clang-tidy 21.1.8, Starcom core: 630 warnings at R1, 11 at R2, 10 at R3.
- Plan steps: 39 of 39 done. F4 moved to the Rocket-Chip pass. F5 dropped. D3 removed from the plan before work started (`fb3ffa84`).
- Open:
  - the 7 F4 rows (the Rocket-Chip pass on the main `AGENT_WHITEBOARD.md` row)
  - the "Noted for later" list in the audit file, Section 2
  - the JSF 130 rule text (the reading is agreed in the audit file; the rule wording is not changed)
  - the comment density rule rework (`AGENT_WHITEBOARD.md` row "Re-evaluate the comment density rule").

## Pushes to `origin/main`

From `git reflog origin/main`. Times are CDT (UTC-5).

| Push time | Commits |
|---|---|
| 2026-10-30 11:02 | `cc1a2368` `20f5da9c` `fb3ffa84` `ea4747e7` `6a616148` `6a30a5e1` `f53e391c` `2ab9344a` |
| 2026-10-30 18:00 | `f9242292` `9376633b` `083bf1a2` `8da54292` `8d5191af` `0194cfe2` |
| 2026-10-31 01:35 | `1a7528a9`, `f81072c7`, `6e2e541e`, `604b499a`, `b83bef89`, `bfc73306`, `ad775c59`, `0e26ee66` (one ref update for each commit, 01:35:06 to 01:35:35) |
| 2026-10-31 11:53 | `764345a0`, `32e93b4d`, `e835d47a`, `fb047778` (one ref update for each commit) |
| 2026-10-31 12:37 | `07cebc5c` |
| 2026-10-31 14:19 | `ccbb05b7` |
| 2026-10-31 14:21 | `0f435c4e` |
| 2026-10-31 15:43 | `47076843` |
| 2026-10-31 16:00 | `8309a435` `e145d4c8` |
| 2026-10-31 17:55 | `b0d597c9` |
| 2026-10-31 18:17 | `d2004f57` `cc546b21` `e20bf54c` `1fd0bd16` |
| 2026-10-31 19:13 | `0516d3bf` |
| 2026-10-31 21:34 | `1660f2af` `8b24f407` |
| 2026-10-31 21:38 | `34502a32` `5abd32b9` |
| night of 10/31 (00:41 on 11/01) | `c6887533` |
| Not pushed | `3b110368` (R3). The gate passed the night of 10/31 (01:14 on 11/01; see R3). |

## Steps

- Verified: what the commit message gives (host tests, tool counts, RP2350 objects).
- Bench: Buzz's gate note in the local `refs/notes/rc-bench` (index `rc-audit-logs\bench_notes_index_2026-10-30_to_11-01.txt`). Each of the 25 notes names a commit that is on `main`, so no commit that had a bench run was changed later. One note covers a range of commits. The format is vehicle / station result, firmware tree, range.
- Radio: the 60 s, 2 dBm two-board run, from its `_score.txt` in `Rocket-Chip\logs\soak\`. The values are vehicle frames sent / station frames heard in the delta window, new station CRC errors in that window, and station RSSI. Each score file gives layout "A" (station on the metal PC, about 1.5 m, antennas up).

| Step | Commit(s) | What changed | PR items and rules | Verified | Bench | Radio |
|---|---|---|---|---|---|---|
| Plan | `cc1a2368`, `20f5da9c`, `fb3ffa84` | Audit report; batch plan; Nathan's decisions (C9 templates, W2 now, D3 removed) | - | Docs only; host ctest 1037/1037 | - | - |
| G1 | `ea4747e7` | 71 comment lines moved out of the `Checks:` block in `starcom/.clang-tidy` | PR-1 | Enabled checks from 156 to 169; probe under old and new config; host ctest 1037/1037 | Not recorded | - |
| G2 | `f53e391c` | Root `.clang-tidy`: 74 comment lines moved out of `Checks:` | PR-2 (config part) | Enabled checks from 153 to 163; `clang-analyzer-core` fires. RC full-tree count not done (RC master audit) | Not recorded | - |
| G3 | `f53e391c` | `HeaderFilterRegex` accepts `/` and `\`; new `ExcludeHeaderFilterRegex` | PR-1b | `copp.cpp`: 6 header warnings show, 0 from third-party code | Not recorded | - |
| G4 | `f53e391c`, `2ab9344a` | Known-bad probe and `tidy_selftest.sh`; CTest `starcom.tidy_selftest`; root-config guard | PR-1 (Tier 1.1) | Self-test 18/18 PASS; old config 17 of 18 FAIL | Not recorded | - |
| T1 | `f53e391c` | `run_clang_tidy.sh` LF only, no BOM; `.gitattributes` keeps it LF | PR-6 | `--all` exits 0 (19 files) | Not recorded | - |
| R1 | `2ab9344a` | Baseline count with the fixed config | Baseline | 630 warnings, 19 files; Goddard's second count is the same | - | - |
| A0 | `6a616148` | Four soak script faults fixed; new `test_soak_two_board.py` | - | New test OK; host ctest 1037/1037 | - | - |
| A1 | `6a30a5e1` | Soak score file gives true counts; baseline run recorded | - | Counts in the audit file ("A1 baseline") | - | Firmware `flight-18e5e6d8` (score file `..._A1_baseline_2dbm_579f0976`): end CRC count 1, RSSI -63 dBm; this file has no delta values |
| C1 | `f9242292` | 12 redundant casts removed | PR-33; JSF 181 | `-Wuseless-cast` from 12 to 0; disassembly identical (MinGW, arm-none-eabi); Starcom tests 2/2 | In the `8da54292` range (below) | - |
| C2 | `9376633b` | 4 signed/unsigned conversions made explicit | PR-34; JSF 162 | `-Wsign-conversion` from 4 to 0; machine code identical; Starcom tests pass | In the `8da54292` range | - |
| C3 | `083bf1a2`, `8da54292` | Path tests, then `macWaitExpiredHalf` split into one helper per state | PR-13; P10-4, JSF 3, LOC-25 | gcov all branches; clang-tidy size and complexity warnings gone; `.text` 16 bytes larger | `8da54292`: 2/2, 3/3, tree `c67aa7d5`, range `f9242292..8da54292` (4) | Split runs (4) and A/B pairs (3) compared with `18e5e6d8`. Pairs, split and base: 408/402 and 413/416; 414/410 and 409/409; 409/410 and 410/403. New CRC errors: 0 and 0, 0 and 0, 2 and 2. RSSI -69 to -70. Audit PR-13: pass |
| T0 | `8d5191af` | Host test for COP-P resend and recovery | Test before C4 | Starcom tests 2/2; fails with the resend blocked | In the `0194cfe2` range | - |
| C4 | `0194cfe2` | COP-P resend and sync-loss counts moved into `CoppEndpoint` | PR-12b; JSF 207, LOC-8, P10-6 | Starcom tests 2/2; 7 RC link-baseline tests; non-const globals on `copp.cpp` from 3 to 1 | 2/2, 3/3, tree `2cd03c22`, range `f9242292..0194cfe2` (6) | 406/411, CRC 0, RSSI -69 |
| C5, C6 | `1a7528a9` | Host adapter cleanup (`ifstream`, `span`, OS-edge helpers); file-local helpers; merged MAC case | PR-14, PR-15, PR-16, PR-20; JSF 17, 22, 26, 28, 31, 108, 182, 200, 215; P10-2; LOC-3 | RP2350 objects identical (5 files); Starcom tests MinGW and WSL; host ctest 1038/1038 | 2/2, 3/3, tree `8867f8f4` (1) | Not recorded |
| C7 | `f81072c7` | Command packet fields in one struct (`CmdSduFields`) | PR-28; JSF 110, LOC-25 | Byte-pin tests pass before and after; host ctest 1040/1040 | 2/2, 3/3, tree `611fbd35`, range `1a7528a9..f81072c7` (2) | 411/408, CRC 0, RSSI -75 |
| C8 | `6e2e541e` | 10 `continue` and 1 loop `break` removed | PR-25; JSF 190, 191 | Tests first; host ctest 1041/1041; both roles build | 2/2, 3/3, tree `640590b1`, range from `1a7528a9` (3) | 408/411, CRC 0, RSSI -75 |
| C9 | `604b499a` | Adapter seams are template policies, not function pointers | PR-23; P10-9, JSF 176, LOC-28/29 | Host ctest 1041/1041; `.elf` section sizes unchanged | 2/2, 3/3, tree `63b37e64`, range from `1a7528a9` (4) | Run 1: 413/408, CRC 3, RSSI -72. Rerun: 412/410, CRC 1, RSSI -70 |
| C10 | `b83bef89` | 151 plain `int`/`unsigned` changed to fixed-width types | PR-27; JSF 209, LOC-17 | RP2350 machine code identical (11 files); host ctest 1041/1041 | 2/2, 3/3, tree `a44eac20`, range from `1a7528a9` (5) | 411/411, CRC 0, RSSI -74 |
| C11 | `bfc73306` | 124 `Result` functions `[[nodiscard]]`; 7 `(void)` discards checked | PR-18; P10-7 | 0 `-Wunused-result`; RP2350 objects identical (16 files); host ctest 1041/1041 | 2/2, 3/3, tree `1c9d852e`, range from `1a7528a9` (6) | 409/403, CRC 0, RSSI -74; 8 station `D` uplinks sent |
| C12 | `ad775c59` | 319 literal suffixes to upper case | PR-19 part 1; JSF 14 | Suffix warnings from 304 to 0; RP2350 objects identical (19 files); host ctest 1041/1041 | 2/2, 3/3, tree `bf567afb`, range from `1a7528a9` (7) | Not recorded |
| C13 | `0e26ee66` | Named constants for magic numbers, each with its source | PR-19 part 2; JSF 151 | Magic-number warnings from 117 to 0; RP2350 objects identical (19 files); host ctest 1041/1041 | 2/2, 3/3, tree `d5c1c5ca`, range from `1a7528a9` (8) | Not recorded |
| C14 | `764345a0` | File intro comments | PR-26; JSF 133 (JSF 130, D.8) | RP2350 code and data identical (debug info only). Host tests: not recorded | 2/2, 3/3, tree `f96ddfde` (1) | Not recorded |
| C15 | `32e93b4d` | Non-ASCII comment characters changed to ASCII (36 files) | PR-29; JSF 9, 13 | Scan finds 0 non-ASCII. Host tests: not recorded | 2/2, 3/3, tree `e4724115` (1) | Not recorded |
| C16 | `e835d47a` | Function-like macros and build-flag `#ifdef`s removed | PR-30; JSF 26, 29; P10-8 | 19 objects identical in each role; host ctest 1042/1042 | 2/2, 3/3, tree `c88424fc` (1) | Run 1 (vehicle antenna blocked by the PCB tub): 393/382, CRC 5, RSSI -82. Rerun (tub removed): 409/407, CRC 0, RSSI -57 |
| C17 | `fb047778` | Unsigned bitwise operands; missing `const` | PR-31 | Core clang-tidy count from 143 to 11; only `mac.cpp.obj` changed, same output for each input; host ctest 1042/1042. Added 9 useless casts (fixed in F1) | 2/2, 3/3, tree `3746848e` (1) | 412/412, CRC 0, RSSI -56 |
| U1 | `07cebc5c` | Station dashboard gets its keys again (`D`, `a`, `x`) | PR-41 (verification blocker) | Host ctest 1043/1043; both images build | 2/2, 3/3, tree `e2b71620` (1). Board check of the keys: not recorded | See C18 |
| C18 | `ccbb05b7` | Final `else` on each listed if/else-if chain | PR-32 part 1; JSF 192 | 40/40 stripped hashes identical; host ctest 1043/1043; clang-tidy core 11 | 2/2, 3/3, tree `105da946` (1) | Sent 411; heard and CRC delta: none in the score file; RSSI -57; 3 station `D` uplinks sent. The file does not show that the vehicle got them |
| W1 | `0f435c4e` | Include guards in 25 headers, no `#pragma once` | PR-24; JSF 27, 35 | 38 objects identical; host ctest 1043/1043 | 2/2, 3/3, tree `8286bb7e` (1) | Not recorded |
| W2 | `47076843`, `8309a435`, `e145d4c8` | Block braces on their own lines (1285); COP-1 path tests; `fop1Tick` and `cop1ReceiveBytes` split | PR-36; JSF 60, 61; P10-4 | Token stream same; 38 objects identical (brace commit); clang-tidy core count from 13 to 11 after the split; host ctest 1043/1043. Made 3 functions 61 lines (fixed in F2) | `47076843`: 2/2, 3/3, tree `b3547f5d`. `e145d4c8`: 2/2, 3/3, tree `0afe9588`, range from `8309a435` (2) | Not recorded |
| W3 | `b0d597c9`, `0516d3bf` | Header names changed from `.hpp` to `.h`; living docs use the new names | PR-32 part 2; JSF 53 | 38 objects match; tidy_selftest 21/21; host ctest 1043/1043 | `b0d597c9`: 2/2, 3/3, tree `3b1271e9`. `0516d3bf`: not recorded | Not recorded |
| D1, D4 | `cc546b21` | Doc claims match the tools and code; DESIGN.md: no virtual dispatch | PR-7, PR-8, PR-11, PR-22, PR-23 | Docs only; Goddard checked each line; tidy_selftest 21/21 | 2/2, 3/3, tree `d73cded7`, range `d2004f57..cc546b21` (2) | - |
| D2 | `d2004f57`, `e20bf54c`, `1fd0bd16` | Test that core code has no Pico or hardware includes; run in RC host tests; fail on empty scan | PR-9 | Planted includes fail; Starcom tests 3/3; host ctest 1044/1044 | `d2004f57` in the `cc546b21` range. `e20bf54c`, `1fd0bd16`: not recorded | - |
| R2 | `1660f2af` | Recount; fix steps F1-F5 and R3 added; V-all rule | PR-35 | Core clang-tidy count from 630 to 11; 16 of 42 rows still FAIL; Goddard recount agrees | In the `8b24f407` range | - |
| F1 | `8b24f407` | 9 useless casts removed in `mac.cpp` | JSF 181 | `-Wuseless-cast` from 9 to 0; `mac.cpp.obj` hash unchanged, both roles | 2/2, 3/3, tree `c6ac759e`, range `1660f2af..8b24f407` (2) | Not recorded |
| F2 | `34502a32`, `5abd32b9` | Branch tests, then the three 61-line functions split | P10-4, LOC-25 | lizard 0 over 60; GCC flags 0; clang-tidy 11 / 328; Starcom 3/3; host 1044/1044 | 2/2, 3/3, tree `30b79a2a`, range `34502a32..5abd32b9` (2) | Not recorded |
| F3 | `c6887533` | Host UDP: one file for each OS, no `#if`, no `g_wsa` | JSF 26, 28, 31, 207, 209; LOC-17 | Starcom 3/3 MinGW and WSL; host 1044/1044; RP2350 hashes unchanged | 2/2, 3/3, tree `03fa9445` (1). Run 1 (00:33, `GATE_F3_c6887533.txt`) stopped because the flash was not recorded. After the flash, the vehicle IMU came up NO-GO. Nathan asked for a restart. Buzz restarted both boards with `picotool reboot -f`, and that cleared it (Buzz; the logs do not show this). Run 2 (00:41, `GATE_F3_c6887533_run2.txt`) passed, with sensors GO. Pushed the night of 10/31 (00:41 on 11/01) | Not recorded |
| F4 | none | Moved to the Rocket-Chip pass (`src/starcom_adapt/`) | JSF 9, 13, 151, 209, 215; LOC-17 | Rows stay FAIL | - | - |
| F5 | none | Dropped (Nathan, 2026-10-31): the density band is an upper cap. Recorded in `c6887533` | JSF 130, D.8 | D.8 regraded PASS | - | - |
| R3 | `3b110368` | Final recount; JSF 130 by full read; 10 definitions fixed with 7 purpose comments; 1 repeat cut; 20 cite fixes; `scripts/ci/comment_coverage.py` | PR-35; JSF 130 | clang-tidy 10; V-gcc 0; lizard max 60; Starcom 3/3 MinGW and WSL; host 1044/1044; RP2350 object hashes unchanged | 2/2, 3/3, tree `a39029e6` (1). Run 1 (01:06, `GATE_R3_run.txt`) stopped partway, after vehicle 2/2. Run 2 (01:08, `GATE_R3_run2.txt`): vehicle 0/2 on a sensor health timeout; the push was blocked. Nathan replugged the boards after run 2. Run 3 (01:14, `GATE_R3_run3.txt`) passed on the recorded flash, with no new build or flash (Buzz). Not a code fault. Not pushed | Not recorded |
