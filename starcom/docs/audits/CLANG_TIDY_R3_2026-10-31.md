# R3 final recount, 2026-10-31

This is the final count for the Starcom code fixes (`STARCOM_AUDIT_2026-10-29.md`, Section 2, row R3). It follows the R2 method (`CLANG_TIDY_R2_2026-10-31.md`). Each row was counted again from the tree. No R2 number was copied.

- Base: `c6887533` (F3). `origin/main` is the same commit. R3 adds only comment edits and `scripts/ci/comment_coverage.py`.
- Tools: clang-tidy 21.1.8 with `starcom/.clang-tidy`; MinGW g++ 15.2.0; lizard 1.21.2.
- Inputs: the 20 translation units in `rc-audit-logs\scbuild` (after F3; `udp_win32.cpp` only). `udp_posix.cpp` is not in that database. It was built and tested in WSL.
- Logs (PC, `rc-audit-logs`): `CLANG_TIDY_R3_v21.1.8.txt`, `CLANG_TIDY_R3_adapt_v21.1.8.txt`, `VGCC_R3.txt`, `LIZARD_R3.txt`, `COMMENT_COVERAGE_R3.txt`, `OBJ_HASH_R3_before.txt`. The `_after` logs are from after the R3 edits.

## clang-tidy: 10 warnings (R2: 11)

| Count | Check | Sites |
|---|---|---|
| 4 | `cppcoreguidelines-pro-type-reinterpret-cast` | `file_replay.cpp:36`; `udp_win32.cpp:21`, `:51`, `:57` |
| 2 | `cppcoreguidelines-avoid-non-const-global-variables` | `copp.cpp:429` `g_tfScratch`; `cop1.cpp:21` `g_cop1TfScratch` |
| 1 | `readability-avoid-nested-conditional-operator` | `pltu.cpp:108` |
| 1 | `readability-container-size-empty` | `space_packet.cpp:52` |
| 1 | `readability-named-parameter` | `space_packet_service.cpp:66` |
| 1 | `readability-identifier-naming` | `copp.cpp:443` `dest_is_local` |

`g_wsa` is gone (F3). The count is the same before and after the R3 edits. The adapter set is 328 (same as R2).

## V-gcc: 20 units, 0 fails, every flag 0

`-Wshadow`, `-Wconversion`, `-Wsign-conversion`, `-Wold-style-cast`, `-Wuseless-cast` (R2: 9), `-Wcast-qual`, `-Wfloat-equal`: all 0, before and after the edits.

## lizard

354 functions. Longest: 60 lines (`macOnNoFramesPending`, `macOnFifoEmpty`). 0 functions over 60. Highest CCN: 20 (`macOnNoFramesPending`, `pump_air_to_send`). Most parameters: 6.

## Comment density (record only, no target)

Core `.cpp` 213/4708 = 4.5%. Host and RP2350 adapters 39/531 = 7.3%. `src/starcom_adapt` 42/693 = 6.1%. Same method as R2.

## JSF 130 check

Method: `scripts/ci/comment_coverage.py` lists candidates. A full read of each covering comment then decides each row.

- Before the edits: 309 functions read, 165 with no comment above or inside, 16 junk candidates.
- 155 of the 165 pass, because the name says the purpose (codec, init, accessor and book-event names; `farmPw` and `farmNw` are covered by `cop1.h:40`; `readBook16` is the inverse of `writeBook16`).
- 10 definitions (8 names) failed. Each got one purpose comment: `os::acquire` and `os::release` (one comment in `udp_os.h` covers both OS files), `asmPrefixKeep`, `repeatSeq`, `coppHoldSubmit`, `tickDec`, `writeBook16`, `delimitedSize`.
- The 16 junk candidates are book event tags (E7, E9, E10, E11, E37-E39, E41, E43-E46, RE2, RE7, SE7, 7.2.3.2). All pass. After the edits there are 17: the new `uslp.cpp:21` cite line looks like code to the script. It passes.
- Limit: a comment inside a function counts as coverage but skips the junk checks, so a lone tag such as `// E36` can hide a gap.
- Limit: claim comments are checked only with `--since`, and only when the comment names a changed function. File-level "No state." lines never appear.
- Claim comments were read by hand in full (all files in scope), all TRUE: `cop1.cpp:19` only caller; `copp.cpp:427-428` only user; `mac.cpp:292` plcw_left 0; `uslp.cpp:21` pointer always used; `loopback.h:35` core never sees it; `cop1.h:102` T1 0 never expires; `copp.h:43` synch 0 never expires; `crc.h:15` CRC never covers the ASM; `mac.h:23` 0 = inactive; `udp_win32.cpp:2` no state; the 41 file-intro "No state." lines.
- The full read also found wrong comments. 20 cite lines were fixed (Duke's cite check `r3_cite_check_2026-11-01.md`, each clause confirmed in the book text; and the stale 131.0-B-5 cites). One repeat comment was cut (`ldpc.cpp:46`).
- After the edits: 156 with no comment, and each passes by its name.

## Section 1 FAIL rows

| Row | R2 | R3 evidence | R3 |
|---|---|---|---|
| Tier 1.1 config, positive control | PASS | Config unchanged since R2 | PASS |
| JSF 3 | PASS | Max CCN 20 | PASS |
| JSF 6 | STILL FAIL | Every Starcom FAIL closes. The F4 rows stay | FAIL until F4 (Rocket-Chip pass) |
| JSF 9, 13 | STILL FAIL (adapt) | Starcom 0. `src/starcom_adapt` 12 comment lines (`byte_pump.cpp` 27, 53, 122, 123, 139, 173, 471; `byte_pump.h` 5, 50, 56, 57, 104) | FAIL in `src/starcom_adapt` (F4) |
| JSF 14 | PASS | 0 suffix warnings | PASS |
| JSF 17 | Listed site | `errno` at `udp_posix.cpp:150`, `:168`. Listed in the F3 commit | Listed OS-edge site |
| JSF 22 | PASS | 0 (`fopen` hit is a CMake comment) | PASS |
| JSF 26, 28, 31 | STILL FAIL | 0 non-guard `#if`, `#ifndef`, `#define` | PASS |
| JSF 27, 35 | PASS | `#pragma once` 0. 25 product headers and `heap_trap.h`: each has a unique guard. `probe_hit.h` has none (known). R2 said 28 headers; R3 could not match that number | PASS for product headers |
| JSF 29 | PASS | Product function-like macros 0; test `CHECK` macros 18 | PASS for product sources |
| JSF 53 | PASS | 0 `.hpp` includes of Starcom headers | PASS |
| JSF 60, 61 | PASS | 0 K&R braces in Starcom | PASS for Starcom |
| JSF 108 | Listed site | `fcntl` at `udp_posix.cpp:46`, `:51`. Listed in the F3 commit | Listed OS-edge site |
| JSF 110 | PASS | Max 6 parameters | PASS |
| JSF 130 | STILL FAIL (density) | Graded by purpose (above). 10 gaps and 20 wrong cites fixed; 1 repeat cut | PASS (regraded) |
| JSF 133 | PASS | Unchanged | PASS |
| JSF 151 | STILL FAIL (adapt) | Core 0. `nav_sdu.h:20`, `:22` | FAIL in `src/starcom_adapt` (F4) |
| JSF 162 | PASS | `-Wsign-conversion` 0 | PASS |
| JSF 176, P10-9, LOC-29 | PASS | 0 function pointers | PASS |
| JSF 181 | STILL FAIL | `-Wuseless-cast` 0 | PASS |
| JSF 182 | Listed site | 4 MinGW hits above, plus `udp_posix.cpp:25` (POSIX only). Listed in the F3 commit | Listed OS-edge site |
| JSF 190 | PASS | Product 0. Tests: `test_version.cpp:52`, `:57` | PASS for product sources |
| JSF 191 | PASS | Product 0. Tests: `test_copp.cpp:413`, `:481`; `test_host_io.cpp:343` | PASS for product sources |
| JSF 200, P10-2, LOC-3 | PASS | 0 `for (;;)` | PASS |
| JSF 207 | STILL FAIL | Only the two owned scratch buffers (no finding) | PASS |
| JSF 209, LOC-17 | STILL FAIL | Starcom: only the OS-API `int` sites in the F3 commit list. Adapter: `byte_pump.cpp` `unsigned` 43, 168, 329, 343, 348; `cmd_sdu.h:26` `float` | PASS for Starcom; FAIL in `src/starcom_adapt` (F4) |
| JSF 215 | STILL FAIL | Starcom `.data() +` 0. `cmd_sdu.cpp` 18-22, 33-37 | FAIL in `src/starcom_adapt` (F4) |
| P10-4, LOC-25 | STILL FAIL | Max length 60, max 6 parameters | PASS |
| LOC-8 | PASS | Unchanged | PASS |
| D.8 | PASS (regraded) | Record only | PASS |

`(void)` on unused parameters (record): `cop1.cpp:236`; `mac.cpp` 775, 810, 858, 919, 980, 1059, 1088, 1109, 1135, 1327, 1345. Product `(void)` call discards: 0.

## Tests after the R3 edits

- Starcom MinGW: 3/3. Starcom WSL (builds `udp_posix.cpp`): 3/3 run, `tidy_selftest` skipped (no clang-tidy in WSL). Rocket-Chip host: 1044/1044.
- RP2350 objects (`build_flight`, `build_station_flight`): every object hash is the same before and after. The stripped-ELF hash changes. It also changed in the earlier comment-only steps (W1, C16) after a relink.

Steps done: 39 of 39. The F4 rows stay FAIL. The Rocket-Chip pass owns them.
