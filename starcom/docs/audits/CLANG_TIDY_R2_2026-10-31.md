# clang-tidy R2 recount, 2026-10-31

This is the final count for the Starcom code fixes (`STARCOM_AUDIT_2026-10-29.md`, Section 2, row R2). R2 adds no code change. It counts HEAD again and compares the count with the R1 baseline (`CLANG_TIDY_R1_2026-10-30.md`).

- HEAD: `0516d3bf`.
- Config: `starcom/.clang-tidy` at that HEAD. The last commit that changed the file is `cc546b21`.
- Tool: clang-tidy 21.1.8 (`C:\Program Files\LLVM\bin\clang-tidy.exe`).
- Inputs: the 19 translation units in the MinGW compile database at `rc-audit-logs\scbuild`. The database file time is 2026-10-31 17:58:40. The header rename commit `b0d597c9` is 17:51:02. The generated header in that build is `version.h`. The database has no `.hpp` path. This recount did not reconfigure the build.
- Loop: `powershell -NoProfile -ExecutionPolicy Bypass -File C:\Users\pow-w\Documents\rc-audit-logs\tidy_run.ps1 -Tag R2`.
- Count: `tidycount.py`. A warning is counted one time for each file, line, column, and message. A header warning that many files show is counted one time.
- Raw log: `CLANG_TIDY_R2_2026-10-31.txt` (this folder). The same bytes are `rc-audit-logs\CLANG_TIDY_R2_v21.1.8.txt`.

Known noise: each of the 19 files gives one `clang-diagnostic-unknown-warning-option` error for the GCC-only `-Wstack-usage=1024` flag. The log also prints `Error while processing` for each file because of that error. The check warnings are still in the log. The count does not include these 19 errors. The R1 log has the same error on each file.

## Total: 11 warnings

| Count | Check |
|---|---|
| 4 | `cppcoreguidelines-pro-type-reinterpret-cast` |
| 3 | `cppcoreguidelines-avoid-non-const-global-variables` |
| 1 | `readability-avoid-nested-conditional-operator` |
| 1 | `readability-container-size-empty` |
| 1 | `readability-named-parameter` |
| 1 | `readability-identifier-naming` |

No `clang-analyzer-core` warning shows in the 19 translation units.

The 11 warnings are all in `.cpp` files. The core run has 0 header warnings. R1 had header warnings in `copp.hpp` (4), `cop1.hpp` (3), `uslp.hpp` (1), and `v3.hpp` (1).

| File | Line | Check |
|---|---|---|
| `adapters/host/udp.cpp` | 32 | `g_wsa` non-const global |
| `adapters/host/udp.cpp` | 92, 110, 115 | `reinterpret_cast` |
| `adapters/host/file_replay.cpp` | 36 | `reinterpret_cast` |
| `src/ccsds/copp.cpp` | 429 | `g_tfScratch` non-const global |
| `src/ccsds/copp.cpp` | 443 | `dest_is_local` name |
| `src/ccsds/cop1.cpp` | 21 | `g_cop1TfScratch` non-const global |
| `src/ccsds/pltu.cpp` | 108 | nested conditional |
| `src/ccsds/space_packet.cpp` | 52 | `size` used where `empty` is the check |
| `src/ccsds/space_packet_service.cpp` | 66 | unnamed parameters |

Second count: `tidycount.py` on `CLANG_TIDY_R2_2026-10-31.txt` again gives total 11. The per-check counts are the same.

## R1 to R2, core run

| Check | R1 | R2 |
|---|---:|---:|
| `hicpp-uppercase-literal-suffix`, `readability-uppercase-literal-suffix` | 304 | 0 |
| `hicpp-signed-bitwise` | 179 | 0 |
| `readability-magic-numbers` | 117 | 0 |
| `misc-const-correctness` | 9 | 0 |
| `cppcoreguidelines-avoid-non-const-global-variables` | 5 | 3 |
| `cppcoreguidelines-pro-type-reinterpret-cast` | 5 | 4 |
| `misc-use-internal-linkage` | 2 | 0 |
| `cert-err33-c` | 2 | 0 |
| `readability-identifier-naming` | 1 | 1 |
| `readability-function-cognitive-complexity` | 1 | 0 |
| `readability-function-size` | 1 | 0 |
| `bugprone-branch-clone` | 1 | 0 |
| `readability-avoid-nested-conditional-operator` | 1 | 1 |
| `readability-container-size-empty` | 1 | 1 |
| `readability-named-parameter` | 1 | 1 |
| Total | 630 | 11 |

No core check count went up. No new check name appears in the core run.

Four checks stayed at 1. `dest_is_local` stays by commit `1a7528a9`. That commit says the name meets JSF rule 51. The nested conditional, the `size` check, and the unnamed parameters are the same three R1 hits. Their lines moved.

## Rocket-Chip adapter set

The same script writes `rc-audit-logs\CLANG_TIDY_R2_adapt_v21.1.8.txt`. It runs clang-tidy on `src/starcom_adapt/*.cpp` with the `build_flight` compile database and `starcom/.clang-tidy`. `tidycount.py` gives total 328.

| Group | Count |
|---|---:|
| Files under `src/starcom_adapt` | 104 |
| Starcom headers, seen from those files | 66 |
| Other Rocket-Chip headers | 158 |

The 104 warnings in the adapter's own files:

| Count | Check |
|---|---|
| 65 | `readability-identifier-naming` |
| 20 | `misc-use-internal-linkage` |
| 7 | `cppcoreguidelines-avoid-non-const-global-variables` |
| 3 | `misc-const-correctness` |
| 3 | `hicpp-signed-bitwise` (`cmd_sdu.cpp:15`, `:31` twice) |
| 2 | `readability-magic-numbers` (`nav_sdu.h:20`, `:22`) |
| 2 | `modernize-deprecated-headers` |
| 1 | `cppcoreguidelines-pro-type-member-init` |
| 1 | `bugprone-branch-clone` |

Suffix warnings in the adapter's own files: 0. The 79 suffix warnings are in other Rocket-Chip headers (`mission_profile_data.h`, `radio_config_table.h`, and others).

Many of the 66 Starcom-header warnings call a function declaration a non-const global (`decodePltu`, `encodeSetVr`, and the same form). The core run defines those functions and does not report them. The adapter translation units only include the headers.

## V-gcc

MinGW g++ 15.2.0. The 19 translation units. Each command comes from the compile database. `-Werror`, `-c`, and `-o` are removed. `-fsyntax-only` is added. The added flags are `-Wshadow`, `-Wconversion`, `-Wsign-conversion`, `-Wold-style-cast`, `-Wuseless-cast`, `-Wcast-qual`, and `-Wfloat-equal`.

| Flag | Count |
|---|---:|
| `-Wuseless-cast` | 9 |
| `-Wsign-conversion` | 0 |
| `-Wconversion` | 0 |
| `-Wold-style-cast` | 0 |
| `-Wshadow` | 0 |
| `-Wcast-qual` | 0 |
| `-Wfloat-equal` | 0 |

The 9 casts are all in `src/ccsds/mac.cpp`: lines 1421, 1422, 1423, 1424, 1466, 1517, 1518, 1519, and 1520. Each is `static_cast<std::uint32_t>` on a value g++ already types as `uint32_t`. The core tidy run does not report `readability-redundant-casting` for these lines.

## lizard

lizard 1.21.2, `py -3 -m lizard -l cpp`, on `starcom/src`, `starcom/include`, `starcom/adapters`, and `src/starcom_adapt`. The length column is the line span. Section 1 used that column for the old 104-line function.

Three functions have a length over 60. No function has a CCN over 20. No function has more than 7 parameters.

| Function | File | Lines | Length | CCN | NLOC |
|---|---|---|---:|---:|---:|
| `macOnValidFrame` | `src/ccsds/mac.cpp` | 809-869 | 61 | 13 | 60 |
| `cop1SelectTfdz` | `src/ccsds/cop1.cpp` | 1015-1075 | 61 | 11 | 61 |
| `queueFromCv` | `src/ccsds/mac.cpp` | 195-255 | 61 | 11 | 45 |

`macWaitExpiredHalf` is now lines 621-654. Length 34. CCN 9. The highest CCN is 20 (`macOnNoFramesPending`, length 60, and `pump_air_to_send`, length 51). The tidy `readability-function-size` count is 0. lizard still reports length 61 for the three functions above.

## Source searches

| Search | Result |
|---|---|
| `#pragma once` as a directive under `starcom/` | 0 |
| Non-ASCII in `starcom/src`, `starcom/include`, `starcom/adapters` | 0 |
| Non-ASCII in `src/starcom_adapt` comments | 12 lines (`byte_pump.cpp` 27, 53, 122, 123, 139, 173, 471; `byte_pump.h` 5, 50, 56, 57, 104) |
| Function-like macros in product sources | 0 |
| Function-like `CHECK` macros in `starcom/tests/unit` | 18 |
| `#include` of a Starcom `.hpp` header | 0 |
| `continue` in product sources | 0 |
| `continue` in `starcom/tests/unit/test_version.cpp` | lines 52 and 57 |
| Loop `break` in product sources | 0 |
| Loop `break` in tests | `test_copp.cpp:413`, `:481`; `test_host_io.cpp:343` |
| `(void)` call discards in product sources | 0 |
| `(void)` on an unused parameter | `cop1.cpp:236`; `mac.cpp` lines 776, 811, 873, 934, 995, 1074, 1103, 1124, 1150, 1342, 1360 |
| `for (;;)` | 0 |
| `<cstdio>`, `fopen`, `fread`, `fclose` in product sources | 0 |
| `.data() +` | 0 |
| K&R `) {` and `else {` in `starcom/src`, `starcom/include`, `starcom/adapters` | 0 |

`src/starcom_adapt` still places the opening brace at the end of the line. Commit `47076843` changes Starcom only. The audit records that the adapter waits for the Rocket-Chip brace pass.

## Section 1 FAIL rows

| Row | Closing check | Evidence | Result |
|---|---|---|---|
| Tier 1.1 config | Comments out of the `Checks` block. Probe run. | `cppcoreguidelines-avoid-goto` is its own entry (line 145). `clang-analyzer-core.*` is an entry (line 236). The probe fires `avoid-goto`, `NullDereference`, and `DivideZero`. `probe_vendor.hpp` has no warning. | PASS |
| Tier 1.1 positive control | Same probe. | `tidy_probe.cpp:35` `goto done` fires `cppcoreguidelines-avoid-goto`. | PASS |
| JSF 3 | lizard CCN | Maximum CCN is 20. `macWaitExpiredHalf` CCN is 9. | PASS |
| JSF 6 | Every other shall FAIL in this table is closed, so no in-file deviation note is required. | Rows below are still FAIL. Those sites have no in-file deviation note. | STILL FAIL |
| JSF 9 | Non-ASCII in Starcom comments | 0 in `starcom/src`, `starcom/include`, and `starcom/adapters`. 12 comment lines remain in `src/starcom_adapt` (list above). | STILL FAIL in `src/starcom_adapt` |
| JSF 13 | Same bytes as JSF 9 | Same 12 lines. No wide string literal in the product sources. | STILL FAIL in `src/starcom_adapt` |
| JSF 14 | Uppercase suffix count | Core 304 to 0. Adapter own files 0. | PASS |
| JSF 17 | `errno` | `udp.cpp:205` and `:241`, POSIX branch only. Commit `1a7528a9` lists one `errno` read after each failing POSIX `sendto` and `recvfrom`. | Remaining listed site, commit `1a7528a9` |
| JSF 22 | `<cstdio>` and `fopen` / `fread` / `fclose` | 0 | PASS |
| JSF 26 | `#if` / `#ifdef` / `#ifndef` other than an include guard | `udp.cpp` still has `#if defined(_WIN32)` at lines 10, 30, 101, 136, 145, 154, 186, and 219, plus the matching `#else` lines. Commit `1a7528a9` does not list these directives in its remaining-site list. | STILL FAIL |
| JSF 27 | `#pragma once` and include guards | Directive count 0. 28 product headers have a guard. Each guard name is unique. `tests/tidy_probe/starcom/include/probe_hit.h` still has no guard. The audit already records that the probe header was outside the guard change. | PASS for the product headers |
| JSF 28 | `#ifndef` only as a guard | `udp.cpp:11` `#ifndef WIN32_LEAN_AND_MEAN` | STILL FAIL |
| JSF 29 | Function-like macros | 0 in product sources. 18 `CHECK` macros remain in unit tests. | PASS for product sources |
| JSF 31 | `#define` only as a guard | `udp.cpp:12` `#define WIN32_LEAN_AND_MEAN` | STILL FAIL |
| JSF 35 | A header blocks a second inclusion | Same evidence as JSF 27. Product headers have guards. `probe_hit.h` does not. | PASS for the product headers |
| JSF 53 | Header names end in `.h` | Starcom headers in source control end in `.h`. No `#include` of a Starcom `.hpp` header. `tl/expected.hpp` and `probe_vendor.hpp` stay as third-party or as the vendor stand-in. | PASS |
| JSF 60 | Opening brace on its own line | 0 `) {` in Starcom sources. The adapter still uses the old brace form. Commit `47076843` is Starcom only. | PASS for Starcom |
| JSF 61 | Brace line has no other code token | 0 `else {` in Starcom sources. Same adapter limit as JSF 60. | PASS for Starcom |
| JSF 108 | `fcntl` varargs | `udp.cpp:120` and `:125`, in `setNonBlocking`, POSIX only. Commit `1a7528a9` lists both calls. | Remaining listed site, commit `1a7528a9` |
| JSF 110 | More than 7 parameters | lizard: 0 functions. `pack_cmd_sdu_user`, `unpack_cmd_sdu_user`, and `pump_pack_cmd_packet` each have 3 parameters. | PASS |
| JSF 130 | Comment density, with JSF 133 and rule D.8 | Core `.cpp` 209/4702 = 4.4% (Goddard recount; the first count, 224, also took block-comment lines that do not start with `*`). Host and RP2350 adapters 27/436 = 6.2%. `src/starcom_adapt` 42/693 = 6.1%. The band is 15% to 25%. One core file is inside the band (`space_packet_service.cpp`, 21.2%). | STILL FAIL |
| JSF 133 | Introductory comment in each source file | Each product `.cpp` and `.h` has one. `version.h.in` has the comment on lines 4 and 5, after the guard. | PASS |
| JSF 151 | `readability-magic-numbers` | Core 117 to 0. Adapter own files: `nav_sdu.h:20` (40) and `:22` (44), both in `static_assert`. | STILL FAIL |
| JSF 162 | `g++ -Wsign-conversion` | 0 on the 19 translation units. | PASS |
| JSF 176 | Function-pointer declarations | 0 in product sources. The adapter seams are templates. | PASS |
| JSF 181 | `g++ -Wuseless-cast` and redundant-cast tidy | 9 sites, all `mac.cpp` (lines listed under V-gcc). The old 12 sites are gone. These 9 are new. | STILL FAIL |
| JSF 182 | `reinterpret_cast` | 4 sites: `file_replay.cpp:36`, `udp.cpp:92`, `:110`, `:115`. Commit `1a7528a9` lists these four OS-edge casts. | Remaining listed site, commit `1a7528a9` |
| JSF 190 | `continue` | 0 in product sources, including the old sites in `ldpc.cpp`, `pltu.cpp`, `file_replay.cpp`, and `byte_pump.cpp`. Two `continue` statements remain in `test_version.cpp`. | PASS for product sources |
| JSF 191 | Loop `break` | 0 in product sources, including `cop1.cpp` and `file_replay.cpp`. Three loop `break` statements remain in unit tests (lines above). | PASS for product sources |
| JSF 200 | `for (;;)` | 0. `file_replay.cpp` no longer has that loop. | PASS |
| JSF 207 | Non-const globals | `g_coppSyncLossN` and `g_coppRetransmitN` are gone. `g_tfScratch` (`copp.cpp:429`) and `g_cop1TfScratch` (`cop1.cpp:21`) have an owner comment. The plan records those two buffers as no finding. `g_wsa` remains at `udp.cpp:32`. | STILL FAIL at `udp.cpp:32` |
| JSF 209 | Plain `int`, `unsigned`, `short`, `long`, `float`, `double` in code | 0 in `starcom/src` and `starcom/include`. `udp.cpp` still uses `int` at lines 32, 63, 65, 118, 120, 187, 188, 192, 205, 220, 224, and 241. `byte_pump.cpp` still uses `unsigned` at lines 43, 168, 329, 343, and 348. `cmd_sdu.h:26` is `std::array<float, 5>`. | STILL FAIL |
| JSF 215 | Pointer arithmetic | The old `file_replay.cpp` `.data() +` sites are 0. `cmd_sdu.cpp` still adds to a pointer at lines 18, 19, 20, 21, 22, 33, 34, 35, 36, and 37. | STILL FAIL |
| P10-2 | Fixed loop bound | The `for (;;)` loop is gone. | PASS |
| P10-4 | Function length 60 lines or less | Three functions have length 61 (table above). | STILL FAIL |
| P10-9 | No function pointers | 0 in product sources. | PASS |
| LOC-3 | Same as P10-2 | 0 `for (;;)`. | PASS |
| LOC-8 | Counters have one owner | The two process counters are gone. The scratch buffers name their owner and their one caller. | PASS |
| LOC-17 | Same as JSF 209 | Same plain `int`, `unsigned`, and `float` sites. | STILL FAIL |
| LOC-25 | Length and parameter count | Parameter count is 7 or less for every function. Length fails for the same three functions as P10-4. | STILL FAIL |
| LOC-29 | Same as P10-9 | 0 function pointers in product sources. | PASS |
| D.8 | Comment density on `.cpp` files | Same percentages as JSF 130. | STILL FAIL |

Comment density counts a line that starts with `//`, `/*`, or `*` after the indent. The denominator is the non-blank lines. Headers are excluded. This is the method in `standards/CODING_STANDARDS.md` under Comment Density.
