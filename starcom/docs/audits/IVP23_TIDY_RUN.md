# IVP 23 — first host clang-tidy run (core only)

**Living report:** [`IVP23_REPORT.md`](IVP23_REPORT.md). This file is the first-run log only. Historical: taken on the pre-camelBack `docs/starcom-sad-draft` fork. Re-run on `starcom_dev` before treating counts as current.

**Date:** 2026-08-31 ~10:50 AM CDT (America/Chicago)  
**Tree (at run):** `C:\Users\pow-w\Documents\Rocket-Chip-starcom-ivp23`  
**Branch (at run):** `grok/starcom-ivp23`  
**HEAD:** `68b0daf8e2d32e614e68ead16ca3f9d0f244126f` (`docs/starcom-sad-draft` @ 68b0daf)  
**Tool:** `C:\Program Files\LLVM\bin\clang-tidy.exe` via `starcom\scripts\run_clang_tidy.ps1`  
**Config:** `--config-file starcom/.clang-tidy` (not parent RC `.clang-tidy`)  
**Compile DB:** `-p starcom/build` (host Ninja). No arm-none-eabi / Pico extra-args.  
**Flags:** no `--fix`. Tests not scanned. Adapters not scanned.  
**Raw log:** `starcom/docs/audits/CLANG_TIDY_2026-08-31_core.txt`  
**Committed:** no.

## Files scanned (12)

`src/ccsds/{clcw,conv,cop1,copp,crc32,ldpc,mac,plcw,pltu,space_packet,uslp,v3}.cpp`

All 12 TUs parsed. No clang-tidy parse/fatal errors. Script exit 0.

Frontend "NNNNN warnings generated" lines in the log are MSVC STL / system-header noise (HeaderFilterRegex keeps them off the tidy report). They are **not** counted below.

## Totals

- **328** tidy diagnostics (warning lines in core + `include/starcom` via HeaderFilterRegex)
- **0** `readability-identifier-naming`
- **0** `clang-analyzer-*`
- **0** `cppcoreguidelines-*`
- **1** `bugprone-*`

Naming is **not** the bulk. Bulk is unsigned-suffix style + protocol bit/byte literals.

### Check ids (unique diagnostics; aliased pair counted once as 226)

| Count | Check id |
|------:|----------|
| 226 | `hicpp-uppercase-literal-suffix` + `readability-uppercase-literal-suffix` (same hits) |
| 81 | `readability-magic-numbers` |
| 9 | `readability-function-size` |
| 7 | `readability-function-cognitive-complexity` |
| 2 | `misc-use-internal-linkage` |
| 1 | `readability-else-after-return` |
| 1 | `bugprone-branch-clone` |
| 1 | `readability-container-size-empty` |

(If counting each alias separately: 226 + 226 for the suffix pair; still 328 diagnostics.)

### Top files

| Hits | File |
|-----:|------|
| 88 | `uslp.cpp` |
| 34 | `cop1.cpp` |
| 32 | `v3.cpp` |
| 29 | `space_packet.cpp` |
| 26 | `copp.cpp` |
| 25 | `clcw.cpp` |
| 24 | `pltu.cpp` |
| 23 | `mac.cpp` |
| 16 | `ldpc.cpp` |
| 12 | `crc32.cpp` |
| 10 | `conv.cpp` |
| 9 | `plcw.cpp` |

## Split: naming vs real checks

### `readability-identifier-naming` — 0

Public ICD identifiers (CLCW / COP-1 / COP-P / USLP / MAC / PLTU field and function names) were **not** flagged. Disposition only: **do not rewrite public ICD names** to satisfy house `kCamelCase` / `g_` rules. A later increment may NOLINT or ignore-regex ICD tokens; this sitting does not rename them.

### Style bulk (not defects)

- **226** lowercase `u` suffixes (`0x3Fu`, `1u`, …). House check wants `U`. Mechanical, not protocol-wrong.
- **81** magic numbers: CCSDS bit masks (`0x07`, `0x3F`, `0x80`, …), window/FSN widths (`254`, `127`), bit loops (`7`). Many are Blue Book field widths. Named constants later; not this increment.
- **function-size / cognitive-complexity:** protocol state machines over P10 60-line / CC 25 bar:
  - `mac_tick` 220 lines, CC 52
  - `apply_state` 156 lines
  - `decode_uslp` 114 / CC 30; `encode_uslp` 101 / CC 35
  - `fop_1_on_clcw` 93 / CC 41
  - `copp_bytes_to_send` 81 / CC 27; `cop1_bytes_to_send` 77
  - `encoded_pltu_size` 66 / CC 33; `copp_receive_bytes` 64 / CC 26
- `misc-use-internal-linkage`: `copp_encode_uslp`, `repeat_seq` (file-local helpers).
- `readability-else-after-return`: `mac_tick` ~608 (E80/E82 vs E85).
- `readability-container-size-empty`: `space_packet.cpp:40` (`size()` vs `empty()`).

### bugprone / clang-analyzer / cppcoreguidelines

| Check | Count | Disposition |
|-------|------:|-------------|
| `clang-analyzer-*` | 0 | No null-deref / div-by-zero / dead-store from this pass. |
| `cppcoreguidelines-pro-type-reinterpret-cast` (and rest) | 0 | Core is not MMIO. |
| `bugprone-reserved-identifier` | 0 | No Pico `__StackBottom` class noise. |
| `bugprone-branch-clone` | 1 | `mac.cpp:780` `mac_phy`: `MacState::s1` sets `p.receive = false` then a second case-list (`s11`…`s71`) does the same. Documented TX-only vs idle grouping, not a logic bug. Merge cases later if desired. |

No clang-analyzer or bugprone hit looks like a live functional defect in this increment.

## Out of scope (this increment)

- Adapters (`adapters/host/file_replay.cpp` `fopen` CRT deprecation). Full `ninja` still fails there. Core TUs are in `compile_commands.json`; tidy of `src/ccsds` did not need a fopen fix.
- Tests (exceptions allowed; not in the core bar).
- `--fix`, ICD identifier renames, commit, merge, `--no-verify`.

## Next (not done here)

1. Optional: `u` → `U` and named Blue Book masks (style gate).
2. Optional: split `mac_tick` / `apply_state` / USLP encode-decode if P10-4 is a hard bar for Starcom.
3. Later: adapters tidy after host CRT / fopen policy.
