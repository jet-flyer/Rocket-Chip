# IVP 23 remainder — house identifiers (living)

**Increment:** 23 initial pass already landed on `main` (`688ff00`, public-verb camelBack + gated tidy subset). This file is the **remainder**, not a new increment.
**Methods:** R, I. Not a Blue Book claims file.
**Tree:** `C:\Users\pow-w\Documents\starcom_dev` on `grok/sc-dev`.
**Do not** merge until the remainder pass. Do not `--fix`. Do not `--no-verify`.
**First run (historical):** [`IVP23_TIDY_RUN.md`](IVP23_TIDY_RUN.md) + raw log `CLANG_TIDY_2026-08-31_core.txt` (taken on `docs/starcom-sad-draft` @ `68b0daf`, pre-camelBack). Re-run on this tree before treating those counts as current.

clang-tidy for the standalone library lives **in** `starcom/` (extract-ready).

## No accepted deviation

Owner 2026-09-02: **zero Starcom rows** in `standards/ACCEPTED_STANDARDS_DEVIATIONS.md`. When house camelBack and a Blue Book / ICD name disagree, look up how NASA trees resolve it — do not mint a deviation.

| Source | What they do with CCSDS identifiers |
|--------|--------------------------------------|
| [NASA F´ style guidelines](https://github.com/nasa/fprime/wiki/F%C2%B4-Style-Guidelines) (2026-06-11) | Functions and locals: **lower camel case**. Types: **PascalCase**. Constants and enumeration *values*: ALL_UPPER_WITH_UNDERSCORE. CCSDS stack (`Svc/Ccsds/Types`, e.g. `SpacePacketSubfields::ApidMask = 0x07FF`) uses the F´ scheme, not Blue Book snake_case. |
| [cFS identifier convention](https://github.com/nasa/cFE/blob/main/docs/cFS_IdentifierNamingConvention.md) | CamelCase preferred for terms; existing common-use case preserved. Command names are ActionSubject (`ResetCounters`). C macros are ALL_CAPS (`CFE_MSG_APID_MASK`). |
| House (`CODING_STANDARDS.md`) | camelBack functions/variables, `k`CamelCase constants, CamelCase types. **Supersedes JSF 45/51/52** (which wanted lowercase snake_case). Same direction as F´ functions/types. Constants already chose `k`CamelCase over F´ ALL_CAPS — that is the existing house pick, not a Starcom exception. |

**Resolution:** C++ tokens follow the house scheme. Blue Book field names live in comments and CONFORMANCE. Public verbs are already camelBack (`decodePltu`). `#pragma once` is the existing **project-wide** exception, not a Starcom row.

`Error` enumerators (`uslp_truncated`, `bad_asm`, …) are the closed ICD set. **Do not rename them.** cFS: existing common-use case is preserved. F´ house style applies when a type is born, not as a retrofit of a closed enum.

## Tooling

- `starcom/.clang-tidy` — house check list, `FunctionCase: camelBack`.
- `CMAKE_EXPORT_COMPILE_COMMANDS ON` in `starcom/CMakeLists.txt`.
- `starcom/scripts/run_clang_tidy.sh` and `run_clang_tidy.ps1` (core default; `--all` for adapters).

Stay **off:** bounds checks and include-cleaner (codec octet walks). `CheckedFunctions` emptied. No `__StackBottom` ignore.

## First core run (2026-08-31, stale fork)

12 TUs (`src/ccsds/*.cpp`). **328** diagnostics. `readability-identifier-naming` = **0** on that snake_case tree. Re-run after porting — camelBack verbs are already on this tree.

Bulk then: 226 lowercase `u` suffixes; **81** `readability-magic-numbers` (Blue Book field-width literals). One `bugprone-branch-clone` in `mac.cpp` `mac_phy` (TX-only vs idle grouping, not a live bug). Function-size: protocol state machines — **leave** (`macTick` / `apply_state` / USLP encode/decode / `fop1OnClcw`).

## JSF AV Rule 151 method (the 81)

Name at the declaration. Book number as the initializer. Cite the clause on that declaration. One declaration, not a parallel mask list. Pack site uses the symbol. Not bare `0x07u`. Not `IgnoredIntegerValues`. Not a CCSDS tidy rule.

House maps 151 as **Split**: tidy is the mechanical gate; Grey residual is named + sourced (book number as initializer).

Starcom shape: F'-style `constexpr` / `enum` in codec headers (C++20, not cFS macros).

**IRL citations (Goddard, 2026-08-31).** Not a Blue Book walk.

JSF AV C++ 2RDU00001 Rev C (Dec 2005), [JSF-AV-rules.pdf](https://stroustrup.com/JSF-AV-rules.pdf):

- AV 151 (will): "Numeric values in code will not be used; symbolic values will be used instead." Sole exception: a class/structure constructor may initialize an array member with numeric values. Note: `0` and `1` may be used when they are fundamental logic. There is no protocol-field carve-out.
- AV 6 (shall): each deviation from a shall rule is documented in the file that contains the deviation.
- AV 7 (will): approval is not required when the use matches an exception written into that rule.

F´ (`nasa/fprime` `devel`, `Svc/Ccsds/Types/Types.fpp`): `SpacePacketSubfields::ApidMask = 0x07FF`. Pack/unpack uses the symbol. That is the Starcom shape: named constant, book number as initializer, symbol at the pack site.

Do not `IgnoredIntegerValues` the 81.

### The 81, by kind

**A. Book field masks already half-named in headers (reuse, do not duplicate).** Pack site still has a bare mask.

- `0x07u` (14): 3-bit fields. CLCW status, V-3 `port_id` / length high bits, USLP `vcf_count_len` / SCID-VCID nibbles, Space Packet APID high bits. `kSetVrDirectiveType` is the **value** `0x03` (211.0-B-6 Annex B1.5 type `011`), not the 3-bit mask. Not 232.0 `0x82` (`kCop1SetVr0`).
- `0x3Fu` (4): 6-bit VCID / Space Packet seq high bits.
- `0x1Fu` (4): 5-bit UPID.
- `0x0Fu` (5): 4-bit MAP / SCID nibble.
- `0b1100u`: TFVN check. Use existing `kTfvnUslp`.
- `6u` / `6` on truncated USLP length: use `kUslpTruncatedMin`.
- `127` / `127u`: use existing `kFopPSentCap`.
- `254` / `254u` (FARM `w` cap): name once in `cop1.hpp`, cite the 232.1 window clause.

**B. Octet indices and header sizes.** Prefer existing `k*Size` (`kSpacePacketHeaderSize` 6, `kV3HeaderSize` 5, `kUslpPrimaryHeaderMin` 7, `kUslpTruncatedMin` 6).

**C. Bit positions / endian shifts.** Name as shift helpers or field-bit offsets on the same declaration that owns the mask. `24` is BE32, not a 211.2 field.

**D. Octet bit-walks (`7` / `7u`).** `crc32.cpp`, `conv.cpp`, `ldpc.cpp`. Source is "octet has 8 bits". Name once (`kOctetMsb`). Do not cite 232.0 or 211.0.

**E. MAC sequence-slot `ss`.** Name the whole 0-7 SS set together (enum citing 211.0-B-6 §6.2.2).

**F. `0xF8u` SET_VR spare mask.** Cite **211.0-B-6 Annex B1.5**, not 232.0 `0x82`.

Tidy ignore (do **not** widen): `IgnoredIntegerValues` `0;1;2;3;4;8;16;32;64;100;255;256;1000`. That is why CLCW `0x80u` / `0x40u` flags never appear in the 81. 151 still wants those named. Name them in the same pass. Do **not** put 232.0 / 211.0 field-width literals on that ignore list.

## Remainder (not a rename sitting)

Owner 2026-09-02: do **not** add code just to rename things. IRL (F´ Types.fpp, cFS `CFE_MSG_APID_MASK`): a field mask is named when the codec is written, on the type that owns the field. We already have `kTfvnV3`, `kIdleApid`, `kFopPSentCap`, `kSetVrDirectiveType`. Use those at the pack site if a literal is still bare.

Do not: `Error::uslp_truncated` → `uslpTruncated`; `MacSs::ss0` for book-numbered slots; a `1u`→`1U` pass; a parallel mask list; split `macTick`; widen `IgnoredIntegerValues`.

## Out of this remainder

- `--fix`, commit-to-main, merge.
- PHY / 211.1 / FSK / FPGA (held on the Starcom whiteboard).
- RC firmware (`i2c_master`, SPI/RFM95, …).
