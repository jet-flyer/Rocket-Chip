# Starcom host testing and verification

**Status:** living

How we prove codecs on the host. What to prove, and in what order, is [`IVP.md`](IVP.md). Claims are [`CONFORMANCE.md`](CONFORMANCE.md). Handshake: [`ICD.md`](ICD.md).

Add tests in the same sitting as the code they prove. The suite grows with the functionality; there is no separate “write all tests later” pass. Inputs and outputs for codecs are the Blue Book wire formats. We do not generate large random corpora until a later hardening sitting (IVP increment 6).

## Run

```
cmake -S starcom -B starcom/build -G Ninja
cmake --build starcom/build
ctest --test-dir starcom/build --output-on-failure
```

Today that is one binary: `starcom.unit`. Tests may use exceptions; the core does not.

ASan+UBSan (when the toolchain has libasan/libubsan):

```
cmake -S starcom -B starcom/build-san -G Ninja -DSTARCOM_SANITIZE=ON
cmake --build starcom/build-san
ctest --test-dir starcom/build-san --output-on-failure
```

Turns off the D-5 `--wrap=malloc` trap (ASan owns the allocator). This tree's MinGW g++ 15.2 has the flags but not the libs.

## When a codec lands

Do these in the same sitting as the `.cpp`. Names come from the IVP table for that increment.

1. **One accept golden.** Canned octets. Hex remainder (if any) computed from the cited book — for PLTU CRC, 211.2 Annex C. Put the hex in the test.
2. **Round-trip.** `decode(encode(x))` matches `x` on the fields that codec owns. Extra octets after a complete candidate are ignored (no stream search).
3. **Reject table.** One case per IVP reject name. Assert the `Error` value from the ICD. Analysis-only names (today: `asm-in-crc`) are assertions on a helper, not decoder errors.
4. **Composition** when the pieces exist. Example still ahead: one Space Packet of N user octets inside a V-3 inside a PLTU is 18+N octets (SAD figure).
5. **D-5 heap trap.** Arm around the new calls. Keep the positive-control allocation so a silent trap failure is visible. Codec paths stay at zero counts.

The book wins if a test comment and Annex C / Fig 3-1 disagree. Fix the test (and ICD if the handshake was wrong) in the same commit.

## What waits

- **Shared on-disk golden files / generated corpora** — not yet. Named IVP vectors as `constexpr` octets. Increment 6 prefix smoke is `tests/unit/test_fuzz.cpp` (bounded lengths, TFVN/ASM fills), not a random dump.
- Radio, Pico SDK, FPGA testbench — adapters / ports. Same golden vectors when those sittings run.

## Hooks (later, keep small)

Rocket-Chip’s commit hook runs the whole firmware host suite and can ask for hardware. Starcom does not copy that.

When a Starcom hook exists it should be **one** check: if the staged paths are under `starcom/`, run `ctest --test-dir starcom/build`. No HW reseat, no 860 firmware tests, no extra policy layers. Add it when the first codec commit is on a branch we push; not as a second framework.

## Changelog

Starcom [`CHANGELOG.md`](../CHANGELOG.md) is for **major pushes** (first tagged cut, public extract, a push that changes the supported API). Ordinary codec sittings and graph snapshots do not mint a row.
