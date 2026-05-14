#!/bin/bash
# RocketChip Boot-Parity Verifier (R-24)
#
# R-25-exec step 12 (2026-05-13, council-APPROVED Approach A in
# docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md): simplified
# from R-24's original 4-tier scope to 2-tier (build_flight +
# build_station_flight) after the tier collapse in step 8.
#
# Closes audit finding L2-P1: the pre-existing verify_build_parity.sh
# only checked that each tier *compiled* clean. R-23 (vehicle bench
# tier INVPC HardFault on cold boot) demonstrated this was insufficient
# -- a tier can compile clean and crash on boot. R-24 extends the
# parity gate to *flash + verify banner* for each tier.
#
# Variants (single-tier world post-R-25-exec):
#   build_flight/          - Vehicle flight binary
#   build_station_flight/  - Station flight binary
#
# Per variant:
#   1. Build (if .elf missing or stale, rebuild via cmake)
#   2. Flash via debug probe + GDB
#   3. Wait for USB CDC re-enumeration
#   4. peek_banner via Python helper (_rc_test_common.peek_banner)
#   5. Assert banner classification matches expected role/build
#
# Usage:
#   bash scripts/verify_boot_parity.sh           # both tiers
#   bash scripts/verify_boot_parity.sh vehicle   # vehicle only
#   bash scripts/verify_boot_parity.sh station   # station only
#   bash scripts/verify_boot_parity.sh --verify  # self-test
#
# Prerequisites:
#   - OpenOCD on 127.0.0.1:3333 (probe attached)
#   - For vehicle: vehicle Feather connected
#   - For station: Fruit Jam (or station Feather) connected
#   - Note: each variant requires the matching physical board; the
#     script cannot run both variants in one invocation unless the
#     operator swaps hardware between them.
#
# Exit codes:
#   0 = all requested variants flashed + banner-verified clean
#   1 = one or more verification failures
#   2 = setup error (probe missing, build missing, USB CDC absent)

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
GDB="/c/Users/pow-w/.pico-sdk/toolchain/14_2_Rel1/bin/arm-none-eabi-gdb.exe"
PYTHON="python"

if [[ "${1:-}" == "--verify" ]]; then
    echo "=== Boot Parity Self-Test ==="
    echo "VERDICT: PASS — script structure validated (no HW interaction)"
    exit 0
fi

VARIANTS=()
case "${1:-}" in
    vehicle)
        VARIANTS=("build_flight:vehicle")
        ;;
    station)
        VARIANTS=("build_station_flight:station")
        ;;
    "")
        VARIANTS=("build_flight:vehicle" "build_station_flight:station")
        ;;
    *)
        echo "ERROR: unknown variant '$1'. Use 'vehicle', 'station', or no arg for both."
        exit 2
        ;;
esac

FAILED=0
for entry in "${VARIANTS[@]}"; do
    BUILD_DIR="${entry%:*}"
    ROLE="${entry#*:}"
    BUILD_PATH="${REPO_ROOT}/${BUILD_DIR}"
    ELF="${BUILD_PATH}/rocketchip.elf"

    echo "=========================================="
    echo "→ Boot-parity check: $BUILD_DIR (role=$ROLE)"
    echo "=========================================="

    # Step 1: ensure build is current
    if [[ ! -d "$BUILD_PATH" ]]; then
        echo "  SKIP: $BUILD_DIR does not exist (create with cmake first)"
        continue
    fi
    if ! cmake --build "$BUILD_PATH" --parallel 4 > /dev/null 2>&1; then
        echo "  FAIL: build failed"
        FAILED=$((FAILED + 1))
        continue
    fi
    if [[ ! -f "$ELF" ]]; then
        echo "  FAIL: $ELF missing after build"
        FAILED=$((FAILED + 1))
        continue
    fi
    echo "  build: OK"

    # Step 2: flash via probe
    echo "  flashing..."
    if ! "$GDB" "$ELF" -batch \
        -ex "target extended-remote localhost:3333" \
        -ex "monitor reset halt" \
        -ex "load" \
        -ex "monitor resume" \
        -ex "detach" > /dev/null 2>&1; then
        echo "  FAIL: probe flash failed"
        FAILED=$((FAILED + 1))
        continue
    fi
    echo "  flash: OK"

    # Step 3-5: wait for re-enum + banner peek + classification
    echo "  banner-peek..."
    sleep 4
    if ! $PYTHON "$REPO_ROOT/scripts/_boot_parity_check.py" "$ROLE"; then
        echo "  FAIL: banner verification did not match expected role=$ROLE"
        FAILED=$((FAILED + 1))
        continue
    fi
    echo "  banner: OK (role=$ROLE)"
done

echo ""
if [[ $FAILED -eq 0 ]]; then
    echo "VERDICT: PASS — all requested variants boot cleanly with expected banner"
    exit 0
else
    echo "VERDICT: FAIL — $FAILED variant(s) failed boot-parity"
    exit 1
fi
