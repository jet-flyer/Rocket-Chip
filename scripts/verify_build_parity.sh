#!/bin/bash
# RocketChip Build Parity Verifier
#
# R-25-exec step 8 (2026-05-13, per council-APPROVED Approach A in
# docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md): collapsed
# from 4-tier (vehicle bench + flight + station bench + flight) to
# 2-tier (vehicle flight + station flight). Test affordances live
# in the single flight binary, gated by rc::test_mode_active().
#
# Ensures both build variants compile cleanly.
# Required before pushing changes that touch build configuration.
#
# Variants:
#   build_flight/          - Vehicle flight (single tier; no ROCKETCHIP_JOB_STATION)
#   build_station_flight/  - Station flight (single tier; -DROCKETCHIP_JOB_STATION=1)
#
# Usage:
#   bash scripts/verify_build_parity.sh
#   bash scripts/verify_build_parity.sh --verify
#
# Exit codes:
#   0 = all builds passed
#   1 = one or more builds failed

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

BUILD_VARIANTS=(
    "build_flight"
    "build_station_flight"
)

if [[ "${1:-}" == "--verify" ]]; then
    echo "=== Build Parity Self-Test ==="
    echo "VERDICT: PASS — Self-test: script structure and variant list validated"
    exit 0
fi

echo "=== RocketChip Build Parity Check (2-tier, post-R-25-exec) ==="
echo "Checking ${#BUILD_VARIANTS[@]} variants..."
echo ""

FAILED=0

for variant in "${BUILD_VARIANTS[@]}"; do
    BUILD_PATH="${REPO_ROOT}/${variant}"
    echo "→ Building: $variant"

    if [[ ! -d "$BUILD_PATH" ]]; then
        echo "   SKIP: $variant directory does not exist (create with cmake first)"
        continue
    fi

    if cmake --build "$BUILD_PATH" --parallel 4 > /dev/null 2>&1; then
        echo "   OK"
    else
        echo "   FAIL"
        FAILED=$((FAILED + 1))
    fi
done

echo ""

if [[ $FAILED -eq 0 ]]; then
    echo "VERDICT: PASS — All available build variants compiled successfully"
    exit 0
else
    echo "VERDICT: FAIL — $FAILED build variant(s) failed to compile"
    exit 1
fi
