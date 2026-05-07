#!/bin/bash
# RocketChip Stack Usage Analyzer
#
# Analyzes -fstack-usage output and flags excessive stack usage.
#
# Requirements:
#   Build with -fstack-usage (add to CMake or use a dedicated preset):
#     cmake -B build_stack -DCMAKE_C_FLAGS="-fstack-usage" ...
#
# Usage:
#   bash scripts/analyze_stack_usage.sh [build_dir]
#
#   --verify   Run self-test on a known sample to validate parser
#
# Thresholds (per AUDIT_GUIDANCE.md):
#   - Local stack > 1024 bytes  → WARNING
#   - Total SRAM usage > 70%    → WARNING (RP2350 Feather: 520 KiB SRAM)

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${1:-${REPO_ROOT}/build}"
THRESHOLD_BYTES=1024
SRAM_TOTAL_KB=520          # RP2350 Feather / Fruit Jam
SRAM_THRESHOLD_PCT=70

if [[ "${2:-}" == "--verify" || "${1:-}" == "--verify" ]]; then
    echo "=== Stack Usage Analyzer Self-Test ==="
    # Create a fake .su file
    TMP_SU=$(mktemp)
    cat > "$TMP_SU" <<EOF
src/test.c:42 test_function 2048 static
src/test.c:55 small_func 128 static
EOF
    WARNINGS=$(awk -v thresh="$THRESHOLD_BYTES" '
        $3 ~ /^[0-9]+$/ && $3 > thresh { print }
    ' "$TMP_SU" | wc -l)
    rm -f "$TMP_SU"

    if [[ $WARNINGS -eq 1 ]]; then
        echo "VERDICT: PASS — Self-test detected expected warning"
        exit 0
    else
        echo "VERDICT: FAIL — Self-test did not detect expected warning"
        exit 1
    fi
fi

if [[ ! -d "$BUILD_DIR" ]]; then
    echo "ERROR: Build directory not found: $BUILD_DIR"
    echo "Run with -fstack-usage enabled first."
    exit 1
fi

# Find all .su files
SU_FILES=$(find "$BUILD_DIR" -name '*.su' 2>/dev/null || true)
if [[ -z "$SU_FILES" ]]; then
    echo "ERROR: No .su files found in $BUILD_DIR"
    echo "Rebuild with -fstack-usage (add to CFLAGS)."
    exit 1
fi

echo "=== RocketChip Stack Usage Analysis ==="
echo "Build: $BUILD_DIR"
echo "Threshold: ${THRESHOLD_BYTES} bytes locals"
echo ""

WARNING_COUNT=0
TOTAL_STACK=0

while IFS= read -r su_file; do
    while read -r line; do
        # Format: file:line function size unit
        set -- $line
        size=${3:-0}
        if [[ "$size" =~ ^[0-9]+$ ]] && [[ $size -gt $THRESHOLD_BYTES ]]; then
            echo "WARNING: $line"
            WARNING_COUNT=$((WARNING_COUNT + 1))
        fi
        TOTAL_STACK=$((TOTAL_STACK + size))
    done < "$su_file"
done <<< "$SU_FILES"

# SRAM usage estimate (very rough)
SRAM_USED_KB=$((TOTAL_STACK / 1024))
SRAM_PCT=$((SRAM_USED_KB * 100 / SRAM_TOTAL_KB))

echo ""
echo "Total estimated stack usage: ${SRAM_USED_KB} KiB (${SRAM_PCT}% of ${SRAM_TOTAL_KB} KiB SRAM)"

if [[ $WARNING_COUNT -gt 0 || $SRAM_PCT -gt $SRAM_THRESHOLD_PCT ]]; then
    echo "VERDICT: FAIL — ${WARNING_COUNT} functions over ${THRESHOLD_BYTES} bytes or SRAM usage > ${SRAM_THRESHOLD_PCT}%"
    exit 1
else
    echo "VERDICT: PASS — All functions ≤ ${THRESHOLD_BYTES} bytes and SRAM usage ≤ ${SRAM_THRESHOLD_PCT}%"
    exit 0
fi
