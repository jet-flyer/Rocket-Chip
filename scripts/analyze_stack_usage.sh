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
    # Create a fake .su file using gcc's actual format (tab-separated, with
    # multi-word names that previously broke the parser)
    TMP_SU=$(mktemp)
    printf 'src/test.cpp:42:5:test_function\t2048\tstatic\n'                >  "$TMP_SU"
    printf 'src/test.cpp:55:5:small_func\t128\tstatic\n'                    >> "$TMP_SU"
    printf 'src/test.cpp:99:1:int main(int, char**)\t256\tstatic\n'         >> "$TMP_SU"
    WARNINGS=$(awk -F'\t' -v thresh="$THRESHOLD_BYTES" '
        NF >= 2 && $(NF-1) ~ /^[0-9]+$/ && $(NF-1) > thresh { print }
    ' "$TMP_SU" | wc -l)
    TOTAL=$(awk -F'\t' '
        NF >= 2 && $(NF-1) ~ /^[0-9]+$/ { total += $(NF-1) }
        END { print (total + 0) }
    ' "$TMP_SU")
    rm -f "$TMP_SU"

    # Expected: 1 warning (the 2048-byte function), total = 2048 + 128 + 256 = 2432
    if [[ $WARNINGS -eq 1 && $TOTAL -eq 2432 ]]; then
        echo "VERDICT: PASS — Self-test detected expected warning + correct total"
        exit 0
    else
        echo "VERDICT: FAIL — Self-test detected $WARNINGS warning(s) (expected 1) and total=$TOTAL (expected 2432)"
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

# gcc -fstack-usage emits TAB-separated columns:
#   file:line:col:funcname<TAB>size<TAB>qualifier
# The funcname column may contain spaces and parentheses (e.g.,
# "int main(int, char**)"), so whitespace-splitting (`set -- $line`) is wrong.
# Awk's NF-1 column reliably gives the size regardless of what's in the
# funcname column. Without this fix, lines with multi-word names produced
# size="static" and crashed bash arithmetic with
# "line 80: static: syntax error: invalid arithmetic operator".

WARNING_COUNT=0
TOTAL_STACK=0
WARNINGS_OUTPUT=""

while IFS= read -r su_file; do
    [[ -z "$su_file" ]] && continue
    # Sum sizes and collect over-threshold lines via awk (handles TAB correctly)
    awk_output=$(awk -F'\t' -v thresh="$THRESHOLD_BYTES" '
        NF >= 2 && $(NF-1) ~ /^[0-9]+$/ {
            sz = $(NF-1) + 0
            total += sz
            if (sz > thresh) {
                # Emit a WARNING line with the original record so reporting is unchanged
                print "WARN\t" $0
            }
        }
        END { print "TOTAL\t" (total + 0) }
    ' "$su_file")

    # Process awk's emitted records
    while IFS=$'\t' read -r tag rest; do
        case "$tag" in
            WARN)
                echo "WARNING: $rest"
                WARNING_COUNT=$((WARNING_COUNT + 1))
                ;;
            TOTAL)
                TOTAL_STACK=$((TOTAL_STACK + rest))
                ;;
        esac
    done <<< "$awk_output"
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
