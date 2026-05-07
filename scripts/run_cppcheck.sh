#!/bin/bash
# RocketChip Cppcheck Runner
#
# Runs Cppcheck on production source files.
# Complements clang-tidy (focuses on bugprone patterns clang-tidy is weaker on).
#
# Usage:
#   bash scripts/run_cppcheck.sh
#   bash scripts/run_cppcheck.sh --verify
#
# Output: logs/cppcheck-YYYY-MM-DD/

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CPPCHECK="cppcheck"
BUILD_DIR="${REPO_ROOT}/build"
OUT_DIR="${REPO_ROOT}/logs/cppcheck-$(date +%Y-%m-%d)"

# PRODUCTION_FILES (same list as run_clang_tidy.sh)
PRODUCTION_FILES=($(grep -o 'src/[^[:space:]]*\.cpp' "${REPO_ROOT}/CMakeLists.txt" | \
    grep -v 'eskf_codegen\.cpp' | sort -u))

if [[ "${1:-}" == "--verify" ]]; then
    echo "=== Cppcheck Self-Test ==="
    TMP_DIR=$(mktemp -d)
    cat > "$TMP_DIR/test.cpp" << 'EOF'
int bad_func() {
    int x;
    return x;   // uninitialized use - should be caught
}
EOF
    OUT=$(cppcheck --enable=warning,style,performance,portability --quiet --inline-suppr \
        --suppress=missingIncludeSystem "$TMP_DIR/test.cpp" 2>&1 || true)
    rm -rf "$TMP_DIR"

    if echo "$OUT" | grep -q "uninitialized"; then
        echo "VERDICT: PASS — Self-test detected expected finding"
        exit 0
    else
        echo "VERDICT: FAIL — Self-test did not detect uninitialized variable"
        echo "Output was: $OUT"
        exit 1
    fi
fi

if ! command -v "$CPPCHECK" &> /dev/null; then
    echo "ERROR: cppcheck not found in PATH"
    exit 1
fi

mkdir -p "$OUT_DIR"

echo "=== RocketChip Cppcheck Audit ==="
echo "Files: ${#PRODUCTION_FILES[@]}"
echo "Output: $OUT_DIR/"
echo ""

TOTAL_ISSUES=0

for f in "${PRODUCTION_FILES[@]}"; do
    if [[ -f "$f" ]]; then
        ISSUES=$(cppcheck --enable=warning,style,performance,portability \
            --quiet --inline-suppr --suppress=missingIncludeSystem \
            --template="{file}:{line}: {severity}: {message}" \
            "$f" 2>&1 | tee -a "${OUT_DIR}/cppcheck.txt" || true)

        COUNT=$(echo "$ISSUES" | grep -c ":" || true)
        TOTAL_ISSUES=$((TOTAL_ISSUES + COUNT))
    fi
done

echo ""
echo "Total issues: $TOTAL_ISSUES"
echo "Details: ${OUT_DIR}/cppcheck.txt"

if [[ $TOTAL_ISSUES -eq 0 ]]; then
    echo "VERDICT: PASS — cppcheck: 0 issues across ${#PRODUCTION_FILES[@]} files"
    exit 0
else
    echo "VERDICT: FAIL — cppcheck: $TOTAL_ISSUES issues across ${#PRODUCTION_FILES[@]} files"
    exit 1
fi
