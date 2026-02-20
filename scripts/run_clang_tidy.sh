#!/bin/bash
# RocketChip clang-tidy audit runner
#
# Runs clang-tidy across all production source files and writes
# per-file output to a timestamped directory.
#
# Prerequisites:
#   - LLVM/clang-tidy installed (C:/Program Files/LLVM/bin/)
#   - Target build completed: cmake -B build -G Ninja && ninja -C build
#     (generates compile_commands.json)
#
# Usage:
#   bash scripts/run_clang_tidy.sh           # Full audit
#   bash scripts/run_clang_tidy.sh src/main.cpp  # Single file

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CLANG_TIDY="C:/Program Files/LLVM/bin/clang-tidy.exe"
BUILD_DIR="${REPO_ROOT}/build"
TOOLCHAIN="C:/Users/pow-w/.pico-sdk/toolchain/14_2_Rel1"

# Verify prerequisites
if [[ ! -x "${CLANG_TIDY}" ]]; then
    echo "ERROR: clang-tidy not found at ${CLANG_TIDY}"
    exit 1
fi
if [[ ! -f "${BUILD_DIR}/compile_commands.json" ]]; then
    echo "ERROR: compile_commands.json not found. Run: cmake -B build -G Ninja && ninja -C build"
    exit 1
fi

# Extra args for ARM cross-compilation context
EXTRA_ARGS=(
    --extra-arg="--sysroot=${TOOLCHAIN}/arm-none-eabi"
    --extra-arg="--target=armv8m.main-none-eabi"
    --extra-arg="-isystem"
    --extra-arg="${TOOLCHAIN}/arm-none-eabi/include/c++/14.2.1"
    --extra-arg="-isystem"
    --extra-arg="${TOOLCHAIN}/arm-none-eabi/include/c++/14.2.1/arm-none-eabi/thumb/v8-m.main+fp/softfp"
    --extra-arg="-Wno-format-security"
    --extra-arg="-Wno-gnu-zero-variadic-macro-arguments"
)

# All production source files (matches ROCKETCHIP_SOURCES in CMakeLists.txt)
PRODUCTION_FILES=(
    src/main.cpp
    src/drivers/ws2812_status.cpp
    src/drivers/i2c_bus.cpp
    src/drivers/icm20948.cpp
    src/drivers/baro_dps310.cpp
    src/drivers/gps_pa1010d.cpp
    src/drivers/gps_uart.cpp
    src/calibration/calibration_data.cpp
    src/calibration/calibration_manager.cpp
    src/calibration/calibration_storage.cpp
    src/cli/rc_os.cpp
    src/math/vec3.cpp
    src/math/quat.cpp
    src/fusion/baro_kf.cpp
    src/fusion/eskf.cpp
    src/fusion/wmm_declination.cpp
    src/fusion/mahony_ahrs.cpp
)

# If specific files passed as args, use those instead
if [[ $# -gt 0 ]]; then
    PRODUCTION_FILES=("$@")
fi

# Output directory
TIMESTAMP=$(date +%Y-%m-%d)
OUT_DIR="${REPO_ROOT}/logs/clang-tidy-${TIMESTAMP}"
mkdir -p "${OUT_DIR}"

echo "=== RocketChip clang-tidy Audit ==="
echo "Tool: $("${CLANG_TIDY}" --version 2>&1 | grep 'LLVM version')"
echo "Files: ${#PRODUCTION_FILES[@]}"
echo "Output: ${OUT_DIR}/"
echo ""

TOTAL_WARNINGS=0
SUMMARY_FILE="${OUT_DIR}/summary.txt"
> "${SUMMARY_FILE}"

for src in "${PRODUCTION_FILES[@]}"; do
    FILE_PATH="${REPO_ROOT}/${src}"
    if [[ ! -f "${FILE_PATH}" ]]; then
        echo "SKIP: ${src} (not found)"
        continue
    fi

    BASENAME=$(basename "${src}" .cpp)
    OUT_FILE="${OUT_DIR}/${BASENAME}.txt"

    echo -n "  ${src} ... "

    # Run clang-tidy, capture output
    "${CLANG_TIDY}" "${FILE_PATH}" -p "${BUILD_DIR}/" "${EXTRA_ARGS[@]}" \
        2>&1 | grep -v "^[0-9]* warnings generated" | grep -v "^Suppressed" | grep -v "^Use -header-filter" \
        > "${OUT_FILE}" || true

    # Count warnings (lines matching "warning:" in our code)
    COUNT=$(grep -c "warning:" "${OUT_FILE}" 2>/dev/null || echo 0)
    TOTAL_WARNINGS=$((TOTAL_WARNINGS + COUNT))

    echo "${COUNT} warnings"
    printf "%-50s %d\n" "${src}" "${COUNT}" >> "${SUMMARY_FILE}"
done

echo ""
echo "Total: ${TOTAL_WARNINGS} warnings across ${#PRODUCTION_FILES[@]} files"
echo "Summary: ${SUMMARY_FILE}"
printf "\nTotal: %d warnings\n" "${TOTAL_WARNINGS}" >> "${SUMMARY_FILE}"
