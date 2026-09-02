#!/usr/bin/env bash
# Starcom host clang-tidy. Core first. Not the Rocket-Chip firmware gate.
# Requires: cmake -S . -B build -G Ninja  (this directory is starcom/)
set -eu
STARCOM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CT="${CLANG_TIDY:-C:/Program Files/LLVM/bin/clang-tidy.exe}"
BUILD="${BUILD_DIR:-${STARCOM}/build}"
CONFIG="${STARCOM}/.clang-tidy"
COMPDB="${BUILD}/compile_commands.json"
SCOPE="core"

if [[ "${1:-}" == "--all" ]]; then
  SCOPE="all"
fi

if [[ ! -f "${COMPDB}" ]]; then
  echo "run_clang_tidy: no ${COMPDB} — configure first: cmake -S . -B build -G Ninja" >&2
  exit 2
fi
if [[ ! -x "${CT}" && ! -f "${CT}" ]]; then
  echo "run_clang_tidy: clang-tidy not found at ${CT}" >&2
  exit 2
fi

mapfile -t FILES < <(cd "${STARCOM}" && ls src/ccsds/*.cpp)
if [[ "${SCOPE}" == "all" ]]; then
  mapfile -t MORE < <(cd "${STARCOM}" && find adapters -name '*.cpp' 2>/dev/null | sort)
  FILES+=("${MORE[@]}")
fi

OUT_DIR="${STARCOM}/docs/audits"
mkdir -p "${OUT_DIR}"
STAMP="$(date +%Y-%m-%d)"
OUT="${OUT_DIR}/CLANG_TIDY_${STAMP}_${SCOPE}.txt"

FAIL=0
: > "${OUT}"
echo "starcom clang-tidy  scope=${SCOPE}  -p ${BUILD}  --config-file ${CONFIG}" | tee -a "${OUT}"
for f in "${FILES[@]}"; do
  echo "---- ${f} ----" | tee -a "${OUT}"
  # --config-file: parent Rocket-Chip .clang-tidy must not win while incubating.
  if ! "${CT}" "${STARCOM}/${f}" -p "${BUILD}" --config-file="${CONFIG}" --quiet >>"${OUT}" 2>&1; then
    FAIL=1
  fi
done
echo "wrote ${OUT}"
exit "${FAIL}"
