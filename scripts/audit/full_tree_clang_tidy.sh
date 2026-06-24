#!/usr/bin/env bash
# Single source of truth for the GATED clang-tidy checks — the subset that BLOCKS
# commits. Both the pre-commit hook (per staged src/*.cpp) and the milestone
# full-tree sweep (SESSION_CHECKLIST item 17) call THIS, so the gated-check list
# lives in exactly one place (previously hard-coded in two — LL 40 dual-hardcode).
#
#   no args      -> full tree (every authored src/*.cpp minus exemptions)
#   file args    -> just those files (the hook passes its staged files)
#
# Gated checks (what each enforces; thresholds/options live in .clang-tidy):
#   readability-function-size                  JSF AV Rule 1 (<=60 lines) + JPL-25 (ParameterThreshold=6)
#   readability-function-cognitive-complexity  JSF AV Rule 3 (<=25)
#   bugprone-unused-return-value               P10-7 / JPL-14 (CheckedFunctions: i2c_bus/gps/flash_safe_execute)
#   bugprone-reserved-identifier               CERT DCL37-C (vendored __Stack* are NOLINT'd per TP-2)
# Plus scripts/audit/check_warning_gate_coverage.py rides the same gate (LL 43).
#
# Exemptions (match item 17): src/cli/**, eskf_codegen.cpp.
# Toolchain paths per docs/agents/SESSION_CHECKLIST.md item 17 / DEBUG_PROBE_NOTES.md.
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CT="${CLANG_TIDY:-C:/Program Files/LLVM/bin/clang-tidy.exe}"
BUILD="${BUILD_DIR:-${REPO}/build}"
TC="C:/Users/pow-w/.pico-sdk/toolchain/14_2_Rel1"

GATED='-*,readability-function-size,readability-function-cognitive-complexity,bugprone-unused-return-value,bugprone-reserved-identifier'
GREP_RE='warning:.*\[(readability-function-(size|cognitive-complexity)|bugprone-(unused-return-value|reserved-identifier))\]'

if [[ ! -f "${BUILD}/compile_commands.json" ]]; then
    echo "full_tree_clang_tidy: no ${BUILD}/compile_commands.json — configure first (cmake -B build -G Ninja)."
    exit 2
fi

if [[ $# -gt 0 ]]; then
    mapfile -t FILES < <(printf '%s\n' "$@")
else
    mapfile -t FILES < <(cd "${REPO}" && git ls-files 'src/*.cpp' | grep -v '^src/cli/' | grep -v 'eskf_codegen\.cpp$')
fi

EXTRA=(
    --extra-arg=--sysroot="${TC}/arm-none-eabi"
    --extra-arg=--target=armv8m.main-none-eabi
    --extra-arg=-isystem --extra-arg="${TC}/arm-none-eabi/include/c++/14.2.1"
    --extra-arg=-isystem --extra-arg="${TC}/arm-none-eabi/include/c++/14.2.1/arm-none-eabi/thumb/v8-m.main+fp/softfp"
    --extra-arg=-Wno-format-security
    --extra-arg=-Wno-gnu-zero-variadic-macro-arguments
)

FAIL=0
for f in "${FILES[@]}"; do
    # honor exemptions even when files are passed explicitly (e.g. by the hook)
    case "$f" in
        src/cli/*|*eskf_codegen.cpp) continue ;;
        src/*.cpp) ;;
        *) continue ;;
    esac
    out="$("${CT}" "${REPO}/${f}" -p "${BUILD}/" --checks="${GATED}" "${EXTRA[@]}" 2>/dev/null | grep -E "${GREP_RE}")"
    if [[ -n "${out}" ]]; then
        echo "${out}"
        FAIL=1
    fi
done

# Compiler-flag coverage (per-file COMPILE_OPTIONS drop) rides the same gate — LL 43.
if ! python "${REPO}/scripts/audit/check_warning_gate_coverage.py" "${BUILD}/compile_commands.json"; then
    FAIL=1
fi

if [[ ${FAIL} -ne 0 ]]; then
    echo ""
    echo "GATED clang-tidy / coverage violations above — fix, decompose, or log an accepted deviation before commit."
    exit 1
fi
echo "full_tree_clang_tidy: clean ($([ $# -gt 0 ] && echo "${#FILES[@]} file(s)" || echo "full tree"))."
exit 0
