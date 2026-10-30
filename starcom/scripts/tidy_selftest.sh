#!/usr/bin/env bash
# Known-bad probe for starcom/.clang-tidy. The probe is not a build target.
set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STARCOM="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROBE="${STARCOM}/tests/tidy_probe"
CT="${CLANG_TIDY:-C:/Program Files/LLVM/bin/clang-tidy.exe}"
CONFIG="${CONFIG:-${STARCOM}/.clang-tidy}"

if [[ ! -f "${CT}" ]]; then
  echo "tidy_selftest: clang-tidy not found at ${CT}"
  exit 2
fi

# A relative CONFIG is from the caller's cwd. Resolve it before the probe cd.
if [[ "${CONFIG}" != /* && ! "${CONFIG}" =~ ^[A-Za-z]: ]]; then
  CONFIG="$(cd "$(dirname "${CONFIG}")" && pwd)/$(basename "${CONFIG}")"
fi

ver_out="$("${CT}" --version 2>&1 || true)"
found="unknown"
if [[ "${ver_out}" =~ ([0-9]+\.[0-9]+\.[0-9]+) ]]; then
  found="${BASH_REMATCH[1]}"
fi
if [[ ! "${found}" =~ ^21\.1\.[0-9]+$ ]]; then
  echo "tidy_selftest: needs clang-tidy 21.1.x, found ${found}"
  exit 2
fi

# Absolute -I so the header path has a slash before third_party.
# The exclude regex requires that slash. A relative -Ithird_party/include
# prints third_party/include\... and the vendor header leaks.
diag="$(
  cd "${PROBE}"
  "${CT}" tidy_probe.cpp --config-file="${CONFIG}" --quiet -- \
    -std=c++20 \
    -I"${PROBE}/starcom/include" \
    -I"${PROBE}/third_party/include" 2>&1 || true
)"

warns=""
vendor_hit=0
while IFS= read -r line || [[ -n "${line}" ]]; do
  line="${line%$'\r'}"
  if [[ "${line}" =~ ^(.+):([0-9]+):[0-9]+:[[:space:]]+(warning|error|note): ]]; then
    path="${BASH_REMATCH[1]//\\//}"
    base="${path##*/}"
    if [[ "${base}" == "probe_vendor.hpp" ]]; then
      vendor_hit=1
    fi
  fi
  if [[ "${line}" =~ ^(.+):([0-9]+):[0-9]+:[[:space:]]+warning:.*\[([^]]+)\][[:space:]]*$ ]]; then
    path="${BASH_REMATCH[1]//\\//}"
    base="${path##*/}"
    ln="${BASH_REMATCH[2]}"
    checks="${BASH_REMATCH[3]}"
    warns+="${base}|${ln}|${checks}"$'\n'
  fi
done <<< "${diag}"

tags=""
collect_expects() {
  local file="$1"
  local base n line
  base="$(basename "${file}")"
  n=0
  while IFS= read -r line || [[ -n "${line}" ]]; do
    n=$((n + 1))
    line="${line%$'\r'}"
    # EXPECT-NOT has no space after EXPECT, so this pattern skips it.
    if [[ "${line}" =~ //[[:space:]]*EXPECT[[:space:]]+([A-Za-z0-9._-]+) ]]; then
      tags+="${base}|${n}|${BASH_REMATCH[1]}"$'\n'
    fi
  done < "${file}"
}

collect_expects "${PROBE}/tidy_probe.cpp"
collect_expects "${PROBE}/starcom/include/probe_hit.hpp"

if [[ -z "${tags}" ]]; then
  echo "tidy_selftest: no EXPECT tags"
  exit 1
fi

check_fired() {
  local want_base="$1" want_line="$2" want_check="$3"
  local w b rest ln checks padded
  while IFS= read -r w; do
    [[ -z "${w}" ]] && continue
    b="${w%%|*}"
    rest="${w#*|}"
    ln="${rest%%|*}"
    checks="${rest#*|}"
    if [[ "${b}" == "${want_base}" && "${ln}" == "${want_line}" ]]; then
      padded=",${checks// /},"
      if [[ "${padded}" == *",${want_check},"* ]]; then
        return 0
      fi
    fi
  done <<< "${warns}"
  return 1
}

pass=0
fail=0
total=0
while IFS= read -r tag; do
  [[ -z "${tag}" ]] && continue
  base="${tag%%|*}"
  rest="${tag#*|}"
  ln="${rest%%|*}"
  check="${rest#*|}"
  total=$((total + 1))
  if check_fired "${base}" "${ln}" "${check}"; then
    echo "PASS ${base}:${ln} ${check}"
    pass=$((pass + 1))
  else
    echo "FAIL ${base}:${ln} ${check}"
    fail=$((fail + 1))
  fi
done <<< "${tags}"

total=$((total + 1))
if [[ "${vendor_hit}" -eq 0 ]]; then
  echo "PASS EXPECT-NOT probe_vendor.hpp"
  pass=$((pass + 1))
else
  echo "FAIL EXPECT-NOT probe_vendor.hpp"
  fail=$((fail + 1))
fi

list_out="$("${CT}" --list-checks --config-file="${CONFIG}" 2>&1 || true)"
count=0
while IFS= read -r line || [[ -n "${line}" ]]; do
  line="${line%$'\r'}"
  if [[ "${line}" =~ ^[[:space:]]+[^[:space:]] ]]; then
    count=$((count + 1))
  fi
done <<< "${list_out}"

total=$((total + 1))
if [[ "${count}" -ge 169 ]]; then
  echo "PASS check-count ${count}"
  pass=$((pass + 1))
else
  echo "FAIL check-count ${count}"
  fail=$((fail + 1))
fi

# Rocket-Chip root .clang-tidy: it had the same comment fault. Check its
# check count and the analyzer tags in tidy_probe.cpp. Skip when Starcom is
# not inside the Rocket-Chip tree (no root config).
ROOT_CONFIG="${ROOT_CONFIG:-${STARCOM}/../.clang-tidy}"
ROOT_MIN="${ROOT_MIN:-163}"
if [[ -f "${ROOT_CONFIG}" ]]; then
  root_diag="$(
    cd "${PROBE}"
    "${CT}" tidy_probe.cpp --config-file="${ROOT_CONFIG}" --quiet -- \
      -std=c++20 \
      -I"${PROBE}/starcom/include" \
      -I"${PROBE}/third_party/include" 2>&1 || true
  )"
  starcom_warns="${warns}"
  warns=""
  while IFS= read -r line || [[ -n "${line}" ]]; do
    line="${line%$'\r'}"
    if [[ "${line}" =~ ^(.+):([0-9]+):[0-9]+:[[:space:]]+warning:.*\[([^]]+)\][[:space:]]*$ ]]; then
      path="${BASH_REMATCH[1]//\\//}"
      warns+="${path##*/}|${BASH_REMATCH[2]}|${BASH_REMATCH[3]}"$'\n'
    fi
  done <<< "${root_diag}"
  while IFS= read -r tag; do
    [[ -z "${tag}" ]] && continue
    base="${tag%%|*}"
    rest="${tag#*|}"
    ln="${rest%%|*}"
    check="${rest#*|}"
    [[ "${base}" == "tidy_probe.cpp" && "${check}" == clang-analyzer-core.* ]] || continue
    total=$((total + 1))
    if check_fired "${base}" "${ln}" "${check}"; then
      echo "PASS root ${base}:${ln} ${check}"
      pass=$((pass + 1))
    else
      echo "FAIL root ${base}:${ln} ${check}"
      fail=$((fail + 1))
    fi
  done <<< "${tags}"
  warns="${starcom_warns}"
  root_list="$("${CT}" --list-checks --config-file="${ROOT_CONFIG}" 2>&1 || true)"
  rcount=0
  while IFS= read -r line || [[ -n "${line}" ]]; do
    line="${line%$'\r'}"
    if [[ "${line}" =~ ^[[:space:]]+[^[:space:]] ]]; then
      rcount=$((rcount + 1))
    fi
  done <<< "${root_list}"
  total=$((total + 1))
  if [[ "${rcount}" -ge "${ROOT_MIN}" ]]; then
    echo "PASS root check-count ${rcount}"
    pass=$((pass + 1))
  else
    echo "FAIL root check-count ${rcount} (need ${ROOT_MIN})"
    fail=$((fail + 1))
  fi
else
  echo "NOTE root config not found (${ROOT_CONFIG}); root checks skipped"
fi

if [[ "${fail}" -eq 0 ]]; then
  echo "tidy_selftest: ${pass}/${total} PASS"
  exit 0
fi
echo "tidy_selftest: FAIL (${fail} of ${total})"
exit 1
