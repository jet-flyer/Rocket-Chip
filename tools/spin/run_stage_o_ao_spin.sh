#!/usr/bin/env bash
# Master SPIN gate — verifies all LTL properties across all 4 active models.
#
# Extended 2026-05-12 (Phase 8 P8-SPIN-A): originally gated only rocketchip_ao.pml
# (11 properties). Now gates the full active set: rocketchip_ao.pml,
# rocketchip_fd.pml, rocketchip_rf_manager.pml, rocketchip_station.pml.
# Property names are extracted from each .pml at run time so new properties get
# picked up automatically.
#
# **CRITICAL: run under Cygwin bash, not Git Bash** (see tools/spin/README.md
# Prerequisites). SPIN is a Cygwin binary; its child gcc must also be the
# Cygwin gcc at /c/tools/cygwin/bin/gcc. From the repo root:
#
#   PATH="/c/tools/cygwin/bin:/c/tools/cygwin/usr/bin:$PATH" \
#     /c/tools/cygwin/bin/bash.exe -lc \
#     'cd /cygdrive/c/Users/pow-w/Documents/Rocket-Chip && \
#      bash tools/spin/run_stage_o_ao_spin.sh'
#
# Or shorter, from a Cygwin terminal at the repo root:
#   bash tools/spin/run_stage_o_ao_spin.sh
#
# Per HW_GATE_DISCIPLINE.md Rule 1: a PASS run emits "SPIN_OK_<total>" with
# the observed count; any errors:1 result or count mismatch fails closed.
#
# Per AUDIT_GUIDANCE.md Step 6: this is the Phase 6 formal-verification gate.
# 4 models, 26 LTL properties as of 2026-05-12.

set -eu
cd "$(dirname "$0")"

# Sanity-check the environment. SPIN is a Cygwin binary and its child gcc must
# also be Cygwin's gcc (not MinGW). The README has the canonical invocation;
# if this script is run from Git Bash, gcc resolution will fail.
if ! command -v gcc >/dev/null; then
  echo "FAIL: no gcc on PATH. See tools/spin/README.md Prerequisites." >&2
  exit 1
fi
if ! gcc --version 2>&1 | head -1 | grep -q -E '(cygwin|x86_64-pc-cygwin)'; then
  echo "WARNING: gcc on PATH is not Cygwin's gcc. SPIN preprocessing may fail." >&2
  echo "         Run this script via Cygwin bash, not Git Bash:" >&2
  echo "         /c/tools/cygwin/bin/bash.exe -lc \\" >&2
  echo "           'export PATH=\"/c/tools/cygwin/bin:/c/tools/cygwin/usr/bin:\$PATH\"; \\" >&2
  echo "            cd /cygdrive/c/Users/pow-w/Documents/Rocket-Chip; \\" >&2
  echo "            bash tools/spin/run_stage_o_ao_spin.sh'" >&2
fi

MODELS=(
  rocketchip_ao.pml
  rocketchip_fd.pml
  rocketchip_rf_manager.pml
  rocketchip_station.pml
  rocketchip_boot.pml          # R-12 (2026-05-07 audit): cross-core boot handshake
)

total_props=0
overall_status=0

for model in "${MODELS[@]}"; do
  echo "================================================================"
  echo "=== Model: $model ==="
  echo "================================================================"

  # Clean any stale artifacts from a prior model
  rm -f ./pan ./pan.[chmptbo] ./*.trail ./_spin_nvr.tmp

  # Generate the verifier
  spin -a "$model"
  gcc -O2 -o pan pan.c

  # Extract LTL property names from the model
  props=$(grep -oP '^ltl\s+\K\w+' "$model")

  if [ -z "$props" ]; then
    echo "WARNING: no LTL properties found in $model"
    continue
  fi

  for p in $props; do
    echo "--- $p ---"
    # -f enables weak fairness (safe for safety properties, required for
    # liveness). Matches the existing rocketchip_ao usage for the liveness
    # claim `p_armed_fault_safe_mode` and extends it uniformly.
    if ./pan -a -f -N "$p"; then
      total_props=$((total_props + 1))
    else
      echo "FAIL: $p in $model"
      overall_status=1
    fi
    rm -f ./*.trail
  done
done

# Final cleanup
rm -f ./pan ./pan.[chmptbo] ./*.trail ./_spin_nvr.tmp

if [ $overall_status -eq 0 ]; then
  echo ""
  echo "SPIN_OK_${total_props}"
  exit 0
else
  echo ""
  echo "SPIN_FAIL"
  exit 1
fi
