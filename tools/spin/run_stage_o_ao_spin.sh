#!/usr/bin/env bash
# Stage O: 8 safety properties on rocketchip_ao.pml (see README Quick Start)
set -eu
cd "$(dirname "$0")"
spin -a rocketchip_ao.pml
gcc -O2 -o pan pan.c
for p in p_no_pyro_idle p_drogue_before_main p_pyro_requires_armed \
         p_drogue_once p_main_once p_logger_gets_all p_telem_gets_all p_led_gets_all; do
  echo "=== $p ==="
  ./pan -a -N "$p"
  rm -f ./*.trail
done
# Fault / HealthMonitor LTLs (same model; added after IVP-104)
for p in p_fault_blocks_arm p_fault_latch_holds p_armed_fault_safe_mode; do
  echo "=== $p ==="
  if [ "$p" = "p_armed_fault_safe_mode" ]; then
    ./pan -a -f -N "$p"
  else
    ./pan -a -N "$p"
  fi
  rm -f ./*.trail
done
echo "SPIN_OK_ALL_11"
