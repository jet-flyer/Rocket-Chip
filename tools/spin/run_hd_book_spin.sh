#!/bin/bash
# Book half-duplex model.
#   (no defines)     every claim errors: 0
#   -DSHORT_WINDOW   peer still radiates; E44 reloads; every claim errors: 0
#   -DCODE_MISS      silent S51/S52 and a listen that ends on that silence;
#                    p_no_dual_s50 must report errors, the other four must not
#   -DCODE_E38       flight reload of Send_Duration; every claim errors: 0
#                    unless NEED_PLCW is still set at the expiry tick, which
#                    this send does not do
# pan's exit status stays 0 when a claim fails. The count is the
# number after "errors:" on the state-vector line, not field 2
# ("State-vector 48 byte, ... errors: 0").
set -u
export PATH="/c/tools/cygwin/bin:/usr/bin:$PATH"
cd "$(dirname "$0")"

fail=0

run_one() {
  local defs="$1"
  local tag="$2"
  local expect="$3"
  echo "=== $tag ==="
  rm -f pan.c pan.h pan.m pan.b pan.t pan.p pan_hd.exe *.trail
  # shellcheck disable=SC2086
  spin $defs -a proximity1_hd.pml
  gcc -O2 -o pan_hd pan.c
  local prop out log states bad
  for prop in p_no_dual_s50 p_gap_bounded p_lock_keeps_abort p_hold_not_commit p_abort_reopens; do
    echo "--- $prop ---"
    log="pan_${tag}_${prop}.txt"
    ./pan_hd -a -N "$prop" > "$log" 2>&1 || true
    cat "$log"
    out=$(sed -n 's/.*errors:[[:space:]]*\([0-9][0-9]*\).*/\1/p' "$log" | tail -n 1)
    states=$(sed -n 's/^[[:space:]]*\([0-9][0-9]*\) states, stored.*/\1/p' "$log" | tail -n 1)
    rm -f *.trail "$log"
    bad=0
    if [ -z "$out" ]; then
      echo "FAIL $tag $prop no errors count"
      bad=1
    elif [ "$expect" = pass ] && [ "$out" != "0" ]; then
      echo "FAIL $tag $prop errors=$out"
      bad=1
    elif [ "$expect" = fail ] && [ "$prop" = p_no_dual_s50 ] && [ "$out" = "0" ]; then
      echo "FAIL $tag expected an error on $prop"
      bad=1
    elif [ "$expect" = fail ] && [ "$prop" != p_no_dual_s50 ] && [ "$out" != "0" ]; then
      echo "FAIL $tag $prop errors=$out"
      bad=1
    fi
    if [ "$bad" -eq 0 ]; then
      echo "OK $tag $prop errors=$out states=${states:-?}"
    else
      fail=1
    fi
  done
  rm -f pan.c pan.h pan.m pan.b pan.t pan.p pan_hd.exe
}

run_one "" book pass
run_one "-DSHORT_WINDOW" short pass
run_one "-DCODE_E38" code_e38 pass
run_one "-DCODE_MISS" miss fail
if [ "$fail" -ne 0 ]; then
  echo "HD_BOOK_SPIN_FAIL"
  exit 1
fi
echo "HD_BOOK_SPIN_DONE"
