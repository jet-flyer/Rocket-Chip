# OPT-IVP-01 Row 10: dual-core / MPU clarification run (narrower disturbance than staggered Tier-1 soak halts).
#
# Replaces Tier-1 soak periodic halts (every 60 s) with ONE halt after passive run.
# Rationale from docs/BENCH_TEST_PROCEDURE.md: halting mid-tight-codegen can leave bogus
# fault context; GDB also reports BOTH HardFault AND MemManage through the SAME C symbol
# (memmanage_fault_handler) because init_early_hw() maps both vectors to one handler —
# seeing "Handler HardFault" is NOT proof MemManage/stack guard fired.
#
# Prerequisites: Pico SDK OpenOCD listening on localhost:3333; bench ELF current.
#
# Usage (repo root):
#   arm-none-eabi-gdb build\rocketchip.elf -batch -x scripts/opt_ivp01_row10_dualcore_watch.gdb
#
# PASS (human + log):
# - OpenOCD lines show rp2350.cm0 and rp2350.cm1 in Thread (or IDLE app code), not stuck in Handler.
# - CFSR / HFSR reads below are ~0 OR only sticky bits cleared by subsequent fault path review.
#
# Typical wall time ~3 min (depends on GDB shell sleep precision).

target extended-remote localhost:3333

echo \n=== OPT-IVP-01 Row 10: dual-core passive watch ===\n

monitor halt
monitor reset halt
echo [load] Program fresh image (dual-core deterministic resume per DEBUG_PROBE / FLASHING)...\n
load

monitor resume
echo [run] Passive 140s — no intermediate halts (avoid periodic halt artefacts)...\n
shell sleep 140

monitor halt
echo === After passive run: SCB sticky fault registers (view as hex) ===\n
echo SCB.CFSR 0xE000ED28 (MMFSR/BFSR/UFSR):\n
x/1xw 0xE000ED28

echo SCB.HFSR 0xE000ED2C:\n
x/1xw 0xE000ED2C

echo SCB.SHCSR 0xE000ED24:\n
x/1xw 0xE000ED24

echo GDB current-thread PC:\n
print $pc

echo === Interpreter: read OpenOCD lines above for [rp2350.cm0]/[rp2350.cm1] modes ===\n
echo PASS if cores run application threads; FAIL if perpetual Handler + non-zero sticky fault — investigate MPU/stack.\n

monitor resume
echo === Done (target running). Log this output for docs/baselines/ row 10. ===\n
disconnect
quit
