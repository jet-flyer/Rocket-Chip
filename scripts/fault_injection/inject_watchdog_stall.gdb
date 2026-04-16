# Fault Injection: Watchdog Stall
# Skips watchdog kick for N iterations → PIO watchdog should fire
#
# Usage: arm-none-eabi-gdb build/rocketchip.elf -batch -x scripts/fault_injection/inject_watchdog_stall.gdb

target extended-remote localhost:3333

echo \n=== Watchdog Stall Injection ===\n

monitor halt
echo [pre] System running normally\n

echo [inject] Skipping watchdog kick for 1000 idle iterations...\n
call fault_force_watchdog_stall(1000)

echo [resume] PIO watchdog should detect stall within timeout...\n
monitor resume
shell sleep 8

monitor halt
echo [verify] After 8s:\n
print g_watchdogReboot

echo === Done ===\n
disconnect
quit
