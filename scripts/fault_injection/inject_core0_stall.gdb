# Fault Injection: Core 0 Stall
# Stalls the idle bridge → watchdog/PIO timers should fire
#
# Usage: arm-none-eabi-gdb build/rocketchip.elf -batch -x scripts/fault_injection/inject_core0_stall.gdb

target extended-remote localhost:3333

echo \n=== Core 0 Stall Injection ===\n

monitor halt
echo [pre] System running normally\n

echo [inject] Setting g_fault_core0_stall = true...\n
call fault_force_core0_stall()

echo [resume] Core 0 idle bridge will spin — watchdog should fire in ~5s...\n
monitor resume
shell sleep 8

monitor halt
echo [verify] After 8s — check if watchdog rebooted:\n
print g_watchdogReboot

echo [cleanup] Clearing stall flag...\n
call fault_force_core0_stall_clear()

echo === Done ===\n
disconnect
quit
