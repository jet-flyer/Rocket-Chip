# Fault Injection: PIO State Machine Halt
# Disables PIO2 backup timer SMs → tests "who watches the watchman"
#
# Usage: arm-none-eabi-gdb build/rocketchip.elf -batch -x scripts/fault_injection/inject_pio_sm_halt.gdb

target extended-remote localhost:3333

echo \n=== PIO SM Halt Injection ===\n

monitor halt
echo [pre] Reading PIO2 CTRL register...\n
print/x *0x50300000

echo [inject] Disabling all PIO2 state machines...\n
call fault_force_pio_sm_halt()

echo [post] PIO2 CTRL after halt:\n
print/x *0x50300000

echo [resume] Backup timers disabled — observe if any upper-layer monitor detects this...\n
monitor resume
shell sleep 10

monitor halt
echo [verify] System state after 10s:\n
info threads

echo === Done ===\n
disconnect
quit
