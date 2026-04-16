# Fault Injection: AO Queue Flood
# Publishes N dummy SIG_SENSOR_DATA events → expect queue overflow assertion
#
# Usage: arm-none-eabi-gdb build/rocketchip.elf -batch -x scripts/fault_injection/inject_queue_flood.gdb

target extended-remote localhost:3333

echo \n=== AO Queue Flood Injection ===\n

monitor halt
echo [pre] System running normally\n

echo [inject] Publishing 100 dummy events...\n
call fault_force_ao_queue_flood(0, 100)

echo [resume] Check for QP assertion (qf_actq id=130)...\n
monitor resume
shell sleep 3

monitor halt
echo [verify] System state after flood:\n
info threads

echo === Done ===\n
disconnect
quit
