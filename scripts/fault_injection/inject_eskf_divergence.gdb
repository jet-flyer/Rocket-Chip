# Fault Injection: ESKF Divergence
# Forces ESKF unhealthy → expect CR-1 reset path, health degradation
#
# Usage: arm-none-eabi-gdb build/rocketchip.elf -batch -x scripts/fault_injection/inject_eskf_divergence.gdb

target extended-remote localhost:3333

echo \n=== ESKF Divergence Injection ===\n

monitor halt
echo [pre] Capturing ESKF state...\n
print g_eskfInitialized
print g_eskf.v

echo [inject] Calling fault_force_eskf_unhealthy()...\n
call fault_force_eskf_unhealthy()

echo [post] ESKF state after injection:\n
print g_eskfInitialized

echo [resume] Continuing execution — watch for CR-1 recovery...\n
continue &
shell sleep 5
monitor halt

echo [verify] State after 5s:\n
print g_eskfInitialized

echo === Done ===\n
disconnect
quit
