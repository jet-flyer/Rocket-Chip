# Replay: Big Daddy F15-6 Nominal Profile (abbreviated)
# Injects key phase-transition samples via GDB call.
# Full CSV replay via serial is available but GDB is more reliable for testing.
#
# Usage: arm-none-eabi-gdb build/rocketchip.elf -batch -x scripts/fault_injection/replay_nominal.gdb

target extended-remote localhost:3333

echo \n=== Replay: Big Daddy F15 Nominal ===\n

monitor halt
echo [setup] Starting replay inject...\n
call replay_inject_start()

echo [pad] Injecting pad samples (1g down, stationary)...\n
# 10 samples at 100Hz = 0.1s of pad data
call replay_inject_sample(0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f, 101325.0f, 329500000, -967500000, 0, 1)
call replay_inject_sample(0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f, 101325.0f, 0, 0, 0, 0)
call replay_inject_sample(0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f, 101325.0f, 0, 0, 0, 0)
call replay_inject_sample(0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f, 101325.0f, 0, 0, 0, 0)
call replay_inject_sample(0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f, 101325.0f, 0, 0, 0, 0)
call replay_inject_sample(0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f, 101325.0f, 329500000, -967500000, 0, 1)
call replay_inject_sample(0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f, 101325.0f, 0, 0, 0, 0)
call replay_inject_sample(0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f, 101325.0f, 0, 0, 0, 0)
call replay_inject_sample(0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f, 101325.0f, 0, 0, 0, 0)
call replay_inject_sample(0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f, 101325.0f, 0, 0, 0, 0)

# Let ESKF initialize from stationary data
monitor resume
shell sleep 2
monitor halt
echo [check] ESKF initialized:\n
print g_eskfInitialized

echo [boost] Injecting boost samples (-40 m/s^2 = ~4g thrust)...\n
# 20 samples of high accel (boost phase)
call replay_inject_sample(0.0f, 0.0f, -40.0f, 0.0f, 0.0f, 0.0f, 101000.0f, 329500000, -967500000, 50000, 1)
call replay_inject_sample(0.0f, 0.0f, -40.0f, 0.0f, 0.0f, 0.0f, 100800.0f, 0, 0, 0, 0)
call replay_inject_sample(0.0f, 0.0f, -40.0f, 0.0f, 0.0f, 0.0f, 100600.0f, 0, 0, 0, 0)
call replay_inject_sample(0.0f, 0.0f, -40.0f, 0.0f, 0.0f, 0.0f, 100400.0f, 0, 0, 0, 0)
call replay_inject_sample(0.0f, 0.0f, -40.0f, 0.0f, 0.0f, 0.0f, 100200.0f, 0, 0, 0, 0)
call replay_inject_sample(0.0f, 0.0f, -40.0f, 0.0f, 0.0f, 0.0f, 100000.0f, 329500000, -967500000, 100000, 1)
call replay_inject_sample(0.0f, 0.0f, -40.0f, 0.0f, 0.0f, 0.0f, 99800.0f, 0, 0, 0, 0)
call replay_inject_sample(0.0f, 0.0f, -40.0f, 0.0f, 0.0f, 0.0f, 99600.0f, 0, 0, 0, 0)
call replay_inject_sample(0.0f, 0.0f, -40.0f, 0.0f, 0.0f, 0.0f, 99400.0f, 0, 0, 0, 0)
call replay_inject_sample(0.0f, 0.0f, -40.0f, 0.0f, 0.0f, 0.0f, 99200.0f, 0, 0, 0, 0)
call replay_inject_sample(0.0f, 0.0f, -40.0f, 0.0f, 0.0f, 0.0f, 99000.0f, 329500000, -967500000, 150000, 1)
call replay_inject_sample(0.0f, 0.0f, -40.0f, 0.0f, 0.0f, 0.0f, 98800.0f, 0, 0, 0, 0)
call replay_inject_sample(0.0f, 0.0f, -40.0f, 0.0f, 0.0f, 0.0f, 98600.0f, 0, 0, 0, 0)
call replay_inject_sample(0.0f, 0.0f, -40.0f, 0.0f, 0.0f, 0.0f, 98400.0f, 0, 0, 0, 0)
call replay_inject_sample(0.0f, 0.0f, -40.0f, 0.0f, 0.0f, 0.0f, 98200.0f, 0, 0, 0, 0)
call replay_inject_sample(0.0f, 0.0f, -40.0f, 0.0f, 0.0f, 0.0f, 98000.0f, 329500000, -967500000, 200000, 1)
call replay_inject_sample(0.0f, 0.0f, -40.0f, 0.0f, 0.0f, 0.0f, 97800.0f, 0, 0, 0, 0)
call replay_inject_sample(0.0f, 0.0f, -40.0f, 0.0f, 0.0f, 0.0f, 97600.0f, 0, 0, 0, 0)
call replay_inject_sample(0.0f, 0.0f, -40.0f, 0.0f, 0.0f, 0.0f, 97400.0f, 0, 0, 0, 0)
call replay_inject_sample(0.0f, 0.0f, -40.0f, 0.0f, 0.0f, 0.0f, 97200.0f, 0, 0, 0, 0)

# Let ESKF process boost data
monitor resume
shell sleep 2
monitor halt

echo [verify] Checking ESKF velocity (should be large negative = upward):\n
print g_eskf.v

echo [cleanup] Stopping replay...\n
call replay_inject_stop()

echo === Done ===\n
disconnect
quit
