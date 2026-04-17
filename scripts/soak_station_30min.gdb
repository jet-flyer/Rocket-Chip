set pagination off

target extended-remote localhost:3333

# Station soak — Core 1 is idle on station by design.
# Focus: Core 0 stability, QP queue watermarks (AO_Radio, AO_Telemetry), uptime.
# Only AO_Radio + AO_Telemetry are exported as globals on station; others are
# started differently (not pursued here — Stage 16C will clarify).
# See AGENT_WHITEBOARD Stage 16C note for station runtime shape caveats.

monitor halt
printf "=== T=0 ===\n"
print $pc
print $msp
print AO_Radio->eQueue.nMin
print AO_Radio->eQueue.nFree
print AO_Telemetry->eQueue.nMin
print AO_Telemetry->eQueue.nFree
monitor resume

shell sleep 300

monitor halt
printf "=== T=5min ===\n"
print $pc
print $msp
print AO_Radio->eQueue.nMin
print AO_Telemetry->eQueue.nMin
monitor resume

shell sleep 300

monitor halt
printf "=== T=10min ===\n"
print $pc
print $msp
print AO_Radio->eQueue.nMin
print AO_Telemetry->eQueue.nMin
monitor resume

shell sleep 300

monitor halt
printf "=== T=15min ===\n"
print $pc
print $msp
print AO_Radio->eQueue.nMin
print AO_Telemetry->eQueue.nMin
monitor resume

shell sleep 300

monitor halt
printf "=== T=20min ===\n"
print $pc
print $msp
print AO_Radio->eQueue.nMin
print AO_Telemetry->eQueue.nMin
monitor resume

shell sleep 300

monitor halt
printf "=== T=25min ===\n"
print $pc
print $msp
print AO_Radio->eQueue.nMin
print AO_Telemetry->eQueue.nMin
monitor resume

shell sleep 300

monitor halt
printf "=== T=30min ===\n"
print $pc
print $msp
print AO_Radio->eQueue.nMin
print AO_Radio->eQueue.nFree
print AO_Telemetry->eQueue.nMin
print AO_Telemetry->eQueue.nFree
monitor resume

quit
