set pagination off
set logging overwrite on
set logging file logs/soak_station_30min.log
set logging on

# IVP-132a.4 station soak (council-reviewed).
# T=0 preconditions MUST pass before timed phase. Each snapshot captures
# per-AO ring indices (activity proxy) + peripheral state + SPI errors.

target extended-remote localhost:3333
monitor halt

# --------------------------------------------------------------------
# T=0 PRECONDITIONS (hard-fail if anything wrong)
# --------------------------------------------------------------------
printf "\n========== T=0 PRECONDITIONS ==========\n"
call diag_stats_t0_preconditions()

# Direct register reads (in case printf didn't flush to serial)
printf "\n[GDB-direct readbacks]\n"
printf "rfm95w RegVersion: 0x%x\n", rfm95w_read_version(10)
printf "kBuildForFlight:   %d\n", (int)kBuildForFlight
printf "g_spi_error_count: %lu\n", (unsigned long)g_spi_error_count._M_i

# Radio AO state
print ((RadioAo*)AO_Radio)->state.initialized
print ((RadioAo*)AO_Radio)->state.radio.initialized
print ((RadioAo*)AO_Radio)->state.scheduler.phase
print ((RadioAo*)AO_Radio)->state.radio.cs_pin
print ((RadioAo*)AO_Radio)->state.radio.rst_pin
print ((RadioAo*)AO_Radio)->state.radio.irq_pin
print ((RadioAo*)AO_Radio)->state.rx_count

printf "========================================\n"
monitor resume

# --------------------------------------------------------------------
# TIMED PHASE — 30 min, snapshot every 5 min
# Each snapshot: $pc / $msp / AO queue ring indices / rx_count /
#                spi_error_count. Ring-index change = AO activity.
# --------------------------------------------------------------------

shell sleep 300
monitor halt
printf "\n===== T=5min =====\n"
print $pc
print $msp
print AO_Radio->eQueue.head
print AO_Radio->eQueue.tail
print AO_Radio->eQueue.nMin
print ((RadioAo*)AO_Radio)->state.rx_count
print ((RadioAo*)AO_Radio)->state.rx_crc_errors
print AO_Telemetry->eQueue.head
print AO_Telemetry->eQueue.tail
print AO_Telemetry->eQueue.nMin
print g_spi_error_count._M_i
monitor resume

shell sleep 300
monitor halt
printf "\n===== T=10min =====\n"
print $pc
print $msp
print AO_Radio->eQueue.head
print AO_Radio->eQueue.tail
print AO_Radio->eQueue.nMin
print ((RadioAo*)AO_Radio)->state.rx_count
print ((RadioAo*)AO_Radio)->state.rx_crc_errors
print AO_Telemetry->eQueue.head
print AO_Telemetry->eQueue.tail
print AO_Telemetry->eQueue.nMin
print g_spi_error_count._M_i
monitor resume

shell sleep 300
monitor halt
printf "\n===== T=15min =====\n"
print $pc
print $msp
print AO_Radio->eQueue.head
print AO_Radio->eQueue.tail
print AO_Radio->eQueue.nMin
print ((RadioAo*)AO_Radio)->state.rx_count
print ((RadioAo*)AO_Radio)->state.rx_crc_errors
print AO_Telemetry->eQueue.head
print AO_Telemetry->eQueue.tail
print AO_Telemetry->eQueue.nMin
print g_spi_error_count._M_i
monitor resume

shell sleep 300
monitor halt
printf "\n===== T=20min =====\n"
print $pc
print $msp
print AO_Radio->eQueue.head
print AO_Radio->eQueue.tail
print AO_Radio->eQueue.nMin
print ((RadioAo*)AO_Radio)->state.rx_count
print ((RadioAo*)AO_Radio)->state.rx_crc_errors
print AO_Telemetry->eQueue.head
print AO_Telemetry->eQueue.tail
print AO_Telemetry->eQueue.nMin
print g_spi_error_count._M_i
monitor resume

shell sleep 300
monitor halt
printf "\n===== T=25min =====\n"
print $pc
print $msp
print AO_Radio->eQueue.head
print AO_Radio->eQueue.tail
print AO_Radio->eQueue.nMin
print ((RadioAo*)AO_Radio)->state.rx_count
print ((RadioAo*)AO_Radio)->state.rx_crc_errors
print AO_Telemetry->eQueue.head
print AO_Telemetry->eQueue.tail
print AO_Telemetry->eQueue.nMin
print g_spi_error_count._M_i
monitor resume

shell sleep 300
monitor halt
printf "\n===== T=30min =====\n"
print $pc
print $msp
print AO_Radio->eQueue.head
print AO_Radio->eQueue.tail
print AO_Radio->eQueue.nMin
print AO_Radio->eQueue.nFree
print ((RadioAo*)AO_Radio)->state.rx_count
print ((RadioAo*)AO_Radio)->state.rx_crc_errors
print AO_Telemetry->eQueue.head
print AO_Telemetry->eQueue.tail
print AO_Telemetry->eQueue.nMin
print AO_Telemetry->eQueue.nFree
print g_spi_error_count._M_i
printf "\n[Final T=0 block for comparison]\n"
call diag_stats_t0_preconditions()
monitor resume

set logging off
quit
