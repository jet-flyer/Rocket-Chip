set pagination off

target extended-remote localhost:3333

monitor halt
printf "=== T=0 ===\n"
print g_sensorSeqlock.data.imu_read_count
print g_sensorSeqlock.data.baro_read_count
print g_sensorSeqlock.data.gps_read_count
print g_sensorSeqlock.data.core1_loop_count
print g_sensorSeqlock.data.imu_error_count
print g_sensorSeqlock.data.baro_error_count
print $msp
monitor resume

shell sleep 300

monitor halt
printf "=== T=5min ===\n"
print g_sensorSeqlock.data.imu_read_count
print g_sensorSeqlock.data.baro_read_count
print g_sensorSeqlock.data.core1_loop_count
print g_sensorSeqlock.data.imu_error_count
print g_sensorSeqlock.data.baro_error_count
print $msp
print $pc
monitor resume

shell sleep 300

monitor halt
printf "=== T=10min ===\n"
print g_sensorSeqlock.data.imu_read_count
print g_sensorSeqlock.data.baro_read_count
print g_sensorSeqlock.data.core1_loop_count
print g_sensorSeqlock.data.imu_error_count
print g_sensorSeqlock.data.baro_error_count
print $msp
print $pc
monitor resume

shell sleep 300

monitor halt
printf "=== T=15min ===\n"
print g_sensorSeqlock.data.imu_read_count
print g_sensorSeqlock.data.baro_read_count
print g_sensorSeqlock.data.core1_loop_count
print g_sensorSeqlock.data.imu_error_count
print g_sensorSeqlock.data.baro_error_count
print $msp
print $pc
monitor resume

shell sleep 300

monitor halt
printf "=== T=20min ===\n"
print g_sensorSeqlock.data.imu_read_count
print g_sensorSeqlock.data.baro_read_count
print g_sensorSeqlock.data.core1_loop_count
print g_sensorSeqlock.data.imu_error_count
print g_sensorSeqlock.data.baro_error_count
print $msp
print $pc
monitor resume

shell sleep 300

monitor halt
printf "=== T=25min ===\n"
print g_sensorSeqlock.data.imu_read_count
print g_sensorSeqlock.data.baro_read_count
print g_sensorSeqlock.data.core1_loop_count
print g_sensorSeqlock.data.imu_error_count
print g_sensorSeqlock.data.baro_error_count
print $msp
print $pc
monitor resume

shell sleep 300

monitor halt
printf "=== T=30min ===\n"
print g_sensorSeqlock.data.imu_read_count
print g_sensorSeqlock.data.baro_read_count
print g_sensorSeqlock.data.gps_read_count
print g_sensorSeqlock.data.core1_loop_count
print g_sensorSeqlock.data.imu_error_count
print g_sensorSeqlock.data.baro_error_count
print $msp
print $pc
print g_sensorSeqlock.data.last_imu_temp_c
print g_sensorSeqlock.data.last_baro_temp_c
monitor resume

quit
