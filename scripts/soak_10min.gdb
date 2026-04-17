set pagination off
set print pretty on

target extended-remote localhost:3333

# T=0
monitor halt
printf "=== T=0 ===\n"
print g_sensorSeqlock.data.imu_read_count
print g_sensorSeqlock.data.baro_read_count
print g_sensorSeqlock.data.gps_read_count
print g_sensorSeqlock.data.core1_loop_count
print g_sensorSeqlock.data.imu_error_count
print g_sensorSeqlock.data.baro_error_count
print g_sensorSeqlock.data.last_ax
print g_sensorSeqlock.data.last_ay
print g_sensorSeqlock.data.last_az
monitor resume

shell sleep 120

monitor halt
printf "=== T=120 ===\n"
print g_sensorSeqlock.data.imu_read_count
print g_sensorSeqlock.data.baro_read_count
print g_sensorSeqlock.data.core1_loop_count
print g_sensorSeqlock.data.imu_error_count
print g_sensorSeqlock.data.baro_error_count
monitor resume

shell sleep 120

monitor halt
printf "=== T=240 ===\n"
print g_sensorSeqlock.data.imu_read_count
print g_sensorSeqlock.data.baro_read_count
print g_sensorSeqlock.data.core1_loop_count
print g_sensorSeqlock.data.imu_error_count
print g_sensorSeqlock.data.baro_error_count
monitor resume

shell sleep 120

monitor halt
printf "=== T=360 ===\n"
print g_sensorSeqlock.data.imu_read_count
print g_sensorSeqlock.data.baro_read_count
print g_sensorSeqlock.data.core1_loop_count
print g_sensorSeqlock.data.imu_error_count
print g_sensorSeqlock.data.baro_error_count
monitor resume

shell sleep 120

monitor halt
printf "=== T=480 ===\n"
print g_sensorSeqlock.data.imu_read_count
print g_sensorSeqlock.data.baro_read_count
print g_sensorSeqlock.data.core1_loop_count
print g_sensorSeqlock.data.imu_error_count
print g_sensorSeqlock.data.baro_error_count
monitor resume

shell sleep 120

monitor halt
printf "=== T=600 ===\n"
print g_sensorSeqlock.data.imu_read_count
print g_sensorSeqlock.data.baro_read_count
print g_sensorSeqlock.data.gps_read_count
print g_sensorSeqlock.data.core1_loop_count
print g_sensorSeqlock.data.imu_error_count
print g_sensorSeqlock.data.baro_error_count
print g_sensorSeqlock.data.last_ax
print g_sensorSeqlock.data.last_ay
print g_sensorSeqlock.data.last_az
monitor resume

quit
