# 5-minute passive soak via GDB (no serial port, no Python).
#
# Captures diag_stats globals via direct memory read at T=0 and T=300s.
# The device runs normally the whole time; GDB does minimal halt/resume.
#
# Usage: arm-none-eabi-gdb build/rocketchip.elf -batch -x scripts/soak_gdb.gdb

target extended-remote localhost:3333

echo \n=== IVP-132 5-min Soak (GDB) ===\n

monitor halt
echo [T=0] Initial state:\n
print/d 'rc::s_count'
print g_eskfInitialized
printf "IMU reads: "
print/d g_sensorSeqlock.data.imu_read_count
printf "Baro reads: "
print/d g_sensorSeqlock.data.baro_read_count
printf "GPS reads: "
print/d g_sensorSeqlock.data.gps_read_count
printf "Core1 loops: "
print/d g_sensorSeqlock.data.core1_loop_count
printf "IMU errors: "
print/d g_sensorSeqlock.data.imu_error_count

monitor resume
echo [running] 5 minute soak...\n
shell sleep 60
monitor halt
echo [T=60s]:\n
print/d g_sensorSeqlock.data.imu_read_count

monitor resume
shell sleep 60
monitor halt
echo [T=120s]:\n
print/d g_sensorSeqlock.data.imu_read_count

monitor resume
shell sleep 60
monitor halt
echo [T=180s]:\n
print/d g_sensorSeqlock.data.imu_read_count

monitor resume
shell sleep 60
monitor halt
echo [T=240s]:\n
print/d g_sensorSeqlock.data.imu_read_count

monitor resume
shell sleep 60
monitor halt
echo \n=== [T=300s] Final ===\n
print g_eskfInitialized
printf "IMU reads: "
print/d g_sensorSeqlock.data.imu_read_count
printf "Baro reads: "
print/d g_sensorSeqlock.data.baro_read_count
printf "GPS reads: "
print/d g_sensorSeqlock.data.gps_read_count
printf "Core1 loops: "
print/d g_sensorSeqlock.data.core1_loop_count
printf "IMU errors: "
print/d g_sensorSeqlock.data.imu_error_count
printf "Baro errors: "
print/d g_sensorSeqlock.data.baro_error_count
printf "GPS errors: "
print/d g_sensorSeqlock.data.gps_error_count

monitor resume
echo === Done ===\n
disconnect
quit
