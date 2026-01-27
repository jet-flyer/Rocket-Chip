/**
 * @file AP_GyroFFT.h
 * @brief Stub AP_GyroFFT for RocketChip
 *
 * Gyro FFT analysis is disabled.
 */
#pragma once

#ifndef HAL_GYROFFT_ENABLED
#define HAL_GYROFFT_ENABLED 0
#endif

#if !HAL_GYROFFT_ENABLED

class AP_GyroFFT {
public:
    static AP_GyroFFT* get_singleton() { return nullptr; }

    void init(uint32_t target_looptime_us) { (void)target_looptime_us; }
    void sample_gyros() {}
    void update() {}
    void update_freq_info(float loop_rate_hz) { (void)loop_rate_hz; }
};

#endif // !HAL_GYROFFT_ENABLED
