/**
 * @file AP_Notify.h
 * @brief Stub AP_Notify for RocketChip
 *
 * LED/buzzer notifications - stub interface with minimal flags.
 */
#pragma once

#include <cstdint>

class AP_Notify {
public:
    // Notification flags (used by ArduPilot subsystems)
    struct notify_flags_type {
        bool initialising;
        bool calibrating;
        bool armed;
        bool pre_arm_check;
        bool save_trim;
        bool esc_calibration;
        bool failsafe_battery;
        bool failsafe_gcs;
        bool failsafe_radio;
        bool parachute_release;
        bool ekf_bad;
        bool autopilot_mode;
        bool gps_status;
        bool gps_num_sats;
        bool gps_fusion;
        bool have_pos_abs;
        bool vehicle_lost;
        bool compass_cal_running;
        bool gyro_calibrated;
        bool leak_detected;
        bool firmware_update;
        bool waiting_for_throw;
        bool powering_off;
    };

    static AP_Notify* get_singleton() { return _singleton; }

    void init() {}
    void update() {}

    // Static flags (ArduPilot code accesses AP_Notify::flags directly)
    static notify_flags_type flags;

    // Singleton instance
    static AP_Notify* _singleton;
};

// Event notify functions - stub implementations
inline void AP_Notify_events_set_armed(bool armed) { (void)armed; }
inline void AP_Notify_events_send_flight_mode_change() {}
