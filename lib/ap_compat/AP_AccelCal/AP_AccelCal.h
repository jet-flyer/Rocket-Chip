/**
 * @file AP_AccelCal.h
 * @brief Minimal AP_AccelCal_Client interface for RocketChip
 *
 * This provides just the client interface needed for calibration callbacks
 * without the full GCS-dependent AP_AccelCal class from ArduPilot.
 */

#pragma once

#include <AP_AccelCal/AccelCalibrator.h>

/**
 * @brief Interface for accelerometer calibration clients
 *
 * Implement this interface to receive calibration completion callbacks.
 * The _acal_save_calibrations() method is called when calibration succeeds.
 */
class AP_AccelCal_Client {
public:
    virtual ~AP_AccelCal_Client() = default;

    // Getters - override to provide calibration state
    virtual bool _acal_get_saving() { return false; }
    virtual bool _acal_get_ready_to_sample() { return true; }
    virtual bool _acal_get_fail() { return false; }
    virtual AccelCalibrator* _acal_get_calibrator(uint8_t instance) = 0;

    // Events - override to handle calibration lifecycle
    virtual void _acal_save_calibrations() = 0;
    virtual void _acal_event_success() {}
    virtual void _acal_event_cancellation() {}
    virtual void _acal_event_failure() {}
};
