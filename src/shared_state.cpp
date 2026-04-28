// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file shared_state.cpp
 * @brief Definitions for globals declared in include/rocketchip/shared_state.h (OPT-IVP-02).
 *
 * Single translation unit keeps cross-core state in one place for review and linking.
 */

#include "rocketchip/shared_state.h"

bool g_neopixelInitialized = false;
bool g_i2cInitialized = false;
bool g_imuInitialized = false;
bool g_baroInitialized = false;
bool g_baroContinuous = false;
bool g_gpsInitialized = false;
bool g_spiInitialized = false;

bool g_imuInitAttempted = false;
bool g_baroInitAttempted = false;
bool g_gpsInitAttempted = false;

size_t g_psramSize = 0;
bool g_psramSelfTestPassed = false;
bool g_psramFlashSafePassed = false;

bool g_calStorageInitialized = false;

gps_transport_t g_gpsTransport = GPS_TRANSPORT_NONE;
bool (*g_gpsFnUpdate)() = nullptr;
bool (*g_gpsFnGetData)(gps_data_t*) = nullptr;
bool (*g_gpsFnHasFix)() = nullptr;

icm20948_t g_imu{};

sensor_seqlock_t g_sensorSeqlock{};

std::atomic<bool> g_startSensorPhase{false};
std::atomic<bool> g_sensorPhaseDone{false};
std::atomic<bool> g_calReloadPending{false};
std::atomic<bool> g_core1PauseI2C{false};
std::atomic<bool> g_core1I2CPaused{false};
std::atomic<bool> g_core1LockoutReady{false};

bool g_sensorPhaseActive = false;
