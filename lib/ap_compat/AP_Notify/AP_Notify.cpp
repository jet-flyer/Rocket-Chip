/**
 * @file AP_Notify.cpp
 * @brief Minimal AP_Notify static member definitions
 *
 * This provides the static variables that ArduPilot subsystems reference
 * (flags, events). The actual LED/buzzer update logic is not implemented
 * since RocketChip uses its own notification system.
 *
 * Uses the real ArduPilot AP_Notify.h from sparse checkout for type definitions.
 */

// Define config before including header
#ifndef CONFIG_NOTIFY_DEVICES_MAX
#define CONFIG_NOTIFY_DEVICES_MAX 6
#endif

#include <AP_Notify/AP_Notify.h>

// Define static members as required by ArduPilot subsystems
AP_Notify* AP_Notify::_singleton = nullptr;
AP_Notify::notify_flags_and_values_type AP_Notify::flags = {};
AP_Notify::notify_events_type AP_Notify::events = {};

// NotifyDevice static members
NotifyDevice* AP_Notify::_devices[CONFIG_NOTIFY_DEVICES_MAX];
uint8_t AP_Notify::_num_devices = 0;
