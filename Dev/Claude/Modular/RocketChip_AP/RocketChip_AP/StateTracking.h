// StateTracking.h - State machine and flight phase detection
#pragma once

#include <Arduino.h>
#include "config.h"

// Forward declarations to avoid circular dependencies
class SensorManager;
class DataLogger;

// ===== SYSTEM STATES =====
enum class SystemState {
  INITIALIZING,  // Power-on initialization
  CALIBRATING,   // Sensor calibration
  IDLE,          // Ready, not logging
  PREFLIGHT,     // Armed for launch detection
  ACTIVE,        // Actively logging
  ERROR          // Critical failure
};

// ===== FLIGHT STATISTICS =====
struct FlightStats {
  float maxAcceleration;
  float maxAltitude;
  float launchAltitude;
  unsigned long flightTime;
  unsigned long launchTime;
};

// ===== STATE TRACKING CLASS =====
class StateTracking {
public:
  StateTracking();
  void begin(SensorManager* sensors, DataLogger* logger);
  
  // State management
  SystemState getCurrentState() const { return currentState; }
  bool setState(SystemState newState);
  const char* getStateName() const;
  const char* getStateName(SystemState state) const;
  
  // Update and detection
  void update();
  bool checkAutoLaunch();
  bool checkAutoLanding();
  
  // Flight statistics
  void updateFlightStats(const struct SensorData& data);
  const FlightStats& getFlightStats() const { return flightStats; }
  void resetFlightStats();
  
  // Configuration
  void setLaunchThreshold(float g) { launchThreshold = g; }
  float getLaunchThreshold() const { return launchThreshold; }
  
  // Test mode
  void setTetheredMode(bool enabled) { tetheredMode = enabled; }
  bool isTetheredMode() const { return tetheredMode; }
  
  // Debug
  void setDebugMode(uint8_t mode) { debugMode = mode; }
  
private:
  // State
  SystemState currentState;
  SystemState previousState;
  
  // References to other modules
  SensorManager* sensorMgr;
  DataLogger* dataLogger;
  
  // Launch detection
  float launchThreshold;
  bool launchDetected;
  unsigned long launchTime;
  
  // Landing detection
  unsigned long stillnessStartTime;
  bool landingDetected;
  
  // Flight statistics
  FlightStats flightStats;
  
  // Configuration
  bool tetheredMode;
  uint8_t debugMode;
  
  // State validation
  bool isValidTransition(SystemState from, SystemState to) const;
};