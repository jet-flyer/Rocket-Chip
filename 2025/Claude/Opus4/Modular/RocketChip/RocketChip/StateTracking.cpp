// StateTracking.cpp - State machine and flight phase detection implementation
#include "StateTracking.h"
#include "SensorManager.h"
#include "DataLogger.h"
#include "SerialInterface.h"  // For DEBUG_STATUS

// State transition validation table
const uint8_t validTransitions[] = {
  // From INITIALIZING
  (1<<(int)SystemState::CALIBRATING) | (1<<(int)SystemState::ERROR),
  // From CALIBRATING
  (1<<(int)SystemState::IDLE) | (1<<(int)SystemState::ERROR),
  // From IDLE
  (1<<(int)SystemState::PREFLIGHT) | (1<<(int)SystemState::CALIBRATING) | (1<<(int)SystemState::ERROR),
  // From PREFLIGHT
  (1<<(int)SystemState::ACTIVE) | (1<<(int)SystemState::IDLE) | (1<<(int)SystemState::ERROR),
  // From ACTIVE
  (1<<(int)SystemState::IDLE) | (1<<(int)SystemState::ERROR),
  // From ERROR
  0  // No exits from ERROR state
};

StateTracking::StateTracking() :
  currentState(SystemState::INITIALIZING),
  previousState(SystemState::INITIALIZING),
  sensorMgr(nullptr),
  dataLogger(nullptr),
  launchThreshold(LAUNCH_ACCEL_THRESHOLD),
  launchDetected(false),
  launchTime(0),
  stillnessStartTime(0),
  landingDetected(false),
  tetheredMode(false),
  debugMode(DEBUG_STATUS) {
  resetFlightStats();
}

void StateTracking::begin(SensorManager* sensors, DataLogger* logger) {
  sensorMgr = sensors;
  dataLogger = logger;
}

bool StateTracking::setState(SystemState newState) {
  if (!isValidTransition(currentState, newState) && currentState != newState) {
    if (debugMode >= DEBUG_STATUS) {
      Serial.print(F("ERROR: Invalid transition "));
      Serial.print(getStateName(currentState));
      Serial.print(F(" -> "));
      Serial.println(getStateName(newState));
    }
    currentState = SystemState::ERROR;
    return false;
  }
  
  if (currentState != newState) {
    previousState = currentState;
    currentState = newState;
    
    if (debugMode >= DEBUG_STATUS) {
      Serial.print(F("State: "));
      Serial.print(getStateName(previousState));
      Serial.print(F(" -> "));
      Serial.println(getStateName(currentState));
    }
    
    // Handle state entry actions
    switch (newState) {
      case SystemState::PREFLIGHT:
        #if CAPTURE_PRELAUNCH
          if (dataLogger) {
            dataLogger->setPreflightCapture(true);
          }
        #endif
        launchDetected = false;
        flightStats.launchAltitude = sensorMgr->getAltitude();
        stillnessStartTime = 0;
        break;
        
      case SystemState::ACTIVE:
        flightStats.launchTime = millis();
        break;
        
      case SystemState::IDLE:
        if (dataLogger) {
          dataLogger->setPreflightCapture(false);
        }
        break;
        
      default:
        break;
    }
  }
  
  return true;
}

bool StateTracking::isValidTransition(SystemState from, SystemState to) const {
  int fromIdx = (int)from;
  int toIdx = (int)to;
  
  if (fromIdx < 0 || fromIdx > 5 || toIdx < 0 || toIdx > 5) {
    return false;
  }
  
  return (validTransitions[fromIdx] & (1 << toIdx)) != 0;
}

void StateTracking::update() {
  if (!sensorMgr) return;
  
  // Check if calibration is complete
  if (currentState == SystemState::CALIBRATING) {
    if (sensorMgr->getCalibrationState() == CAL_IDLE) {
      setState(SystemState::IDLE);
    }
  }
}

bool StateTracking::checkAutoLaunch() {
  if (currentState != SystemState::PREFLIGHT || !AUTO_START_ON_LAUNCH) {
    return false;
  }
  
  if (!launchDetected && sensorMgr->getTotalAccel() > launchThreshold) {
    launchDetected = true;
    launchTime = millis();
    flightStats.launchTime = launchTime;
    flightStats.launchAltitude = sensorMgr->getAltitude();
    flightStats.maxAcceleration = sensorMgr->getTotalAccel();
    flightStats.maxAltitude = flightStats.launchAltitude;
    
    setState(SystemState::ACTIVE);
    return true;
  }
  
  return false;
}

bool StateTracking::checkAutoLanding() {
  #if !AUTO_STOP_ON_LANDING
    return false;
  #endif
  
  if (currentState != SystemState::ACTIVE || !dataLogger->isLogging()) {
    return false;
  }
  
  float totalAccel = sensorMgr->getTotalAccel();
  
  // Check if we're still (near 1G)
  if (totalAccel > 0.9 && totalAccel < 1.1) {
    if (stillnessStartTime == 0) {
      stillnessStartTime = millis();
    } else if (millis() - stillnessStartTime >= LANDING_STILLNESS_TIME) {
      // We've been still for required time
      landingDetected = true;
      setState(SystemState::IDLE);
      return true;
    }
  } else {
    // Reset if we move
    stillnessStartTime = 0;
  }
  
  return false;
}

void StateTracking::updateFlightStats(const SensorData& data) {
  if (currentState == SystemState::ACTIVE && dataLogger->isLogging()) {
    if (data.totalAccel > flightStats.maxAcceleration) {
      flightStats.maxAcceleration = data.totalAccel;
    }
    if (data.altitude > flightStats.maxAltitude) {
      flightStats.maxAltitude = data.altitude;
    }
    flightStats.flightTime = millis() - flightStats.launchTime;
  }
}

void StateTracking::resetFlightStats() {
  flightStats.maxAcceleration = 0;
  flightStats.maxAltitude = 0;
  flightStats.launchAltitude = 0;
  flightStats.flightTime = 0;
  flightStats.launchTime = 0;
  launchDetected = false;
  landingDetected = false;
  stillnessStartTime = 0;
}

const char* StateTracking::getStateName() const {
  return getStateName(currentState);
}

const char* StateTracking::getStateName(SystemState state) const {
  switch (state) {
    case SystemState::INITIALIZING: return "INITIALIZING";
    case SystemState::CALIBRATING: return "CALIBRATING";
    case SystemState::IDLE: return "IDLE";
    case SystemState::PREFLIGHT: return "PREFLIGHT";
    case SystemState::ACTIVE: return "ACTIVE";
    case SystemState::ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}