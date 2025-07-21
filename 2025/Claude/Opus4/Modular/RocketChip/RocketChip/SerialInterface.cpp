// SerialInterface.cpp - User command handling and output implementation
#include "SerialInterface.h"
#include "SensorManager.h"
#include "DataLogger.h"
#include "StateTracking.h"

SerialInterface::SerialInterface() :
  sensorMgr(nullptr),
  dataLogger(nullptr),
  stateTracker(nullptr),
  debugMode(DEBUG_STATUS),
  tetheredModeActive(false),
  testAccelValue(1.0),
  lastDataOutput(0),
  lastStatusPrint(0) {
}

void SerialInterface::begin() {
  // Check if serial is connected
  if (SERIAL_DEBUG) {
    unsigned long startTime = millis();
    while (!Serial && (millis() - startTime < 3000)) {
      delay(10);
    }
    
    if (Serial) {
      // Auto-enable tethered mode when USB connected
      debugMode = DEBUG_TETHERED;
      tetheredModeActive = true;
      printWelcomeMessage();
      Serial.println(F(">>> TETHERED MODE - Type 'h' for help\n"));
    } else {
      debugMode = DEBUG_OFF;
    }
  }
}

void SerialInterface::setModules(SensorManager* sensors, DataLogger* logger, StateTracking* tracker) {
  sensorMgr = sensors;
  dataLogger = logger;
  stateTracker = tracker;
  
  // Sync debug modes
  if (sensorMgr) sensorMgr->setDebugMode(debugMode);
  if (dataLogger) dataLogger->setDebugMode(debugMode);
  if (stateTracker) stateTracker->setDebugMode(debugMode);
}

void SerialInterface::update() {
  // Check for calibration input FIRST (before consuming serial data)
  if (sensorMgr && sensorMgr->isWaitingForInput() && Serial.available()) {
    // Just check for any input (Enter key) and consume the line
    String input = Serial.readStringUntil('\n');
    sensorMgr->handleCalibrationInput();
    return;  // Don't process as command
  }
  
  // Handle serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.length() > 0) {
      handleCommand(command.charAt(0), command);
    }
  }
  
  // Output verbose data if enabled
  if (debugMode == DEBUG_VERBOSE && (millis() - lastDataOutput >= (1000 / VERBOSE_DATA_RATE))) {
    lastDataOutput = millis();
    outputVerboseData();
  }
  
  // Print periodic status
  if (debugMode == DEBUG_STATUS && stateTracker) {
    SystemState state = stateTracker->getCurrentState();
    if (state != SystemState::IDLE && state != SystemState::CALIBRATING && 
        (millis() - lastStatusPrint >= 5000)) {
      lastStatusPrint = millis();
      printStatus();
    }
  }
}

void SerialInterface::handleCommand(char cmd, const String& fullCommand) {
  switch (cmd) {
    case 's':
    case 'S':
      handleStartStop();
      break;
      
    case 'p':
    case 'P':
      handlePreflight();
      break;
      
    case 'c':
      handleCalibration(CalPhase::PHASE_GYRO_ONLY);
      break;
      
    case 'C':
      handleCalibration(CalPhase::PHASE_FULL_CAL);
      break;
      
    case 'A':
      handleCalibration(CalPhase::PHASE_ACCEL_ONLY);
      break;
      
    case 'M':
      handleCalibration(CalPhase::PHASE_MAG_ONLY);
      break;
      
    case 'd':
      handleDebugMode(fullCommand);
      break;
      
    case 't':
    case 'T':
      handleTestAccel();
      break;
      
    case 'm':
      handleMemoryInfo();
      break;
      
    case 'X':
      handleFactoryReset();
      break;
      
    case 'h':
    case 'H':
      printHelp();
      break;
      
    default:
      Serial.print(F("Unknown command: '"));
      Serial.print(fullCommand);
      Serial.println(F("'. Type 'h' for help."));
      break;
  }
}

void SerialInterface::handleStartStop() {
  if (!stateTracker || !dataLogger) return;
  
  SystemState state = stateTracker->getCurrentState();
  
  if (state == SystemState::IDLE || state == SystemState::ACTIVE || state == SystemState::PREFLIGHT) {
    if (!dataLogger->isLogging()) {
      // Start logging
      if (state == SystemState::IDLE) {
        // Manual start from IDLE - go through PREFLIGHT then ACTIVE
        stateTracker->setState(SystemState::PREFLIGHT);
        stateTracker->setState(SystemState::ACTIVE);
        dataLogger->startLogging();
        Serial.println(F(">>> MANUAL LOGGING STARTED"));
      } else if (state == SystemState::PREFLIGHT) {
        // Start from PREFLIGHT
        stateTracker->setState(SystemState::ACTIVE);
        dataLogger->startLogging();
      }
    } else {
      // Stop logging
      dataLogger->stopLogging();
      stateTracker->setState(SystemState::IDLE);
      printFlightSummary(stateTracker->getFlightStats());
      stateTracker->resetFlightStats();
    }
  }
}

void SerialInterface::handlePreflight() {
  if (!stateTracker) return;
  
  SystemState state = stateTracker->getCurrentState();
  
  if (state == SystemState::IDLE) {
    stateTracker->setState(SystemState::PREFLIGHT);
    Serial.println(F(">>> PREFLIGHT MODE"));
    #if CAPTURE_PRELAUNCH
      Serial.print(F("    Pre-buffer ON, launch>"));
    #else
      Serial.print(F("    Launch>"));
    #endif
    Serial.print(stateTracker->getLaunchThreshold());
    Serial.println(F("G"));
  } else if (state == SystemState::PREFLIGHT) {
    stateTracker->setState(SystemState::IDLE);
    Serial.println(F(">>> IDLE MODE"));
  }
}

void SerialInterface::handleCalibration(CalPhase phase) {
  if (!stateTracker || !sensorMgr) return;
  
  if (stateTracker->getCurrentState() == SystemState::IDLE) {
    stateTracker->setState(SystemState::CALIBRATING);
    sensorMgr->startCalibration(phase);
  } else {
    Serial.println(F("Must be in IDLE mode to calibrate"));
  }
}

void SerialInterface::handleDebugMode(const String& command) {
  if (command.length() == 1) {
    // Just 'd' - disable tethered mode
    if (tetheredModeActive) {
      tetheredModeActive = false;
      setDebugMode(DEBUG_STATUS);
      Serial.println(F(">>> TETHERED MODE DISABLED"));
    } else {
      Serial.println(F("Tethered mode is not active"));
    }
  } else {
    // Extract mode number
    int mode = command.charAt(1) - '0';
    if (mode >= 0 && mode <= 3) {
      setDebugMode((DebugMode)mode);
    } else {
      Serial.println(F("Invalid mode. Use d0-d3"));
    }
  }
}

void SerialInterface::setDebugMode(DebugMode mode) {
  debugMode = mode;
  
  // Update tethered mode flag
  tetheredModeActive = (mode == DEBUG_TETHERED);
  if (stateTracker) {
    stateTracker->setTetheredMode(tetheredModeActive);
  }
  
  // Sync with other modules
  if (sensorMgr) sensorMgr->setDebugMode(mode);
  if (dataLogger) dataLogger->setDebugMode(mode);
  if (stateTracker) stateTracker->setDebugMode(mode);
  
  Serial.print(F("Debug mode set to: "));
  switch (debugMode) {
    case DEBUG_OFF:
      Serial.println(F("OFF"));
      break;
    case DEBUG_STATUS:
      Serial.println(F("STATUS"));
      break;
    case DEBUG_VERBOSE:
      Serial.println(F("VERBOSE (10Hz)"));
      break;
    case DEBUG_TETHERED:
      Serial.println(F("TETHERED MODE"));
      break;
  }
}

void SerialInterface::handleTestAccel() {
  if (!stateTracker || !sensorMgr) return;
  
  if (debugMode == DEBUG_TETHERED && stateTracker->getCurrentState() == SystemState::PREFLIGHT) {
    testAccelValue = stateTracker->getLaunchThreshold() + 1.0;
    sensorMgr->setTestAcceleration(testAccelValue);
    Serial.print(F(">>> TEST: Simulating "));
    Serial.print(testAccelValue);
    Serial.println(F("G acceleration!"));
  } else {
    Serial.println(F("Tethered mode must be active and in PREFLIGHT state"));
  }
}

void SerialInterface::handleMemoryInfo() {
  Serial.println(F("\nMEMORY:"));
  
  // Filesystem info
  FSInfo fs_info;
  if (LittleFS.info(fs_info)) {
    Serial.print(F("Flash: "));
    Serial.print(fs_info.usedBytes * 100 / fs_info.totalBytes);
    Serial.print(F("% ("));
    Serial.print(fs_info.usedBytes / 1024);
    Serial.print(F("/"));
    Serial.print(fs_info.totalBytes / 1024);
    Serial.println(F("KB)"));
  }
  
  Serial.print(F("RAM: "));
  Serial.print(rp2040.getFreeHeap() / 1024);
  Serial.print(F("/"));
  Serial.print(rp2040.getTotalHeap() / 1024);
  Serial.println(F("KB free"));
  
  if (dataLogger) {
    uint32_t bufferUsed, bufferSize;
    dataLogger->getMemoryInfo(bufferUsed, bufferSize);
    Serial.print(F("Buffer: "));
    Serial.print(bufferUsed * 100 / bufferSize);
    Serial.print(F("% ("));
    Serial.print(bufferUsed / 1024);
    Serial.print(F("KB/"));
    Serial.print(bufferSize / (1024 * 1024));
    Serial.print(F("MB)"));
    
    if (dataLogger->isLogging()) {
      uint32_t secondsRemaining = dataLogger->getSecondsRemaining();
      Serial.print(F(" ~"));
      Serial.print(secondsRemaining / 60);
      Serial.print(F("m left"));
    }
    Serial.println();
  }
  Serial.println();
}

void SerialInterface::handleFactoryReset() {
  Serial.println(F(">>> DELETE ALL DATA? Type 'YES':"));
  
  String response = "";
  unsigned long timeout = millis() + 10000;
  while (millis() < timeout && response.length() < 10) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        break;
      }
      response += c;
    }
    delay(10);
  }
  
  if (response == "YES") {
    Serial.println(F(">>> Formatting filesystem..."));
    
    // Unmount filesystem
    LittleFS.end();
    delay(100);
    
    // Format the filesystem
    if (LittleFS.format()) {
      Serial.println(F(">>> Format complete."));
      delay(100);
      
      // Try to remount to ensure clean state
      if (LittleFS.begin()) {
        Serial.println(F(">>> Filesystem ready. Restarting..."));
      } else {
        Serial.println(F(">>> Warning: Filesystem mount failed. Restarting anyway..."));
      }
    } else {
      Serial.println(F(">>> Format failed! Restarting anyway..."));
    }
    
    delay(1000);
    rp2040.restart();
  } else {
    Serial.println(F(">>> Cancelled"));
  }
}

void SerialInterface::print(const char* msg) {
  if (debugMode != DEBUG_OFF) {
    Serial.print(msg);
  }
}

void SerialInterface::println(const char* msg) {
  if (debugMode != DEBUG_OFF) {
    Serial.println(msg);
  }
}

void SerialInterface::print(const __FlashStringHelper* msg) {
  if (debugMode != DEBUG_OFF) {
    Serial.print(msg);
  }
}

void SerialInterface::println(const __FlashStringHelper* msg) {
  if (debugMode != DEBUG_OFF) {
    Serial.println(msg);
  }
}

void SerialInterface::printWelcomeMessage() {
  Serial.println();
  Serial.println(F("==========================================="));
  Serial.print(F("       "));
  Serial.print(DEVICE_NAME);
  Serial.print(F(" v"));
  Serial.println(FIRMWARE_VERSION);
  Serial.println(F("==========================================="));
  Serial.print(F("Buffer: "));
  Serial.print(PSRAM_BUFFER_SIZE / (1024 * 1024));
  Serial.print(F("MB | Rates: "));
  Serial.print(IMU_RATE_HZ);
  Serial.print(F("/"));
  Serial.print(BARO_RATE_HZ);
  Serial.print(F("/"));
  Serial.print(LOG_RATE_HZ);
  Serial.println(F(" Hz"));
  #if AUTO_START_ON_LAUNCH
    Serial.print(F("Auto-start: YES"));
  #endif
  #if AUTO_STOP_ON_LANDING
    Serial.print(F(" | Auto-stop: YES"));
  #endif
  Serial.println();
}

void SerialInterface::printHelp() {
  Serial.println(F("\n=== COMMANDS ==="));
  Serial.println(F("s - Start/stop logging"));
  Serial.println(F("p - Enter/exit preflight mode"));
  Serial.println(F("c - Recalibrate gyro | C - Full cal"));
  Serial.println(F("A - Accel cal only | M - Mag cal only"));
  Serial.println(F("X - Factory reset (delete all)"));
  Serial.println(F("d0-3 - Debug: 0=Off 1=Status 2=Verbose 3=Tethered"));
  Serial.println(F("d - Disable tethered mode"));
  Serial.println(F("t - Trigger test accel (tethered+preflight)"));
  Serial.println(F("m - Memory usage | h - Help"));
  Serial.print(F("\nLaunch: "));
  Serial.print(LAUNCH_ACCEL_THRESHOLD);
  Serial.print(F("G | Pre-buffer: "));
  Serial.print(PRELAUNCH_BUFFER_SECONDS);
  Serial.println(F("s"));
  Serial.println();
}

void SerialInterface::printStatus() {
  if (!stateTracker || !sensorMgr || !dataLogger) return;
  
  SystemState state = stateTracker->getCurrentState();
  if (state == SystemState::PREFLIGHT || state == SystemState::ACTIVE) {
    const SensorData& data = sensorMgr->getData();
    
    Serial.print(F("["));
    switch (state) {
      case SystemState::PREFLIGHT: Serial.print(F("PREFLT")); break;
      case SystemState::ACTIVE: Serial.print(F("ACTIVE")); break;
      default: break;
    }
    Serial.print(F("] Alt: "));
    Serial.print(data.altitude, 1);
    Serial.print(F("m | "));
    Serial.print(data.totalAccel, 1);
    Serial.print(F("G | Buf: "));
    
    uint32_t bufferUsed, bufferSize;
    dataLogger->getMemoryInfo(bufferUsed, bufferSize);
    Serial.print(bufferUsed * 100 / bufferSize);
    Serial.println(F("%"));
  }
}

void SerialInterface::printFlightSummary(const FlightStats& stats) {
  Serial.print(F("\nFLIGHT: "));
  Serial.print(stats.flightTime / 1000);
  Serial.print(F("s, "));
  Serial.print(stats.maxAcceleration, 1);
  Serial.print(F("G max, "));
  Serial.print(stats.maxAltitude, 1);
  Serial.println(F("m alt"));
}

void SerialInterface::outputVerboseData() {
  if (!sensorMgr) return;
  
  const SensorData& data = sensorMgr->getData();
  
  // CSV format for easy import
  Serial.print(millis());
  Serial.print(F(","));
  Serial.print(data.totalAccel, 2);
  Serial.print(F(","));
  Serial.print(data.roll * 57.29578, 1);  // Convert to degrees
  Serial.print(F(","));
  Serial.print(data.pitch * 57.29578, 1);
  Serial.print(F(","));
  Serial.print(data.yaw * 57.29578, 1);
  Serial.print(F(","));
  Serial.print(data.altitude, 1);
  Serial.print(F(","));
  Serial.print(data.temperature, 1);
  Serial.println();
}