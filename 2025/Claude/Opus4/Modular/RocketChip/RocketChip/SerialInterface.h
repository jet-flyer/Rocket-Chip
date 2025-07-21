// SerialInterface.h - User command handling and output
#pragma once

#include <Arduino.h>
#include <LittleFS.h>
#include "config.h"
#include "SensorManager.h"  // Need this for CalPhase enum

// ===== DEBUG MODES =====
enum DebugMode {
  DEBUG_OFF,        // No output
  DEBUG_STATUS,     // Status messages only (default)
  DEBUG_VERBOSE,    // Include 10Hz data output
  DEBUG_TETHERED    // Tethered mode with manual triggers
};

// Forward declarations
class DataLogger;
class StateTracking;
struct FlightStats;

// ===== SERIAL INTERFACE CLASS =====
class SerialInterface {
public:
  SerialInterface();
  void begin();
  void setModules(SensorManager* sensors, DataLogger* logger, StateTracking* tracker);
  
  // Main update - call from loop
  void update();
  
  // Output functions
  void print(const char* msg);
  void println(const char* msg);
  void print(const __FlashStringHelper* msg);
  void println(const __FlashStringHelper* msg);
  void printWelcomeMessage();
  void printHelp();
  void printStatus();
  void printFlightSummary(const FlightStats& stats);
  void outputVerboseData();
  
  // Debug mode
  DebugMode getDebugMode() const { return debugMode; }
  void setDebugMode(DebugMode mode);
  
private:
  // References to other modules
  SensorManager* sensorMgr;
  DataLogger* dataLogger;
  StateTracking* stateTracker;
  
  // Configuration
  DebugMode debugMode;
  bool tetheredModeActive;
  float testAccelValue;
  
  // Timing
  unsigned long lastDataOutput;
  unsigned long lastStatusPrint;
  
  // Command handling
  void handleCommand(char cmd, const String& fullCommand);
  void handleStartStop();
  void handlePreflight();
  void handleCalibration(CalPhase phase);
  void handleDebugMode(const String& command);
  void handleTestAccel();
  void handleMemoryInfo();
  void handleFactoryReset();
};