// DataLogger.h - Data logging, PSRAM management, and telemetry
#pragma once

#include <Arduino.h>
#include <LittleFS.h>
#include <MAVLink.h>
#include "config.h"

// Forward declarations
class SensorManager;
struct SensorData;
enum class SystemState;

class DataLogger {
public:
  DataLogger();
  bool begin();
  
  // Logging control
  bool startLogging();
  bool stopLogging();
  bool isLogging() const { return logging; }
  bool isPreflightCapture() const { return capturePrelaunch; }
  void setPreflightCapture(bool enable) { capturePrelaunch = enable; }
  
  // Data logging
  void logData(const SensorData& data, SystemState state);
  void update();  // Called periodically to handle flushing
  
  // Telemetry
  void sendHeartbeat();
  void sendAttitude(const SensorData& data);
  void sendHighresIMU(const SensorData& data);
  void sendScaledPressure(const SensorData& data);
  
  // Status
  bool hasStoredData() const { return storedDataExists; }
  uint32_t getFileNumber() const { return logFileNumber; }
  uint32_t getBytesWritten() const { return totalBytesWritten; }
  
  // Memory info
  void getMemoryInfo(uint32_t& bufferUsed, uint32_t& bufferSize);
  uint32_t getSecondsRemaining();
  
  // Debug
  void setDebugMode(uint8_t mode) { debugMode = mode; }
  
private:
  // PSRAM buffers
  uint8_t* psramBuffer;
  uint8_t* prelaunchBuffer;
  volatile uint32_t psramWritePos;
  volatile uint32_t psramReadPos;
  volatile uint32_t prelaunchWritePos;
  volatile bool psramFlushNeeded;
  uint32_t actualPSRAMSize;
  
  // File system
  File logFile;
  uint32_t logFileNumber;
  uint32_t totalBytesWritten;
  bool storedDataExists;
  
  // State
  volatile bool logging;
  volatile bool capturePrelaunch;
  
  // Debug
  uint8_t debugMode;
  
  // Internal methods
  bool setupPSRAMBuffers();
  void openLogFile();
  void flushPSRAMToFlash();
  void dumpPrelaunchBuffer();
  void writeMAVLinkMessage(mavlink_message_t& msg);
};