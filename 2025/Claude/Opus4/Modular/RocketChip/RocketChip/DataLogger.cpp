// DataLogger.cpp - Data logging, PSRAM management, and telemetry implementation
#include "DataLogger.h"
#include "SensorManager.h"
#include "StateTracking.h"
#include "SerialInterface.h"  // For DEBUG_STATUS

DataLogger::DataLogger() :
  psramBuffer(nullptr),
  prelaunchBuffer(nullptr),
  psramWritePos(0),
  psramReadPos(0),
  prelaunchWritePos(0),
  psramFlushNeeded(false),
  actualPSRAMSize(PSRAM_BUFFER_SIZE),
  logFileNumber(0),
  totalBytesWritten(0),
  storedDataExists(false),
  logging(false),
  capturePrelaunch(false),
  debugMode(DEBUG_STATUS) {
}

bool DataLogger::begin() {
  // Setup PSRAM buffers
  if (!setupPSRAMBuffers()) {
    return false;
  }
  
  // Find next available log file number
  while (LittleFS.exists(String(F("/log_")) + String(logFileNumber) + F(".mavlink"))) {
    logFileNumber++;
  }
  
  storedDataExists = (logFileNumber > 0);
  
  return true;
}

bool DataLogger::setupPSRAMBuffers() {
  #if !defined(RP2350_PSRAM_CS)
    if (debugMode >= DEBUG_STATUS) {
      Serial.println(F("ERROR: RP2350 PSRAM not detected in build!"));
    }
    return false;
  #endif
  
  // Allocate main PSRAM buffer
  psramBuffer = (uint8_t*)pmalloc(PSRAM_BUFFER_SIZE);
  
  if (!psramBuffer) {
    if (debugMode >= DEBUG_STATUS) {
      Serial.print(F("ERROR: PSRAM allocation failed ("));
      Serial.print(PSRAM_BUFFER_SIZE / (1024 * 1024));
      Serial.println(F(" MB)"));
    }
    return false;
  }
  
  actualPSRAMSize = PSRAM_BUFFER_SIZE;
  
  // Allocate pre-launch buffer (use regular malloc)
  prelaunchBuffer = (uint8_t*)malloc(PRELAUNCH_BUFFER_SIZE);
  if (!prelaunchBuffer) {
    if (debugMode >= DEBUG_STATUS) {
      Serial.println(F("ERROR: Pre-launch buffer failed!"));
    }
    return false;
  }
  
  return true;
}

bool DataLogger::startLogging() {
  if (logging) return true;
  
  // Dump pre-launch buffer if we have data
  if (capturePrelaunch && prelaunchBuffer) {
    dumpPrelaunchBuffer();
  }
  
  logging = true;
  capturePrelaunch = false;
  
  if (debugMode >= DEBUG_STATUS) {
    Serial.println(F(">>> LOGGING STARTED"));
  }
  
  return true;
}

bool DataLogger::stopLogging() {
  if (!logging) return true;
  
  logging = false;
  psramFlushNeeded = true;
  
  // Wait for flush
  delay(100);
  update();
  
  if (logFile) {
    uint32_t fileSize = logFile.size();
    logFile.close();
    storedDataExists = true;
    
    if (debugMode >= DEBUG_STATUS) {
      Serial.print(F(">>> LOGGING STOPPED - Saved "));
      Serial.print(fileSize / 1024.0, 1);
      Serial.println(F(" KB"));
    }
  }
  
  // Reset for next flight
  totalBytesWritten = 0;
  logFileNumber++;
  
  return true;
}

void DataLogger::logData(const SensorData& data, SystemState state) {
  // Create MAVLink messages
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;
  uint8_t tempBuffer[300];
  uint16_t totalLen = 0;
  
  // HIGHRES_IMU message
  #if MAVLINK_USE_HIGHRES_IMU
  mavlink_msg_highres_imu_pack(
    MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg,
    data.timestamp,
    data.accel_x, data.accel_y, data.accel_z,
    data.gyro_x, data.gyro_y, data.gyro_z,
    data.mag_x, data.mag_y, data.mag_z,
    data.pressure,
    0,  // diff_pressure
    data.altitude,
    data.temperature,
    0x3FFF,  // fields_updated (all fields)
    0        // id
  );
  
  len = mavlink_msg_to_send_buffer(buf, &msg);
  memcpy(tempBuffer, buf, len);
  totalLen = len;
  #endif
  
  // ATTITUDE message
  mavlink_msg_attitude_pack(
    MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg,
    millis(),
    data.roll, data.pitch, data.yaw,
    data.gyro_x, data.gyro_y, data.gyro_z
  );
  
  len = mavlink_msg_to_send_buffer(buf, &msg);
  memcpy(tempBuffer + totalLen, buf, len);
  totalLen += len;
  
  // Write to appropriate buffer
  if (capturePrelaunch && prelaunchBuffer) {
    // Circular buffer for pre-launch
    for (uint16_t i = 0; i < totalLen; i++) {
      prelaunchBuffer[prelaunchWritePos] = tempBuffer[i];
      prelaunchWritePos = (prelaunchWritePos + 1) % PRELAUNCH_BUFFER_SIZE;
    }
  }
  
  if (logging) {
    if (psramBuffer) {
      // Check buffer space
      uint32_t bytesInBuffer = (psramWritePos >= psramReadPos) ? 
                              (psramWritePos - psramReadPos) : 
                              (actualPSRAMSize - psramReadPos + psramWritePos);
      
      if (bytesInBuffer + totalLen < actualPSRAMSize) {
        // Write to circular buffer
        for (uint16_t i = 0; i < totalLen; i++) {
          psramBuffer[psramWritePos] = tempBuffer[i];
          psramWritePos = (psramWritePos + 1) % actualPSRAMSize;
        }
        
        // Check if we need to flush
        if (bytesInBuffer + totalLen >= (actualPSRAMSize * PSRAM_FLUSH_THRESHOLD / 100)) {
          psramFlushNeeded = true;
        }
      } else {
        // Buffer full
        psramFlushNeeded = true;
      }
    } else {
      // No PSRAM, write directly
      if (!logFile) {
        openLogFile();
      }
      if (logFile) {
        logFile.write(tempBuffer, totalLen);
        totalBytesWritten += totalLen;
      }
    }
  }
}

void DataLogger::update() {
  // Flush PSRAM to flash when needed
  if (psramFlushNeeded || (!logging && psramReadPos != psramWritePos)) {
    flushPSRAMToFlash();
  }
}

void DataLogger::dumpPrelaunchBuffer() {
  if (!prelaunchBuffer || prelaunchWritePos == 0) {
    return;
  }
  
  if (debugMode >= DEBUG_STATUS) {
    Serial.println(F(">>> Dumping pre-launch buffer..."));
  }
  
  // Calculate data size
  uint32_t dataSize = min(prelaunchWritePos, (uint32_t)PRELAUNCH_BUFFER_SIZE);
  uint32_t startPos = (prelaunchWritePos >= PRELAUNCH_BUFFER_SIZE) ? 
                      prelaunchWritePos : 0;
  
  // Copy to main buffer or file
  if (psramBuffer) {
    for (uint32_t i = 0; i < dataSize; i++) {
      uint32_t readPos = (startPos + i) % PRELAUNCH_BUFFER_SIZE;
      psramBuffer[psramWritePos] = prelaunchBuffer[readPos];
      psramWritePos = (psramWritePos + 1) % actualPSRAMSize;
    }
  } else {
    if (!logFile) {
      openLogFile();
    }
    if (logFile) {
      const uint32_t chunkSize = 1024;
      uint8_t chunk[chunkSize];
      
      for (uint32_t written = 0; written < dataSize; written += chunkSize) {
        uint32_t thisChunk = min(chunkSize, dataSize - written);
        for (uint32_t i = 0; i < thisChunk; i++) {
          uint32_t readPos = (startPos + written + i) % PRELAUNCH_BUFFER_SIZE;
          chunk[i] = prelaunchBuffer[readPos];
        }
        logFile.write(chunk, thisChunk);
        totalBytesWritten += thisChunk;
      }
    }
  }
  
  if (debugMode >= DEBUG_STATUS) {
    Serial.print(F("    Pre-launch: +"));
    Serial.print(dataSize / 1024.0, 1);
    Serial.println(F(" KB"));
  }
  
  // Clear for next use
  prelaunchWritePos = 0;
}

void DataLogger::flushPSRAMToFlash() {
  if (!psramBuffer || psramReadPos == psramWritePos) {
    return;
  }
  
  // Open log file if needed
  if (!logFile) {
    openLogFile();
    if (!logFile) return;
  }
  
  // Calculate bytes to flush
  uint32_t bytesToFlush = (psramWritePos >= psramReadPos) ? 
                          (psramWritePos - psramReadPos) : 
                          (actualPSRAMSize - psramReadPos + psramWritePos);
  
  // Only flush if we have enough data
  if (bytesToFlush < MIN_FLUSH_SIZE && !psramFlushNeeded && logging) {
    return;
  }
  
  // Flush in chunks
  const uint32_t chunkSize = 4096;
  uint8_t chunk[chunkSize];
  
  while (bytesToFlush > 0) {
    uint32_t thisChunk = min(chunkSize, bytesToFlush);
    
    for (uint32_t i = 0; i < thisChunk; i++) {
      chunk[i] = psramBuffer[psramReadPos];
      psramReadPos = (psramReadPos + 1) % actualPSRAMSize;
    }
    
    logFile.write(chunk, thisChunk);
    totalBytesWritten += thisChunk;
    bytesToFlush -= thisChunk;
  }
  
  logFile.flush();
  psramFlushNeeded = false;
  
  // Check file size limit
  if (totalBytesWritten >= MAX_LOG_FILE_SIZE) {
    logFile.close();
    logFileNumber++;
    totalBytesWritten = 0;
  }
}

void DataLogger::openLogFile() {
  String filename = String(F("/log_")) + String(logFileNumber) + F(".mavlink");
  
  logFile = LittleFS.open(filename, "w");
  if (!logFile) {
    if (debugMode >= DEBUG_STATUS) {
      Serial.println(F("ERROR: Cannot create log file"));
      
      FSInfo fs_info;
      if (LittleFS.info(fs_info)) {
        Serial.print(F("Free space: "));
        Serial.print((fs_info.totalBytes - fs_info.usedBytes) / 1024);
        Serial.println(F(" KB"));
      }
    }
    return;
  }
  
  if (debugMode >= DEBUG_STATUS) {
    Serial.print(F(">>> Logging to: "));
    Serial.println(filename);
  }
}

void DataLogger::getMemoryInfo(uint32_t& bufferUsed, uint32_t& bufferSize) {
  bufferSize = actualPSRAMSize;
  if (psramBuffer) {
    bufferUsed = (psramWritePos >= psramReadPos) ? 
                 (psramWritePos - psramReadPos) : 
                 (actualPSRAMSize - psramReadPos + psramWritePos);
  } else {
    bufferUsed = 0;
  }
}

uint32_t DataLogger::getSecondsRemaining() {
  if (!psramBuffer || !logging) return 0;
  
  uint32_t bufferUsed, bufferSize;
  getMemoryInfo(bufferUsed, bufferSize);
  
  uint32_t bytesRemaining = bufferSize - bufferUsed;
  return bytesRemaining / BYTES_PER_SECOND;
}

// Telemetry functions
void DataLogger::sendHeartbeat() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_heartbeat_pack(
    MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg,
    MAV_TYPE_ROCKET,  // MAVLink already defines this as 9
    MAV_AUTOPILOT_INVALID,
    MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
    0,
    MAV_STATE_ACTIVE
  );
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void DataLogger::sendAttitude(const SensorData& data) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_attitude_pack(
    MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg,
    millis(),
    data.roll, data.pitch, data.yaw,
    data.gyro_x, data.gyro_y, data.gyro_z
  );
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void DataLogger::sendHighresIMU(const SensorData& data) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_highres_imu_pack(
    MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg,
    data.timestamp,
    data.accel_x, data.accel_y, data.accel_z,
    data.gyro_x, data.gyro_y, data.gyro_z,
    data.mag_x, data.mag_y, data.mag_z,
    data.pressure,
    0,  // diff_pressure
    data.altitude,
    data.temperature,
    0x3FFF,  // fields_updated
    0        // id
  );
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void DataLogger::sendScaledPressure(const SensorData& data) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  int16_t temp_scaled = (int16_t)(data.temperature * 100);
  
  mavlink_msg_scaled_pressure_pack(
    MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg,
    millis(),
    data.pressure,
    0.0f,  // press_diff
    temp_scaled,
    0      // temperature_press_diff
  );
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}