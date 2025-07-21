// RocketChip.ino - Main sketch for Rocket Chip tracking board
// Target: Adafruit RP2350 Feather with 8MB PSRAM
// Core: Earle Philhower's Pico Core
// Sensors: ICM20948 (9-axis IMU) + DPS310 (Pressure/Altitude)

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_DPS310.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_NeoPixel.h>
#include <LittleFS.h>
#include <MAVLink.h>  // MAVLink 2.0 library
#include "config.h"   // Configuration settings

// NeoPixel configuration
#define NEOPIXEL_COUNT 1

// PSRAM Buffer Configuration
#define PSRAM_BUFFER_SIZE (4 * 1024 * 1024)  // 4MB main buffer in PSRAM
#define PSRAM_FLUSH_THRESHOLD (PSRAM_BUFFER_SIZE * 0.75)  // Flush at 75% full
#define MIN_FLUSH_SIZE (64 * 1024)  // Minimum 64KB before flushing to flash

// Pre-launch buffer configuration
#define PRELAUNCH_BUFFER_SECONDS 5  // Seconds of pre-launch data to keep
#define PRELAUNCH_BUFFER_SIZE (50 * 140 * PRELAUNCH_BUFFER_SECONDS)  // ~35KB for 5 seconds

// Create sensor objects
Adafruit_ICM20948 icm;
Adafruit_DPS310 dps;

// AHRS (Attitude and Heading Reference System) - using Madgwick filter
Adafruit_Madgwick filter;

// NeoPixel for status indication
Adafruit_NeoPixel pixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// PSRAM buffers for hybrid storage
uint8_t* psramBuffer = nullptr;          // Main logging buffer
uint8_t* prelaunchBuffer = nullptr;      // Circular buffer for pre-launch data
volatile uint32_t psramWritePos = 0;
volatile uint32_t psramReadPos = 0;
volatile uint32_t prelaunchWritePos = 0;
volatile bool psramFlushNeeded = false;
volatile bool capturePrelaunch = false;

// File system for data logging
File logFile;
uint32_t logFileNumber = 0;
uint32_t totalBytesWritten = 0;

// Timing variables
unsigned long lastSensorRead = 0;
unsigned long lastLogWrite = 0;
unsigned long lastStatusUpdate = 0;
unsigned long lastPrintStatus = 0;
unsigned long lastLedUpdate = 0;

// Sensor data cache (shared between cores)
volatile struct {
  float accel_x, accel_y, accel_z;  // m/s^2
  float gyro_x, gyro_y, gyro_z;     // rad/s
  float mag_x, mag_y, mag_z;        // uT
  float pressure;                    // hPa
  float temperature;                 // Celsius
  float altitude;                    // meters
  float roll, pitch, yaw;            // radians
  uint32_t timestamp;                // microseconds
  float totalAccel;                  // magnitude of acceleration vector
} sensorData;

// Launch detection variables
float launchThreshold = LAUNCH_ACCEL_THRESHOLD;  // G's to detect launch
bool launchDetected = false;
unsigned long launchTime = 0;

// Flight statistics
float maxAcceleration = 0;
float maxAltitude = 0;
float launchAltitude = 0;

// System state
enum SystemState {
  STATE_INITIALIZING,  // Pulsing yellow
  STATE_CALIBRATING,   // Solid yellow
  STATE_IDLE,          // Solid blue (or blue/green if data available)
  STATE_PREFLIGHT,     // Slow blinking blue
  STATE_ACTIVE,        // Fast blinking green
  STATE_ERROR          // Solid red
};

volatile SystemState currentState = STATE_INITIALIZING;
volatile bool isLogging = false;
volatile bool hasStoredData = false;
volatile bool sensorsReady = false;

// LED animation variables - ADD THESE HERE
float ledBrightness = 0;
float ledPhase = 0;
bool ledToggle = false;
unsigned long ledToggleTime = 0;

// Debug and output modes
enum DebugMode {
  DEBUG_OFF,        // No output
  DEBUG_STATUS,     // Status messages only (default)
  DEBUG_VERBOSE,    // Include 10Hz data output
  DEBUG_TEST        // Test mode with manual triggers
};

DebugMode debugMode = DEBUG_STATUS;
unsigned long lastDataOutput = 0;

// Calibration data
struct CalibrationData {
  float accelOffset[3] = {0, 0, 0};
  float gyroOffset[3] = {0, 0, 0};
  float magOffset[3] = {0, 0, 0};
  float magScale[3] = {1, 1, 1};
  bool isCalibrated = false;
} calibration;

// Test mode variables
bool testModeActive = false;
float testAccelValue = 1.0;  // G's for test mode

// Function prototypes
void initSensors();
void readSensors();
void updateAHRS();
void logDataMAVLink();
void flushPSRAMToFlash();
void openLogFile();
void dumpPrelaunchBuffer();
void updateStatusLED();
void checkSystemReady();
void calibrateSensors();
void applyCalibration();
void printWelcomeMessage();
void printHelp();
void printStatus();
void printFlightSummary();
void outputVerboseData();
void handleSerialCommand();
void simulateTestAcceleration();
void core1_setup();
void core1_loop();

// MAVLink functions
void sendMAVLinkHeartbeat();
void sendMAVLinkAttitude();
void sendMAVLinkHighresIMU();
void sendMAVLinkScaledPressure();

void setup() {
  Serial.begin(SERIAL_BAUD);
  
  // Initialize NeoPixel
  pixel.begin();
  pixel.setBrightness(NEOPIXEL_BRIGHTNESS);
  pixel.show();
  
  // Set initial state
  currentState = STATE_INITIALIZING;
  
  // Check if serial is connected for debug mode
  bool serialConnected = false;
  if (SERIAL_DEBUG) {
    // Wait briefly for serial connection
    unsigned long startTime = millis();
    while (!Serial && (millis() - startTime < 3000)) {
      delay(10);
    }
    
    if (Serial) {
      serialConnected = true;
      debugMode = DEBUG_STATUS;
      printWelcomeMessage();
    } else {
      debugMode = DEBUG_OFF;
    }
  }

  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C
  
  // Initialize file system
  if (!LittleFS.begin()) {
    if (debugMode >= DEBUG_STATUS) {
      Serial.println("Error: Failed to mount file system!");
    }
    currentState = STATE_ERROR;
    return;
  }
  
  // Allocate PSRAM buffers
  psramBuffer = (uint8_t*)malloc(PSRAM_BUFFER_SIZE);  // RP2350 automatically uses PSRAM for large allocations
  if (!psramBuffer) {
    if (debugMode >= DEBUG_STATUS) {
      Serial.println("Warning: PSRAM allocation failed, using direct write mode");
    }
    // System will still work but with direct flash writes
  } else {
    if (debugMode >= DEBUG_STATUS) {
      Serial.print("Main PSRAM buffer allocated: ");
      Serial.print(PSRAM_BUFFER_SIZE / (1024 * 1024));
      Serial.println(" MB");
    }
  }
  
  // Allocate pre-launch buffer
  prelaunchBuffer = (uint8_t*)malloc(PRELAUNCH_BUFFER_SIZE);
  if (prelaunchBuffer && debugMode >= DEBUG_STATUS) {
    Serial.print("Pre-launch buffer allocated: ");
    Serial.print(PRELAUNCH_BUFFER_SIZE / 1024);
    Serial.println(" KB");
  }
  
  // Find next available log file number
  while (LittleFS.exists(String("/log_") + String(logFileNumber) + ".mavlink")) {
    logFileNumber++;
  }
  
  hasStoredData = (logFileNumber > 0);
  
  // Initialize sensors
  initSensors();
  
  // Initialize AHRS
  filter.begin(SENSOR_RATE_HZ);
  
  // Wait for system to be still before calibrating
  checkSystemReady();
  
  if (debugMode >= DEBUG_STATUS && Serial) {
    Serial.println("Core 0 ready!");
  }
}

void setup1() {
  // Core 1 handles UI and status
  core1_setup();
}

void loop() {
  // Core 0: Handle sensors and logging
  
  // Read sensors at specified rate
  if (micros() - lastSensorRead >= (1000000 / SENSOR_RATE_HZ)) {
    lastSensorRead = micros();
    if (sensorsReady) {
      readSensors();
      updateAHRS();
    }
  }
  
  // Log data at specified rate
  if ((isLogging || capturePrelaunch) && (micros() - lastLogWrite >= (1000000 / LOG_RATE_HZ))) {
    lastLogWrite = micros();
    logDataMAVLink();
  }
  
  // Flush PSRAM to flash when needed
  if (psramFlushNeeded || (!isLogging && psramReadPos != psramWritePos)) {
    flushPSRAMToFlash();
  }
  
  // Send MAVLink heartbeat every second
  if (millis() - lastStatusUpdate >= 1000) {
    lastStatusUpdate = millis();
    if (debugMode >= DEBUG_VERBOSE) {
      sendMAVLinkHeartbeat();
    }
  }
  
  // Output verbose data if enabled
  if (debugMode == DEBUG_VERBOSE && (millis() - lastDataOutput >= 100)) {
    lastDataOutput = millis();
    outputVerboseData();
  }
  
  // Print debug status periodically in status mode
  if (debugMode == DEBUG_STATUS && (millis() - lastPrintStatus >= 5000)) {
    lastPrintStatus = millis();
    printStatus();
  }
}

void loop1() {
  // Core 1: Handle UI and LED status
  core1_loop();
}

void initSensors() {
  if (debugMode >= DEBUG_STATUS) {
    Serial.println("Initializing sensors...");
  }
  
  // Initialize ICM20948
  if (!icm.begin_I2C()) {
    if (debugMode >= DEBUG_STATUS) {
      Serial.println("Failed to find ICM20948!");
    }
    currentState = STATE_ERROR;
    return;
  }
  
  if (debugMode >= DEBUG_STATUS) {
    Serial.println("ICM20948 found!");
  }
  
  // Configure ICM20948
  icm.setAccelRange(ACCEL_RANGE);
  icm.setGyroRange(GYRO_RANGE);
  icm.setAccelRateDivisor(0); // Maximum rate
  icm.setGyroRateDivisor(0);   // Maximum rate
  icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
  
  // Initialize DPS310
  if (!dps.begin_I2C()) {
    if (debugMode >= DEBUG_STATUS) {
      Serial.println("Failed to find DPS310!");
    }
    currentState = STATE_ERROR;
    return;
  }
  
  if (debugMode >= DEBUG_STATUS) {
    Serial.println("DPS310 found!");
  }
  
  // Configure DPS310
  dps.configurePressure(PRESSURE_RATE, PRESSURE_SAMPLES);
  dps.configureTemperature(PRESSURE_RATE, PRESSURE_SAMPLES);
  
  sensorsReady = true;
  if (debugMode >= DEBUG_STATUS) {
    Serial.println("Sensors initialized!");
  }
}

void readSensors() {
  // Read ICM20948
  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);
  
  // Read DPS310
  sensors_event_t pressure_event, temp_event;
  dps.getEvents(&temp_event, &pressure_event);
  
  // Initial pressure for altitude calculation
  static float initialPressure = 1013.25;  // Will be calibrated on first read
  static bool pressureCalibrated = false;
  
  if (!pressureCalibrated && pressure_event.pressure > 0) {
    initialPressure = pressure_event.pressure;
    pressureCalibrated = true;
  }
  
  // Update shared data structure
  noInterrupts();
  sensorData.timestamp = micros();
  sensorData.accel_x = accel.acceleration.x;
  sensorData.accel_y = accel.acceleration.y;
  sensorData.accel_z = accel.acceleration.z;
  sensorData.gyro_x = gyro.gyro.x;
  sensorData.gyro_y = gyro.gyro.y;
  sensorData.gyro_z = gyro.gyro.z;
  sensorData.mag_x = mag.magnetic.x;
  sensorData.mag_y = mag.magnetic.y;
  sensorData.mag_z = mag.magnetic.z;
  sensorData.pressure = pressure_event.pressure;
  sensorData.temperature = temp_event.temperature;
  sensorData.altitude = dps.readAltitude(SEA_LEVEL_PRESSURE);
  interrupts();
  
  // Apply calibration
  applyCalibration();
  
  // Apply test mode override if active
  if (testModeActive && currentState == STATE_PREFLIGHT) {
    simulateTestAcceleration();
  }
}

void updateAHRS() {
  // Update the filter
  filter.update(sensorData.gyro_x * 57.29578, // Convert to deg/s
                sensorData.gyro_y * 57.29578,
                sensorData.gyro_z * 57.29578,
                sensorData.accel_x,
                sensorData.accel_y,
                sensorData.accel_z,
                sensorData.mag_x,
                sensorData.mag_y,
                sensorData.mag_z);
  
  // Store orientation (convert to radians for MAVLink)
  noInterrupts();
  sensorData.roll = filter.getRoll() * 0.01745329;
  sensorData.pitch = filter.getPitch() * 0.01745329;
  sensorData.yaw = filter.getYaw() * 0.01745329;
  
  // Calculate total acceleration magnitude
  sensorData.totalAccel = sqrt(sensorData.accel_x * sensorData.accel_x +
                               sensorData.accel_y * sensorData.accel_y +
                               sensorData.accel_z * sensorData.accel_z) / 9.81; // Convert to G's
  interrupts();
  
  // Launch detection in preflight mode
  if (currentState == STATE_PREFLIGHT && !launchDetected && AUTO_START_ON_LAUNCH) {
    if (sensorData.totalAccel > launchThreshold) {
      launchDetected = true;
      launchTime = millis();
      // Flag for Core 1 to handle the state change
      // (We don't want to change states from Core 0)
      Serial.println("LAUNCH DETECTED!");
    }
  }
}

void checkSystemReady() {
  if (debugMode >= DEBUG_STATUS) {
    Serial.println("Calibrating sensors...");
  }
  
  currentState = STATE_CALIBRATING;
  
  // Perform sensor calibration
  calibrateSensors();
  
  // Load saved calibration if available
  // TODO: Load from LittleFS
  
  currentState = STATE_IDLE;
  
  if (debugMode >= DEBUG_STATUS) {
    Serial.println("System ready!");
    Serial.println("Type 'h' for help");
  }
}

void calibrateSensors() {
  // Simple gyro calibration - average readings while stationary
  const int numSamples = 100;
  float gyroSum[3] = {0, 0, 0};
  
  for (int i = 0; i < numSamples; i++) {
    sensors_event_t accel, gyro, mag, temp;
    icm.getEvent(&accel, &gyro, &temp, &mag);
    
    gyroSum[0] += gyro.gyro.x;
    gyroSum[1] += gyro.gyro.y;
    gyroSum[2] += gyro.gyro.z;
    
    delay(10);
  }
  
  // Calculate gyro offsets
  calibration.gyroOffset[0] = gyroSum[0] / numSamples;
  calibration.gyroOffset[1] = gyroSum[1] / numSamples;
  calibration.gyroOffset[2] = gyroSum[2] / numSamples;
  
  calibration.isCalibrated = true;
  
  if (debugMode >= DEBUG_STATUS) {
    Serial.println("Calibration complete!");
    Serial.print("Gyro offsets: ");
    Serial.print(calibration.gyroOffset[0], 4); Serial.print(", ");
    Serial.print(calibration.gyroOffset[1], 4); Serial.print(", ");
    Serial.println(calibration.gyroOffset[2], 4);
  }
}

void applyCalibration() {
  // Apply calibration offsets to sensor data
  if (calibration.isCalibrated) {
    sensorData.gyro_x -= calibration.gyroOffset[0];
    sensorData.gyro_y -= calibration.gyroOffset[1];
    sensorData.gyro_z -= calibration.gyroOffset[2];
  }
}

void logDataMAVLink() {
  // Create MAVLink messages
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;
  uint8_t tempBuffer[300]; // Temporary buffer for both messages
  uint16_t totalLen = 0;
  
  // Prepare HIGHRES_IMU message
  mavlink_msg_highres_imu_pack(
    MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg,
    sensorData.timestamp,           // time_usec
    sensorData.accel_x,            // xacc
    sensorData.accel_y,            // yacc
    sensorData.accel_z,            // zacc
    sensorData.gyro_x,             // xgyro
    sensorData.gyro_y,             // ygyro
    sensorData.gyro_z,             // zgyro
    sensorData.mag_x,              // xmag
    sensorData.mag_y,              // ymag
    sensorData.mag_z,              // zmag
    sensorData.pressure,           // abs_pressure
    0,                             // diff_pressure
    sensorData.altitude,           // pressure_alt
    sensorData.temperature,        // temperature
    0x3FFF,                        // fields_updated (all fields)
    0                              // id
  );
  
  len = mavlink_msg_to_send_buffer(buf, &msg);
  memcpy(tempBuffer, buf, len);
  totalLen = len;
  
  // Add ATTITUDE message
  mavlink_msg_attitude_pack(
    MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg,
    millis(),                      // time_boot_ms
    sensorData.roll,               // roll
    sensorData.pitch,              // pitch
    sensorData.yaw,                // yaw
    sensorData.gyro_x,             // rollspeed
    sensorData.gyro_y,             // pitchspeed
    sensorData.gyro_z              // yawspeed
  );
  
  len = mavlink_msg_to_send_buffer(buf, &msg);
  memcpy(tempBuffer + totalLen, buf, len);
  totalLen += len;
  
  // If in preflight mode, write to pre-launch circular buffer
  if (capturePrelaunch && prelaunchBuffer) {
    for (uint16_t i = 0; i < totalLen; i++) {
      prelaunchBuffer[prelaunchWritePos] = tempBuffer[i];
      prelaunchWritePos = (prelaunchWritePos + 1) % PRELAUNCH_BUFFER_SIZE;
    }
  }
  
  // If actively logging, write to main buffer or file
  if (isLogging) {
    if (psramBuffer) {
      // Check if buffer has space
      uint32_t bytesInBuffer = (psramWritePos >= psramReadPos) ? 
                              (psramWritePos - psramReadPos) : 
                              (PSRAM_BUFFER_SIZE - psramReadPos + psramWritePos);
      
      if (bytesInBuffer + totalLen < PSRAM_BUFFER_SIZE) {
        // Write to circular buffer
        for (uint16_t i = 0; i < totalLen; i++) {
          psramBuffer[psramWritePos] = tempBuffer[i];
          psramWritePos = (psramWritePos + 1) % PSRAM_BUFFER_SIZE;
        }
        
        // Check if we need to flush
        if (bytesInBuffer + totalLen >= (PSRAM_BUFFER_SIZE * PSRAM_FLUSH_THRESHOLD / 100)) {
          psramFlushNeeded = true;
        }
      } else {
        // Buffer full, force flush
        psramFlushNeeded = true;
      }
    } else {
      // No PSRAM, write directly to file
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

void dumpPrelaunchBuffer() {
  if (!prelaunchBuffer || prelaunchWritePos == 0) {
    return; // No pre-launch data to dump
  }
  
  if (debugMode >= DEBUG_STATUS) {
    Serial.println(">>> Dumping pre-launch buffer...");
  }
  
  // Calculate how much data we have (up to buffer size)
  uint32_t dataSize = min(prelaunchWritePos, (uint32_t)PRELAUNCH_BUFFER_SIZE);
  uint32_t startPos = (prelaunchWritePos >= PRELAUNCH_BUFFER_SIZE) ? 
                      prelaunchWritePos : 0;
  
  // Copy pre-launch data to main buffer or file
  if (psramBuffer) {
    // Copy to main PSRAM buffer
    for (uint32_t i = 0; i < dataSize; i++) {
      uint32_t readPos = (startPos + i) % PRELAUNCH_BUFFER_SIZE;
      psramBuffer[psramWritePos] = prelaunchBuffer[readPos];
      psramWritePos = (psramWritePos + 1) % PSRAM_BUFFER_SIZE;
    }
  } else {
    // Write directly to file
    if (!logFile) {
      openLogFile();
    }
    if (logFile) {
      // Write in chunks to avoid large stack allocation
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
    Serial.print("    Added ");
    Serial.print(dataSize / 1024.0, 1);
    Serial.println(" KB of pre-launch data");
  }
}

void flushPSRAMToFlash() {
  if (!psramBuffer || psramReadPos == psramWritePos) {
    return; // Nothing to flush
  }
  
  // Open log file if not already open
  if (!logFile) {
    openLogFile();
    if (!logFile) return;
  }
  
  // Calculate bytes to flush
  uint32_t bytesToFlush = (psramWritePos >= psramReadPos) ? 
                          (psramWritePos - psramReadPos) : 
                          (PSRAM_BUFFER_SIZE - psramReadPos + psramWritePos);
  
  // Only flush if we have enough data or flush is forced
  if (bytesToFlush < MIN_FLUSH_SIZE && !psramFlushNeeded) {
    return;
  }
  
  // Flush data in chunks
  const uint32_t chunkSize = 4096;
  uint8_t chunk[chunkSize];
  
  while (bytesToFlush > 0) {
    uint32_t thisChunk = min(chunkSize, bytesToFlush);
    
    // Copy from circular buffer to chunk
    for (uint32_t i = 0; i < thisChunk; i++) {
      chunk[i] = psramBuffer[psramReadPos];
      psramReadPos = (psramReadPos + 1) % PSRAM_BUFFER_SIZE;
    }
    
    // Write chunk to file
    logFile.write(chunk, thisChunk);
    totalBytesWritten += thisChunk;
    bytesToFlush -= thisChunk;
  }
  
  // Sync to flash
  logFile.flush();
  psramFlushNeeded = false;
  
  // Check file size limit
  if (totalBytesWritten >= 10 * 1024 * 1024) { // 10MB limit per file
    logFile.close();
    logFileNumber++;
    totalBytesWritten = 0;
  }
}

void openLogFile() {
  String filename = String("/log_") + String(logFileNumber) + ".mavlink";
  logFile = LittleFS.open(filename, "w");
  if (!logFile) {
    if (debugMode >= DEBUG_STATUS) {
      Serial.println("Failed to create log file!");
    }
    currentState = STATE_ERROR;
    return;
  }
  if (debugMode >= DEBUG_STATUS) {
    Serial.print("Logging to: ");
    Serial.println(filename);
  }
}

// Core 1 functions
void core1_setup() {
  if (debugMode >= DEBUG_STATUS && Serial) {
    Serial.println("Core 1: UI and Status");
  }
}

void core1_loop() {
  // Handle serial commands
  if (Serial.available()) {
    handleSerialCommand();
  }
  
  // Check for automatic launch detection
  if (launchDetected && currentState == STATE_PREFLIGHT && !isLogging) {
    isLogging = true;
    currentState = STATE_ACTIVE;
    capturePrelaunch = false;
    // Dump pre-launch buffer
    if (prelaunchBuffer) {
      dumpPrelaunchBuffer();
    }
    if (debugMode >= DEBUG_STATUS) {
      Serial.println("AUTO-LAUNCH: Detected " + String(sensorData.totalAccel, 1) + "G acceleration!");
      Serial.println("Logging started automatically");
    }
  }
  
  // Update LED
  updateStatusLED();
  
  // Small delay to not hog the CPU
  delay(10);
}

void updateStatusLED() {
  unsigned long currentTime = millis();
  
  switch (currentState) {
    case STATE_INITIALIZING:
      // Pulsing yellow (1 Hz sine wave)
      ledPhase += 0.0628; // 2*PI / 100 for 1Hz at 10ms updates
      ledBrightness = (sin(ledPhase) + 1.0) * 0.5;
      pixel.setPixelColor(0, pixel.Color(255 * ledBrightness, 255 * ledBrightness, 0));
      break;
      
    case STATE_CALIBRATING:
      // Solid yellow
      pixel.setPixelColor(0, pixel.Color(255, 255, 0));
      break;
      
    case STATE_IDLE:
      if (hasStoredData) {
        // Alternating blue/green at 0.5 Hz
        if (currentTime - ledToggleTime >= 1000) {
          ledToggle = !ledToggle;
          ledToggleTime = currentTime;
        }
        if (ledToggle) {
          pixel.setPixelColor(0, pixel.Color(0, 0, 255)); // Blue
        } else {
          pixel.setPixelColor(0, pixel.Color(0, 255, 0)); // Green
        }
      } else {
        // Solid blue
        pixel.setPixelColor(0, pixel.Color(0, 0, 255));
      }
      break;
      
    case STATE_PREFLIGHT:
      // Slow blinking blue (1 Hz)
      if (currentTime - ledToggleTime >= 500) {
        ledToggle = !ledToggle;
        ledToggleTime = currentTime;
      }
      if (ledToggle) {
        pixel.setPixelColor(0, pixel.Color(0, 0, 255));
      } else {
        pixel.setPixelColor(0, pixel.Color(0, 0, 0));
      }
      break;
      
    case STATE_ACTIVE:
      // Fast blinking green (5 Hz)
      if (currentTime - ledToggleTime >= 100) {
        ledToggle = !ledToggle;
        ledToggleTime = currentTime;
      }
      if (ledToggle) {
        pixel.setPixelColor(0, pixel.Color(0, 255, 0));
      } else {
        pixel.setPixelColor(0, pixel.Color(0, 0, 0));
      }
      break;
      
    case STATE_ERROR:
      // Solid red
      pixel.setPixelColor(0, pixel.Color(255, 0, 0));
      break;
  }
  
  pixel.show();
}

// MAVLink transmission functions (for serial output when connected)
void sendMAVLinkHeartbeat() {
  if (!Serial || debugMode < DEBUG_VERBOSE) return;  // Only send in verbose mode
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_heartbeat_pack(
    MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg,
    12,                            // type (MAV_TYPE_ROCKET = 12)
    0,                             // autopilot (MAV_AUTOPILOT_GENERIC = 0)
    1 | (isLogging ? 128 : 0),     // base_mode (custom enabled + armed if logging)
    0,                             // custom_mode
    currentState == STATE_ERROR ? 5 : 
    (isLogging ? 4 : 3)            // system_status (critical/active/standby)
  );
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void sendMAVLinkAttitude() {
  if (!Serial || debugMode < DEBUG_VERBOSE) return;
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_attitude_pack(
    MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg,
    millis(),
    sensorData.roll,
    sensorData.pitch,
    sensorData.yaw,
    sensorData.gyro_x,
    sensorData.gyro_y,
    sensorData.gyro_z
  );
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void printWelcomeMessage() {
  Serial.println();
  Serial.println("===========================================");
  Serial.println("       ROCKET CHIP FLIGHT TRACKER");
  Serial.println("              Version 1.0");
  Serial.println("===========================================");
  Serial.println();
  Serial.println("Status messages enabled. Type 'h' for help.");
  Serial.println();
}

void printHelp() {
  Serial.println();
  Serial.println("=== COMMAND MENU ===");
  Serial.println("s - Start/stop logging");
  Serial.println("p - Enter/exit preflight mode");
  Serial.println("c - Recalibrate sensors");
  Serial.println("d - Change debug mode:");
  Serial.println("    0 = Off, 1 = Status only");
  Serial.println("    2 = Verbose (10Hz data)");
  Serial.println("    3 = Test mode");
  Serial.println("t - (Test mode) Trigger test acceleration");
  Serial.println("m - Show memory usage");
  Serial.println("h - Show this help");
  Serial.println("==================");
  Serial.println();
}

void handleSerialCommand() {
  char cmd = Serial.read();
  
  switch (cmd) {
    case 's':
    case 'S':
      if (currentState == STATE_IDLE || currentState == STATE_ACTIVE || currentState == STATE_PREFLIGHT) {
        isLogging = !isLogging;
        if (isLogging) {
          currentState = STATE_ACTIVE;
          capturePrelaunch = false;
          launchDetected = false;
          if (prelaunchBuffer && capturePrelaunch) {
            dumpPrelaunchBuffer();
          }
          if (debugMode >= DEBUG_STATUS) {
            Serial.println(">>> LOGGING STARTED");
          }
        } else {
          currentState = STATE_IDLE;
          psramFlushNeeded = true;
          delay(100);
          if (logFile) {
            uint32_t fileSize = logFile.size();
            logFile.close();
            hasStoredData = true;
            if (debugMode >= DEBUG_STATUS) {
              printFlightSummary();
              Serial.print(">>> LOGGING STOPPED - Saved ");
              Serial.print(fileSize / 1024.0, 1);
              Serial.println(" KB");
            }
          }
        }
      }
      break;
      
    case 'p':
    case 'P':
      if (currentState == STATE_IDLE) {
        currentState = STATE_PREFLIGHT;
        capturePrelaunch = true;
        prelaunchWritePos = 0;
        launchDetected = false;
        testModeActive = false;
        if (debugMode >= DEBUG_STATUS) {
          Serial.println(">>> PREFLIGHT MODE ACTIVE");
          Serial.println("    - Recording pre-launch buffer");
          Serial.println("    - Launch detection enabled (>" + String(launchThreshold) + "G)");
        }
      } else if (currentState == STATE_PREFLIGHT) {
        currentState = STATE_IDLE;
        capturePrelaunch = false;
        testModeActive = false;
        if (debugMode >= DEBUG_STATUS) {
          Serial.println(">>> IDLE MODE");
        }
      }
      break;
      
    case 'c':
    case 'C':
      if (currentState == STATE_IDLE) {
        calibrateSensors();
      } else {
        Serial.println("Must be in IDLE mode to calibrate");
      }
      break;
      
    case 'd':
    case 'D':
      {
        // Wait for mode number
        while (!Serial.available()) delay(10);
        int mode = Serial.read() - '0';
        if (mode >= 0 && mode <= 3) {
          debugMode = (DebugMode)mode;
          Serial.print("Debug mode set to: ");
          switch (debugMode) {
            case DEBUG_OFF: Serial.println("OFF"); break;
            case DEBUG_STATUS: Serial.println("STATUS"); break;
            case DEBUG_VERBOSE: Serial.println("VERBOSE (10Hz)"); break;
            case DEBUG_TEST: 
              Serial.println("TEST MODE");
              testModeActive = true;
              break;
          }
        }
      }
      break;
      
    case 't':
    case 'T':
      if (debugMode == DEBUG_TEST && currentState == STATE_PREFLIGHT) {
        testAccelValue = launchThreshold + 1.0;
        Serial.println(">>> TEST: Simulating " + String(testAccelValue) + "G acceleration!");
      }
      break;
      
    case 'm':
    case 'M':
      Serial.println("=== MEMORY USAGE ===");
      
      #ifdef RP2350_PSRAM_CS
      Serial.print("Total PSRAM: ");
      Serial.print(rp2040.getTotalPSRAMHeap() / (1024 * 1024));
      Serial.print(" MB, Free: ");
      Serial.print(rp2040.getFreePSRAMHeap() / (1024 * 1024));
      Serial.println(" MB");
      #endif
      
      if (psramBuffer) {
        uint32_t mainBufferUsed = (psramWritePos >= psramReadPos) ? 
                                  (psramWritePos - psramReadPos) : 
                                  (PSRAM_BUFFER_SIZE - psramReadPos + psramWritePos);
        Serial.print("Main buffer: ");
        Serial.print(mainBufferUsed / 1024);
        Serial.print(" KB used of ");
        Serial.print(PSRAM_BUFFER_SIZE / (1024 * 1024));
        Serial.println(" MB");
        Serial.print("Buffer usage: ");
        Serial.print(mainBufferUsed * 100 / PSRAM_BUFFER_SIZE);
        Serial.println("%");
        
        // Show time remaining at current rate
        uint32_t bytesPerSecond = LOG_RATE_HZ * 140; // Approximate
        uint32_t secondsRemaining = (PSRAM_BUFFER_SIZE - mainBufferUsed) / bytesPerSecond;
        Serial.print("Time until flush: ~");
        Serial.print(secondsRemaining / 60);
        Serial.print(" min ");
        Serial.print(secondsRemaining % 60);
        Serial.println(" sec");
      } else {
        Serial.println("No main buffer (direct write mode)");
      }
      break;
      
    case 'h':
    case 'H':
      printHelp();
      break;
      
    case '\n':
    case '\r':
      // Ignore newlines
      break;
      
    default:
      if (debugMode >= DEBUG_STATUS) {
        Serial.println("Unknown command. Type 'h' for help.");
      }
      break;
  }
}

void outputVerboseData() {
  // Output data in a clean CSV format at 10Hz
  Serial.print(millis());
  Serial.print(",");
  Serial.print(sensorData.totalAccel, 2);
  Serial.print(",");
  Serial.print(filter.getRoll(), 1);
  Serial.print(",");
  Serial.print(filter.getPitch(), 1);
  Serial.print(",");
  Serial.print(filter.getYaw(), 1);
  Serial.print(",");
  Serial.print(sensorData.altitude, 1);
  Serial.print(",");
  Serial.println(sensorData.temperature, 1);
}

void simulateTestAcceleration() {
  // Override acceleration for test mode
  sensorData.accel_z = -testAccelValue * 9.81; // Simulate upward acceleration
  testAccelValue = 1.0; // Reset after one reading
}

void printStatus() {
  if (debugMode < DEBUG_STATUS) return;
  
  Serial.print("[");
  switch(currentState) {
    case STATE_INITIALIZING: Serial.print("INIT"); break;
    case STATE_CALIBRATING: Serial.print("CAL"); break;
    case STATE_IDLE: Serial.print("IDLE"); break;
    case STATE_PREFLIGHT: Serial.print("PREFLT"); break;
    case STATE_ACTIVE: Serial.print("ACTIVE"); break;
    case STATE_ERROR: Serial.print("ERROR"); break;
  }
  Serial.print("] ");
  
  // Show key metrics
  Serial.print("Alt: ");
  Serial.print(sensorData.altitude, 1);
  Serial.print("m | ");
  Serial.print(sensorData.totalAccel, 1);
  Serial.print("G | ");
  
  if (psramBuffer) {
    uint32_t bufferPercent = ((psramWritePos >= psramReadPos) ? 
                             (psramWritePos - psramReadPos) : 
                             (PSRAM_BUFFER_SIZE - psramReadPos + psramWritePos)) * 100 / PSRAM_BUFFER_SIZE;
    Serial.print("Buf: ");
    Serial.print(bufferPercent);
    Serial.print("%");
  }
  
  if (testModeActive) {
    Serial.print(" | TEST MODE");
  }
  
  Serial.println();
}

void printFlightSummary() {
  // Print a summary of the flight that was just logged
  if (launchDetected && launchTime > 0) {
    uint32_t flightDuration = (millis() - launchTime) / 1000;
    Serial.println();
    Serial.println("=== FLIGHT SUMMARY ===");
    Serial.print("Flight duration: ");
    Serial.print(flightDuration / 60);
    Serial.print(" min ");
    Serial.print(flightDuration % 60);
    Serial.println(" sec");
    Serial.print("Max acceleration: ");
    Serial.print(maxAcceleration, 1);
    Serial.println(" G");
    Serial.print("Max altitude: ");
    Serial.print(maxAltitude - launchAltitude, 1);
    Serial.println(" m AGL");
    Serial.print("Max altitude: ");
    Serial.print(maxAltitude, 1);
    Serial.println(" m ASL");
    Serial.println("===================");
  }
}