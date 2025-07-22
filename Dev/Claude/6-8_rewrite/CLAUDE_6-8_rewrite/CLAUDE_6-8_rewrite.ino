// RocketChip.ino - Main sketch for Rocket Chip tracking board
// Target: Adafruit RP2350 Feather with 8MB PSRAM
// Core: Earle Philhower's Pico Core (Dual-core operation)
// Sensors: ICM20948 (9-axis IMU) + DPS310 (Pressure/Altitude)
// Version: 2.0 - Complete rewrite with fixed state machine and input handling

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

// ===== SYSTEM CONFIGURATION =====
// Remove redundant definitions - use config.h instead
#define NEOPIXEL_COUNT 1

// ===== SYSTEM STATE DEFINITIONS =====
enum SystemState {
  STATE_INITIALIZING,  // Pulsing yellow - POWER-ON ONLY
  STATE_CALIBRATING,   // Solid yellow
  STATE_IDLE,          // Solid blue (or blue/green if data available)
  STATE_PREFLIGHT,     // Slow blinking blue
  STATE_ACTIVE,        // Fast blinking green
  STATE_ERROR          // Solid red - CRITICAL FAILURE ONLY
};

// State transition validation using bitfields
const uint8_t validTransitions[] = {
  (1<<STATE_CALIBRATING) | (1<<STATE_ERROR),                              // From INITIALIZING
  (1<<STATE_IDLE) | (1<<STATE_ERROR),                                     // From CALIBRATING  
  (1<<STATE_PREFLIGHT) | (1<<STATE_CALIBRATING) | (1<<STATE_ERROR),      // From IDLE
  (1<<STATE_ACTIVE) | (1<<STATE_IDLE) | (1<<STATE_ERROR),                // From PREFLIGHT
  (1<<STATE_IDLE) | (1<<STATE_ERROR),                                     // From ACTIVE
  0                                                                        // From ERROR (no exits)
};

// Safe state transition macro
#define SET_STATE(newState) do { \
  SystemState oldState = currentState; \
  if ((validTransitions[currentState] & (1<<newState)) || currentState == newState) { \
    currentState = newState; \
    if (debugMode >= DEBUG_STATUS && oldState != newState) { \
      Serial.print("State: "); Serial.print(getStateName(oldState)); \
      Serial.print(" -> "); Serial.println(getStateName(newState)); \
    } \
  } else { \
    if (debugMode >= DEBUG_STATUS) { \
      Serial.print("ERROR: Invalid transition "); Serial.print(getStateName(oldState)); \
      Serial.print(" -> "); Serial.println(getStateName(newState)); \
    } \
    currentState = STATE_ERROR; \
  } \
} while(0)

// ===== CALIBRATION STATE DEFINITIONS =====
enum CalState { 
  CAL_IDLE,
  CAL_GYRO_COLLECTING,
  CAL_ACCEL_WAITING,      // Waiting for user to press enter or position device
  CAL_ACCEL_COLLECTING,   // Collecting data for current position
  CAL_MAG_WAITING,        // Waiting for user to start
  CAL_MAG_COLLECTING      // Collecting magnetometer data
};

enum CalPhase {
  PHASE_NONE,
  PHASE_FIRST_TIME,
  PHASE_GYRO_ONLY,
  PHASE_ACCEL_ONLY,
  PHASE_MAG_ONLY,
  PHASE_FULL_CAL
};

// ===== DEBUG MODES =====
enum DebugMode {
  DEBUG_OFF,        // No output
  DEBUG_STATUS,     // Status messages only (default)
  DEBUG_VERBOSE,    // Include 10Hz data output
  DEBUG_TETHERED    // Tethered mode with manual triggers
};

// ===== GLOBAL OBJECTS =====
Adafruit_ICM20948 icm;
Adafruit_DPS310 dps;
Adafruit_Madgwick filter;
Adafruit_NeoPixel pixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// ===== PSRAM BUFFERS =====
uint8_t* psramBuffer = nullptr;          // Main logging buffer
uint8_t* prelaunchBuffer = nullptr;      // Circular buffer for pre-launch data
volatile uint32_t psramWritePos = 0;
volatile uint32_t psramReadPos = 0;
volatile uint32_t prelaunchWritePos = 0;
volatile bool psramFlushNeeded = false;
volatile bool capturePrelaunch = false;
uint32_t actualPSRAMSize = PSRAM_BUFFER_SIZE;

// ===== FILE SYSTEM =====
File logFile;
uint32_t logFileNumber = 0;
uint32_t totalBytesWritten = 0;

// ===== TIMING VARIABLES =====
unsigned long lastIMURead = 0;
unsigned long lastBaroRead = 0;
unsigned long lastLogWrite = 0;
unsigned long lastStatusUpdate = 0;
unsigned long lastPrintStatus = 0;
unsigned long lastLedUpdate = 0;

// ===== SENSOR DATA CACHE =====
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

// ===== LAUNCH DETECTION =====
float launchThreshold = LAUNCH_ACCEL_THRESHOLD;
bool launchDetected = false;
unsigned long launchTime = 0;
float maxAcceleration = 0;
float maxAltitude = 0;
float launchAltitude = 0;
unsigned long stillnessStartTime = 0;

// ===== SYSTEM STATE =====
volatile SystemState currentState = STATE_INITIALIZING;
volatile bool isLogging = false;
volatile bool hasStoredData = false;
volatile bool sensorsReady = false;

// ===== LED ANIMATION =====
float ledBrightness = 0;
float ledPhase = 0;
bool ledToggle = false;
unsigned long ledToggleTime = 0;

// ===== DEBUG AND OUTPUT =====
DebugMode debugMode = DEBUG_STATUS;
unsigned long lastDataOutput = 0;

// ===== CALIBRATION DATA =====
struct CalibrationData {
  float accelOffset[3] = {0, 0, 0};
  float gyroOffset[3] = {0, 0, 0};
  float magOffset[3] = {0, 0, 0};
  float magScale[3] = {1, 1, 1};
  float accelScale[3] = {1, 1, 1};
  bool isCalibrated = false;
  bool accelCalibrated = false;
  bool magCalibrated = false;
  uint32_t magicNumber = 0xCAFEBABE;
} calibration;

// ===== CALIBRATION STATE =====
CalState calState = CAL_IDLE;
CalPhase currentCalPhase = PHASE_NONE;
bool waitingForInput = false;
unsigned long inputPromptTime = 0;
int calStep = 0;
int calSamples = 0;
float calData[6][3];
float magMin[3] = {1000, 1000, 1000};
float magMax[3] = {-1000, -1000, -1000};

// Gyro calibration with stillness detection
float gyroVariance[3] = {0, 0, 0};
float gyroLastSample[3] = {0, 0, 0};
bool gyroIsStill = true;
unsigned long calStartTime = 0;
unsigned long calLastUpdate = 0;

// Magnetometer calibration coverage tracking
float magLastVector[3] = {0, 0, 0};
float magTotalRotation = 0;
int magProgressCounter = 0;  // Kept for compatibility, but not used

// ===== TETHERED MODE =====
bool tetheredModeActive = false;
float testAccelValue = 1.0;

// ===== FUNCTION PROTOTYPES =====
void initSensors();
void readIMU();
void readBarometer();
void updateAHRS();
void logDataMAVLink();
void flushPSRAMToFlash();
void openLogFile();
void dumpPrelaunchBuffer();
void updateStatusLED();
void checkSystemReady();
void printWelcomeMessage();
void printHelp();
void printStatus();
void printFlightSummary();
void outputVerboseData();
void handleSerialCommand();
void simulateTestAcceleration();
void core1_setup();
void core1_loop();
void setupPSRAMBuffers();
const char* getStateName(SystemState state);

// Calibration functions
bool loadCalibration();
void saveCalibration();
void updateCalibration();
void startCalibrationPhase(CalPhase phase);
void startGyroCollection();
void collectGyroData();
void startAccelCalibration();
void showAccelPosition(int position);
void collectAccelData();
void processAccelCalibration();
void startMagCalibration();
void collectMagData();
void completeCalibration();

// MAVLink functions
void sendMAVLinkHeartbeat();
void sendMAVLinkAttitude();
void sendMAVLinkHighresIMU();
void sendMAVLinkScaledPressure();

// ===== MAIN SETUP =====
void setup() {
  Serial.begin(SERIAL_BAUD);
  
  // Initialize NeoPixel
  pixel.begin();
  pixel.setBrightness(NEOPIXEL_BRIGHTNESS);
  pixel.setPixelColor(0, pixel.Color(255, 255, 0)); // Yellow for init
  pixel.show();
  
  // Set initial state (only place INITIALIZING can be set)
  currentState = STATE_INITIALIZING;
  
  // Check if serial is connected for debug mode
  if (SERIAL_DEBUG) {
    // Wait briefly for serial connection
    unsigned long startTime = millis();
    while (!Serial && (millis() - startTime < 3000)) {
      delay(10);
    }
    
    if (Serial) {
      // Automatically enter tethered mode when connected via USB
      debugMode = DEBUG_TETHERED;
      tetheredModeActive = true;
      printWelcomeMessage();
      Serial.println(">>> TETHERED MODE - Type 'h' for help\n");
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
    SET_STATE(STATE_ERROR);
    return;
  }
  
  // Setup PSRAM buffers
  setupPSRAMBuffers();
  
  // Find next available log file number
  while (LittleFS.exists(String("/log_") + String(logFileNumber) + ".mavlink")) {
    logFileNumber++;
  }
  
  hasStoredData = (logFileNumber > 0);
  
  // Initialize sensors
  initSensors();
  
  // Initialize AHRS
  filter.begin(IMU_RATE_HZ);  // AHRS runs at IMU rate
  #ifdef MADGWICK_BETA
    filter.setBeta(MADGWICK_BETA);
  #endif
  
  // Check system and calibrate
  checkSystemReady();
}

// ===== PSRAM BUFFER SETUP =====
void setupPSRAMBuffers() {
  #if !defined(RP2350_PSRAM_CS)
    if (debugMode >= DEBUG_STATUS) {
      Serial.println("ERROR: RP2350 PSRAM not detected in build!");
    }
    SET_STATE(STATE_ERROR);
    return;
  #endif
  
  // Allocate main PSRAM buffer - MUST succeed
  psramBuffer = (uint8_t*)pmalloc(PSRAM_BUFFER_SIZE);
  
  if (!psramBuffer) {
    if (debugMode >= DEBUG_STATUS) {
      Serial.print("ERROR: PSRAM allocation failed (");
      Serial.print(PSRAM_BUFFER_SIZE / (1024 * 1024));
      Serial.println(" MB)");
    }
    SET_STATE(STATE_ERROR);
    return;
  }
  
  actualPSRAMSize = PSRAM_BUFFER_SIZE;
  
  // Allocate pre-launch buffer (use regular malloc)
  prelaunchBuffer = (uint8_t*)malloc(PRELAUNCH_BUFFER_SIZE);
  if (!prelaunchBuffer) {
    if (debugMode >= DEBUG_STATUS) {
      Serial.println("ERROR: Pre-launch buffer failed!");
    }
    SET_STATE(STATE_ERROR);
    return;
  }
}

// ===== CORE 1 SETUP (UI AND STATUS) =====
void setup1() {
  core1_setup();
}

// ===== MAIN LOOP (CORE 0 - SENSORS AND LOGGING) =====
void loop() {
  // Read IMU at high rate (1000Hz)
  if (micros() - lastIMURead >= (1000000 / IMU_RATE_HZ)) {
    lastIMURead = micros();
    if (sensorsReady) {
      readIMU();
      // Update calibration if active
      if (calState != CAL_IDLE) {
        updateCalibration();
      } else {
        // Only update AHRS when not calibrating
        updateAHRS();
      }
    }
  }
  
  // Read barometer at lower rate (100Hz)
  if (micros() - lastBaroRead >= (1000000 / BARO_RATE_HZ)) {
    lastBaroRead = micros();
    if (sensorsReady) {
      readBarometer();
    }
  }
  
  // Skip logging operations during calibration
  if (calState != CAL_IDLE) {
    return;
  }
  
  // Log data at specified rate
  if ((isLogging || capturePrelaunch) && (micros() - lastLogWrite >= (1000000 / LOG_RATE_HZ))) {
    lastLogWrite = micros();
    logDataMAVLink();
    
    // Track max values during flight
    if (isLogging) {
      if (sensorData.totalAccel > maxAcceleration) {
        maxAcceleration = sensorData.totalAccel;
      }
      if (sensorData.altitude > maxAltitude) {
        maxAltitude = sensorData.altitude;
      }
    }
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
  if (debugMode == DEBUG_VERBOSE && (millis() - lastDataOutput >= (1000 / VERBOSE_DATA_RATE))) {
    lastDataOutput = millis();
    outputVerboseData();
  }
  
  // Print status periodically (but not in IDLE or CALIBRATING states)
  if (debugMode == DEBUG_STATUS && currentState != STATE_IDLE && currentState != STATE_CALIBRATING && 
      (millis() - lastPrintStatus >= 5000)) {
    lastPrintStatus = millis();
    printStatus();
  }
}

// ===== LOOP 1 (CORE 1 - UI AND LED) =====
void loop1() {
  core1_loop();
}

// ===== SENSOR INITIALIZATION =====
void initSensors() {
  if (debugMode >= DEBUG_STATUS) {
    Serial.print("Initializing sensors... ");
  }
  
  unsigned long startTime = millis();
  
  // Initialize ICM20948
  while (!icm.begin_I2C()) {
    if (millis() - startTime > SENSOR_INIT_TIMEOUT) {
      if (debugMode >= DEBUG_STATUS) {
        Serial.println("ICM20948 timeout!");
      }
      SET_STATE(STATE_ERROR);
      return;
    }
    delay(100);
  }
  
  // Configure ICM20948
  icm.setAccelRange(ACCEL_RANGE);
  icm.setGyroRange(GYRO_RANGE);
  icm.setAccelRateDivisor(0); // Maximum rate
  icm.setGyroRateDivisor(0);   // Maximum rate
  icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
  
  // Initialize DPS310
  startTime = millis();
  while (!dps.begin_I2C()) {
    if (millis() - startTime > SENSOR_INIT_TIMEOUT) {
      if (debugMode >= DEBUG_STATUS) {
        Serial.println("DPS310 timeout!");
      }
      SET_STATE(STATE_ERROR);
      return;
    }
    delay(100);
  }
  
  // Configure DPS310
  dps.configurePressure(PRESSURE_RATE, PRESSURE_SAMPLES);
  dps.configureTemperature(PRESSURE_RATE, PRESSURE_SAMPLES);
  
  sensorsReady = true;
  if (debugMode >= DEBUG_STATUS) {
    Serial.println("OK");
  }
}

// ===== SENSOR READING =====
void readIMU() {
  // Read ICM20948 only (high rate sensors)
  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);
  
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
  interrupts();
  
  // Apply calibration offsets
  if (calibration.isCalibrated) {
    sensorData.gyro_x -= calibration.gyroOffset[0];
    sensorData.gyro_y -= calibration.gyroOffset[1]; 
    sensorData.gyro_z -= calibration.gyroOffset[2];
  }
  
  if (calibration.accelCalibrated) {
    sensorData.accel_x = (sensorData.accel_x - calibration.accelOffset[0]) * calibration.accelScale[0];
    sensorData.accel_y = (sensorData.accel_y - calibration.accelOffset[1]) * calibration.accelScale[1];
    sensorData.accel_z = (sensorData.accel_z - calibration.accelOffset[2]) * calibration.accelScale[2];
  }
  
  if (calibration.magCalibrated) {
    sensorData.mag_x = (sensorData.mag_x - calibration.magOffset[0]) * calibration.magScale[0];
    sensorData.mag_y = (sensorData.mag_y - calibration.magOffset[1]) * calibration.magScale[1];
    sensorData.mag_z = (sensorData.mag_z - calibration.magOffset[2]) * calibration.magScale[2];
  }
  
  // Apply tethered mode override if active
  if (tetheredModeActive && currentState == STATE_PREFLIGHT) {
    simulateTestAcceleration();
  }
}

void readBarometer() {
  // Read DPS310 only (low rate sensors)
  sensors_event_t pressure_event, temp_event;
  dps.getEvents(&temp_event, &pressure_event);
  
  // Update shared data structure
  noInterrupts();
  sensorData.pressure = pressure_event.pressure;
  sensorData.temperature = temp_event.temperature;
  sensorData.altitude = dps.readAltitude(SEA_LEVEL_PRESSURE);
  interrupts();
}

// ===== AHRS UPDATE =====
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
      launchAltitude = sensorData.altitude;
      maxAcceleration = sensorData.totalAccel;
      maxAltitude = sensorData.altitude;
    }
  }
  
  // Simple landing detection - just check if we're still
  #if AUTO_STOP_ON_LANDING
  if (currentState == STATE_ACTIVE && isLogging) {
    if (sensorData.totalAccel > 0.9 && sensorData.totalAccel < 1.1) {
      if (stillnessStartTime == 0) {
        stillnessStartTime = millis();
      } else if (millis() - stillnessStartTime >= LANDING_STILLNESS_TIME) {
        // We've been still for 5 seconds - trigger landing in Core 1
        launchDetected = false; // This will trigger auto-stop in Core 1
      }
    } else {
      stillnessStartTime = 0; // Reset if we move
    }
  }
  #endif
}

// ===== CALIBRATION SYSTEM =====
void checkSystemReady() {
  SET_STATE(STATE_CALIBRATING);
  
  // Try to load calibration
  if (loadCalibration()) {
    if (!calibration.accelCalibrated || !calibration.magCalibrated) {
      // Need first time setup
      startCalibrationPhase(PHASE_FIRST_TIME);
    } else {
      // Just do gyro calibration
      startCalibrationPhase(PHASE_GYRO_ONLY);
    }
  } else {
    // No calibration file - first time setup
    startCalibrationPhase(PHASE_FIRST_TIME);
  }
}

bool loadCalibration() {
  if (!LittleFS.exists(CALIBRATION_FILE)) return false;
  
  File calFile = LittleFS.open(CALIBRATION_FILE, "r");
  if (!calFile) return false;
  
  CalibrationData tempCal;
  size_t bytesRead = calFile.read((uint8_t*)&tempCal, sizeof(tempCal));
  calFile.close();
  
  if (bytesRead == sizeof(tempCal) && tempCal.magicNumber == 0xCAFEBABE) {
    calibration = tempCal;
    if (debugMode >= DEBUG_STATUS) {
      Serial.print("Cal loaded: Accel=");
      Serial.print(calibration.accelCalibrated ? "Y" : "N");
      Serial.print(" Mag=");
      Serial.println(calibration.magCalibrated ? "Y" : "N");
    }
    return true;
  }
  return false;
}

void saveCalibration() {
  calibration.magicNumber = 0xCAFEBABE;
  File calFile = LittleFS.open(CALIBRATION_FILE, "w");
  if (calFile) {
    calFile.write((uint8_t*)&calibration, sizeof(calibration));
    calFile.close();
    if (debugMode >= DEBUG_STATUS) {
      Serial.println("✓ Calibration saved");
    }
  }
}

void updateCalibration() {
  // Early exit if not calibrating
  if (calState == CAL_IDLE) return;
  
  // Check for user input if we're waiting for it
  if (waitingForInput && Serial.available()) {
    // Clear all input
    while (Serial.available()) {
      Serial.read();
    }
    waitingForInput = false;
    
    // Handle the input based on current state
    switch (calState) {
      case CAL_ACCEL_WAITING:
        if (calStep == -1) {
          // Initial state - show first position
          calStep = 0;
          showAccelPosition(1);
        } else if (calStep >= 0 && calStep <= 5) {
          // Ready to collect for current position
          if (debugMode >= DEBUG_STATUS) {
            Serial.println("Collecting data...");
          }
          calState = CAL_ACCEL_COLLECTING;
          calSamples = 0;
        }
        // If calStep is out of bounds, do nothing - this shouldn't happen
        break;
        
      case CAL_MAG_WAITING:
        if (debugMode >= DEBUG_STATUS) {
          Serial.println("Start moving now! Progress:");
        }
        calState = CAL_MAG_COLLECTING;
        calSamples = 0;
        magProgressCounter = 0;
        break;
    }
    return;
  }
  
  // Handle active calibration states
  switch (calState) {
    case CAL_GYRO_COLLECTING:
      collectGyroData();
      break;
      
    case CAL_ACCEL_COLLECTING:
      collectAccelData();
      break;
      
    case CAL_MAG_COLLECTING:
      collectMagData();
      break;
  }
}

void startCalibrationPhase(CalPhase phase) {
  currentCalPhase = phase;
  
  switch (phase) {
    case PHASE_FIRST_TIME:
      if (debugMode >= DEBUG_STATUS) {
        Serial.println("\n🚀 FIRST TIME SETUP");
      }
      delay(100); // Small delay to ensure serial output completes
      startGyroCollection();
      break;
      
    case PHASE_GYRO_ONLY:
      startGyroCollection();
      break;
      
    case PHASE_ACCEL_ONLY:
      startAccelCalibration();
      break;
      
    case PHASE_MAG_ONLY:
      startMagCalibration();
      break;
      
    case PHASE_FULL_CAL:
      if (debugMode >= DEBUG_STATUS) {
        Serial.println("\n📊 FULL CALIBRATION");
      }
      delay(100);
      startGyroCollection();
      break;
      
    default:
      break;
  }
}

void startGyroCollection() {
  if (debugMode >= DEBUG_STATUS) {
    Serial.println("📍 GYRO CAL - Keep still 5 sec");
  }
  calState = CAL_GYRO_COLLECTING;
  calSamples = 0;
  calibration.gyroOffset[0] = 0;
  calibration.gyroOffset[1] = 0;
  calibration.gyroOffset[2] = 0;
  gyroVariance[0] = gyroVariance[1] = gyroVariance[2] = 0;
  gyroIsStill = true;
  calStartTime = millis();
  calLastUpdate = millis();
}

void collectGyroData() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - calStartTime;
  
  // Check for stillness after first 100ms
  if (elapsedTime > 100 && calSamples > 10) {
    float movement = 0;
    for (int i = 0; i < 3; i++) {
      float diff = ((float*)&sensorData.gyro_x)[i] - gyroLastSample[i];
      movement += fabs(diff);
    }
    
    // If movement exceeds threshold (0.05 rad/s), restart
    if (movement > 0.05) {
      if (debugMode >= DEBUG_STATUS) {
        Serial.println(" Movement! Restarting...");
      }
      calSamples = 0;
      calibration.gyroOffset[0] = 0;
      calibration.gyroOffset[1] = 0;
      calibration.gyroOffset[2] = 0;
      calStartTime = millis();
      calLastUpdate = millis();  // Reset the update timer too
      return;
    }
  }
  
  // Store current sample for next comparison
  gyroLastSample[0] = sensorData.gyro_x;
  gyroLastSample[1] = sensorData.gyro_y;
  gyroLastSample[2] = sensorData.gyro_z;
  
  // Accumulate samples
  calibration.gyroOffset[0] += sensorData.gyro_x;
  calibration.gyroOffset[1] += sensorData.gyro_y;
  calibration.gyroOffset[2] += sensorData.gyro_z;
  calSamples++;
  
  // Show progress every second
  if (currentTime - calLastUpdate >= 1000 && debugMode >= DEBUG_STATUS) {
    calLastUpdate = currentTime;
    int currentSecond = elapsedTime / 1000 + 1;
    if (currentSecond <= 5) {  // Only show 1-5
      Serial.print(currentSecond);
      Serial.print("... ");
    }
  }
  
  // Complete after specified time
  if (elapsedTime >= GYRO_CALIBRATION_TIME_MS) {
    // Calculate averages
    calibration.gyroOffset[0] /= calSamples;
    calibration.gyroOffset[1] /= calSamples;
    calibration.gyroOffset[2] /= calSamples;
    calibration.isCalibrated = true;
    
    if (debugMode >= DEBUG_STATUS) {
      Serial.println("✓");
      Serial.print("Gyro done: ");
      Serial.print(calSamples);
      Serial.print(" samples @ ");
      Serial.print(calSamples * 1000 / GYRO_CALIBRATION_TIME_MS);
      Serial.print("Hz, offsets=");
      Serial.print(calibration.gyroOffset[0], 3);
      Serial.print(",");
      Serial.print(calibration.gyroOffset[1], 3);
      Serial.print(",");
      Serial.println(calibration.gyroOffset[2], 3);
    }
    
    // Decide what to do next based on phase
    switch (currentCalPhase) {
      case PHASE_FIRST_TIME:
        if (!calibration.accelCalibrated) {
          startAccelCalibration();
        } else if (!calibration.magCalibrated) {
          startMagCalibration();
        } else {
          completeCalibration();
        }
        break;
        
      case PHASE_FULL_CAL:
        startAccelCalibration();
        break;
        
      case PHASE_GYRO_ONLY:
      default:
        completeCalibration();
        break;
    }
  }
}

void startAccelCalibration() {
  if (debugMode >= DEBUG_STATUS) {
    Serial.println("🎯 ACCEL CAL - 6 positions, 2sec each");
    Serial.println("Press Enter to begin...");
  }
  
  calState = CAL_ACCEL_WAITING;
  calStep = -1;  // Start at -1 so first action shows position 1
  calSamples = 0;
  memset(calData, 0, sizeof(calData));
  waitingForInput = true;
  inputPromptTime = millis();
}

void showAccelPosition(int position) {
  const char* positions[] = {
    "FLAT (PCB up)",
    "LEFT SIDE",
    "RIGHT SIDE",
    "NOSE UP", 
    "NOSE DOWN",
    "UPSIDE DOWN"
  };
  
  if (debugMode >= DEBUG_STATUS) {
    Serial.printf("Position %d/6: %s - Enter when ready\n", position, positions[position - 1]);
  }
  
  calState = CAL_ACCEL_WAITING;
  waitingForInput = true;
  inputPromptTime = millis();
}

void collectAccelData() {
  if (calSamples == 0) {
    // Clear data for this position
    calData[calStep][0] = 0;
    calData[calStep][1] = 0;
    calData[calStep][2] = 0;
    calStartTime = millis(); // Reset timer for THIS position
  }
  
  calData[calStep][0] += sensorData.accel_x;
  calData[calStep][1] += sensorData.accel_y;
  calData[calStep][2] += sensorData.accel_z;
  calSamples++;
  
  unsigned long elapsedTime = millis() - calStartTime;
  
  // Show progress every 0.4 seconds
  if ((elapsedTime / 400) > ((elapsedTime - 10) / 400) && debugMode >= DEBUG_STATUS) {
    Serial.print(".");
  }
  
  if (elapsedTime >= ACCEL_CALIBRATION_TIME_MS) {
    // Average the samples
    calData[calStep][0] /= calSamples;
    calData[calStep][1] /= calSamples;
    calData[calStep][2] /= calSamples;
    
    if (debugMode >= DEBUG_STATUS) {
      Serial.print(" ✓ (");
      Serial.print(calSamples);
      Serial.println(" samples)");
    }
    
    // Check if we have more positions to calibrate
    if (calStep < 5) {  // Positions 0-4 need another position
      calStep++;
      calSamples = 0; // Reset for next position
      showAccelPosition(calStep + 1);
    } else {  // calStep == 5, we're done with all 6 positions
      // Process calibration data directly
      processAccelCalibration();
    }
  }
}

void processAccelCalibration() {
  // Detect board orientation based on which axis shows ±1g in flat position
  float flat_x = calData[0][0];
  float flat_y = calData[0][1];
  float flat_z = calData[0][2];
  
  // Find which axis has the strongest gravity signal when flat
  float abs_x = fabs(flat_x);
  float abs_y = fabs(flat_y);
  float abs_z = fabs(flat_z);
  
  // Standard orientation: Z-axis should show -9.81 m/s² when flat
  // Calculate offsets and scale factors based on 6-point calibration
  // Positions: 0=flat, 1=left, 2=right, 3=nose up, 4=nose down, 5=upside down
  
  // X-axis calibration from nose up (3) and nose down (4)
  calibration.accelOffset[0] = (calData[3][0] + calData[4][0]) / 2.0;
  calibration.accelScale[0] = 19.62 / (calData[3][0] - calData[4][0]);
  
  // Y-axis calibration from left (1) and right (2)
  calibration.accelOffset[1] = (calData[1][1] + calData[2][1]) / 2.0;
  calibration.accelScale[1] = 19.62 / (calData[1][1] - calData[2][1]);
  
  // Z-axis calibration from flat (0) and upside down (5)
  calibration.accelOffset[2] = (calData[0][2] + calData[5][2]) / 2.0;
  calibration.accelScale[2] = 19.62 / (calData[0][2] - calData[5][2]);
  
  calibration.accelCalibrated = true;
  
  if (debugMode >= DEBUG_STATUS) {
    Serial.print("Accel done: ");
    if (abs_z > abs_x && abs_z > abs_y) {
      Serial.println("Standard mount (Z vertical)");
    } else if (abs_x > abs_y && abs_x > abs_z) {
      Serial.println("Side mount (X vertical)");
    } else {
      Serial.println("Edge mount (Y vertical)");
    }
  }
  
  // Next step based on phase
  switch (currentCalPhase) {
    case PHASE_FIRST_TIME:
    case PHASE_FULL_CAL:
      if (!calibration.magCalibrated) {
        startMagCalibration();
      } else {
        completeCalibration();
      }
      break;
      
    case PHASE_ACCEL_ONLY:
    default:
      completeCalibration();
      break;
  }
}

void startMagCalibration() {
  if (debugMode >= DEBUG_STATUS) {
    Serial.println("\n🧭 MAG CAL - Rotate slowly in ALL directions");
    Serial.println("Target: All axes need 85% coverage");
    if (IMU_RATE_HZ < 1000) {
      Serial.print("Note: Running at ");
      Serial.print(IMU_RATE_HZ);
      Serial.println("Hz - may take longer");
    }
    Serial.println("Press Enter to begin...");
  }
  
  calState = CAL_MAG_WAITING;
  magMin[0] = magMin[1] = magMin[2] = 1000;
  magMax[0] = magMax[1] = magMax[2] = -1000;
  magTotalRotation = 0;
  waitingForInput = true;
  inputPromptTime = millis();
}

void collectMagData() {
  // Track min/max values
  for (int i = 0; i < 3; i++) {
    float val = ((float*)&sensorData.mag_x)[i];
    if (val < magMin[i]) magMin[i] = val;
    if (val > magMax[i]) magMax[i] = val;
  }
  
  calSamples++;
  
  // Show progress every 2 seconds
  static unsigned long lastMagUpdate = 0;
  if (calSamples == 1) {
    // Reset the timer when starting collection
    lastMagUpdate = millis();
  }
  
  unsigned long currentTime = millis();
  if (currentTime - lastMagUpdate >= 2000 && debugMode >= DEBUG_STATUS) {
    lastMagUpdate = currentTime;
    
    // Calculate coverage for each axis
    float axisRange[3];
    int axisCoverage[3];
    bool allAxesGood = true;
    
    for (int i = 0; i < 3; i++) {
      axisRange[i] = magMax[i] - magMin[i];
      // Coverage is based on achieving the minimum range
      axisCoverage[i] = (int)((axisRange[i] / MAG_MIN_RANGE) * 100);
      if (axisCoverage[i] > 100) axisCoverage[i] = 100;
      if (axisCoverage[i] < MAG_GOOD_COVERAGE) allAxesGood = false;
    }
    
    // Update progress display on new line
    Serial.print("X:");
    Serial.print(axisCoverage[0]);
    Serial.print("% Y:");
    Serial.print(axisCoverage[1]);
    Serial.print("% Z:");
    Serial.print(axisCoverage[2]);
    Serial.print("% [");
    Serial.print((int)axisRange[0]);
    Serial.print(",");
    Serial.print((int)axisRange[1]);
    Serial.print(",");
    Serial.print((int)axisRange[2]);
    Serial.print(" uT]");
    
    if (!allAxesGood) {
      Serial.println(" - Keep rotating!");
    } else {
      Serial.println(" - Ready!");
    }
  }
  
  // Check completion criteria - all axes need good coverage
  bool complete = true;
  for (int i = 0; i < 3; i++) {
    float range = magMax[i] - magMin[i];
    if ((range / MAG_MIN_RANGE * 100) < MAG_GOOD_COVERAGE) {
      complete = false;
      break;
    }
  }
  
  if (complete && calSamples > 1000) {  // At least 1000 samples for good calibration
    // Calculate calibration values
    for (int i = 0; i < 3; i++) {
      calibration.magOffset[i] = (magMin[i] + magMax[i]) / 2.0;
      calibration.magScale[i] = 2.0 / (magMax[i] - magMin[i]);
    }
    calibration.magCalibrated = true;
    
    if (debugMode >= DEBUG_STATUS) {
      Serial.println("\n\n✓ Mag calibration complete!");
      Serial.print(calSamples);
      Serial.print(" samples collected. Final coverage: ");
      
      // Show final coverage
      for (int i = 0; i < 3; i++) {
        float range = magMax[i] - magMin[i];
        int coverage = (int)((range / MAG_MIN_RANGE) * 100);
        if (coverage > 100) coverage = 100;
        
        Serial.print(i == 0 ? "X:" : (i == 1 ? " Y:" : " Z:"));
        Serial.print(coverage);
        Serial.print("%");
      }
      Serial.println();
    }
    
    completeCalibration();
  }
}

void completeCalibration() {
  calState = CAL_IDLE;
  SET_STATE(STATE_IDLE);
  saveCalibration();
  currentCalPhase = PHASE_NONE;
  waitingForInput = false;
  
  if (debugMode >= DEBUG_STATUS) {
    Serial.println("\n✅ CALIBRATION COMPLETE!");
    Serial.println("System in IDLE mode. Type 'h' for help\n");
  }
}

// ===== DATA LOGGING =====
void logDataMAVLink() {
  // Create MAVLink messages
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;
  uint8_t tempBuffer[300]; // Temporary buffer for both messages
  uint16_t totalLen = 0;
  
  // Prepare HIGHRES_IMU message (if enabled in config)
  #if MAVLINK_USE_HIGHRES_IMU
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
  #endif
  
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
    return;
  }
  
  if (debugMode >= DEBUG_STATUS) {
    Serial.println(">>> Dumping pre-launch buffer...");
  }
  
  // Calculate how much data we have
  uint32_t dataSize = min(prelaunchWritePos, (uint32_t)PRELAUNCH_BUFFER_SIZE);
  uint32_t startPos = (prelaunchWritePos >= PRELAUNCH_BUFFER_SIZE) ? 
                      prelaunchWritePos : 0;
  
  // Copy pre-launch data to main buffer or file
  if (psramBuffer) {
    // Copy to main PSRAM buffer
    for (uint32_t i = 0; i < dataSize; i++) {
      uint32_t readPos = (startPos + i) % PRELAUNCH_BUFFER_SIZE;
      psramBuffer[psramWritePos] = prelaunchBuffer[readPos];
      psramWritePos = (psramWritePos + 1) % actualPSRAMSize;
    }
  } else {
    // Write directly to file
    if (!logFile) {
      openLogFile();
    }
    if (logFile) {
      // Write in chunks
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
    Serial.print("    Pre-launch: +");
    Serial.print(dataSize / 1024.0, 1);
    Serial.println(" KB");
  }
}

void flushPSRAMToFlash() {
  if (!psramBuffer || psramReadPos == psramWritePos) {
    return;
  }
  
  // Open log file if not already open
  if (!logFile) {
    openLogFile();
    if (!logFile) return;
  }
  
  // Calculate bytes to flush
  uint32_t bytesToFlush = (psramWritePos >= psramReadPos) ? 
                          (psramWritePos - psramReadPos) : 
                          (actualPSRAMSize - psramReadPos + psramWritePos);
  
  // Only flush if we have enough data or flush is forced
  if (bytesToFlush < MIN_FLUSH_SIZE && !psramFlushNeeded && isLogging) {
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
      psramReadPos = (psramReadPos + 1) % actualPSRAMSize;
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
  if (totalBytesWritten >= MAX_LOG_FILE_SIZE) {
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
      Serial.println("ERROR: Cannot create log file");
      
      // Check filesystem info
      FSInfo fs_info;
      if (LittleFS.info(fs_info)) {
        Serial.print("Free space: ");
        Serial.print((fs_info.totalBytes - fs_info.usedBytes) / 1024);
        Serial.println(" KB");
      }
    }
    SET_STATE(STATE_ERROR);
    return;
  }
  
  if (debugMode >= DEBUG_STATUS) {
    Serial.print(">>> Logging to: ");
    Serial.println(filename);
  }
}

// ===== CORE 1 FUNCTIONS =====
void core1_setup() {
  // Core 1 handles UI and LED status
}

void core1_loop() {
  // Handle serial commands
  if (Serial.available()) {
    handleSerialCommand();
  }
  
  // Check for automatic launch detection
  if (launchDetected && currentState == STATE_PREFLIGHT && !isLogging) {
    SET_STATE(STATE_ACTIVE);
    isLogging = true;
    capturePrelaunch = false;
    
    // Dump pre-launch buffer
    if (prelaunchBuffer) {
      dumpPrelaunchBuffer();
    }
    
    if (debugMode >= DEBUG_STATUS) {
      Serial.print("AUTO-LAUNCH: ");
      Serial.print(sensorData.totalAccel, 1);
      Serial.println("G detected!");
    }
  }
  
  // Check for automatic landing detection
  #if AUTO_STOP_ON_LANDING
  if (!launchDetected && currentState == STATE_ACTIVE && isLogging) {
    // Stop logging
    SET_STATE(STATE_IDLE);
    isLogging = false;
    psramFlushNeeded = true;
    delay(100);
    
    if (logFile) {
      uint32_t fileSize = logFile.size();
      logFile.close();
      hasStoredData = true;
      
      if (debugMode >= DEBUG_STATUS) {
        Serial.println("AUTO-STOP: Landing detected");
        printFlightSummary();
        Serial.print(">>> LOGGING STOPPED - Saved ");
        Serial.print(fileSize / 1024.0, 1);
        Serial.println(" KB");
      }
    }
    
    // Reset stats
    maxAcceleration = 0;
    maxAltitude = 0;
    launchAltitude = 0;
    stillnessStartTime = 0;
  }
  #endif
  
  // Update LED
  if (millis() - lastLedUpdate >= LED_UPDATE_RATE) {
    lastLedUpdate = millis();
    updateStatusLED();
  }
  
  // Small delay to not hog the CPU
  delay(10);
}

// ===== LED STATUS INDICATION =====
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
      // Solid yellow during calibration
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

// ===== USER INTERFACE =====
void printWelcomeMessage() {
  Serial.println();
  Serial.println("===========================================");
  Serial.print("       ");
  Serial.print(DEVICE_NAME);
  Serial.print(" v");
  Serial.println(FIRMWARE_VERSION);
  Serial.println("===========================================");
  Serial.print("Buffer: ");
  Serial.print(PSRAM_BUFFER_SIZE / (1024 * 1024));
  Serial.print("MB | Rates: ");
  Serial.print(IMU_RATE_HZ);
  Serial.print("/");
  Serial.print(BARO_RATE_HZ);
  Serial.print("/");
  Serial.print(LOG_RATE_HZ);
  Serial.println(" Hz");
  #if AUTO_START_ON_LAUNCH
    Serial.print("Auto-start: YES");
  #endif
  #if AUTO_STOP_ON_LANDING
    Serial.print(" | Auto-stop: YES");
  #endif
  Serial.println();
}

void printHelp() {
  Serial.println("\n=== COMMANDS ===");
  Serial.println("s - Start/stop logging");
  Serial.println("p - Enter/exit preflight mode");
  Serial.println("c - Recalibrate gyro | C - Full cal");
  Serial.println("A - Accel cal only | M - Mag cal only");
  Serial.println("X - Factory reset (delete all)");
  Serial.println("d0-3 - Debug: 0=Off 1=Status 2=Verbose 3=Tethered");
  Serial.println("d - Disable tethered mode");
  Serial.println("t - Trigger test accel (tethered+preflight)");
  Serial.println("m - Memory usage | h - Help");
  Serial.print("\nLaunch: ");
  Serial.print(LAUNCH_ACCEL_THRESHOLD);
  Serial.print("G | Pre-buffer: ");
  Serial.print(PRELAUNCH_BUFFER_SECONDS);
  Serial.println("s");
  Serial.println();
}

void handleSerialCommand() {
  if (!Serial.available()) return;
  
  String command = Serial.readStringUntil('\n');
  command.trim();
  
  if (command.length() == 0) return;
  
  char cmd = command.charAt(0);
  
  switch (cmd) {
    case 's':
    case 'S':
      if (currentState == STATE_IDLE || currentState == STATE_ACTIVE || currentState == STATE_PREFLIGHT) {
        if (!isLogging) {
          // Start logging
          if (currentState == STATE_IDLE) {
            // Manual start from IDLE
            SET_STATE(STATE_PREFLIGHT);
            #if CAPTURE_PRELAUNCH
              capturePrelaunch = true;
              prelaunchWritePos = 0;
            #endif
            launchDetected = false;
            
            // Then immediately go to ACTIVE
            SET_STATE(STATE_ACTIVE);
            isLogging = true;
            capturePrelaunch = false;
            if (debugMode >= DEBUG_STATUS) {
              Serial.println(">>> MANUAL LOGGING STARTED");
            }
          } else if (currentState == STATE_PREFLIGHT) {
            // Start from PREFLIGHT
            SET_STATE(STATE_ACTIVE);
            isLogging = true;
            capturePrelaunch = false;
            launchDetected = false;
            if (prelaunchBuffer) {
              dumpPrelaunchBuffer();
            }
            if (debugMode >= DEBUG_STATUS) {
              Serial.println(">>> LOGGING STARTED");
            }
          }
        } else {
          // Stop logging
          SET_STATE(STATE_IDLE);
          isLogging = false;
          psramFlushNeeded = true;
          delay(100); // Allow flush to complete
          
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
          
          // Reset flight statistics
          maxAcceleration = 0;
          maxAltitude = 0;
          launchAltitude = 0;
          launchDetected = false;
          stillnessStartTime = 0;
        }
      }
      break;
      
    case 'p':
    case 'P':
      if (currentState == STATE_IDLE) {
        // Enter preflight mode
        SET_STATE(STATE_PREFLIGHT);
        #if CAPTURE_PRELAUNCH
          capturePrelaunch = true;
          prelaunchWritePos = 0;
        #endif
        launchDetected = false;
        launchAltitude = sensorData.altitude;
        stillnessStartTime = 0;
        
        if (debugMode >= DEBUG_STATUS) {
          Serial.println(">>> PREFLIGHT MODE");
          #if CAPTURE_PRELAUNCH
            Serial.print("    Pre-buffer ON, launch>");
          #else  
            Serial.print("    Launch>");
          #endif
          Serial.print(launchThreshold);
          Serial.println("G");
        }
      } else if (currentState == STATE_PREFLIGHT) {
        // Exit preflight mode
        SET_STATE(STATE_IDLE);
        capturePrelaunch = false;
        if (debugMode >= DEBUG_STATUS) {
          Serial.println(">>> IDLE MODE");
        }
      }
      break;
      
    case 'd': {
      // If alone, disable tethered mode
      if (command.length() == 1) {
        if (tetheredModeActive) {
          tetheredModeActive = false;
          debugMode = DEBUG_STATUS;
          Serial.println(">>> TETHERED MODE DISABLED");
          Serial.println(">>> Switched to status mode");
        } else {
          Serial.println("Tethered mode is not active");
        }
      } else {
        // Extract mode number from command (e.g., "d2" or "D1")
        int mode = -1;
        if (command.length() >= 2) {
          mode = command.charAt(1) - '0';
        }
        
        if (mode >= 0 && mode <= 3) {
          debugMode = (DebugMode)mode;
          Serial.print("Debug mode set to: ");
          switch (debugMode) {
            case DEBUG_OFF: 
              Serial.println("OFF"); 
              tetheredModeActive = false;
              break;
            case DEBUG_STATUS: 
              Serial.println("STATUS"); 
              tetheredModeActive = false;
              break;
            case DEBUG_VERBOSE: 
              Serial.println("VERBOSE (10Hz)"); 
              tetheredModeActive = false;
              break;
            case DEBUG_TETHERED: 
              Serial.println("TETHERED MODE");
              tetheredModeActive = true;
              break;
          }
        } else {
          Serial.println("Invalid mode. Use d0-d3");
        }
      }
      break;
    }
      
    case 'c':
      if (currentState == STATE_IDLE) {
        SET_STATE(STATE_CALIBRATING);
        startCalibrationPhase(PHASE_GYRO_ONLY);
      } else {
        Serial.println("Must be in IDLE mode to calibrate");
      }
      break;
      
    case 'C':
      if (currentState == STATE_IDLE) {
        SET_STATE(STATE_CALIBRATING);
        startCalibrationPhase(PHASE_FULL_CAL);
      } else {
        Serial.println("Must be in IDLE mode for full calibration");
      }
      break;
      
    case 'A':
      if (currentState == STATE_IDLE) {
        SET_STATE(STATE_CALIBRATING);
        startCalibrationPhase(PHASE_ACCEL_ONLY);
      } else {
        Serial.println("Must be in IDLE mode for calibration");
      }
      break;
      
    case 'M':
      if (currentState == STATE_IDLE) {
        SET_STATE(STATE_CALIBRATING);
        startCalibrationPhase(PHASE_MAG_ONLY);
      } else {
        Serial.println("Must be in IDLE mode for calibration");
      }
      break;
      
    case 'X': {
      Serial.println(">>> DELETE ALL DATA? Type 'YES':");
      
      // Wait for response
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
        // Delete all files
        File root = LittleFS.open("/", "r");
        if (root) {
          int fileCount = 0;
          File file = root.openNextFile();
          while (file) {
            String fileName = String("/") + file.name();
            file.close();
            LittleFS.remove(fileName);
            fileCount++;
            file = root.openNextFile();
          }
          root.close();
          
          Serial.print(">>> Deleted ");
          Serial.print(fileCount);
          Serial.println(" files. Restarting...");
          delay(1000);
          
          // Restart RP2040/RP2350
          rp2040.restart();
        }
      } else {
        Serial.println(">>> Cancelled");
      }
      break;
    }
      
    case 't':
    case 'T':
      if (debugMode == DEBUG_TETHERED && currentState == STATE_PREFLIGHT) {
        testAccelValue = launchThreshold + 1.0;
        Serial.print(">>> TEST: Simulating ");
        Serial.print(testAccelValue);
        Serial.println("G acceleration!");
      } else {
        Serial.println("Tethered mode must be active and in PREFLIGHT state");
      }
      break;
      
    case 'm': {
      Serial.println("\nMEMORY:");
      
      // Filesystem info
      FSInfo fs_info;
      if (LittleFS.info(fs_info)) {
        Serial.print("Flash: ");
        Serial.print(fs_info.usedBytes * 100 / fs_info.totalBytes);
        Serial.print("% (");
        Serial.print(fs_info.usedBytes / 1024);
        Serial.print("/");
        Serial.print(fs_info.totalBytes / 1024);
        Serial.println("KB)");
      }
      
      Serial.print("RAM: ");
      Serial.print(rp2040.getFreeHeap() / 1024);
      Serial.print("/");
      Serial.print(rp2040.getTotalHeap() / 1024);
      Serial.println("KB free");
      
      if (psramBuffer) {
        uint32_t bufferUsed = (psramWritePos >= psramReadPos) ? 
                             (psramWritePos - psramReadPos) : 
                             (actualPSRAMSize - psramReadPos + psramWritePos);
        Serial.print("Buffer: ");
        Serial.print(bufferUsed * 100 / actualPSRAMSize);
        Serial.print("% (");
        Serial.print(bufferUsed / 1024);
        Serial.print("KB/");
        Serial.print(actualPSRAMSize / (1024 * 1024));
        Serial.print("MB)");
        
        // Time remaining
        if (isLogging) {
          uint32_t secondsRemaining = (actualPSRAMSize - bufferUsed) / BYTES_PER_SECOND;
          Serial.print(" ~");
          Serial.print(secondsRemaining / 60);
          Serial.print("m left");
        }
        Serial.println();
      }
      Serial.println();
      break;
    }
      
    case 'h':
    case 'H':
      printHelp();
      break;
      
    default:
      Serial.print("Unknown command: '");
      Serial.print(command);
      Serial.println("'. Type 'h' for help.");
      break;
  }
}

// ===== OUTPUT FUNCTIONS =====
void outputVerboseData() {
  // CSV format for easy import
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
  Serial.print(sensorData.temperature, 1);
  Serial.println();
}

void printStatus() {
  // Don't print in verbose mode to avoid cluttering output
  if (debugMode == DEBUG_VERBOSE) return;
  
  // Only print status for active states
  if (currentState == STATE_PREFLIGHT || currentState == STATE_ACTIVE) {
    Serial.print("[");
    switch (currentState) {
      case STATE_PREFLIGHT: Serial.print("PREFLT"); break;
      case STATE_ACTIVE: Serial.print("ACTIVE"); break;
      default: break;
    }
    Serial.print("] Alt: ");
    Serial.print(sensorData.altitude, 1);
    Serial.print("m | ");
    Serial.print(sensorData.totalAccel, 1);
    Serial.print("G | Buf: ");
    
    if (psramBuffer) {
      uint32_t bufferUsed = (psramWritePos >= psramReadPos) ? 
                           (psramWritePos - psramReadPos) : 
                           (actualPSRAMSize - psramReadPos + psramWritePos);
      Serial.print(bufferUsed * 100 / actualPSRAMSize);
    } else {
      Serial.print("0");
    }
    Serial.println("%");
  }
}

void printFlightSummary() {
  Serial.print("\nFLIGHT: ");
  Serial.print((millis() - launchTime) / 1000);
  Serial.print("s, ");
  Serial.print(maxAcceleration, 1);
  Serial.print("G max, ");
  Serial.print(maxAltitude, 1);
  Serial.println("m alt");
}

// ===== MAVLINK FUNCTIONS =====
void sendMAVLinkHeartbeat() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_heartbeat_pack(
    MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg,
    MAV_TYPE_ROCKET,           // type
    MAV_AUTOPILOT_INVALID,     // autopilot
    MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, // base_mode
    0,                         // custom_mode
    MAV_STATE_ACTIVE           // system_status
  );
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void sendMAVLinkAttitude() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_attitude_pack(
    MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg,
    millis(),                  // time_boot_ms
    sensorData.roll,           // roll
    sensorData.pitch,          // pitch
    sensorData.yaw,            // yaw
    sensorData.gyro_x,         // rollspeed
    sensorData.gyro_y,         // pitchspeed
    sensorData.gyro_z          // yawspeed
  );
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void sendMAVLinkHighresIMU() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_highres_imu_pack(
    MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg,
    sensorData.timestamp,      // time_usec
    sensorData.accel_x,        // xacc
    sensorData.accel_y,        // yacc
    sensorData.accel_z,        // zacc
    sensorData.gyro_x,         // xgyro
    sensorData.gyro_y,         // ygyro
    sensorData.gyro_z,         // zgyro
    sensorData.mag_x,          // xmag
    sensorData.mag_y,          // ymag
    sensorData.mag_z,          // zmag
    sensorData.pressure,       // abs_pressure
    0,                         // diff_pressure
    sensorData.altitude,       // pressure_alt
    sensorData.temperature,    // temperature
    0x3FFF,                    // fields_updated (all fields)
    0                          // id
  );
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void sendMAVLinkScaledPressure() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // The scaled_pressure message expects temperatures in centi-degrees (0.01 degrees)
  int16_t temp_scaled = (int16_t)(sensorData.temperature * 100);
  
  mavlink_msg_scaled_pressure_pack(
    MAVLINK_SYS_ID, MAVLINK_COMP_ID, &msg,
    millis(),                  // time_boot_ms
    sensorData.pressure,       // press_abs (hPa)
    0.0f,                      // press_diff (hPa) - we don't have differential pressure
    temp_scaled,               // temperature (centi-degrees Celsius)
    0                          // temperature_press_diff (centi-degrees) - we don't have this
  );
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

// ===== HELPER FUNCTIONS =====
const char* getStateName(SystemState state) {
  switch (state) {
    case STATE_INITIALIZING: return "INITIALIZING";
    case STATE_CALIBRATING: return "CALIBRATING";
    case STATE_IDLE: return "IDLE";
    case STATE_PREFLIGHT: return "PREFLIGHT";
    case STATE_ACTIVE: return "ACTIVE";
    case STATE_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

void simulateTestAcceleration() {
  if (tetheredModeActive && testAccelValue > launchThreshold) {
    // Temporarily override the total acceleration
    sensorData.totalAccel = testAccelValue;
    
    // Reset test value after one use
    testAccelValue = 1.0;
  }
}