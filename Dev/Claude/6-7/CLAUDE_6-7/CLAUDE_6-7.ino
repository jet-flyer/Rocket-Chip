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
#define PSRAM_BUFFER_SIZE (7 * 1024 * 1024)  // 7MB main buffer in PSRAM
#define PSRAM_FLUSH_THRESHOLD (PSRAM_BUFFER_SIZE * 0.75)  // Flush at 75% full
#define MIN_FLUSH_SIZE (64 * 1024)  // Minimum 64KB before flushing to flash

// Pre-launch buffer configuration
#define PRELAUNCH_BUFFER_SECONDS 5  // Seconds of pre-launch data to keep
#define PRELAUNCH_BUFFER_SIZE (50 * 140 * PRELAUNCH_BUFFER_SECONDS)  // ~35KB for 5 seconds

// System state
enum SystemState {
  STATE_INITIALIZING,  // Pulsing yellow - POWER-ON ONLY
  STATE_CALIBRATING,   // Solid yellow
  STATE_IDLE,          // Solid blue (or blue/green if data available)
  STATE_PREFLIGHT,     // Slow blinking blue
  STATE_ACTIVE,        // Fast blinking green
  STATE_ERROR          // Solid red - CRITICAL FAILURE ONLY
};

// Compact state transition validation using bitfields
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

// Helper function for state names
const char* getStateName(SystemState state) {
  const char* names[] = {"INIT", "CAL", "IDLE", "PREFLT", "ACTIVE", "ERROR"};
  return (state < 6) ? names[state] : "UNKNOWN";
}

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
uint32_t actualPSRAMSize = PSRAM_BUFFER_SIZE; // Track actual allocated size

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

volatile SystemState currentState = STATE_INITIALIZING;
volatile bool isLogging = false;
volatile bool hasStoredData = false;
volatile bool sensorsReady = false;

// LED animation variables
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

// Enhanced calibration data structure
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

// Enhanced calibration state tracking
enum CalState { 
  CAL_IDLE, 
  CAL_GYRO, 
  CAL_ACCEL_SETUP,
  CAL_ACCEL_WAIT_POS,
  CAL_ACCEL_COLLECTING,
  CAL_MAG_SETUP,
  CAL_MAG_WAIT_START,
  CAL_MAG_COLLECTING,
  CAL_FIRST_TIME_SETUP
} calState = CAL_IDLE;
int calStep = 0;
int calSamples = 0;
float calData[6][3]; // For 6-point accel calibration
float magMin[3] = {1000, 1000, 1000};
float magMax[3] = {-1000, -1000, -1000};
bool firstTimeSetup = false;

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
void setupPSRAMBuffers();

// Enhanced calibration functions
bool loadCalibration();
void saveCalibration();
void startFirstTimeSetup();
void startGyroCalibration();
void startAccelCalibration();
void startMagCalibration();
void updateCalibration();

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
  
  // Set initial state (only place INITIALIZING can be set)
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
    SET_STATE(STATE_ERROR);
    return;
  }
  
  // Setup PSRAM buffers with proper pmalloc()
  setupPSRAMBuffers();
  
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

void setupPSRAMBuffers() {
  #if !defined(RP2350_PSRAM_CS)
    if (debugMode >= DEBUG_STATUS) {
      Serial.println("ERROR: RP2350 PSRAM not detected in build!");
    }
    SET_STATE(STATE_ERROR);
    return;
  #endif
  
  // Try to allocate main PSRAM buffer with pmalloc() - start with 7MB target
  uint32_t targetSize = PSRAM_BUFFER_SIZE;
  while (targetSize >= (512 * 1024) && !psramBuffer) {
    psramBuffer = (uint8_t*)pmalloc(targetSize);  // Use pmalloc() for PSRAM
    
    if (psramBuffer) {
      actualPSRAMSize = targetSize;
      if (debugMode >= DEBUG_STATUS) {
        Serial.print("PSRAM buffer: ");
        Serial.print(targetSize / (1024 * 1024));
        Serial.println(" MB allocated");
      }
      break;
    } else {
      targetSize /= 2;
    }
  }
  
  if (!psramBuffer) {
    if (debugMode >= DEBUG_STATUS) {
      Serial.println("Warning: PSRAM allocation failed, using direct write mode");
    }
  }
  
  // Allocate pre-launch buffer (use regular malloc for small sizes)
  prelaunchBuffer = (uint8_t*)malloc(PRELAUNCH_BUFFER_SIZE);
  if (prelaunchBuffer && debugMode >= DEBUG_STATUS) {
    Serial.print("Pre-launch buffer: ");
    Serial.print(PRELAUNCH_BUFFER_SIZE / 1024);
    Serial.println(" KB allocated");
  }
}

void setup1() {
  // Core 1 handles UI and status
  core1_setup();
}

void loop() {
  // Core 0: Handle sensors and logging
  
  // Read sensors at specified rate (always read for calibration)
  if (micros() - lastSensorRead >= (1000000 / SENSOR_RATE_HZ)) {
    lastSensorRead = micros();
    if (sensorsReady) {
      readSensors();
      // Only do AHRS if not calibrating
      if (calState == CAL_IDLE) {
        updateAHRS();
      }
    }
  }
  
  // Don't do logging operations during calibration
  if (calState != CAL_IDLE) {
    return;
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
    SET_STATE(STATE_ERROR);
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
    SET_STATE(STATE_ERROR);
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
  
  // Handle calibration sampling
  if (calState != CAL_IDLE) {
    updateCalibration();
  }
  
  // Apply calibration to sensor data
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
  
  // Apply test mode override if active
  if (testModeActive && currentState == STATE_PREFLIGHT) {
    simulateTestAcceleration();
  }
}

void updateCalibration() {
  // Check for user input on states that need it
  bool userInput = false;
  if (Serial.available()) {
    // Clear the input buffer
    while (Serial.available()) {
      Serial.read();
    }
    userInput = true;
  }
  
  switch (calState) {
    case CAL_FIRST_TIME_SETUP:
      if (userInput) {
        // Start with accelerometer calibration (skip gyro for first-time setup)
        if (!calibration.accelCalibrated) {
          startAccelCalibration();
        } else if (!calibration.magCalibrated) {
          startMagCalibration();
        } else {
          // Both are calibrated, just do quick gyro and finish
          startGyroCalibration();
        }
      }
      break;
      
    case CAL_ACCEL_SETUP:
      if (userInput) {
        // Start first position
        if (debugMode >= DEBUG_STATUS) {
          Serial.println("\nPosition 1/6: LEVEL (normal orientation, flat)");
          Serial.println("Press Enter when in position...");
        }
        calState = CAL_ACCEL_WAIT_POS;
        calStep = 0;
        calSamples = 0;
      }
      break;
      
    case CAL_ACCEL_WAIT_POS:
      if (userInput) {
        if (debugMode >= DEBUG_STATUS) {
          Serial.println("Collecting data...");
        }
        calState = CAL_ACCEL_COLLECTING;
        calSamples = 0;
      }
      break;
      
    case CAL_MAG_SETUP:
      if (userInput) {
        if (debugMode >= DEBUG_STATUS) {
          Serial.println("Start moving now! Progress:");
        }
        calState = CAL_MAG_COLLECTING;
        calSamples = 0;
      }
      break;
    
    case CAL_GYRO:
      // Collect gyro samples
      calibration.gyroOffset[0] += sensorData.gyro_x;
      calibration.gyroOffset[1] += sensorData.gyro_y;
      calibration.gyroOffset[2] += sensorData.gyro_z;
      calSamples++;
      
      if (calSamples % 20 == 0 && debugMode >= DEBUG_STATUS) Serial.print(".");
      
      if (calSamples >= 100) {
        calibration.gyroOffset[0] /= calSamples;
        calibration.gyroOffset[1] /= calSamples;
        calibration.gyroOffset[2] /= calSamples;
        calibration.isCalibrated = true;
        
        if (debugMode >= DEBUG_STATUS) {
          Serial.println(" ✓");
          Serial.printf("Gyro offsets: %.4f, %.4f, %.4f\n", 
                       calibration.gyroOffset[0], calibration.gyroOffset[1], calibration.gyroOffset[2]);
        }
        
        if (firstTimeSetup && !calibration.accelCalibrated) {
          startAccelCalibration();
        } else if (firstTimeSetup && !calibration.magCalibrated) {
          startMagCalibration();
        } else {
          calState = CAL_IDLE;
          SET_STATE(STATE_IDLE);
          if (firstTimeSetup) {
            saveCalibration();
            firstTimeSetup = false;
          }
          if (debugMode >= DEBUG_STATUS) {
            Serial.println("System ready!");
            Serial.println("Type 'h' for help");
          }
        }
      }
      break;
      
    case CAL_ACCEL_COLLECTING:
      calData[calStep][0] += sensorData.accel_x;
      calData[calStep][1] += sensorData.accel_y;
      calData[calStep][2] += sensorData.accel_z;
      calSamples++;
      
      if (calSamples % 10 == 0 && debugMode >= DEBUG_STATUS) Serial.print(".");
      
      if (calSamples >= 50) {
        // Average the samples
        calData[calStep][0] /= calSamples;
        calData[calStep][1] /= calSamples;
        calData[calStep][2] /= calSamples;
        
        if (debugMode >= DEBUG_STATUS) Serial.println(" ✓");
        
        calStep++;
        calSamples = 0;
        
        if (calStep < 6) {
          const char* positions[] = {"LEFT SIDE (90° left roll)", "RIGHT SIDE (90° right roll)", 
                                    "NOSE DOWN (90° forward pitch)", "NOSE UP (90° backward pitch)", 
                                    "UPSIDE DOWN (180° roll)"};
          if (debugMode >= DEBUG_STATUS) {
            Serial.printf("\nPosition %d/6: %s\n", calStep + 1, positions[calStep - 1]);
            Serial.println("Press Enter when in position...");
          }
          calState = CAL_ACCEL_WAIT_POS;
        } else {
          // Process 6-point calibration
          calibration.accelOffset[0] = (calData[3][0] + calData[4][0]) / 2.0; // nose up/down
          calibration.accelOffset[1] = (calData[1][1] + calData[2][1]) / 2.0; // left/right
          calibration.accelOffset[2] = (calData[0][2] + calData[5][2]) / 2.0; // level/inverted
          
          calibration.accelScale[0] = 19.62 / (calData[3][0] - calData[4][0]);
          calibration.accelScale[1] = 19.62 / (calData[1][1] - calData[2][1]);
          calibration.accelScale[2] = 19.62 / (calData[0][2] - calData[5][2]);
          
          calibration.accelCalibrated = true;
          
          if (debugMode >= DEBUG_STATUS) {
            Serial.println("✓ Accel calibration complete");
          }
          
          if (!calibration.magCalibrated) {
            startMagCalibration();
          } else {
            calState = CAL_IDLE;
            SET_STATE(STATE_IDLE);
            saveCalibration();
            firstTimeSetup = false;
            if (debugMode >= DEBUG_STATUS) {
              Serial.println("System ready!");
              Serial.println("Type 'h' for help");
            }
          }
        }
      }
      break;
      
    case CAL_MAG_COLLECTING:
      // Track min/max for hard iron correction
      for (int i = 0; i < 3; i++) {
        float val = ((float*)&sensorData.mag_x)[i];
        if (val < magMin[i]) magMin[i] = val;
        if (val > magMax[i]) magMax[i] = val;
      }
      calSamples++;
      
      if (calSamples % 50 == 0 && debugMode >= DEBUG_STATUS) {
        Serial.printf("%d%% ", calSamples * 100 / 600); // Updated for 600 samples (~12 seconds)
      }
      
      if (calSamples >= 600) { // Increased from 500 to 600 for better calibration
        // Calculate hard iron offsets and scaling
        for (int i = 0; i < 3; i++) {
          calibration.magOffset[i] = (magMin[i] + magMax[i]) / 2.0;
          calibration.magScale[i] = 2.0 / (magMax[i] - magMin[i]);
        }
        calibration.magCalibrated = true;
        
        if (debugMode >= DEBUG_STATUS) {
          Serial.println("✓ Mag calibration complete");
        }
        
        calState = CAL_IDLE;
        SET_STATE(STATE_IDLE);
        saveCalibration();
        firstTimeSetup = false;
        if (debugMode >= DEBUG_STATUS) {
          Serial.println("System ready!");
          Serial.println("Type 'h' for help");
        }
      }
      break;
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
    Serial.println("Checking calibration...");
  }
  
  // Try to load calibration
  if (loadCalibration()) {
    if (!calibration.accelCalibrated || !calibration.magCalibrated) {
      firstTimeSetup = true;
      SET_STATE(STATE_CALIBRATING);
      startFirstTimeSetup();
    } else {
      // Just do gyro calibration
      SET_STATE(STATE_CALIBRATING);
      startGyroCalibration();
    }
  } else {
    // No calibration file - first time setup
    firstTimeSetup = true;
    SET_STATE(STATE_CALIBRATING);
    startFirstTimeSetup();
  }
}

bool loadCalibration() {
  if (!LittleFS.exists("/calibration.dat")) return false;
  
  File calFile = LittleFS.open("/calibration.dat", "r");
  if (!calFile) return false;
  
  CalibrationData tempCal;
  size_t bytesRead = calFile.read((uint8_t*)&tempCal, sizeof(tempCal));
  calFile.close();
  
  if (bytesRead == sizeof(tempCal) && tempCal.magicNumber == 0xCAFEBABE) {
    calibration = tempCal;
    if (debugMode >= DEBUG_STATUS) {
      Serial.println("✓ Calibration loaded");
      Serial.print("  Accel: "); Serial.println(calibration.accelCalibrated ? "✓" : "✗");
      Serial.print("  Mag: "); Serial.println(calibration.magCalibrated ? "✓" : "✗");
    }
    return true;
  }
  return false;
}

void saveCalibration() {
  calibration.magicNumber = 0xCAFEBABE;
  File calFile = LittleFS.open("/calibration.dat", "w");
  if (calFile) {
    calFile.write((uint8_t*)&calibration, sizeof(calibration));
    calFile.close();
    if (debugMode >= DEBUG_STATUS) {
      Serial.println("✓ Calibration saved");
    }
  }
}

void startFirstTimeSetup() {
  if (debugMode >= DEBUG_STATUS) {
    Serial.println("\n🚀 FIRST TIME SETUP");
    Serial.println("This will calibrate accelerometer and magnetometer.");
    Serial.println("Gyroscope is automatically calibrated on each startup.");
    Serial.println("Please follow instructions carefully for each step.");
    Serial.println("Press Enter when ready to begin...");
  }
  
  calState = CAL_FIRST_TIME_SETUP;
  firstTimeSetup = true;
}

void startGyroCalibration() {
  if (debugMode >= DEBUG_STATUS) {
    Serial.println("\n📍 GYROSCOPE CALIBRATION");
    Serial.println("Keep device still and level...");
  }
  calState = CAL_GYRO;
  calSamples = 0;
  calibration.gyroOffset[0] = calibration.gyroOffset[1] = calibration.gyroOffset[2] = 0;
}

void startAccelCalibration() {
  if (debugMode >= DEBUG_STATUS) {
    Serial.println("\n🎯 ACCELEROMETER CALIBRATION");
    Serial.println("You will position the device in 6 different orientations.");
    Serial.println("Hold each position steady until you see ✓ COMPLETE");
    Serial.println("Take your time - accuracy is important!");
    Serial.println("Press Enter when ready to begin...");
  }
  calState = CAL_ACCEL_SETUP;
  calStep = 0;
  calSamples = 0;
  memset(calData, 0, sizeof(calData));
}

void startMagCalibration() {
  if (debugMode >= DEBUG_STATUS) {
    Serial.println("\n🧭 MAGNETOMETER CALIBRATION");
    Serial.println("Slowly rotate the device in ALL directions for 20 seconds:");
    Serial.println("• Move it in figure-8 patterns");
    Serial.println("• Rotate around all 3 axes");
    Serial.println("• Try to trace an imaginary sphere in the air");
    Serial.println("• Keep moving until progress reaches 100%");
    Serial.println("Press Enter when ready to begin...");
  }
  
  calState = CAL_MAG_SETUP;
  magMin[0] = magMin[1] = magMin[2] = 1000;
  magMax[0] = magMax[1] = magMax[2] = -1000;
}

void calibrateSensors() {
  // Legacy function - now just calls gyro calibration
  startGyroCalibration();
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
                              (actualPSRAMSize - psramReadPos + psramWritePos);
      
      if (bytesInBuffer + totalLen < actualPSRAMSize) {
        // Write to circular buffer
        for (uint16_t i = 0; i < totalLen; i++) {
          psramBuffer[psramWritePos] = tempBuffer[i];
          psramWritePos = (psramWritePos + 1) % actualPSRAMSize;
        }
        
        // Check if we need to flush
        if (bytesInBuffer + totalLen >= (actualPSRAMSize * 75 / 100)) {
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
      psramWritePos = (psramWritePos + 1) % actualPSRAMSize;
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
                          (actualPSRAMSize - psramReadPos + psramWritePos);
  
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
  if (totalBytesWritten >= 10 * 1024 * 1024) { // 10MB limit per file
    logFile.close();
    logFileNumber++;
    totalBytesWritten = 0;
  }
}

void openLogFile() {
  String filename = String("/log_") + String(logFileNumber) + ".mavlink";
  
  if (debugMode >= DEBUG_STATUS) {
    Serial.print("Opening log file: ");
    Serial.print(filename);
    Serial.print("... ");
  }
  
  logFile = LittleFS.open(filename, "w");
  if (!logFile) {
    if (debugMode >= DEBUG_STATUS) {
      Serial.println("FAILED!");
      Serial.println(">>> ERROR: Cannot create log file");
      Serial.print(">>> Free space: ");
      
      // Check filesystem info
      FSInfo fs_info;
      if (LittleFS.info(fs_info)) {
        Serial.print(fs_info.totalBytes - fs_info.usedBytes);
        Serial.println(" bytes");
      } else {
        Serial.println("unknown");
      }
    }
    SET_STATE(STATE_ERROR);
    return;
  }
  
  if (debugMode >= DEBUG_STATUS) {
    Serial.println("SUCCESS!");
    Serial.print(">>> Logging to: ");
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
  
  // Check for automatic launch detection (only from preflight to active)
  if (launchDetected && currentState == STATE_PREFLIGHT && !isLogging) {
    SET_STATE(STATE_ACTIVE);
    isLogging = true;
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
  Serial.println("           Version 1.1 Enhanced");
  Serial.println("      Advanced Calibration System");
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
  Serial.println("c - Recalibrate gyroscope only");
  Serial.println("C - Full calibration (gyro + accel + mag)");
  Serial.println("A - Accelerometer calibration only");
  Serial.println("M - Magnetometer calibration only");
  Serial.println("X - Factory reset: delete all data and restart");
  Serial.println("d0-3 - Change debug mode:");
  Serial.println("    d0 = Off, d1 = Status only");
  Serial.println("    d2 = Verbose (10Hz data)");
  Serial.println("    d3 = Test mode");
  Serial.println("t - (Test mode) Trigger test acceleration");
  Serial.println("m - Show memory usage");
  Serial.println("h - Show this help");
  Serial.println("==================");
  Serial.println();
}

void handleSerialCommand() {
  // Only process commands when a complete line is available
  if (!Serial.available()) return;
  
  String command = Serial.readStringUntil('\n');
  command.trim(); // Remove whitespace and newlines
  
  if (command.length() == 0) return; // Ignore empty lines
  
  char cmd = command.charAt(0); // Get first character of command
  
  switch (cmd) {
    case 's':
    case 'S':
      if (currentState == STATE_IDLE || currentState == STATE_ACTIVE || currentState == STATE_PREFLIGHT) {
        if (!isLogging) {
          // Start logging - requires PREFLIGHT or manual from IDLE
          if (currentState == STATE_IDLE) {
            // Manual start from IDLE - go through PREFLIGHT for safety
            SET_STATE(STATE_PREFLIGHT);
            capturePrelaunch = true;
            prelaunchWritePos = 0;
            launchDetected = false;
            testModeActive = false;
            
            // Then immediately go to ACTIVE for manual logging
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
          // Stop logging - transition back to IDLE
          SET_STATE(STATE_IDLE);
          isLogging = false;
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
        // Enter preflight mode - only from IDLE
        SET_STATE(STATE_PREFLIGHT);
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
        // Exit preflight mode - back to IDLE
        SET_STATE(STATE_IDLE);
        capturePrelaunch = false;
        testModeActive = false;
        if (debugMode >= DEBUG_STATUS) {
          Serial.println(">>> IDLE MODE");
        }
      }
      break;
      
    case 'c':
      if (currentState == STATE_IDLE) {
        // Quick gyro recalibration
        SET_STATE(STATE_CALIBRATING);
        startGyroCalibration();
      } else {
        Serial.println("Must be in IDLE mode to calibrate");
      }
      break;
      
    case 'C': // Full calibration
      if (currentState == STATE_IDLE) {
        firstTimeSetup = true;
        SET_STATE(STATE_CALIBRATING);
        startFirstTimeSetup();
        Serial.println(">>> FULL CALIBRATION STARTED");
      } else {
        Serial.println("Must be in IDLE mode for full calibration");
      }
      break;
      
    case 'A': // Accel only
      if (currentState == STATE_IDLE) {
        SET_STATE(STATE_CALIBRATING);
        if (debugMode >= DEBUG_STATUS) {
          Serial.println(">>> ACCEL CALIBRATION STARTED");
        }
        firstTimeSetup = false; // This is manual, not first-time
        startAccelCalibration();
      } else {
        Serial.println("Must be in IDLE mode for calibration");
      }
      break;
      
    case 'M': // Mag only  
      if (currentState == STATE_IDLE) {
        SET_STATE(STATE_CALIBRATING);
        if (debugMode >= DEBUG_STATUS) {
          Serial.println(">>> MAG CALIBRATION STARTED");
        }
        firstTimeSetup = false; // This is manual, not first-time
        startMagCalibration();
      } else {
        Serial.println("Must be in IDLE mode for calibration");
      }
      break;
      
    case 'X': { // Delete all data with confirmation - need braces for variable scope
      Serial.println(">>> WARNING: This will delete ALL stored data!");
      Serial.println(">>> - All calibration data");
      Serial.println(">>> - All flight log files");
      Serial.println(">>> Type 'YES' to confirm or any other key to cancel:");
      
      // Wait for response
      String response = "";
      unsigned long timeout = millis() + 10000; // 10 second timeout
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
        bool deletedCalibration = false;
        bool deletedLogs = false;
        
        // Delete calibration file
        if (LittleFS.exists("/calibration.dat")) {
          LittleFS.remove("/calibration.dat");
          deletedCalibration = true;
        }
        
        // Delete all log files
        int logCount = 0;
        for (int i = 0; i < 100; i++) {
          String filename = String("/log_") + String(i) + ".mavlink";
          if (LittleFS.exists(filename)) {
            LittleFS.remove(filename);
            logCount++;
          }
        }
        
        Serial.println(">>> DATA DELETION COMPLETE:");
        if (deletedCalibration) {
          Serial.println(">>>   ✓ Calibration data deleted");
        }
        if (logCount > 0) {
          Serial.print(">>>   ✓ ");
          Serial.print(logCount);
          Serial.println(" log files deleted");
        }
        if (!deletedCalibration && logCount == 0) {
          Serial.println(">>>   (No data found to delete)");
        }
        
        Serial.println(">>> RESTARTING FOR FRESH SETUP...");
        delay(1000);
        
        // Trigger software reset
        #ifdef ARDUINO_ARCH_RP2040
        rp2040.restart();
        #else
        // Fallback for other architectures
        NVIC_SystemReset();
        #endif
      } else {
        Serial.println(">>> Factory reset cancelled");
      }
      break;
    }
      
    case 'd':
    case 'D': {
      // Extract mode number from command (e.g., "d2" or "D1")
      int mode = -1;
      if (command.length() >= 2) {
        mode = command.charAt(1) - '0';
      }
      
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
      } else {
        Serial.println("Usage: d0, d1, d2, or d3");
      }
      break;
    }
      
    case 't':
    case 'T':
      if (debugMode == DEBUG_TEST && currentState == STATE_PREFLIGHT) {
        testAccelValue = launchThreshold + 1.0;
        Serial.println(">>> TEST: Simulating " + String(testAccelValue) + "G acceleration!");
      }
      break;
      
    case 'm': {
      Serial.println("=== MEMORY USAGE ===");
      
      // Filesystem info
      FSInfo fs_info;
      if (LittleFS.info(fs_info)) {
        Serial.print("Flash filesystem: ");
        Serial.print(fs_info.usedBytes / 1024);
        Serial.print(" KB used of ");
        Serial.print(fs_info.totalBytes / 1024);
        Serial.print(" KB total (");
        Serial.print(fs_info.usedBytes * 100 / fs_info.totalBytes);
        Serial.println("% full)");
      }
      
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
                                  (actualPSRAMSize - psramReadPos + psramWritePos);
        Serial.print("Main buffer: ");
        Serial.print(mainBufferUsed / 1024);
        Serial.print(" KB used of ");
        Serial.print(actualPSRAMSize / (1024 * 1024));
        Serial.println(" MB");
        Serial.print("Buffer usage: ");
        Serial.print(mainBufferUsed * 100 / actualPSRAMSize);
        Serial.println("%");
        
        // Show time remaining at current rate
        uint32_t bytesPerSecond = LOG_RATE_HZ * 140; // Approximate
        uint32_t secondsRemaining = (actualPSRAMSize - mainBufferUsed) / bytesPerSecond;
        Serial.print("Time until flush: ~");
        Serial.print(secondsRemaining / 60);
        Serial.print(" min ");
        Serial.print(secondsRemaining % 60);
        Serial.println(" sec");
      } else {
        Serial.println("No main buffer (direct write mode)");
      }
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
  Serial.print(getStateName(currentState));
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
                             (actualPSRAMSize - psramReadPos + psramWritePos)) * 100 / actualPSRAMSize;
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