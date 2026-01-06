// RocketChip.ino - Main sketch for Rocket Chip tracking board
// Target: Adafruit RP2350 Feather with 8MB PSRAM
// Version: 2.1 - Modular architecture

// Save Arduino's sq macro if it exists
#ifdef sq
#define ARDUINO_SQ sq
#undef sq
#endif

// Include AP libraries first
#include "libraries/AP_Common/AP_Common.h"
#include "libraries/AP_Math/AP_Math.h"

// Now include Arduino headers
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <LittleFS.h>

// Restore Arduino's sq macro if we saved it
#ifdef ARDUINO_SQ
#undef sq
#define sq ARDUINO_SQ
#undef ARDUINO_SQ
#endif

#include "config.h"
#include "SensorManager.h"
#include "DataLogger.h"
#include "StateTracking.h"
#include "SerialInterface.h"

// ===== GLOBAL OBJECTS =====
SensorManager sensors;
DataLogger logger;
StateTracking stateTracker;
SerialInterface serial;
Adafruit_NeoPixel pixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// ===== TIMING VARIABLES =====
unsigned long lastSensorUpdate = 0;
unsigned long lastLogUpdate = 0;
unsigned long lastStatusUpdate = 0;
unsigned long lastLedUpdate = 0;

// ===== LED ANIMATION =====
float ledBrightness = 0;
float ledPhase = 0;
bool ledToggle = false;
unsigned long ledToggleTime = 0;

// ===== FUNCTION PROTOTYPES =====
void updateStatusLED();
void core1_setup();
void core1_loop();

// ===== MAIN SETUP =====
void setup() {
  Serial.begin(115200);
  
  // === TEMPORARY TEST CODE ===
  Serial.println("Testing AP_Math...");
  Vector3f v1(1.0f, 2.0f, 3.0f);
  Serial.print("Vector length: ");
  Serial.println(v1.length());
  // === END TEST CODE ===
  //Serial.begin(SERIAL_BAUD);
  
  // Initialize I2C before anything else
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock
  delay(100); // Give I2C time to stabilize
  
  // Initialize NeoPixel
  pixel.begin();
  pixel.setBrightness(NEOPIXEL_BRIGHTNESS);
  pixel.setPixelColor(0, pixel.Color(255, 255, 0)); // Yellow for init
  pixel.show();
  
  // Initialize serial interface
  serial.begin();
  
  // Initialize file system
  if (!LittleFS.begin()) {
    serial.println(F("Error: Failed to mount file system!"));
    stateTracker.setState(SystemState::ERROR);
    return;
  }
  
  // Initialize data logger (includes PSRAM setup)
  if (!logger.begin()) {
    serial.println(F("Error: Failed to initialize data logger!"));
    stateTracker.setState(SystemState::ERROR);
    return;
  }
  
  // Initialize sensors
  if (!sensors.begin()) {
    serial.println(F("Error: Failed to initialize sensors!"));
    stateTracker.setState(SystemState::ERROR);
    return;
  }
  
  // Initialize state tracking
  stateTracker.begin(&sensors, &logger);
  
  // Check if we need initial calibration
  if (!sensors.isCalibrated()) {
    serial.println(F("No calibration found - starting first time setup"));
    stateTracker.setState(SystemState::CALIBRATING);
    sensors.startCalibration(CalPhase::PHASE_FIRST_TIME);
  } else {
    // Just do quick gyro calibration
    stateTracker.setState(SystemState::CALIBRATING);
    sensors.startCalibration(CalPhase::PHASE_GYRO_ONLY);
  }
  
  serial.setModules(&sensors, &logger, &stateTracker);
}

// ===== MAIN LOOP (CORE 0 - SENSORS AND LOGGING) =====
void loop() {
  unsigned long currentTime = micros();
  
  // Update sensors at high rate (1000Hz for IMU, 100Hz for baro)
  if (currentTime - lastSensorUpdate >= (1000000 / IMU_RATE_HZ)) {
    lastSensorUpdate = currentTime;
    
    sensors.updateIMU();
    
    // Update barometer at lower rate
    static uint8_t baroCounter = 0;
    if (++baroCounter >= (IMU_RATE_HZ / BARO_RATE_HZ)) {
      baroCounter = 0;
      sensors.updateBarometer();
    }
    
    // Update calibration if in calibration mode
    // This ensures calibration gets sensor data at full rate
    if (stateTracker.getCurrentState() == SystemState::CALIBRATING) {
      sensors.updateCalibration();
    }
    
    // Update state tracking (flight detection, etc)
    stateTracker.update();
  }
  
  // Log data at specified rate
  if ((logger.isLogging() || logger.isPreflightCapture()) && 
      (currentTime - lastLogUpdate >= (1000000 / LOG_RATE_HZ))) {
    lastLogUpdate = currentTime;
    
    const SensorData& data = sensors.getData();
    logger.logData(data, stateTracker.getCurrentState());
    
    // Track flight statistics
    stateTracker.updateFlightStats(data);
  }
  
  // Periodic status update
  if (millis() - lastStatusUpdate >= 1000) {
    lastStatusUpdate = millis();
    
    // Flush any pending data
    logger.update();
    
    // Send telemetry heartbeat if in verbose mode
    if (serial.getDebugMode() == DEBUG_VERBOSE) {
      logger.sendHeartbeat();
    }
  }
}

// ===== CORE 1 SETUP =====
void setup1() {
  core1_setup();
}

// ===== LOOP 1 (CORE 1 - UI AND LED) =====
void loop1() {
  core1_loop();
}

// ===== CORE 1 IMPLEMENTATION =====
void core1_setup() {
  // Core 1 handles UI and LED status
}

void core1_loop() {
  // Handle serial commands
  serial.update();
  
  // Auto launch/landing detection
  if (stateTracker.checkAutoLaunch()) {
    logger.startLogging();
    serial.println(F("AUTO-LAUNCH DETECTED!"));
  }
  
  if (stateTracker.checkAutoLanding()) {
    logger.stopLogging();
    serial.println(F("AUTO-LANDING DETECTED!"));
    serial.printFlightSummary(stateTracker.getFlightStats());
  }
  
  // Update LED
  if (millis() - lastLedUpdate >= LED_UPDATE_RATE) {
    lastLedUpdate = millis();
    updateStatusLED();
  }
  
  delay(10); // Don't hog CPU
}

// ===== LED STATUS INDICATION =====
void updateStatusLED() {
  unsigned long currentTime = millis();
  SystemState state = stateTracker.getCurrentState();
  
  switch (state) {
    case SystemState::INITIALIZING:
      // Pulsing yellow
      ledPhase += 0.0628;
      ledBrightness = (sin(ledPhase) + 1.0) * 0.5;
      pixel.setPixelColor(0, pixel.Color(255 * ledBrightness, 255 * ledBrightness, 0));
      break;
      
    case SystemState::CALIBRATING:
      // Solid yellow
      pixel.setPixelColor(0, pixel.Color(255, 255, 0));
      break;
      
    case SystemState::IDLE:
      if (logger.hasStoredData()) {
        // Alternating blue/green
        if (currentTime - ledToggleTime >= 1000) {
          ledToggle = !ledToggle;
          ledToggleTime = currentTime;
        }
        pixel.setPixelColor(0, ledToggle ? pixel.Color(0, 0, 255) : pixel.Color(0, 255, 0));
      } else {
        // Solid blue
        pixel.setPixelColor(0, pixel.Color(0, 0, 255));
      }
      break;
      
    case SystemState::PREFLIGHT:
      // Slow blinking blue
      if (currentTime - ledToggleTime >= 500) {
        ledToggle = !ledToggle;
        ledToggleTime = currentTime;
      }
      pixel.setPixelColor(0, ledToggle ? pixel.Color(0, 0, 255) : pixel.Color(0, 0, 0));
      break;
      
    case SystemState::ACTIVE:
      // Fast blinking green
      if (currentTime - ledToggleTime >= 100) {
        ledToggle = !ledToggle;
        ledToggleTime = currentTime;
      }
      pixel.setPixelColor(0, ledToggle ? pixel.Color(0, 255, 0) : pixel.Color(0, 0, 0));
      break;
      
    case SystemState::ERROR:
      // Solid red
      pixel.setPixelColor(0, pixel.Color(255, 0, 0));
      break;
  }
  
  pixel.show();
}