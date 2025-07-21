// RocketChip_MB.ino - Main sketch for Rocket Chip tracking board
// Target: Multi-board support (RP2350 HSTX, ESP32-S3 TFT)
// Version: 2.2 - Multi-board architecture

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <LittleFS.h>
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

// ===== PLATFORM-SPECIFIC VARIABLES =====
#if defined(BOARD_ESP32S3_TFT)
  TaskHandle_t Core1TaskHandle = NULL;
#endif

// ===== FUNCTION PROTOTYPES =====
void updateStatusLED();
void core1_setup();
void core1_loop();
#if defined(BOARD_ESP32S3_TFT)
  void core1_task(void* parameter);
#endif

// ===== MAIN SETUP =====
void setup() {
  Serial.begin(SERIAL_BAUD);
  
  // Print board info early
  Serial.println();
  Serial.print(F("RocketChip v"));
  Serial.print(FIRMWARE_VERSION);
  Serial.print(F(" on "));
  Serial.println(BOARD_NAME);
  
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
  
  // Platform-specific core setup
  #if defined(BOARD_ESP32S3_TFT)
    // Create Core 1 task for ESP32-S3
    xTaskCreatePinnedToCore(
      core1_task,           // Function
      "Core1Task",          // Name
      10000,                // Stack size
      NULL,                 // Parameters
      1,                    // Priority
      &Core1TaskHandle,     // Task handle
      1                     // Core number
    );
  #endif
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

// ===== PLATFORM-SPECIFIC CORE 1 SETUP =====
#if defined(BOARD_RP2350_HSTX)
// RP2350 uses automatic dual core setup
void setup1() {
  core1_setup();
}

void loop1() {
  core1_loop();
}
#endif

#if defined(BOARD_ESP32S3_TFT)
// ESP32-S3 uses manual task creation
void core1_task(void* parameter) {
  core1_setup();
  
  while(true) {
    core1_loop();
    vTaskDelay(1); // Yield to other tasks
  }
}
#endif

// ===== CORE 1 IMPLEMENTATION =====
void core1_setup() {
  // Core 1 handles UI and LED status
  #if HAS_BUTTONS
    // Initialize buttons for ESP32-S3 TFT
    pinMode(BUTTON_A, INPUT_PULLUP);
    pinMode(BUTTON_B, INPUT_PULLUP);
    pinMode(BUTTON_C, INPUT_PULLUP);
  #endif
}

void core1_loop() {
  // Handle serial commands
  serial.update();
  
  // Handle button input if available
  #if HAS_BUTTONS
    static bool lastButtonA = HIGH, lastButtonB = HIGH, lastButtonC = HIGH;
    bool buttonA = digitalRead(BUTTON_A);
    bool buttonB = digitalRead(BUTTON_B);
    bool buttonC = digitalRead(BUTTON_C);
    
    // Button A - Start/Stop logging (on press)
    if (lastButtonA == HIGH && buttonA == LOW) {
      serial.println(F("Button A - Start/Stop"));
      // Simulate 's' command
      Serial.println("s");
    }
    
    // Button B - Preflight mode (on press)
    if (lastButtonB == HIGH && buttonB == LOW) {
      serial.println(F("Button B - Preflight"));
      // Simulate 'p' command
      Serial.println("p");
    }
    
    // Button C - Help/Status (on press)
    if (lastButtonC == HIGH && buttonC == LOW) {
      serial.println(F("Button C - Help"));
      // Simulate 'h' command
      Serial.println("h");
    }
    
    lastButtonA = buttonA;
    lastButtonB = buttonB;
    lastButtonC = buttonC;
  #endif
  
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