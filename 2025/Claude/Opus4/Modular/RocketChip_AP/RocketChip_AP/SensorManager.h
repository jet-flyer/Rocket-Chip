// SensorManager.h - Sensor reading and calibration management
#pragma once

#include <Arduino.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_DPS310.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHRS.h>
#include "config.h"

// ===== CALIBRATION TYPES =====
enum CalState { 
  CAL_IDLE,
  CAL_GYRO_COLLECTING,
  CAL_ACCEL_WAITING,
  CAL_ACCEL_COLLECTING,
  CAL_MAG_WAITING,
  CAL_MAG_COLLECTING
};

enum CalPhase {
  PHASE_NONE,
  PHASE_FIRST_TIME,
  PHASE_GYRO_ONLY,
  PHASE_ACCEL_ONLY,
  PHASE_MAG_ONLY,
  PHASE_FULL_CAL
};

// ===== SENSOR DATA STRUCTURE =====
struct SensorData {
  float accel_x, accel_y, accel_z;  // m/s^2
  float gyro_x, gyro_y, gyro_z;     // rad/s
  float mag_x, mag_y, mag_z;        // uT
  float pressure;                    // hPa
  float temperature;                 // Celsius
  float altitude;                    // meters
  float roll, pitch, yaw;            // radians
  uint32_t timestamp;                // microseconds
  float totalAccel;                  // magnitude in G's
};

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
};

// ===== SENSOR MANAGER CLASS =====
class SensorManager {
public:
  SensorManager();
  bool begin();
  
  // Sensor updates
  void updateIMU();
  void updateBarometer();
  void updateAHRS();
  
  // Data access
  const SensorData& getData() const { return sensorData; }
  float getTotalAccel() const { return sensorData.totalAccel; }
  float getAltitude() const { return sensorData.altitude; }
  
  // Calibration
  bool isCalibrated() const { return calibration.isCalibrated; }
  bool startCalibration(CalPhase phase);
  bool updateCalibration();  // Call from main loop during calibration
  CalState getCalibrationState() const { return calState; }
  bool isWaitingForInput() const { return waitingForInput; }
  void handleCalibrationInput();  // Call when user presses enter
  
  // Test mode support
  void setTestAcceleration(float g);
  
  // Debug output
  void setDebugMode(uint8_t mode) { debugMode = mode; }
  
private:
  // Sensor objects
  Adafruit_ICM20948 icm;
  Adafruit_DPS310 dps;
  Adafruit_Madgwick filter;
  
  // Sensor data
  SensorData sensorData;
  
  // Calibration
  CalibrationData calibration;
  volatile CalState calState;  // Shared between cores
  CalPhase currentCalPhase;
  volatile bool waitingForInput;  // Read by Core 1, written by Core 0
  unsigned long inputPromptTime;
  
  // Calibration working variables
  volatile int calStep;  // Also make this volatile to be safe
  int calSamples;
  float calData[6][3];
  float magMin[3], magMax[3];
  float gyroVariance[3];
  float gyroLastSample[3];
  bool gyroIsStill;
  float testAccelOverride;
  bool testAccelActive;
  
  // Debug
  uint8_t debugMode;
  
  // Internal methods
  bool initSensors();
  void applyCalibration();
  bool loadCalibration();
  void saveCalibration();
  
  // Calibration phases
  void startGyroCollection();
  void collectGyroData();
  void startAccelCalibration();
  void showAccelPosition(int position);
  void collectAccelData();
  void processAccelCalibration();
  void startMagCalibration();
  void collectMagData();
  void completeCalibration();
};