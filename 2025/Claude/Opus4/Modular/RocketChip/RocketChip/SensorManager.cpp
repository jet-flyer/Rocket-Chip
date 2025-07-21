// SensorManager.cpp - Sensor reading and calibration implementation
#include "SensorManager.h"
#include "SerialInterface.h"  // For DEBUG_STATUS
#include <LittleFS.h>

SensorManager::SensorManager() : 
  calState(CAL_IDLE),
  currentCalPhase(PHASE_NONE),
  waitingForInput(false),
  calStep(0),
  calSamples(0),
  testAccelOverride(1.0),
  testAccelActive(false),
  debugMode(DEBUG_STATUS) {
}

bool SensorManager::begin() {
  if (!initSensors()) {
    return false;
  }
  
  // Load calibration if available
  loadCalibration();
  
  // Initialize AHRS
  filter.begin(IMU_RATE_HZ);
  #ifdef MADGWICK_BETA
    filter.setBeta(MADGWICK_BETA);
  #endif
  
  return true;
}

bool SensorManager::initSensors() {
  if (debugMode >= DEBUG_STATUS) {
    Serial.print(F("Initializing sensors... "));
  }
  
  unsigned long startTime = millis();
  
  // Initialize ICM20948
  while (!icm.begin_I2C()) {
    if (millis() - startTime > SENSOR_INIT_TIMEOUT) {
      if (debugMode >= DEBUG_STATUS) {
        Serial.println(F("ICM20948 timeout!"));
      }
      return false;
    }
    delay(100);
  }
  
  // Configure ICM20948
  icm.setAccelRange(ACCEL_RANGE);
  icm.setGyroRange(GYRO_RANGE);
  icm.setAccelRateDivisor(0); // Maximum rate
  icm.setGyroRateDivisor(0);  // Maximum rate
  icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
  
  // Initialize DPS310
  startTime = millis();
  while (!dps.begin_I2C()) {
    if (millis() - startTime > SENSOR_INIT_TIMEOUT) {
      if (debugMode >= DEBUG_STATUS) {
        Serial.println(F("DPS310 timeout!"));
      }
      return false;
    }
    delay(100);
  }
  
  // Configure DPS310
  dps.configurePressure(PRESSURE_RATE, PRESSURE_SAMPLES);
  dps.configureTemperature(PRESSURE_RATE, PRESSURE_SAMPLES);
  
  if (debugMode >= DEBUG_STATUS) {
    Serial.println(F("OK"));
  }
  
  return true;
}

void SensorManager::updateIMU() {
  // Read ICM20948
  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);
  
  // Update sensor data
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
  
  // Apply calibration
  applyCalibration();
  
  // Update AHRS if not calibrating
  if (calState == CAL_IDLE) {
    updateAHRS();
  }
  
  // Apply test acceleration if active
  if (testAccelActive) {
    sensorData.totalAccel = testAccelOverride;
    testAccelActive = false;
  }
}

void SensorManager::updateBarometer() {
  sensors_event_t pressure_event, temp_event;
  dps.getEvents(&temp_event, &pressure_event);
  
  sensorData.pressure = pressure_event.pressure;
  sensorData.temperature = temp_event.temperature;
  sensorData.altitude = dps.readAltitude(SEA_LEVEL_PRESSURE);
}

void SensorManager::updateAHRS() {
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
  
  // Store orientation
  sensorData.roll = filter.getRoll() * 0.01745329;
  sensorData.pitch = filter.getPitch() * 0.01745329;
  sensorData.yaw = filter.getYaw() * 0.01745329;
  
  // Calculate total acceleration
  sensorData.totalAccel = sqrt(sensorData.accel_x * sensorData.accel_x +
                              sensorData.accel_y * sensorData.accel_y +
                              sensorData.accel_z * sensorData.accel_z) / 9.81;
}

void SensorManager::applyCalibration() {
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
}

bool SensorManager::loadCalibration() {
  if (!LittleFS.exists(CALIBRATION_FILE)) return false;
  
  File calFile = LittleFS.open(CALIBRATION_FILE, "r");
  if (!calFile) return false;
  
  CalibrationData tempCal;
  size_t bytesRead = calFile.read((uint8_t*)&tempCal, sizeof(tempCal));
  calFile.close();
  
  if (bytesRead == sizeof(tempCal) && tempCal.magicNumber == 0xCAFEBABE) {
    calibration = tempCal;
    if (debugMode >= DEBUG_STATUS) {
      Serial.print(F("Cal loaded: Accel="));
      Serial.print(calibration.accelCalibrated ? "Y" : "N");
      Serial.print(F(" Mag="));
      Serial.println(calibration.magCalibrated ? "Y" : "N");
    }
    return true;
  }
  return false;
}

void SensorManager::saveCalibration() {
  calibration.magicNumber = 0xCAFEBABE;
  File calFile = LittleFS.open(CALIBRATION_FILE, "w");
  if (calFile) {
    calFile.write((uint8_t*)&calibration, sizeof(calibration));
    calFile.close();
    if (debugMode >= DEBUG_STATUS) {
      Serial.println(F("✓ Calibration saved"));
    }
  }
}

bool SensorManager::startCalibration(CalPhase phase) {
  currentCalPhase = phase;
  
  switch (phase) {
    case PHASE_FIRST_TIME:
      if (debugMode >= DEBUG_STATUS) {
        Serial.println(F("\n🚀 FIRST TIME SETUP"));
      }
      delay(100);
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
        Serial.println(F("\n📊 FULL CALIBRATION"));
      }
      delay(100);
      startGyroCollection();
      break;
      
    default:
      return false;
  }
  
  return true;
}

bool SensorManager::updateCalibration() {
  if (calState == CAL_IDLE) return false;
  
  switch (calState) {
    case CAL_GYRO_COLLECTING:
      collectGyroData();
      break;
      
    case CAL_ACCEL_WAITING:
    case CAL_MAG_WAITING:
      // Waiting for user input - handled by handleCalibrationInput()
      // Add timeout check for debugging
      if (waitingForInput && (millis() - inputPromptTime > 30000)) {
        if (debugMode >= DEBUG_STATUS) {
          Serial.print(F("DEBUG: Timeout waiting for input at step "));
          Serial.println(calStep);
        }
      }
      break;
      
    case CAL_ACCEL_COLLECTING:
      collectAccelData();
      break;
      
    case CAL_MAG_COLLECTING:
      collectMagData();
      break;
  }
  
  return true;
}

void SensorManager::handleCalibrationInput() {
  waitingForInput = false;
  
  switch (calState) {
    case CAL_ACCEL_WAITING:
      if (calStep == -1) {
        // First time - show first position
        calStep = 0;
        showAccelPosition(1);
      } else if (calStep >= 0 && calStep <= 5) {
        // User pressed enter for a position - start collecting
        if (debugMode >= DEBUG_STATUS) {
          Serial.println(F("Collecting data..."));
        }
        // Small delay to let sensor stabilize after movement
        delay(200);
        calState = CAL_ACCEL_COLLECTING;
        calSamples = 0;
      }
      break;
      
    case CAL_MAG_WAITING:
      if (debugMode >= DEBUG_STATUS) {
        Serial.println(F("Start moving now! Progress:"));
      }
      calState = CAL_MAG_COLLECTING;
      calSamples = 0;
      break;
  }
}

void SensorManager::startGyroCollection() {
  if (debugMode >= DEBUG_STATUS) {
    Serial.println(F("📍 GYRO CAL - Keep still 5000 samples"));
  }
  calState = CAL_GYRO_COLLECTING;
  calSamples = 0;
  calibration.gyroOffset[0] = 0;
  calibration.gyroOffset[1] = 0;
  calibration.gyroOffset[2] = 0;
  
  // Initialize last samples to current readings
  gyroLastSample[0] = sensorData.gyro_x;
  gyroLastSample[1] = sensorData.gyro_y;
  gyroLastSample[2] = sensorData.gyro_z;
  
  gyroVariance[0] = gyroVariance[1] = gyroVariance[2] = 0;
  gyroIsStill = true;
}

void SensorManager::collectGyroData() {
  // Accumulate data
  calibration.gyroOffset[0] += sensorData.gyro_x;
  calibration.gyroOffset[1] += sensorData.gyro_y;
  calibration.gyroOffset[2] += sensorData.gyro_z;
  calSamples++;
  
  // Check for movement after we have enough samples (100+ samples)
  if (calSamples > 100) {
    float movement = 0;
    for (int i = 0; i < 3; i++) {
      float diff = ((float*)&sensorData.gyro_x)[i] - gyroLastSample[i];
      movement += fabs(diff);
    }
    
    // Use reasonable threshold (0.15 rad/s total across all axes)
    if (movement > 0.15) {
      if (debugMode >= DEBUG_STATUS) {
        Serial.println(F(" Movement! Restarting..."));
      }
      calSamples = 0;
      calibration.gyroOffset[0] = 0;
      calibration.gyroOffset[1] = 0;
      calibration.gyroOffset[2] = 0;
      // Don't update last samples on restart
      return;
    }
  }
  
  // Update last samples for next comparison
  gyroLastSample[0] = sensorData.gyro_x;
  gyroLastSample[1] = sensorData.gyro_y;
  gyroLastSample[2] = sensorData.gyro_z;
  
  // Show progress every 1000 samples
  if (calSamples % 1000 == 0 && debugMode >= DEBUG_STATUS) {
    Serial.print(calSamples / 1000);
    Serial.print(F("... "));
  }
  
  // Complete after 5000 samples
  if (calSamples >= 5000) {
    calibration.gyroOffset[0] /= calSamples;
    calibration.gyroOffset[1] /= calSamples;
    calibration.gyroOffset[2] /= calSamples;
    calibration.isCalibrated = true;
    
    if (debugMode >= DEBUG_STATUS) {
      Serial.println(F("✓"));
      Serial.print(F("Gyro done: "));
      Serial.print(calSamples);
      Serial.println(F(" samples"));
    }
    
    // Next phase
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

void SensorManager::startAccelCalibration() {
  if (debugMode >= DEBUG_STATUS) {
    Serial.println(F("🎯 ACCEL CAL - 6 positions, 2000 samples each"));
    Serial.println(F("Press Enter to begin..."));
  }
  
  calState = CAL_ACCEL_WAITING;
  calStep = -1;
  calSamples = 0;
  memset(calData, 0, sizeof(calData));
  waitingForInput = true;
  inputPromptTime = millis();
}

void SensorManager::showAccelPosition(int position) {
  const char* positions[] = {
    "FLAT (PCB up)",
    "LEFT SIDE",
    "RIGHT SIDE",
    "NOSE UP",
    "NOSE DOWN",
    "UPSIDE DOWN"
  };
  
  if (debugMode >= DEBUG_STATUS) {
    Serial.print(F("Position "));
    Serial.print(position);
    Serial.print(F("/6: "));
    Serial.print(positions[position - 1]);
    Serial.println(F(" - Enter when ready"));
  }
  
  calState = CAL_ACCEL_WAITING;
  waitingForInput = true;
  inputPromptTime = millis();
}

void SensorManager::collectAccelData() {
  // Initialize accumulator on first sample
  if (calSamples == 0) {
    calData[calStep][0] = 0;
    calData[calStep][1] = 0;
    calData[calStep][2] = 0;
  }
  
  // Just accumulate whatever data we get - don't check for zeros
  // The ICM20948 should always return some data, even if it's noise
  calData[calStep][0] += sensorData.accel_x;
  calData[calStep][1] += sensorData.accel_y;
  calData[calStep][2] += sensorData.accel_z;
  calSamples++;
  
  // Print dots every 100 samples
  if (calSamples % 100 == 0 && debugMode >= DEBUG_STATUS) {
    Serial.print(F("."));
  }
  
  // Complete after 2000 samples
  if (calSamples >= 2000) {
    // Calculate average
    calData[calStep][0] /= calSamples;
    calData[calStep][1] /= calSamples;
    calData[calStep][2] /= calSamples;
    
    if (debugMode >= DEBUG_STATUS) {
      Serial.print(F(" ✓ ("));
      Serial.print(calSamples);
      Serial.println(F(" samples)"));
      
      // Show the averages for debugging
      Serial.print(F("  Avg: X="));
      Serial.print(calData[calStep][0], 2);
      Serial.print(F(" Y="));
      Serial.print(calData[calStep][1], 2);
      Serial.print(F(" Z="));
      Serial.println(calData[calStep][2], 2);
    }
    
    // Move to next position
    calStep++;
    calSamples = 0;
    
    if (calStep < 6) {
      // Show next position
      showAccelPosition(calStep + 1);
    } else {
      // All positions done - process calibration
      processAccelCalibration();
    }
  }
}

void SensorManager::processAccelCalibration() {
  // Calculate offsets and scale factors
  calibration.accelOffset[0] = (calData[3][0] + calData[4][0]) / 2.0;
  calibration.accelScale[0] = 19.62 / (calData[3][0] - calData[4][0]);
  
  calibration.accelOffset[1] = (calData[1][1] + calData[2][1]) / 2.0;
  calibration.accelScale[1] = 19.62 / (calData[1][1] - calData[2][1]);
  
  calibration.accelOffset[2] = (calData[0][2] + calData[5][2]) / 2.0;
  calibration.accelScale[2] = 19.62 / (calData[0][2] - calData[5][2]);
  
  calibration.accelCalibrated = true;
  
  if (debugMode >= DEBUG_STATUS) {
    Serial.println(F("Accel calibration complete!"));
  }
  
  // Next phase
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

void SensorManager::startMagCalibration() {
  if (debugMode >= DEBUG_STATUS) {
    Serial.println(F("\n🧭 MAG CAL - Rotate slowly in ALL directions"));
    Serial.println(F("Target: All axes need 85% coverage"));
    Serial.println(F("Press Enter to begin..."));
  }
  
  calState = CAL_MAG_WAITING;
  // Initialize with extreme values that will be immediately replaced
  magMin[0] = magMin[1] = magMin[2] = 999999;
  magMax[0] = magMax[1] = magMax[2] = -999999;
  waitingForInput = true;
  inputPromptTime = millis();
}

void SensorManager::collectMagData() {
  // Track min/max values
  for (int i = 0; i < 3; i++) {
    float val = ((float*)&sensorData.mag_x)[i];
    if (val < magMin[i]) magMin[i] = val;
    if (val > magMax[i]) magMax[i] = val;
  }
  
  calSamples++;
  
  // Show progress every 1000 samples
  if (calSamples % 1000 == 0 && debugMode >= DEBUG_STATUS) {
    float axisRange[3];
    int axisCoverage[3];
    int goodAxes = 0;
    
    for (int i = 0; i < 3; i++) {
      axisRange[i] = magMax[i] - magMin[i];
      
      // Scale the expected range based on the actual values
      float expectedRange = MAG_MIN_RANGE;
      if (fabs(magMax[i]) > 1000 || fabs(magMin[i]) > 1000) {
        expectedRange = MAG_MIN_RANGE * 100;  // Scale up for raw sensor values
      }
      
      axisCoverage[i] = (int)((axisRange[i] / expectedRange) * 100);
      if (axisCoverage[i] > 100) axisCoverage[i] = 100;
      if (axisCoverage[i] >= MAG_GOOD_COVERAGE) goodAxes++;
    }
    
    Serial.print(F("X:"));
    Serial.print(axisCoverage[0]);
    Serial.print(F("% Y:"));
    Serial.print(axisCoverage[1]);
    Serial.print(F("% Z:"));
    Serial.print(axisCoverage[2]);
    Serial.print(F("% ["));
    Serial.print((int)axisRange[0]);
    Serial.print(F(","));
    Serial.print((int)axisRange[1]);
    Serial.print(F(","));
    Serial.print((int)axisRange[2]);
    Serial.print(F(" raw]"));
    
    if (goodAxes < 3) {
      Serial.println(F(" - Keep rotating!"));
    } else {
      Serial.println(F(" - Ready!"));
    }
  }
  
  // Check completion - need all 3 axes to have good coverage
  bool complete = true;
  for (int i = 0; i < 3; i++) {
    float axisRange = magMax[i] - magMin[i];
    float expectedRange = MAG_MIN_RANGE;
    if (fabs(magMax[i]) > 1000 || fabs(magMin[i]) > 1000) {
      expectedRange = MAG_MIN_RANGE * 100;
    }
    
    if ((axisRange / expectedRange * 100) < MAG_GOOD_COVERAGE) {
      complete = false;
      break;
    }
  }
  
  // Complete after 5000 samples minimum AND all axes have coverage
  if (complete && calSamples >= 5000) {
    for (int i = 0; i < 3; i++) {
      calibration.magOffset[i] = (magMin[i] + magMax[i]) / 2.0;
      calibration.magScale[i] = 2.0 / (magMax[i] - magMin[i]);
    }
    calibration.magCalibrated = true;
    
    if (debugMode >= DEBUG_STATUS) {
      Serial.println(F("\n✓ Mag calibration complete!"));
    }
    
    completeCalibration();
  }
}

void SensorManager::completeCalibration() {
  calState = CAL_IDLE;
  saveCalibration();
  currentCalPhase = PHASE_NONE;
  waitingForInput = false;
  
  if (debugMode >= DEBUG_STATUS) {
    Serial.println(F("\n✅ CALIBRATION COMPLETE!"));
    Serial.println(F("System ready. Type 'h' for help\n"));
  }
}

void SensorManager::setTestAcceleration(float g) {
  testAccelOverride = g;
  testAccelActive = true;
}