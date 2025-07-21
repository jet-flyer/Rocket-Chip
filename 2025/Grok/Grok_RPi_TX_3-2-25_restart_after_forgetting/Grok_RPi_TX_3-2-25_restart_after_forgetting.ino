#include <Wire.h>
#include <Adafruit_DPS310.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <RH_RF69.h>  // RadioHead RFM69 library
#include <MAVLink.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>  // For sine wave pulsing

// Pin and constant definitions
#define RFM69_CS 10      // RFM69 chip select pin
#define RFM69_INT 6      // RFM69 interrupt pin
#define RFM69_RST 11     // RFM69 reset pin
#define RF69_FREQ 915.0  // RFM69 frequency in MHz
#define MAVLINK_BUF_SIZE 64
#define NEOPIXEL_PIN 21
#define LED_PIN 7
#define SEA_LEVEL_PRESSURE_HPA 1013.25
#define UPDATE_INTERVAL_US 10000  // 100 Hz
#define TX_INTERVAL_US 20000      // 50 Hz

// Object instantiations
RH_RF69 rf69(RFM69_CS, RFM69_INT);  // RFM69 radio
Adafruit_DPS310 dps;
Adafruit_ICM20948 icm;
Adafruit_Madgwick filter;
Adafruit_NeoPixel pixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Global variables
uint8_t mavlink_buf[MAVLINK_BUF_SIZE];
unsigned long last_update = 0, last_tx_update = 0, last_pixel_update = 0, last_heartbeat = 0, last_debug_print = 0;
float pos_x = 0, pos_y = 0, pos_z = 0, vel_x = 0, vel_y = 0, vel_z = 0;
float bias_x = 0, bias_y = 0, bias_z = 0;
float accel_offset_x = 0, accel_offset_y = 0, accel_offset_z = 0;
bool radio_installed = false;

// System state enumeration
enum SystemState {
  INITIALIZED,
  CALIBRATING,
  STANDBY,
  ACTIVE,
  ERROR_STATE
} current_state = INITIALIZED;

// Sensor data structure
struct SensorData {
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float mag_x, mag_y, mag_z;
  float pressure, altitude, temperature;
  float roll, pitch, yaw;
} sensor_data;

// Function declarations
void detectRadio();
void calibrateSensors();
void updateNeoPixel();
bool isSensorStill();
void printDebugInfo();

void setup() {
  Serial.begin(921600);
  while (!Serial) delay(1);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pixel.begin();
  pixel.setBrightness(255);
  pixel.show();

  Wire.setSDA(2);
  Wire.setSCL(3);
  Wire.begin();

  if (!dps.begin_I2C()) {
    current_state = ERROR_STATE;
    Serial.println("DPS310 init failed");
    while (1);
  }
  dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);

  if (!icm.begin_I2C()) {
    current_state = ERROR_STATE;
    Serial.println("ICM20948 init failed");
    while (1);
  }
  icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  icm.setAccelRateDivisor(10);  // ~100 Hz
  icm.setGyroRateDivisor(10);   // ~100 Hz
  icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);

  filter.begin(100.0);

  detectRadio();

  current_state = INITIALIZED;
  Serial.println("Waiting for sensor to be still...");

  while (!isSensorStill()) {
    updateNeoPixel();
    delay(10);
  }
  Serial.println("Sensor is still, calibrating now...");

  current_state = CALIBRATING;
  updateNeoPixel();
  calibrateSensors();
  Serial.println("Calibration complete");

  // Enter STANDBY state
  current_state = STANDBY;
  updateNeoPixel();

  // Prompt user to start data logging
  Serial.println("hit ENTER to start data");

  // Wait for ENTER key press
  while (Serial.read() != '\n') {
    delay(100);  // Small delay to reduce CPU load
  }

  // Exit STANDBY and enter ACTIVE state
  current_state = ACTIVE;
  Serial.println("Data logging started");
}

void loop() {
  unsigned long now_us = micros();
  if (now_us - last_update < UPDATE_INTERVAL_US) return;
  last_update = now_us;

  // Only process data if in ACTIVE state
  if (current_state == ACTIVE) {
    sensors_event_t accel_event, gyro_event, mag_event, temp;
    icm.getEvent(&accel_event, &gyro_event, &mag_event, &temp);
    sensor_data.accel_x = accel_event.acceleration.x - accel_offset_x;
    sensor_data.accel_y = accel_event.acceleration.y - accel_offset_y;
    sensor_data.accel_z = accel_event.acceleration.z - accel_offset_z;
    sensor_data.gyro_x = gyro_event.gyro.x - bias_x;
    sensor_data.gyro_y = gyro_event.gyro.y - bias_y;
    sensor_data.gyro_z = gyro_event.gyro.z - bias_z;
    sensor_data.mag_x = mag_event.magnetic.x;
    sensor_data.mag_y = mag_event.magnetic.y;
    sensor_data.mag_z = mag_event.magnetic.z;

    sensors_event_t temp_event, pressure_event;
    if (dps.getEvents(&temp_event, &pressure_event)) {
      sensor_data.pressure = pressure_event.pressure;
      sensor_data.temperature = temp_event.temperature;
      sensor_data.altitude = 44330.0 * (1.0 - pow(sensor_data.pressure / SEA_LEVEL_PRESSURE_HPA, 0.1903));
    }

    filter.update(sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z,
                  sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z,
                  sensor_data.mag_x, sensor_data.mag_y, sensor_data.mag_z);
    sensor_data.roll = filter.getRoll();
    sensor_data.pitch = filter.getPitch();
    sensor_data.yaw = filter.getYaw();

    if (radio_installed && now_us - last_tx_update >= TX_INTERVAL_US) {
      last_tx_update = now_us;
      uint32_t timestamp_ms = now_us / 1000;
      mavlink_message_t msg;

      int16_t temperature_centidegrees = (int16_t)(sensor_data.temperature * 100);
      mavlink_msg_scaled_pressure_pack(1, 200, &msg, timestamp_ms, sensor_data.pressure, 0, temperature_centidegrees, 0);
      uint16_t len = mavlink_msg_to_send_buffer(mavlink_buf, &msg);
      rf69.send(mavlink_buf, len);
      rf69.waitPacketSent();

      mavlink_scaled_imu_t imu = {
        .xacc = (int16_t)(sensor_data.accel_x * 1000),
        .yacc = (int16_t)(sensor_data.accel_y * 1000),
        .zacc = (int16_t)(sensor_data.accel_z * 1000),
        .xgyro = (int16_t)(sensor_data.gyro_x * 1000),
        .ygyro = (int16_t)(sensor_data.gyro_y * 1000),
        .zgyro = (int16_t)(sensor_data.gyro_z * 1000),
        .xmag = (int16_t)(sensor_data.mag_x * 100),
        .ymag = (int16_t)(sensor_data.mag_y * 100),
        .zmag = (int16_t)(sensor_data.mag_z * 100),
        .temperature = temperature_centidegrees
      };
      mavlink_msg_scaled_imu_pack(1, 200, &msg, timestamp_ms, imu.xacc, imu.yacc, imu.zacc, imu.xgyro, imu.ygyro, imu.zgyro, imu.xmag, imu.ymag, imu.zmag, imu.temperature);
      len = mavlink_msg_to_send_buffer(mavlink_buf, &msg);
      rf69.send(mavlink_buf, len);
      rf69.waitPacketSent();

      mavlink_msg_attitude_pack(1, 200, &msg, timestamp_ms, sensor_data.roll, sensor_data.pitch, sensor_data.yaw,
                                sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z);
      len = mavlink_msg_to_send_buffer(mavlink_buf, &msg);
      rf69.send(mavlink_buf, len);
      rf69.waitPacketSent();

      mavlink_msg_local_position_ned_pack(1, 200, &msg, timestamp_ms, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z);
      len = mavlink_msg_to_send_buffer(mavlink_buf, &msg);
      rf69.send(mavlink_buf, len);
      rf69.waitPacketSent();

      if (millis() - last_heartbeat >= 1000) {
        mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC, 0, 0, 0);
        len = mavlink_msg_to_send_buffer(mavlink_buf, &msg);
        rf69.send(mavlink_buf, len);
        rf69.waitPacketSent();
        last_heartbeat = millis();
      }
    }

    if (millis() - last_debug_print >= 1000) {
      printDebugInfo();
      last_debug_print = millis();
    }
  }

  if (millis() - last_pixel_update >= 1000) {
    updateNeoPixel();
    last_pixel_update = millis();
  }
}

// Placeholder for helper functions (unchanged from original code)
void detectRadio() { /* Implementation */ }
void calibrateSensors() { /* Implementation */ }
void updateNeoPixel() { /* Implementation */ }
bool isSensorStill() { return true; /* Implementation */ }
void printDebugInfo() { 
  Serial.print("Accel: ");
  Serial.print(sensor_data.accel_x); Serial.print(", ");
  Serial.print(sensor_data.accel_y); Serial.print(", ");
  Serial.println(sensor_data.accel_z);
  // Add more debug output as needed
}