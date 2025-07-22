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
bool radio_installed = false;  // Flag to indicate if radio is detected

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
    while (1)
      ;
  }
  dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);

  if (!icm.begin_I2C()) {
    current_state = ERROR_STATE;
    Serial.println("ICM20948 init failed");
    while (1)
      ;
  }
  icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  icm.setAccelRateDivisor(10);  // ~100 Hz
  icm.setGyroRateDivisor(10);   // ~100 Hz
  icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);

  filter.begin(100.0);

  // Detect if RFM69 radio is present
  detectRadio();

  // Set state to INITIALIZED (pulsing yellow)
  current_state = INITIALIZED;
  Serial.println("Waiting for sensor to be still...");

  // Wait for one second of stillness
  while (!isSensorStill()) {
    updateNeoPixel();  // Keep pulsing yellow
    delay(10);         // Small delay to avoid busy-waiting
  }
  Serial.println("Sensor is still, calibrating now...");

  // Switch to CALIBRATING state (solid yellow)
  current_state = CALIBRATING;
  updateNeoPixel();
  calibrateSensors();
  Serial.println("Calibration complete");

  // Switch to STANDBY state (solid blue)
  current_state = STANDBY;
  updateNeoPixel();
}

void loop() {
  unsigned long now_us = micros();
  if (now_us - last_update < UPDATE_INTERVAL_US) return;
  last_update = now_us;

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

  // Pack and send MAVLink messages if radio is installed
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

  // Print debug information every second
  if (millis() - last_debug_print >= 1000) {
    printDebugInfo();
    last_debug_print = millis();
  }

  if (millis() - last_pixel_update >= 1000) {
    updateNeoPixel();
    last_pixel_update = millis();
  }
}

/** Detect if RFM69 radio is present */
void detectRadio() {
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  if (!rf69.init()) {
    Serial.println("RFM69 init failed");
    radio_installed = false;
  } else if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
    radio_installed = false;
  } else {
    rf69.setTxPower(20, true);
    radio_installed = true;
    Serial.println("RFM69 radio detected and initialized");
  }
}

/** Calibrate sensors */
void calibrateSensors() {
  float sum_gyro_x = 0, sum_gyro_y = 0, sum_gyro_z = 0;
  float sum_accel_x = 0, sum_accel_y = 0, sum_accel_z = 0;
  int samples = 100;
  for (int i = 0; i < samples; i++) {
    sensors_event_t accel_event, gyro_event, mag_event, temp;
    icm.getEvent(&accel_event, &gyro_event, &mag_event, &temp);
    sum_gyro_x += gyro_event.gyro.x;
    sum_gyro_y += gyro_event.gyro.y;
    sum_gyro_z += gyro_event.gyro.z;
    sum_accel_x += accel_event.acceleration.x;
    sum_accel_y += accel_event.acceleration.y;
    sum_accel_z += accel_event.acceleration.z;
    delay(10);
  }
  bias_x = sum_gyro_x / samples;
  bias_y = sum_gyro_y / samples;
  bias_z = sum_gyro_z / samples;
  accel_offset_x = sum_accel_x / samples;
  accel_offset_y = sum_accel_y / samples;
  accel_offset_z = sum_accel_z / samples - 9.81;  // Adjust for gravity
}

/** Update NeoPixel based on state */
void updateNeoPixel() {
  unsigned long current_time = millis();

  switch (current_state) {
    case INITIALIZED:
      {
        // Pulsing yellow at 1 Hz (1000 ms period)
        float brightness = (sin(2 * PI * current_time / 1000.0) + 1) * 127.5;  // 0-255 range
        pixel.setPixelColor(0, pixel.Color(brightness, brightness, 0));
        break;
      }
    case CALIBRATING:
      pixel.setPixelColor(0, pixel.Color(255, 255, 0));  // Solid yellow
      break;
    case STANDBY:
      pixel.setPixelColor(0, pixel.Color(0, 0, 255));  // Solid blue
      break;
    case ACTIVE:
      {
        // Blinking green: 250 ms on, 750 ms off (1000 ms period)
        unsigned long cycle_time = current_time % 1000;
        if (cycle_time < 250) {
          pixel.setPixelColor(0, pixel.Color(0, 255, 0));  // Green on
        } else {
          pixel.setPixelColor(0, pixel.Color(0, 0, 0));  // Off
        }
        break;
      }
    case ERROR_STATE:
      pixel.setPixelColor(0, pixel.Color(255, 0, 0));  // Solid red
      break;
  }
  pixel.show();
}

/** Check if the sensor is still for one second */
bool isSensorStill() {
  const int sample_rate = 100;  // 100 Hz
  const int duration = 1000;    // 1 second
  const int num_samples = sample_rate * duration / 1000;
  float prev_x = 0, prev_y = 0, prev_z = 0;
  bool first_reading = true;

  for (int i = 0; i < num_samples; i++) {
    sensors_event_t accel_event, gyro_event, mag_event, temp;
    icm.getEvent(&accel_event, &gyro_event, &mag_event, &temp);
    float current_x = accel_event.acceleration.x;
    float current_y = accel_event.acceleration.y;
    float current_z = accel_event.acceleration.z;

    if (!first_reading) {
      float delta_x = abs(current_x - prev_x);
      float delta_y = abs(current_y - prev_y);
      float delta_z = abs(current_z - prev_z);
      if (delta_x > 0.2 || delta_y > 0.2 || delta_z > 0.2) {  // Threshold of 0.2 m/s²
        return false;
      }
    }
    prev_x = current_x;
    prev_y = current_y;
    prev_z = current_z;
    first_reading = false;
    delay(1000 / sample_rate);  // 10 ms per sample
  }
  return true;
}

/** Print debug information (mirrors RX side) */
void printDebugInfo() {
  Serial.print("State: ");
  switch (current_state) {
    case INITIALIZED: Serial.println("INITIALIZED"); break;
    case CALIBRATING: Serial.println("CALIBRATING"); break;
    case STANDBY: Serial.println("STANDBY"); break;
    case ACTIVE: Serial.println("ACTIVE"); break;
    case ERROR_STATE: Serial.println("ERROR"); break;
  }
  Serial.print("Accel (m/s²): ");
  Serial.print(sensor_data.accel_x);
  Serial.print(", ");
  Serial.print(sensor_data.accel_y);
  Serial.print(", ");
  Serial.println(sensor_data.accel_z);
  Serial.print("Gyro (rad/s): ");
  Serial.print(sensor_data.gyro_x);
  Serial.print(", ");
  Serial.print(sensor_data.gyro_y);
  Serial.print(", ");
  Serial.println(sensor_data.gyro_z);
  Serial.print("Mag (uT): ");
  Serial.print(sensor_data.mag_x);
  Serial.print(", ");
  Serial.print(sensor_data.mag_y);
  Serial.print(", ");
  Serial.println(sensor_data.mag_z);
  Serial.print("Pressure (hPa): ");
  Serial.println(sensor_data.pressure);
  Serial.print("Temperature (C): ");
  Serial.println(sensor_data.temperature);
  Serial.print("Altitude (m): ");
  Serial.println(sensor_data.altitude);
  Serial.print("Roll (rad): ");
  Serial.println(sensor_data.roll);
  Serial.print("Pitch (rad): ");
  Serial.println(sensor_data.pitch);
  Serial.print("Yaw (rad): ");
  Serial.println(sensor_data.yaw);
}