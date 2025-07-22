#include <Wire.h>
#include <Adafruit_DPS310.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <RH_RF69.h>
#include <MAVLink.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>  // For malloc, etc.

// Pin and constant definitions
#define RFM69_CS 10                        // RFM69 chip select pin
#define RFM69_INT 6                        // RFM69 interrupt pin
#define RFM69_RST 11                       // RFM69 reset pin
#define RF69_FREQ 915.0                    // RFM69 frequency in MHz
#define MAVLINK_BUF_SIZE 64                // Buffer size for MAVLink messages
#define NEOPIXEL_PIN 21                    // NeoPixel pin
#define LED_PIN 7                          // Onboard LED pin
#define LOG_BUFFER_SIZE (2 * 1024 * 1024)  // 7 MB log buffer in PSRAM
#define SEA_LEVEL_PRESSURE_HPA 1013.25     // Sea level pressure for altitude calc
#define AHRS_SENSOR_RATE 100.0             // AHRS update rate in Hz
#define UPDATE_INTERVAL_US 10000           // 100 Hz sensor update interval
#define TX_INTERVAL_US 20000               // 50 Hz transmission interval

// Object instantiations
RH_RF69 rf69(RFM69_CS, RFM69_INT);                               // RFM69 radio
Adafruit_DPS310 dps;                                             // DPS310 pressure sensor
Adafruit_ICM20948 icm;                                           // ICM20948 IMU
Adafruit_Madgwick filter;                                        // Madgwick AHRS filter
Adafruit_NeoPixel pixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);  // NeoPixel

// Global variables
uint8_t mavlink_buf[MAVLINK_BUF_SIZE];  // Buffer for MAVLink messages
uint8_t* log_buffer = nullptr;          // PSRAM log buffer pointer
size_t write_pos = 0;                   // Current write position in log buffer
bool buffer_full = false;               // Flag for log buffer full
unsigned long last_update = 0, last_tx_update = 0, last_pixel_update = 0, last_heartbeat = 0;
bool log_started = false, high_rate_mode = false, radio_installed = false, gps_installed = false;
uint32_t log_index = 0, log_start_time = 0;
float pos_x = 0, pos_y = 0, pos_z = 0, vel_x = 0, vel_y = 0, vel_z = 0;  // Position and velocity
float bias_x = 0, bias_y = 0, bias_z = 0;                                // Gyroscope biases
float accel_offset_x = 0, accel_offset_y = 0, accel_offset_z = 0;        // Accelerometer offsets
float mag_offsets[3] = { 0, 0, 0 };                                      // Magnetometer offsets (optional)
float mag_scale[3] = { 1, 1, 1 };                                        // Magnetometer scaling factors (optional)

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
  float accel_x, accel_y, accel_z;        // Accelerometer data (m/s^2)
  float gyro_x, gyro_y, gyro_z;           // Gyroscope data (rad/s)
  float mag_x, mag_y, mag_z;              // Magnetometer data (uT)
  float pressure, altitude, temperature;  // Pressure (hPa), altitude (m), temp (C)
  float roll, pitch, yaw;                 // Orientation (radians)
  float gps_lat, gps_lon, gps_alt;        // GPS data (degrees, meters)
  bool gps_fix;                           // GPS fix status
} sensor_data;

// Function declarations
void detectRadio();
void calibrateSensors();
void updateNeoPixel();
void logMAVLinkMessage(mavlink_message_t* msg);
void detectGPS();  // Placeholder for GPS detection

void setup() {
  log_buffer = (uint8_t*)malloc(LOG_BUFFER_SIZE);
  Serial.printf("Free heap: %u bytes\n", rp2040.getFreeHeap());
  if (log_buffer == nullptr) {
    Serial.println("Failed to allocate log buffer");
    while (1)
      ;
  }
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

  filter.begin(AHRS_SENSOR_RATE);


  detectRadio();
  calibrateSensors();
  detectGPS();  // Check for GPS presence

  current_state = STANDBY;
  updateNeoPixel();
}

void loop() {
  unsigned long now_us = micros();
  if (now_us - last_update < UPDATE_INTERVAL_US) return;
  last_update = now_us;

  // Read IMU data
  sensors_event_t accel_event, gyro_event, mag_event, temp;
  icm.getEvent(&accel_event, &gyro_event, &mag_event, &temp);
  sensor_data.accel_x = accel_event.acceleration.x - accel_offset_x;
  sensor_data.accel_y = accel_event.acceleration.y - accel_offset_y;
  sensor_data.accel_z = accel_event.acceleration.z - accel_offset_z;
  sensor_data.gyro_x = gyro_event.gyro.x - bias_x;
  sensor_data.gyro_y = gyro_event.gyro.y - bias_y;
  sensor_data.gyro_z = gyro_event.gyro.z - bias_z;
  sensor_data.mag_x = mag_event.magnetic.x;  // Optional calibration not applied
  sensor_data.mag_y = mag_event.magnetic.y;
  sensor_data.mag_z = mag_event.magnetic.z;

  // Read pressure and temperature
  sensors_event_t temp_event, pressure_event;
  if (dps.getEvents(&temp_event, &pressure_event)) {
    sensor_data.pressure = pressure_event.pressure;
    sensor_data.temperature = temp_event.temperature;
    sensor_data.altitude = 44330.0 * (1.0 - pow(sensor_data.pressure / SEA_LEVEL_PRESSURE_HPA, 0.1903));
  }

  // Update AHRS filter
  filter.update(sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z,
                sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z,
                sensor_data.mag_x, sensor_data.mag_y, sensor_data.mag_z);
  sensor_data.roll = filter.getRoll();
  sensor_data.pitch = filter.getPitch();
  sensor_data.yaw = filter.getYaw();

  // Update position and velocity
  float dt = UPDATE_INTERVAL_US / 1000000.0;  // Delta time in seconds
  if (!gps_installed || !sensor_data.gps_fix) {
    // Use dead reckoning when GPS isn’t installed or no fix is available
    vel_x += sensor_data.accel_x * dt;
    vel_y += sensor_data.accel_y * dt;
    vel_z += sensor_data.accel_z * dt;
    pos_x += vel_x * dt;
    pos_y += vel_y * dt;
    pos_z += vel_z * dt;
  } else {
    // Correct position with GPS data when available
    pos_x = sensor_data.gps_lon;  // Assuming longitude for x
    pos_y = sensor_data.gps_lat;  // Assuming latitude for y
    pos_z = sensor_data.gps_alt;  // Altitude
                                  // Velocities could be computed from GPS if available; here we leave them unchanged
  }

  // Pack and send MAVLink messages
  if (radio_installed && now_us - last_tx_update >= TX_INTERVAL_US) {
    last_tx_update = now_us;
    uint32_t timestamp_ms = now_us / 1000;

    mavlink_message_t msg;
    uint16_t len;

    // SCALED_PRESSURE
    int16_t temperature_centidegrees = (int16_t)(sensor_data.temperature * 100);
    mavlink_msg_scaled_pressure_pack(1, 200, &msg, timestamp_ms, sensor_data.pressure, 0, temperature_centidegrees, 0);
    logMAVLinkMessage(&msg);
    len = mavlink_msg_to_send_buffer(mavlink_buf, &msg);
    rf69.send(mavlink_buf, len);
    rf69.waitPacketSent();

    // SCALED_IMU
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
    logMAVLinkMessage(&msg);
    len = mavlink_msg_to_send_buffer(mavlink_buf, &msg);
    rf69.send(mavlink_buf, len);
    rf69.waitPacketSent();

    // ATTITUDE
    mavlink_msg_attitude_pack(1, 200, &msg, timestamp_ms, sensor_data.roll, sensor_data.pitch, sensor_data.yaw,
                              sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z);
    logMAVLinkMessage(&msg);
    len = mavlink_msg_to_send_buffer(mavlink_buf, &msg);
    rf69.send(mavlink_buf, len);
    rf69.waitPacketSent();

    // LOCAL_POSITION_NED
    mavlink_msg_local_position_ned_pack(1, 200, &msg, timestamp_ms, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z);
    logMAVLinkMessage(&msg);
    len = mavlink_msg_to_send_buffer(mavlink_buf, &msg);
    rf69.send(mavlink_buf, len);
    rf69.waitPacketSent();

    // HEARTBEAT (sent every 1 second)
    if (millis() - last_heartbeat >= 1000) {
      mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC, 0, 0, 0);
      logMAVLinkMessage(&msg);
      len = mavlink_msg_to_send_buffer(mavlink_buf, &msg);
      rf69.send(mavlink_buf, len);
      rf69.waitPacketSent();
      last_heartbeat = millis();
    }
  }

  // Update NeoPixel every second
  if (millis() - last_pixel_update >= 1000) {
    updateNeoPixel();
    last_pixel_update = millis();
  }
}

// Helper Functions

/** Detect and initialize the RFM69 radio */
void detectRadio() {
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  if (!rf69.init()) {
    Serial.println("RFM69 init failed");
    while (1)
      ;
  }
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
    while (1)
      ;
  }
  rf69.setTxPower(20, true);  // Max power
  radio_installed = true;
}

/** Calibrate sensors (gyro and accelerometer) */
void calibrateSensors() {
  current_state = CALIBRATING;
  updateNeoPixel();

  float sum_gyro_x = 0, sum_gyro_y = 0, sum_gyro_z = 0;
  float sum_accel_x = 0, sum_accel_y = 0, sum_accel_z = 0;
  int samples = 100;
  for (int i = 0; i < samples; i++) {
    sensors_event_t gyro_event, accel_event;
    icm.getGyroSensor()->getEvent(&gyro_event);
    icm.getAccelerometerSensor()->getEvent(&accel_event);
    sum_gyro_x += gyro_event.gyro.x;
    sum_gyro_y += gyro_event.gyro.y;
    sum_gyro_z += gyro_event.gyro.z;
    sum_accel_x += accel_event.acceleration.x;
    sum_accel_y += accel_event.acceleration.y;
    sum_accel_z += accel_event.acceleration.z;
    delay(10);
  }
  // Gyro biases (should be close to 0 when stationary)
  bias_x = sum_gyro_x / samples;
  bias_y = sum_gyro_y / samples;
  bias_z = sum_gyro_z / samples;
  // Accelerometer offsets (assuming stationary and level: 0g in X/Y, +1g in Z)
  accel_offset_x = sum_accel_x / samples;         // Should be 0 in X
  accel_offset_y = sum_accel_y / samples;         // Should be 0 in Y
  accel_offset_z = sum_accel_z / samples - 9.81;  // Should be +1g in Z, subtract gravity
}

/** Update NeoPixel based on system state */
void updateNeoPixel() {
  switch (current_state) {
    case INITIALIZED: pixel.setPixelColor(0, pixel.Color(255, 255, 0)); break;  // Yellow
    case CALIBRATING: pixel.setPixelColor(0, pixel.Color(0, 255, 0)); break;    // Green
    case STANDBY: pixel.setPixelColor(0, pixel.Color(0, 0, 255)); break;        // Blue
    case ACTIVE: pixel.setPixelColor(0, pixel.Color(255, 0, 0)); break;         // Red
    case ERROR_STATE: pixel.setPixelColor(0, pixel.Color(255, 0, 0)); break;    // Red
  }
  pixel.show();
}

/** Log MAVLink message to PSRAM buffer */
void logMAVLinkMessage(mavlink_message_t* msg) {
  if (buffer_full) return;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
  if (write_pos + len >= LOG_BUFFER_SIZE) {
    buffer_full = true;
    Serial.println("Log buffer full. Logging stopped.");
    return;
  }
  memcpy(log_buffer + write_pos, buf, len);
  write_pos += len;
}

/** Placeholder for GPS detection */
void detectGPS() {
  // TODO: Implement GPS initialization and set gps_installed = true if detected
  gps_installed = false;  // Assume no GPS for now
}