#include <Wire.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_DPS310.h>
#include <Adafruit_AHRS_Madgwick.h>
#include <Adafruit_NeoPixel.h>
#include <MAVLink.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <LittleFS.h>
#include <psram.h> // For pmalloc

// Pin definitions
#define NEOPIXEL_PIN 21 // GPIO 21 for onboard NeoPixel

// Constants
const float GRAVITY = 9.81; // m/s^2
const uint8_t SYSTEM_ID = 1;
const uint8_t COMPONENT_ID = 1;
const size_t MAX_LOG_SIZE = 7 * 1024 * 1024; // 7MB upper limit
const size_t MIN_LOG_SIZE = 1 * 1024 * 1024; // 1MB minimum
const unsigned long UPDATE_INTERVAL = 10; // 10ms for 100Hz
const float STILLNESS_THRESHOLD = 0.2; // m/s^2
const int CALIBRATION_SAMPLES = 100;
const int BUFFER_SIZE = 500; // 5 seconds at 100Hz
const float LAUNCH_ACCEL_THRESHOLD = 2.0 * GRAVITY; // 2g
const unsigned long LAUNCH_DETECT_DURATION = 500; // 0.5 seconds

// State enumeration with NeoPixel indicators
enum SystemState {
  INITIALIZED,  // Pulsing yellow: System initializing
  CALIBRATING,  // Solid yellow: Calibration in progress
  PRE_LAUNCH,   // Slow blinking blue: Armed, waiting for launch
  ACTIVE,       // Fast blinking green: Logging data
  STANDBY,      // Solid blue, flashing if data ready: Idle, ready for commands
  ERROR         // Solid red: Critical error
};

// Flight phase enumeration
enum FlightPhase {
  GROUND,
  BOOST,
  COAST,
  APOGEE,
  DESCENT
};

// Sensor data structure
struct SensorData {
  float gyro_x, gyro_y, gyro_z; // rad/s
  float accel_x, accel_y, accel_z; // m/s^2
  float mag_x, mag_y, mag_z; // uT
  float pressure; // hPa
  float temperature; // C
};

// MAVLink message buffer structure
struct MAVLinkBuffer {
  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;
};

// Circular buffer for pre-launch data
MAVLinkBuffer pre_launch_buffer[BUFFER_SIZE];
int buffer_index = 0;
bool buffer_full = false;

// Global variables
Adafruit_ICM20948 icm;
Adafruit_DPS310 dps;
Adafruit_Madgwick filter;
Adafruit_NeoPixel pixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
SystemState state = INITIALIZED;
FlightPhase current_phase = GROUND;
SensorData sensor_data;
float gyro_bias[3] = {0};
float accel_offset[3] = {0};
float mag_offset[3] = {0};
float ref_pressure = 1013.25; // hPa, updated during calibration
uint8_t* log_buffer = nullptr;
size_t log_size = 0; // Actual allocated size
size_t log_index = 0;
unsigned long last_update = 0;
bool accel_cal_needed = true;
bool mag_cal_needed = true;
bool test_mode = false;
unsigned long launch_detect_start = 0;
bool verbose_mode = false;
bool has_data = false;

// Custom function to calculate altitude from pressure
float pressureToAltitude(float pressure, float seaLevelPressure) {
  return 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Prompt for test mode
  Serial.println("Serial connection detected, enter test mode? Y/N");
  unsigned long start = millis();
  while (millis() - start < 5000) { // 5 seconds timeout
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'Y' || c == 'y') {
        test_mode = true;
        Serial.println("Entering test mode. Available commands:");
        Serial.println("1: Set state to INT");
        Serial.println("2: Set state to CAL");
        Serial.println("3: Set state to PRE");
        Serial.println("4: Set state to ACT");
        Serial.println("5: Set state to SBY");
        Serial.println("6: Set state to ERR");
        Serial.println("A: Set phase to GROUND");
        Serial.println("B: Set phase to BOOST");
        Serial.println("C: Set phase to COAST");
        Serial.println("D: Set phase to APOGEE");
        Serial.println("E: Set phase to DESCENT");
        Serial.println("V: Toggle verbose mode");
        Serial.println("M: Trigger magnetometer calibration");
        break;
      } else if (c == 'N' || c == 'n') {
        test_mode = false;
        break;
      }
    }
  }

  Wire.begin();

  // Initialize LittleFS
  if (!LittleFS.begin()) {
    Serial.println("Failed to initialize LittleFS");
    set_state(ERROR);
    return;
  }

  // Check for existing calibration data
  accel_cal_needed = !LittleFS.exists("accel_bias.txt");
  mag_cal_needed = !LittleFS.exists("mag_offset.txt");

  if (accel_cal_needed || mag_cal_needed) {
    Serial.println("No calibration data detected. Entering calibration wizard.");
  }

  // Load existing calibration data if present
  if (!accel_cal_needed) {
    File accel_file = LittleFS.open("accel_bias.txt", "r");
    if (accel_file) {
      for (int i = 0; i < 3; i++) accel_offset[i] = accel_file.readStringUntil('\n').toFloat();
      accel_file.close();
      Serial.println("Loaded accelerometer calibration from accel_bias.txt.");
    }
  }
  if (!mag_cal_needed) {
    File mag_file = LittleFS.open("mag_offset.txt", "r");
    if (mag_file) {
      for (int i = 0; i < 3; i++) mag_offset[i] = mag_file.readStringUntil('\n').toFloat();
      mag_file.close();
      Serial.println("Loaded magnetometer calibration from mag_offset.txt.");
    }
  }

  // Initialize ICM20948 (IMU)
  if (!icm.begin_I2C()) {
    Serial.println("Failed to initialize ICM20948");
    set_state(ERROR);
    return;
  }
  // Initialize DPS310 (barometer)
  if (!dps.begin_I2C()) {
    Serial.println("Failed to initialize DPS310");
    set_state(ERROR);
    return;
  }

  // Configure sensors
  icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
  dps.setMode(DPS310_CONT_PRESSURE);

  // Calibrate gyroscope on startup
  calibrate_gyro();

  // Initialize AHRS filter
  filter.begin(100);

  // Initialize NeoPixel
  pixel.begin();
  pixel.setBrightness(50);
  pixel.setPixelColor(0, pixel.Color(255, 255, 0)); // Yellow for initializing
  pixel.show();

  // Dynamically allocate log buffer
  size_t target_size = min((size_t)rp2040.getFreePSRAMHeap(), MAX_LOG_SIZE);
  while (target_size >= MIN_LOG_SIZE) {
    log_buffer = (uint8_t*)pmalloc(target_size);
    if (log_buffer) {
      log_size = target_size;
      break;
    }
    target_size -= 1 * 1024 * 1024; // Reduce by 1MB
  }

  if (!log_buffer) {
    Serial.println("Failed to allocate any log buffer");
    set_state(ERROR);
    return;
  }

  // Set initial state to INITIALIZED
  set_state(INITIALIZED);
}

void loop() {
  if (test_mode) {
    // Test mode: Handle user commands
    if (Serial.available()) {
      char c = Serial.read();
      switch (c) {
        case '1': set_state(INITIALIZED); break;
        case '2': set_state(CALIBRATING); break;
        case '3': set_state(PRE_LAUNCH); break;
        case '4': set_state(ACTIVE); break;
        case '5': set_state(STANDBY); break;
        case '6': set_state(ERROR); break;
        case 'A': set_phase(GROUND); break;
        case 'B': set_phase(BOOST); break;
        case 'C': set_phase(COAST); break;
        case 'D': set_phase(APOGEE); break;
        case 'E': set_phase(DESCENT); break;
        case 'V': verbose_mode = !verbose_mode; Serial.println(verbose_mode ? "Verbose mode enabled" : "Verbose mode disabled"); break;
        case 'M': mag_cal_needed = true; set_state(CALIBRATING); break;
      }
    }
  } else {
    // Normal operation
    handle_serial_commands();

    unsigned long current_time = millis();
    if (current_time - last_update >= UPDATE_INTERVAL) {
      last_update = current_time;

      switch (state) {
        case INITIALIZED:
          if (is_sensor_still()) {
            set_state(CALIBRATING);
          }
          break;
        case CALIBRATING:
          if (accel_cal_needed) {
            calibrate_accelerometer();
          } else if (mag_cal_needed) {
            calibrate_magnetometer();
          } else {
            set_state(STANDBY);
          }
          break;
        case PRE_LAUNCH:
          read_sensors();
          update_ahrs();
          log_pre_launch_data();
          if (detect_launch()) {
            set_state(ACTIVE);
            // Copy pre-launch buffer to log
            for (int i = 0; i < BUFFER_SIZE; i++) {
              int idx = (buffer_index + i) % BUFFER_SIZE;
              if (log_index + pre_launch_buffer[idx].len < log_size) {
                memcpy(log_buffer + log_index, pre_launch_buffer[idx].data, pre_launch_buffer[idx].len);
                log_index += pre_launch_buffer[idx].len;
              }
            }
          }
          break;
        case ACTIVE:
          read_sensors();
          update_ahrs();
          log_data();
          update_flight_phase();
          if (verbose_mode) print_sensor_data();
          break;
        case STANDBY:
          // Idle, waiting for commands
          break;
        case ERROR:
          // Error state, no operation
          break;
      }
      update_neopixel();
    }
  }
}

// Calibration functions
void calibrate_gyro() {
  const int samples = CALIBRATION_SAMPLES;
  float sum_gyro_x = 0, sum_gyro_y = 0, sum_gyro_z = 0;
  for (int i = 0; i < samples; i++) {
    sensors_event_t accel, gyro, mag, temp;
    icm.getEvent(&accel, &gyro, &mag, &temp);
    sum_gyro_x += gyro.gyro.x;
    sum_gyro_y += gyro.gyro.y;
    sum_gyro_z += gyro.gyro.z;
    delay(10);
  }
  gyro_bias[0] = sum_gyro_x / samples;
  gyro_bias[1] = sum_gyro_y / samples;
  gyro_bias[2] = sum_gyro_z / samples;
}

void calibrate_accelerometer() {
  if (is_sensor_still()) {
    Serial.println("Calibrating accelerometer. Place the board on a level surface.");
    const int samples = CALIBRATION_SAMPLES;
    float sum_accel_x = 0, sum_accel_y = 0, sum_accel_z = 0;
    for (int i = 0; i < samples; i++) {
      sensors_event_t accel, gyro, mag, temp;
      icm.getEvent(&accel, &gyro, &mag, &temp);
      sum_accel_x += accel.acceleration.x;
      sum_accel_y += accel.acceleration.y;
      sum_accel_z += accel.acceleration.z;
      delay(10);
    }
    accel_offset[0] = sum_accel_x / samples;
    accel_offset[1] = sum_accel_y / samples;
    accel_offset[2] = sum_accel_z / samples - GRAVITY;
    Serial.println("Accelerometer calibration complete. Data saved to accel_bias.txt.");
    File accel_file = LittleFS.open("accel_bias.txt", "w");
    if (accel_file) {
      for (int i = 0; i < 3; i++) accel_file.println(accel_offset[i]);
      accel_file.close();
      accel_cal_needed = false;
    } else {
      Serial.println("Failed to save accelerometer calibration data.");
      set_state(ERROR);
    }
  } else {
    Serial.println("Please keep the board still for calibration.");
  }
}

void calibrate_magnetometer() {
  Serial.println("Calibrating magnetometer. Rotate the board in a figure-eight pattern for 30 seconds.");
  unsigned long start_time = millis();
  float min_mag[3] = {1000, 1000, 1000};
  float max_mag[3] = {-1000, -1000, -1000};
  while (millis() - start_time < 30000) {
    sensors_event_t accel, gyro, mag, temp;
    icm.getEvent(&accel, &gyro, &mag, &temp);
    min_mag[0] = min(min_mag[0], mag.magnetic.x);
    min_mag[1] = min(min_mag[1], mag.magnetic.y);
    min_mag[2] = min(min_mag[2], mag.magnetic.z);
    max_mag[0] = max(max_mag[0], mag.magnetic.x);
    max_mag[1] = max(max_mag[1], mag.magnetic.y);
    max_mag[2] = max(max_mag[2], mag.magnetic.z);
    delay(10);
  }
  mag_offset[0] = (min_mag[0] + max_mag[0]) / 2;
  mag_offset[1] = (min_mag[1] + max_mag[1]) / 2;
  mag_offset[2] = (min_mag[2] + max_mag[2]) / 2;
  Serial.print("Magnetometer calibration complete. Offsets: ");
  Serial.print(mag_offset[0]); Serial.print(", ");
  Serial.print(mag_offset[1]); Serial.print(", ");
  Serial.println(mag_offset[2]);
  File mag_file = LittleFS.open("mag_offset.txt", "w");
  if (mag_file) {
    for (int i = 0; i < 3; i++) mag_file.println(mag_offset[i]);
    mag_file.close();
    mag_cal_needed = false;
  } else {
    Serial.println("Failed to save magnetometer calibration data.");
    set_state(ERROR);
  }
}

// State and phase management
void set_state(SystemState new_state) {
  if (new_state != state) {
    Serial.print(get_state_abbr(state));
    Serial.print("->");
    Serial.println(get_state_abbr(new_state));
    state = new_state;
  }
}

void set_phase(FlightPhase new_phase) {
  if (new_phase != current_phase) {
    Serial.print("Flight phase: ");
    Serial.println(get_phase_name(new_phase));
    if (new_phase == BOOST) {
      Serial.println("Launch detected");
    }
    current_phase = new_phase;
  }
}

String get_state_abbr(SystemState s) {
  switch (s) {
    case INITIALIZED: return "INT";
    case CALIBRATING: return "CAL";
    case PRE_LAUNCH: return "PRE";
    case ACTIVE: return "ACT";
    case STANDBY: return "SBY";
    case ERROR: return "ERR";
    default: return "UNK";
  }
}

String get_phase_name(FlightPhase p) {
  switch (p) {
    case GROUND: return "GROUND";
    case BOOST: return "BOOST";
    case COAST: return "COAST";
    case APOGEE: return "APOGEE";
    case DESCENT: return "DESCENT";
    default: return "UNKNOWN";
  }
}

// Sensor and logging functions
void handle_serial_commands() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "start") set_state(PRE_LAUNCH);
    else if (cmd == "stop") set_state(STANDBY);
    else if (cmd == "calibrate") set_state(CALIBRATING);
  }
}

bool is_sensor_still() {
  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &mag, &temp);
  float accel_magnitude = sqrt(accel.acceleration.x * accel.acceleration.x +
                               accel.acceleration.y * accel.acceleration.y +
                               accel.acceleration.z * accel.acceleration.z);
  return abs(accel_magnitude - GRAVITY) < STILLNESS_THRESHOLD;
}

void read_sensors() {
  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &mag, &temp);
  
  sensors_event_t temp_event, pressure_event;
  dps.getEvents(&temp_event, &pressure_event);

  sensor_data.gyro_x = gyro.gyro.x - gyro_bias[0];
  sensor_data.gyro_y = gyro.gyro.y - gyro_bias[1];
  sensor_data.gyro_z = gyro.gyro.z - gyro_bias[2];
  sensor_data.accel_x = accel.acceleration.x - accel_offset[0];
  sensor_data.accel_y = accel.acceleration.y - accel_offset[1];
  sensor_data.accel_z = accel.acceleration.z - accel_offset[2];
  sensor_data.mag_x = mag.magnetic.x - mag_offset[0];
  sensor_data.mag_y = mag.magnetic.y - mag_offset[1];
  sensor_data.mag_z = mag.magnetic.z - mag_offset[2];
  sensor_data.pressure = pressure_event.pressure;
  sensor_data.temperature = temp_event.temperature;
}

void update_ahrs() {
  filter.update(sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z,
                sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z,
                sensor_data.mag_x, sensor_data.mag_y, sensor_data.mag_z);
}

void log_pre_launch_data() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_heartbeat_pack(SYSTEM_ID, COMPONENT_ID, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  if (buffer_index < BUFFER_SIZE) {
    memcpy(pre_launch_buffer[buffer_index].data, buf, len);
    pre_launch_buffer[buffer_index].len = len;
    buffer_index++;
  } else {
    buffer_index = 0;
    buffer_full = true;
  }
}

bool detect_launch() {
  float accel_magnitude = sqrt(sensor_data.accel_x * sensor_data.accel_x +
                               sensor_data.accel_y * sensor_data.accel_y +
                               sensor_data.accel_z * sensor_data.accel_z);
  if (accel_magnitude > LAUNCH_ACCEL_THRESHOLD) {
    if (launch_detect_start == 0) launch_detect_start = millis();
    if (millis() - launch_detect_start >= LAUNCH_DETECT_DURATION) {
      launch_detect_start = 0;
      return true;
    }
  } else {
    launch_detect_start = 0;
  }
  return false;
}

void log_data() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_raw_imu_pack(SYSTEM_ID, COMPONENT_ID, &msg, micros(),
                           (int16_t)(sensor_data.accel_x * 1000), (int16_t)(sensor_data.accel_y * 1000), (int16_t)(sensor_data.accel_z * 1000),
                           (int16_t)(sensor_data.gyro_x * 1000), (int16_t)(sensor_data.gyro_y * 1000), (int16_t)(sensor_data.gyro_z * 1000),
                           (int16_t)(sensor_data.mag_x), (int16_t)(sensor_data.mag_y), (int16_t)(sensor_data.mag_z),
                           0,  // Sensor ID placeholder
                           (int16_t)(sensor_data.temperature * 100));  // Temperature in centidegrees
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  if (log_index + len < log_size) {
    memcpy(log_buffer + log_index, buf, len);
    log_index += len;
    has_data = true;
  }
}

void update_flight_phase() {
  float accel_magnitude = sqrt(sensor_data.accel_x * sensor_data.accel_x +
                               sensor_data.accel_y * sensor_data.accel_y +
                               sensor_data.accel_z * sensor_data.accel_z);
  float altitude = pressureToAltitude(sensor_data.pressure, ref_pressure);  // Updated to use custom function

  switch (current_phase) {
    case GROUND:
      if (detect_launch()) set_phase(BOOST);
      break;
    case BOOST:
      if (accel_magnitude < GRAVITY) set_phase(COAST);
      break;
    case COAST:
      if (altitude <= 0) set_phase(APOGEE);  // Note: This condition may need adjustment based on actual altitude behavior
      break;
    case APOGEE:
      if (altitude < 0) set_phase(DESCENT);  // Note: This condition may need adjustment based on actual altitude behavior
      break;
    case DESCENT:
      if (is_sensor_still()) set_phase(GROUND);
      break;
  }
}

void print_sensor_data() {
  Serial.print("Accel: "); Serial.print(sensor_data.accel_x); Serial.print(", ");
  Serial.print(sensor_data.accel_y); Serial.print(", "); Serial.println(sensor_data.accel_z);
  Serial.print("Gyro: "); Serial.print(sensor_data.gyro_x); Serial.print(", ");
  Serial.print(sensor_data.gyro_y); Serial.print(", "); Serial.println(sensor_data.gyro_z);
  Serial.print("Mag: "); Serial.print(sensor_data.mag_x); Serial.print(", ");
  Serial.print(sensor_data.mag_y); Serial.print(", "); Serial.println(sensor_data.mag_z);
  Serial.print("Pressure: "); Serial.println(sensor_data.pressure);
  Serial.print("Temp: "); Serial.println(sensor_data.temperature);
}

void update_neopixel() {
  static unsigned long last_blink = 0;
  static bool blink_state = false;
  unsigned long now = millis();

  switch (state) {
    case INITIALIZED:
      // Pulsing yellow
      pixel.setPixelColor(0, pixel.Color(255, 255, 0));
      break;
    case CALIBRATING:
      // Solid yellow
      pixel.setPixelColor(0, pixel.Color(255, 255, 0));
      break;
    case PRE_LAUNCH:
      // Slow blinking blue
      if (now - last_blink >= 500) {
        blink_state = !blink_state;
        pixel.setPixelColor(0, blink_state ? pixel.Color(0, 0, 255) : pixel.Color(0, 0, 0));
        last_blink = now;
      }
      break;
    case ACTIVE:
      // Fast blinking green
      if (now - last_blink >= 100) {
        blink_state = !blink_state;
        pixel.setPixelColor(0, blink_state ? pixel.Color(0, 255, 0) : pixel.Color(0, 0, 0));
        last_blink = now;
      }
      break;
    case STANDBY:
      // Solid blue, flashing if data ready
      if (has_data && now - last_blink >= 1000) {
        blink_state = !blink_state;
        pixel.setPixelColor(0, blink_state ? pixel.Color(0, 0, 255) : pixel.Color(0, 0, 0));
        last_blink = now;
      } else if (!has_data) {
        pixel.setPixelColor(0, pixel.Color(0, 0, 255));
      }
      break;
    case ERROR:
      // Solid red
      pixel.setPixelColor(0, pixel.Color(255, 0, 0));
      break;
  }
  pixel.show();
}