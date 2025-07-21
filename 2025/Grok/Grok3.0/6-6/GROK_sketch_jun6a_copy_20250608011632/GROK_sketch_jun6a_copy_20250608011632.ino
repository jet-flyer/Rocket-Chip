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

// Debug macro
#define DEBUG_PRINTLN(str) if (debug_mode) Serial.println(str)

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
const int MOVING_AVG_WINDOW = 10; // Moving average window size
const float COMPLEMENTARY_ALPHA = 0.9; // Complementary filter gain

// State enumeration with NeoPixel indicators
enum SystemState {
  INITIALIZING,  // Pulsing yellow (1 Hz): System initializing, waiting for stillness
  CALIBRATING,   // Solid yellow: Calibration in progress
  IDLE,          // Solid blue (no data), Alternating blue/green (0.5 Hz, has data): Idle, ready for commands
  PREFLIGHT,     // Slow blinking blue (1 Hz): Preflight checks in progress
  ACTIVE,        // Fast blinking green (5 Hz): Logging data during flight
  ERROR          // Solid red: Critical error
};

// Flight phase enumeration
enum FlightPhase {
  GROUND,
  BOOST,
  COAST,
  APOGEE,
  DESCENT,
  LANDED
};

// Sensor data structure
struct SensorData {
  float gyro_x, gyro_y, gyro_z; // rad/s
  float accel_x, accel_y, accel_z; // m/s^2
  float mag_x, mag_y, mag_z; // uT
  float pressure; // hPa
  float temperature; // C
  float altitude; // m
  float velocity; // m/s
};

// Shared sensor data (IMU updated by Core 1, barometer by Core 0)
volatile SensorData shared_sensor_data;
volatile bool imu_data_ready = false;
volatile bool i2c_ready = false; // Synchronization flag for I2C

// MAVLink message buffer structure
struct MAVLinkBuffer {
  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;
};

// Circular buffer for pre-launch data
MAVLinkBuffer pre_launch_buffer[BUFFER_SIZE];
int buffer_index = 0;
bool buffer_full = false;

// Moving average filter buffers
float gyro_x_avg[MOVING_AVG_WINDOW], gyro_y_avg[MOVING_AVG_WINDOW], gyro_z_avg[MOVING_AVG_WINDOW];
float accel_x_avg[MOVING_AVG_WINDOW], accel_y_avg[MOVING_AVG_WINDOW], accel_z_avg[MOVING_AVG_WINDOW];
int avg_index = 0;

// Global variables
Adafruit_ICM20948 icm;
Adafruit_DPS310 dps;
Adafruit_Madgwick filter;
Adafruit_NeoPixel pixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
SystemState state = INITIALIZING;
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
bool debug_mode = false; // Debugging mode flag
unsigned long launch_detect_start = 0;
bool verbose_mode = false;
bool has_data = false;

// Calibration status tracking
bool nose_calibrated = false;
bool tail_calibrated = false;
bool right_calibrated = false;
bool left_calibrated = false;
bool front_calibrated = false;
bool back_calibrated = false;

// Custom altitude calculation
float pressureToAltitude(float pressure, float seaLevelPressure) {
  return 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));
}

// Moving average filter
void apply_moving_average(float* data, float new_value, float* output) {
  data[avg_index] = new_value;
  float sum = 0;
  for (int i = 0; i < MOVING_AVG_WINDOW; i++) sum += data[i];
  *output = sum / MOVING_AVG_WINDOW;
  avg_index = (avg_index + 1) % MOVING_AVG_WINDOW;
}

// Wait for user to press enter
void wait_for_enter() {
  Serial.println("Press enter when ready.");
  while (!Serial.available()) {
    delay(100);
  }
  while (Serial.available()) {
    Serial.read();
  }
}

// Collect average acceleration data for a given duration
void collect_average_accel(float& ax, float& ay, float& az, unsigned long duration) {
  float sum_x = 0, sum_y = 0, sum_z = 0;
  int count = 0;
  unsigned long start = millis();
  while (millis() - start < duration) {
    while (!imu_data_ready) {
      if (millis() - start > 2000) {
        DEBUG_PRINTLN("Timeout waiting for IMU data in collect_average_accel");
        return;
      }
    }
    sum_x += shared_sensor_data.accel_x;
    sum_y += shared_sensor_data.accel_y;
    sum_z += shared_sensor_data.accel_z;
    count++;
    imu_data_ready = false;
    delay(10);
  }
  ax = sum_x / count;
  ay = sum_y / count;
  az = sum_z / count;
}

// List remaining calibration steps
void list_remaining_calibrations() {
  Serial.println("Remaining accelerometer calibrations:");
  if (!nose_calibrated) Serial.println("- Nose up");
  if (!tail_calibrated) Serial.println("- Tail up");
  if (!right_calibrated) Serial.println("- Right up");
  if (!left_calibrated) Serial.println("- Left up");
  if (!front_calibrated) Serial.println("- Front up");
  if (!back_calibrated) Serial.println("- Back up");
  if (nose_calibrated && tail_calibrated && right_calibrated && 
      left_calibrated && front_calibrated && back_calibrated) {
    Serial.println("All orientations calibrated.");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
    DEBUG_PRINTLN("Waiting for Serial connection...");
  }
  DEBUG_PRINTLN("Serial initialized.");

  // Initialize moving average buffers
  for (int i = 0; i < MOVING_AVG_WINDOW; i++) {
    gyro_x_avg[i] = gyro_y_avg[i] = gyro_z_avg[i] = 0;
    accel_x_avg[i] = accel_y_avg[i] = accel_z_avg[i] = 0;
  }
  DEBUG_PRINTLN("Moving average buffers initialized.");

  // Prompt for test mode
  Serial.println("Serial connection detected, enter test mode? Y/N");
  unsigned long start = millis();
  while (millis() - start < 5000) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'Y' || c == 'y') {
        test_mode = true;
        Serial.println("Entering test mode. Available commands:");
        Serial.println("1: Set state to INI");
        Serial.println("2: Set state to CAL");
        Serial.println("3: Set state to PRE");
        Serial.println("4: Set state to ACT");
        Serial.println("5: Set state to IDL");
        Serial.println("6: Set state to ERR");
        Serial.println("A: Set phase to GROUND");
        Serial.println("B: Set phase to BOOST");
        Serial.println("C: Set phase to COAST");
        Serial.println("D: Set phase to APOGEE");
        Serial.println("E: Set phase to DESCENT");
        Serial.println("F: Set phase to LANDED");
        Serial.println("V: Toggle verbose mode");
        Serial.println("M: Trigger magnetometer calibration");
        Serial.println("Z: Clear calibration data");
        Serial.println("G: Toggle debug mode");
        Serial.println("X: Delete all flash data");
        break;
      } else if (c == 'N' || c == 'n') {
        test_mode = false;
        break;
      }
    }
  }
  DEBUG_PRINTLN("Test mode prompt completed.");

  Wire.begin();
  DEBUG_PRINTLN("I2C bus initialized.");
  i2c_ready = true; // Signal I2C is ready

  // Initialize LittleFS
  if (!LittleFS.begin()) {
    Serial.println("Failed to initialize LittleFS");
    set_state(ERROR);
    return;
  }
  DEBUG_PRINTLN("LittleFS initialized.");

  // Initialize DPS310 (barometer) on Core 0
  DEBUG_PRINTLN("Initializing DPS310...");
  if (!dps.begin_I2C()) {
    Serial.println("Failed to initialize DPS310");
    DEBUG_PRINTLN("DPS310 initialization failed - check I2C wiring or sensor presence.");
    set_state(ERROR);
    return;
  }
  dps.setMode(DPS310_CONT_PRESSURE);
  DEBUG_PRINTLN("DPS310 initialized.");

  // Check for existing calibration data with debug output
  Serial.print("Checking for accel_bias.txt: ");
  if (LittleFS.exists("accel_bias.txt")) {
    Serial.println("found.");
    File accel_file = LittleFS.open("accel_bias.txt", "r");
    if (accel_file) {
      for (int i = 0; i < 3; i++) {
        String line = accel_file.readStringUntil('\n');
        accel_offset[i] = line.toFloat();
        Serial.print("Loaded accel_offset["); Serial.print(i); Serial.print("]: "); Serial.println(accel_offset[i]);
      }
      accel_file.close();
      DEBUG_PRINTLN("Loaded accelerometer calibration from accel_bias.txt.");
      nose_calibrated = tail_calibrated = right_calibrated = 
      left_calibrated = front_calibrated = back_calibrated = true; // Assume complete if file exists
      accel_cal_needed = false;
    } else {
      Serial.println("Failed to open accel_bias.txt for reading.");
      accel_cal_needed = true;
    }
  } else {
    Serial.println("not found.");
    accel_cal_needed = true;
  }

  Serial.print("Checking for mag_offset.txt: ");
  if (LittleFS.exists("mag_offset.txt")) {
    Serial.println("found.");
    File mag_file = LittleFS.open("mag_offset.txt", "r");
    if (mag_file) {
      for (int i = 0; i < 3; i++) {
        String line = mag_file.readStringUntil('\n');
        mag_offset[i] = line.toFloat();
        Serial.print("Loaded mag_offset["); Serial.print(i); Serial.print("]: "); Serial.println(mag_offset[i]);
      }
      mag_file.close();
      DEBUG_PRINTLN("Loaded magnetometer calibration from mag_offset.txt.");
      mag_cal_needed = false;
    } else {
      Serial.println("Failed to open mag_offset.txt for reading.");
      mag_cal_needed = true;
    }
  } else {
    Serial.println("not found.");
    mag_cal_needed = true;
  }

  if (accel_cal_needed && mag_cal_needed) {
    Serial.println("No calibration data detected. Entering calibration wizard.");
  } else if (accel_cal_needed) {
    Serial.println("No accelerometer calibration data detected. Entering calibration wizard.");
  } else if (mag_cal_needed) {
    Serial.println("No magnetometer calibration data detected. Entering calibration wizard.");
  }

  // Initialize NeoPixel
  DEBUG_PRINTLN("Initializing NeoPixel...");
  pixel.begin();
  pixel.setBrightness(50);
  pixel.setPixelColor(0, pixel.Color(255, 255, 0)); // Pulsing yellow for initializing
  pixel.show();
  DEBUG_PRINTLN("NeoPixel initialized.");

  // Dynamically allocate log buffer
  DEBUG_PRINTLN("Allocating log buffer...");
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
  DEBUG_PRINTLN("Log buffer allocated successfully.");

  // Set initial state to INITIALIZING
  set_state(INITIALIZING);
  DEBUG_PRINTLN("Setup completed, entering main loop...");
}

// Setup for Core 1
void setup1() {
  DEBUG_PRINTLN("Core 1: Waiting for I2C initialization...");
  while (!i2c_ready) {
    delay(10); // Wait for Core 0 to initialize I2C
  }
  DEBUG_PRINTLN("Core 1: Setup started");
  DEBUG_PRINTLN("Core 1: Initializing ICM20948...");
  if (!icm.begin_I2C()) {
    Serial.println("Core 1: Failed to initialize ICM20948");
    DEBUG_PRINTLN("ICM20948 initialization failed - check I2C wiring, power, or address conflicts.");
    set_state(ERROR);
    while (1) {
      DEBUG_PRINTLN("Core 1: ICM20948 init failed, halted.");
      delay(1000);
    }
  }
  DEBUG_PRINTLN("Core 1: ICM20948 initialized.");

  // Configure sensors
  icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
  DEBUG_PRINTLN("Core 1: ICM20948 configured.");

  // Initialize AHRS filter
  filter.begin(1000); // Set to 1000 Hz
  DEBUG_PRINTLN("Core 1: AHRS filter initialized.");
  DEBUG_PRINTLN("Core 1: Setup completed.");
}

// Loop for Core 1: High-frequency tasks (1000 Hz)
void loop1() {
  if (state == ERROR) {
    return; // Do nothing if in ERROR state
  }
  unsigned long start = micros();

  // Periodic debug to confirm Core 1 is running
  static unsigned long last_print = 0;
  if (millis() - last_print > 1000) {
    DEBUG_PRINTLN("Core 1: loop running");
    last_print = millis();
  }

  // Read IMU sensors at 1000 Hz
  sensors_event_t accel, gyro, mag, temp;
  if (!icm.getEvent(&accel, &gyro, &mag, &temp)) {
    static unsigned long last_error = 0;
    if (millis() - last_error > 1000) {
      DEBUG_PRINTLN("Core 1: Failed to read ICM20948 data");
      last_error = millis();
    }
    return;
  }

  // Apply biases
  float raw_gyro_x = gyro.gyro.x - gyro_bias[0];
  float raw_gyro_y = gyro.gyro.y - gyro_bias[1];
  float raw_gyro_z = gyro.gyro.z - gyro_bias[2];
  float raw_accel_x = accel.acceleration.x - accel_offset[0];
  float raw_accel_y = accel.acceleration.y - accel_offset[1];
  float raw_accel_z = accel.acceleration.z - accel_offset[2];

  // Apply moving average filter
  float filtered_gyro_x, filtered_gyro_y, filtered_gyro_z;
  float filtered_accel_x, filtered_accel_y, filtered_accel_z;
  apply_moving_average(gyro_x_avg, raw_gyro_x, &filtered_gyro_x);
  apply_moving_average(gyro_y_avg, raw_gyro_y, &filtered_gyro_y);
  apply_moving_average(gyro_z_avg, raw_gyro_z, &filtered_gyro_z);
  apply_moving_average(accel_x_avg, raw_accel_x, &filtered_accel_x);
  apply_moving_average(accel_y_avg, raw_accel_y, &filtered_accel_y);
  apply_moving_average(accel_z_avg, raw_accel_z, &filtered_accel_z);

  // Update AHRS (Madgwick filter)
  filter.update(filtered_gyro_x, filtered_gyro_y, filtered_gyro_z,
                filtered_accel_x, filtered_accel_y, filtered_accel_z,
                mag.magnetic.x - mag_offset[0], mag.magnetic.y - mag_offset[1], mag.magnetic.z - mag_offset[2]);

  // Update shared IMU data
  shared_sensor_data.gyro_x = filtered_gyro_x;
  shared_sensor_data.gyro_y = filtered_gyro_y;
  shared_sensor_data.gyro_z = filtered_gyro_z;
  shared_sensor_data.accel_x = filtered_accel_x;
  shared_sensor_data.accel_y = filtered_accel_y;
  shared_sensor_data.accel_z = filtered_accel_z;
  shared_sensor_data.mag_x = mag.magnetic.x - mag_offset[0];
  shared_sensor_data.mag_y = mag.magnetic.y - mag_offset[1];
  shared_sensor_data.mag_z = mag.magnetic.z - mag_offset[2];
  imu_data_ready = true;

  // Maintain 1000 Hz (1 ms)
  while (micros() - start < 1000) {}
}

// Loop for Core 0: Low-frequency tasks (100 Hz)
void loop() {
  if (state == ERROR) {
    return; // Do nothing in ERROR state
  }

  if (test_mode) {
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd.length() > 0) {
        char c = cmd[0];
        switch (c) {
          case '1': set_state(INITIALIZING); break;
          case '2': set_state(CALIBRATING); break;
          case '3': set_state(PREFLIGHT); break;
          case '4': set_state(ACTIVE); break;
          case '5': set_state(IDLE); break;
          case '6': set_state(ERROR); break;
          case 'A': set_phase(GROUND); break;
          case 'B': set_phase(BOOST); break;
          case 'C': set_phase(COAST); break;
          case 'D': set_phase(APOGEE); break;
          case 'E': set_phase(DESCENT); break;
          case 'F': set_phase(LANDED); break;
          case 'V':
            verbose_mode = !verbose_mode;
            Serial.println(verbose_mode ? "Verbose mode enabled" : "Verbose mode disabled");
            break;
          case 'M':
            mag_cal_needed = true;
            set_state(CALIBRATING);
            break;
          case 'Z':
            if (LittleFS.remove("accel_bias.txt")) {
              Serial.println("Accelerometer calibration data cleared.");
              accel_cal_needed = true;
              nose_calibrated = tail_calibrated = right_calibrated = 
              left_calibrated = front_calibrated = back_calibrated = false;
            } else {
              Serial.println("No accelerometer calibration data to clear.");
            }
            if (LittleFS.remove("mag_offset.txt")) {
              Serial.println("Magnetometer calibration data cleared.");
              mag_cal_needed = true;
            } else {
              Serial.println("No magnetometer calibration data to clear.");
            }
            break;
          case 'G':
            debug_mode = !debug_mode;
            Serial.println(debug_mode ? "Debug mode enabled" : "Debug mode disabled");
            break;
          case 'X':
            Serial.println("All flash data will be deleted, type YES to confirm or any other key to cancel.");
            if (debug_mode) {
              Serial.print("Waiting for confirmation: ");
            }
            String confirmation = Serial.readStringUntil('\n');
            confirmation.trim();
            if (debug_mode) {
              Serial.print("Confirmation received: ");
              Serial.println(confirmation);
            }
            if (confirmation == "YES") {
              if (LittleFS.format()) {
                Serial.println("Flash data deleted, rebooting...");
                delay(500);
                rp2040.reboot();
              } else {
                Serial.println("Failed to delete flash data.");
              }
            } else {
              Serial.println("Operation canceled.");
            }
            break;
        }
      }
    }
  } else {
    handle_serial_commands();

    unsigned long current_time = millis();
    if (current_time - last_update >= UPDATE_INTERVAL) {
      last_update = current_time;

      // Read barometer at 100 Hz
      sensors_event_t temp_event, pressure_event;
      if (!dps.getEvents(&temp_event, &pressure_event)) {
        DEBUG_PRINTLN("Failed to read DPS310 data");
        return;
      }
      shared_sensor_data.pressure = pressure_event.pressure;
      shared_sensor_data.temperature = temp_event.temperature;
      float h_baro = pressureToAltitude(shared_sensor_data.pressure, ref_pressure);
      float dt = 0.01; // 100 Hz = 10 ms
      shared_sensor_data.velocity += shared_sensor_data.accel_z * dt;
      float h_accel = shared_sensor_data.altitude + shared_sensor_data.velocity * dt;
      shared_sensor_data.altitude = COMPLEMENTARY_ALPHA * h_baro + (1 - COMPLEMENTARY_ALPHA) * h_accel;

      if (imu_data_ready) {
        sensor_data = const_cast<SensorData&>(shared_sensor_data);
        imu_data_ready = false;
      }

      switch (state) {
        case INITIALIZING:
          if (is_sensor_still()) {
            calibrate_gyro();
            set_state(CALIBRATING);
          }
          break;
        case CALIBRATING:
          if (accel_cal_needed) {
            calibrate_accelerometer();
          } else if (mag_cal_needed) {
            calibrate_magnetometer();
          } else {
            set_state(IDLE);
          }
          break;
        case IDLE:
          break;
        case PREFLIGHT:
          log_pre_launch_data();
          if (detect_launch()) {
            set_state(ACTIVE);
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
          log_data();
          update_flight_phase();
          if (verbose_mode) print_sensor_data();
          if (current_phase == LANDED) {
            set_state(IDLE);
          }
          break;
        case ERROR:
          break;
      }
      update_neopixel();
    }
  }
}

// Calibration functions
void calibrate_gyro() {
  DEBUG_PRINTLN("Calibrating gyroscope...");
  const int samples = CALIBRATION_SAMPLES;
  float sum_gyro_x = 0, sum_gyro_y = 0, sum_gyro_z = 0;
  for (int i = 0; i < samples; i++) {
    while (!imu_data_ready) {
      static unsigned long start = millis();
      if (millis() - start > 1000) {
        DEBUG_PRINTLN("Timeout waiting for IMU data in calibrate_gyro");
        return;
      }
    }
    sum_gyro_x += shared_sensor_data.gyro_x;
    sum_gyro_y += shared_sensor_data.gyro_y;
    sum_gyro_z += shared_sensor_data.gyro_z;
    imu_data_ready = false;
    delay(10);
  }
  gyro_bias[0] = sum_gyro_x / samples;
  gyro_bias[1] = sum_gyro_y / samples;
  gyro_bias[2] = sum_gyro_z / samples;
  DEBUG_PRINTLN("Gyroscope calibration complete.");
}

void calibrate_accelerometer() {
  Serial.println("Starting accelerometer calibration. Follow the prompts to position the rocket.");

  float ax_nose_up, ay_nose_up, az_nose_up;
  float ax_tail_up, ay_tail_up, az_tail_up;
  float ax_right_up, ay_right_up, az_right_up;
  float ax_left_up, ay_left_up, az_left_up;
  float ax_front_up, ay_front_up, az_front_up;
  float ax_back_up, ay_back_up, az_back_up;

  if (!nose_calibrated) {
    Serial.println("Place the rocket upright with the nose pointing upwards.");
    wait_for_enter();
    collect_average_accel(ax_nose_up, ay_nose_up, az_nose_up, 2000);
    nose_calibrated = true;
    Serial.println("Nose up calibration recorded.");
    list_remaining_calibrations();
  }

  if (!tail_calibrated) {
    Serial.println("Place the rocket inverted with the tail pointing upwards.");
    wait_for_enter();
    collect_average_accel(ax_tail_up, ay_tail_up, az_tail_up, 2000);
    tail_calibrated = true;
    Serial.println("Tail up calibration recorded.");
    list_remaining_calibrations();
  }

  if (!right_calibrated) {
    Serial.println("Place the rocket on its side with the right side upwards.");
    wait_for_enter();
    collect_average_accel(ax_right_up, ay_right_up, az_right_up, 2000);
    right_calibrated = true;
    Serial.println("Right up calibration recorded.");
    list_remaining_calibrations();
  }

  if (!left_calibrated) {
    Serial.println("Place the rocket on its side with the left side upwards.");
    wait_for_enter();
    collect_average_accel(ax_left_up, ay_left_up, az_left_up, 2000);
    left_calibrated = true;
    Serial.println("Left up calibration recorded.");
    list_remaining_calibrations();
  }

  if (!front_calibrated) {
    Serial.println("Place the rocket on its side with the front side upwards.");
    wait_for_enter();
    collect_average_accel(ax_front_up, ay_front_up, az_front_up, 2000);
    front_calibrated = true;
    Serial.println("Front up calibration recorded.");
    list_remaining_calibrations();
  }

  if (!back_calibrated) {
    Serial.println("Place the rocket on its side with the back side upwards.");
    wait_for_enter();
    collect_average_accel(ax_back_up, ay_back_up, az_back_up, 2000);
    back_calibrated = true;
    Serial.println("Back up calibration recorded.");
    list_remaining_calibrations();
  }

  if (nose_calibrated && tail_calibrated && right_calibrated && 
      left_calibrated && front_calibrated && back_calibrated) {
    DEBUG_PRINTLN("All positions calibrated. Calculating biases...");
    accel_offset[0] = (ax_right_up + ax_left_up) / 2;
    accel_offset[1] = (ay_front_up + ay_back_up) / 2;
    accel_offset[2] = (az_nose_up + az_tail_up) / 2;
    
    Serial.println("Biases calculated:");
    Serial.print("X-axis bias: "); Serial.println(accel_offset[0]);
    Serial.print("Y-axis bias: "); Serial.println(accel_offset[1]);
    Serial.print("Z-axis bias: "); Serial.println(accel_offset[2]);
    DEBUG_PRINTLN("Saving biases to accel_bias.txt...");

    File accel_file = LittleFS.open("accel_bias.txt", "w");
    if (accel_file) {
      for (int i = 0; i < 3; i++) {
        accel_file.println(accel_offset[i]);
        accel_file.flush();
      }
      accel_file.close();
      accel_cal_needed = false;
      Serial.println("Accelerometer calibration complete. Biases saved to accel_bias.txt.");
      delay(2000);
    } else {
      Serial.println("Failed to open accel_bias.txt for writing.");
      set_state(ERROR);
    }
  }
}

void calibrate_magnetometer() {
  Serial.println("Calibrating magnetometer. Rotate the rocket in a figure-eight pattern for 30 seconds.");
  unsigned long start_time = millis();
  float min_mag[3] = {1000, 1000, 1000};
  float max_mag[3] = {-1000, -1000, -1000};
  while (millis() - start_time < 30000) {
    while (!imu_data_ready) {
      if (millis() - start_time > 31000) {
        DEBUG_PRINTLN("Timeout waiting for IMU data in calibrate_magnetometer");
        return;
      }
    }
    min_mag[0] = min(min_mag[0], shared_sensor_data.mag_x);
    min_mag[1] = min(min_mag[1], shared_sensor_data.mag_y);
    min_mag[2] = min(min_mag[2], shared_sensor_data.mag_z);
    max_mag[0] = max(max_mag[0], shared_sensor_data.mag_x);
    max_mag[1] = max(max_mag[1], shared_sensor_data.mag_y);
    max_mag[2] = max(max_mag[2], shared_sensor_data.mag_z);
    imu_data_ready = false;
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
    for (int i = 0; i < 3; i++) {
      mag_file.println(mag_offset[i]);
      mag_file.flush();
    }
    mag_file.close();
    mag_cal_needed = false;
    Serial.println("Magnetometer calibration data saved to mag_offset.txt.");
    delay(2000);
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
    if (state == ERROR) {
      pixel.setPixelColor(0, pixel.Color(255, 0, 0)); // Solid red for ERROR
      pixel.show();
    }
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
    case INITIALIZING: return "INI";
    case CALIBRATING: return "CAL";
    case IDLE: return "IDL";
    case PREFLIGHT: return "PRE";
    case ACTIVE: return "ACT";
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
    case LANDED: return "LANDED";
    default: return "UNKNOWN";
  }
}

// Sensor and logging functions
void handle_serial_commands() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "start") set_state(PREFLIGHT);
    else if (cmd == "stop") set_state(IDLE);
    else if (cmd == "calibrate") set_state(CALIBRATING);
  }
}

bool is_sensor_still() {
  unsigned long start = millis();
  while (!imu_data_ready) {
    if (millis() - start > 1000) {
      DEBUG_PRINTLN("Timeout waiting for IMU data in is_sensor_still");
      return false;
    }
  }
  float accel_magnitude = sqrt(shared_sensor_data.accel_x * shared_sensor_data.accel_x +
                               shared_sensor_data.accel_y * shared_sensor_data.accel_y +
                               shared_sensor_data.accel_z * shared_sensor_data.accel_z);
  imu_data_ready = false;
  bool is_still = abs(accel_magnitude - GRAVITY) < STILLNESS_THRESHOLD;
  if (debug_mode) {
    Serial.print("is_sensor_still: accel_magnitude = ");
    Serial.print(accel_magnitude);
    Serial.print(", still = ");
    Serial.println(is_still ? "true" : "false");
  }
  return is_still;
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
                           0, // Sensor ID placeholder
                           (int16_t)(sensor_data.temperature * 100));
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
  float altitude = sensor_data.altitude;

  switch (current_phase) {
    case GROUND:
      if (detect_launch()) set_phase(BOOST);
      break;
    case BOOST:
      if (accel_magnitude < GRAVITY) set_phase(COAST);
      break;
    case COAST:
      if (sensor_data.velocity <= 0) set_phase(APOGEE);
      break;
    case APOGEE:
      if (sensor_data.velocity < 0) set_phase(DESCENT);
      break;
    case DESCENT:
      if (is_sensor_still() && altitude <= 0) set_phase(LANDED);
      break;
    case LANDED:
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
  Serial.print("Altitude: "); Serial.println(sensor_data.altitude);
}

void update_neopixel() {
  static unsigned long last_blink = 0;
  static bool blink_state = false;
  unsigned long now = millis();

  switch (state) {
    case INITIALIZING:
      if (now - last_blink >= 500) {
        blink_state = !blink_state;
        pixel.setPixelColor(0, blink_state ? pixel.Color(255, 255, 0) : pixel.Color(0, 0, 0));
        last_blink = now;
      }
      break;
    case CALIBRATING:
      pixel.setPixelColor(0, pixel.Color(255, 255, 0));
      break;
    case IDLE:
      if (has_data) {
        if (now - last_blink >= 500) {
          blink_state = !blink_state;
          pixel.setPixelColor(0, blink_state ? pixel.Color(0, 0, 255) : pixel.Color(0, 255, 0));
          last_blink = now;
        }
      } else {
        pixel.setPixelColor(0, pixel.Color(0, 0, 255));
      }
      break;
    case PREFLIGHT:
      if (now - last_blink >= 500) {
        blink_state = !blink_state;
        pixel.setPixelColor(0, blink_state ? pixel.Color(0, 0, 255) : pixel.Color(0, 0, 0));
        last_blink = now;
      }
      break;
    case ACTIVE:
      if (now - last_blink >= 100) {
        blink_state = !blink_state;
        pixel.setPixelColor(0, blink_state ? pixel.Color(0, 255, 0) : pixel.Color(0, 0, 0));
        last_blink = now;
      }
      break;
    case ERROR:
      pixel.setPixelColor(0, pixel.Color(255, 0, 0));
      break;
  }
  pixel.show();
}