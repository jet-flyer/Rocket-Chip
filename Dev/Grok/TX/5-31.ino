#include <Wire.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_DPS310.h>
#include <Adafruit_AHRS_Madgwick.h>
#include <Adafruit_NeoPixel.h>
#include <MAVLink.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <LittleFS.h>

// Pin definitions
#define NEOPIXEL_PIN 21 // GPIO 21 for onboard NeoPixel (conflicts with HSTX GPIO 16-23 if HSTX used)

// HSTX Pin Reference (not currently used, for future tracking):
// HSTX likely uses GPIO 16-23 (pins 16-23) based on RP2350 HSTX defaults:
// - GPIO 16: HSTX Data 0
// - GPIO 17: HSTX Data 1
// - GPIO 18: HSTX Data 2
// - GPIO 19: HSTX Data 3
// - GPIO 20: HSTX Data 4
// - GPIO 21: HSTX Data 5 (conflicts with NeoPixel)
// - GPIO 22: HSTX Data 6
// - GPIO 23: HSTX Data 7
// If HSTX is needed, NeoPixel may need to move (e.g., to GPIO 8)

// Constants
const float GRAVITY = 9.81; // m/s^2
const uint8_t SYSTEM_ID = 1;
const uint8_t COMPONENT_ID = 1;
const size_t LOG_SIZE = 8 * 1024 * 1024; // 8MB in PSRAM
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
size_t log_index = 0;
unsigned long last_update = 0;
bool calibration_done = false;
bool has_data = false;
bool verbose_mode = false;
unsigned long launch_detect_start = 0;
unsigned long mag_cal_start_time = 0;
int sample_count = 0;
float mag_min[3] = {1000, 1000, 1000};
float mag_max[3] = {-1000, -1000, -1000};

// Calibration variables
float sum_gyro[3] = {0};
float sum_accel[3] = {0};
float sum_pressure = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin();

  // Initialize LittleFS for calibration storage
  if (!LittleFS.begin()) {
    Serial.println("Failed to initialize LittleFS");
    state = ERROR;
    return;
  }

  // Load existing calibration data if available
  if (LittleFS.exists("/calibration.txt")) {
    load_calibration_data();
    calibration_done = true;
  } else {
    Serial.println("Calibration required. Keep the sensor still to begin.");
  }

  // Initialize ICM20948 (IMU)
  if (!icm.begin_I2C()) {
    Serial.println("Failed to initialize ICM20948");
    state = ERROR;
    return;
  }
  // Initialize DPS310 (barometer)
  if (!dps.begin_I2C()) {
    Serial.println("Failed to initialize DPS310");
    state = ERROR;
    return;
  }

  // Configure sensors
  icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
  dps.setMode(DPS310_CONT_PRESSURE);

  // Initialize AHRS filter
  filter.begin(100);

  // Initialize NeoPixel
  pixel.begin();
  pixel.setBrightness(50);
  pixel.show();

  // Allocate PSRAM log buffer using malloc (PSRAM must be enabled in Tools > PSRAM)
  log_buffer = (uint8_t*)malloc(LOG_SIZE);
  if (log_buffer == nullptr) {
    Serial.println("Failed to allocate log buffer");
    state = ERROR;
    return;
  }

  // Set initial state
  state = calibration_done ? STANDBY : INITIALIZED;
}

void loop() {
  handle_serial_commands();

  unsigned long current_time = millis();
  if (current_time - last_update >= UPDATE_INTERVAL) {
    last_update = current_time;

    switch (state) {
      case INITIALIZED:
        if (is_sensor_still()) {
          state = CALIBRATING;
          Serial.println("Sensor is still, starting calibration...");
        }
        break;
      case CALIBRATING:
        calibrate_sensors();
        break;
      case PRE_LAUNCH:
        read_sensors();
        update_ahrs();
        log_pre_launch_data();
        if (detect_launch()) {
          state = ACTIVE;
          Serial.println("Launch detected, starting full logging...");
          // Copy pre-launch buffer to log
          for (int i = 0; i < BUFFER_SIZE; i++) {
            int idx = (buffer_index + i) % BUFFER_SIZE;
            if (log_index + pre_launch_buffer[idx].len < LOG_SIZE) {
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

// Placeholder functions (to be implemented as needed)
void handle_serial_commands() {}
void load_calibration_data() {}
bool is_sensor_still() { return true; }
void calibrate_sensors() {}
void read_sensors() {}
void update_ahrs() {}
void log_pre_launch_data() {}
bool detect_launch() { return false; }
void log_data() {}
void update_flight_phase() {}
void print_sensor_data() {}
void update_neopixel() {}