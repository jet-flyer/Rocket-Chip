// Transmitter Code

#include <SPI.h>
#include <RFM69.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_AHRS.h> // Include AHRS library
#include <MAVLink.h>
#include <Wire.h>
#include "sensordata.h" // Your sensordata.h file

#define RFM69_CS 8      // Chip select pin for the RFM69
#define RFM69_INT 3     // Interrupt pin (optional, but recommended)
#define RFM69_RST 4     // Reset pin for the RFM69

Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
RFM69 radio(RFM69_CS, RFM69_INT);
Adafruit_AHRS ahrs;  // AHRS instance

// Calibration values (store these after initial setup)
float initialPressure;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // ... (Radio initialization same as before)

  Wire.begin();

  if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }

  if (!mpu.begin()) {
    Serial.println("Could not find MPU6050 chip");
    while (1);
  }
  // ... (MPU6050 settings)

  // Initialize AHRS
  if (!ahrs.begin()) {
    Serial.println("Could not find AHRS sensor");
    while (1);
  }

  // Calibrate: Get initial pressure reading
  initialPressure = bmp.readPressure();
  Serial.print("Initial Pressure (Pa): "); Serial.println(initialPressure);
  delay(1000);
}

void loop() {
  // Read sensor data
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure();

  // Calculate relative altitude (meters, nearest 0.01m)
  float relativeAltitude = (initialPressure - pressure) * 8.31434 / (0.0289644 * 9.80665 * temperature);
  relativeAltitude = round(relativeAltitude * 100.0) / 100.0;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Update AHRS
  ahrs.update(g.gyro.x, g.gyro.y, g.gyro.z, a.acceleration.x, a.acceleration.y, a.acceleration.z, millis()); // Use millis() for time

  // Get orientation data (Euler angles)
  float roll = ahrs.getRoll();
  float pitch = ahrs.getPitch();
  float yaw = ahrs.getYaw();

  // Prepare MAVLink message
  mavlink_message_t msg;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_sensordata_pack(NODEID, TARGETID, &msg, temperature, pressure, relativeAltitude, a.acceleration.x, a.acceleration.y, a.acceleration.z, g.gyro.x, g.gyro.y, g.gyro.z, roll, pitch, yaw); // Include orientation data
  uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

  // Send data via radio
  radio.send(TARGETID, (const void*)buffer, len);
  radio.waitPacketSent();

  delay(100);
}