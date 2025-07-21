#include <SPI.h>
#include <Adafruit_DPS310.h>
#include <RH_RF69.h>
#include <mavlink.h>

// DPS310 Sensor
Adafruit_DPS310 dps;

// RFM69 Radio
#define RFM69_CS 8
#define RFM69_INT 3
#define RFM69_RST 4
#define RFM69_FREQ 915.0 // Change to your frequency!
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// MAVLink System ID (Change as needed)
#define SYSTEM_ID 1

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Wait for serial connection
  }

  // Initialize DPS310
  if (!dps.begin_I2C()) {
    Serial.println("Failed to initialize DPS310");
    while (1) delay(10);
  }
  dps.configurePressure(DPS310_PRES_ULTRAHIGH, DPS310_OSRS_8);
  dps.configureTemperature(DPS310_TEMP_ULTRAHIGH, DPS310_OSRS_8);

  // Initialize RFM69
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);

  if (!rf69.init()) {
    Serial.println("RFM69 initialization failed");
    while (1) delay(10);
  }
  if (!rf69.setFrequency(RFM69_FREQ)) {
    Serial.println("RFM69 frequency set failed");
    while (1) delay(10);
  }
  rf69.setTxPower(20, true); // Max power

  Serial.println("Setup complete");
}

void loop() {
  // Read DPS310 data
  float pressure = dps.readPressure() / 100.0f; // Convert Pa to hPa
  float temperature = dps.readTemperature();

  // Create MAVLink message
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_scaled_pressure_pack(SYSTEM_ID, MAV_COMP_ID_SYSTEM_CONTROL, &msg,
                                    millis(), pressure * 100, temperature * 100, 0); //Times 100 for int conversion.

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send MAVLink message via RFM69
  if (rf69.send(buf, len)) {
    rf69.waitPacketSent();
    Serial.print("Sent: ");
    Serial.print(pressure);
    Serial.print(" hPa, ");
    Serial.print(temperature);
    Serial.println(" C");
  } else {
    Serial.println("RFM69 send failed");
  }

  delay(1000); // Send data every second
}