// Required libraries
#include <Wire.h>
#include <SPI.h>        // Explicitly include SPI for RP2350
#include <Adafruit_DPS310.h>
#include <RH_RF69.h>
#include <MAVLink.h>

// RFM69 Configuration
#define RFM69_FREQ    915.0
#define RFM69_CS      10    // "B" - Chip Select
#define RFM69_RST     11    // "A" - Reset
#define RFM69_INT     6     // "D" - Interrupt

// SPI pins for RP2350 Feather (default Feather pinout)
#define SPI_SCK       18    // SCK
#define SPI_MOSI      19    // MOSI
#define SPI_MISO      16    // MISO

// Structure to hold sensor data (moved before usage)
struct SensorData {
  float pressure;
  float temperature;
  uint32_t timestamp;
};

// Initialize sensor and radio objects
Adafruit_DPS310 dps;
RH_RF69 rf69(RFM69_CS, RFM69_INT);  // Use default hardware_spi from RadioHead

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("Starting setup...");

  Wire.begin();
  
  Serial.println("Initializing DPS310...");
  if (!dps.begin_I2C()) {
    Serial.println("Failed to find DPS310");
    while (1) delay(10);
  }
  Serial.println("DPS310 initialized successfully");
  
  dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);

  Serial.println("Initializing RFM69...");
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  Serial.println("Reset pin set low");

  // Reset sequence
  digitalWrite(RFM69_RST, HIGH);
  Serial.println("Reset pin set high");
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  Serial.println("Reset pin set low again");
  delay(100);

  // Initialize SPI explicitly
  Serial.println("Initializing SPI...");
  SPI.begin();
  SPI.setSCK(SPI_SCK);
  SPI.setTX(SPI_MOSI);
  SPI.setRX(SPI_MISO);
  SPI.setCS(RFM69_CS);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));  // 1MHz, standard SPI mode

  Serial.print("Attempting RFM69 init with CS=");
  Serial.print(RFM69_CS);
  Serial.print(", INT=");
  Serial.print(RFM69_INT);
  Serial.print(", SPI Pins: SCK=");
  Serial.print(SPI_SCK);
  Serial.print(", MOSI=");
  Serial.print(SPI_MOSI);
  Serial.print(", MISO=");
  Serial.println(SPI_MISO);

  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed!");
    Serial.println("Checking SPI communication...");
    // Test SPI by reading a register
    uint8_t regVal = rf69.spiRead(0x01);  // Read RFM69 register 0x01 (OpMode)
    Serial.print("RFM69 Register 0x01 (OpMode) value: 0x");
    Serial.println(regVal, HEX);
    Serial.println("Expected some non-zero value if communication works");
    Serial.println("Check wiring: CS to pin 10, RST to pin 11, INT to pin 6");
    Serial.println("SPI: SCK to pin 18, MOSI to pin 19, MISO to pin 16");
    Serial.println("Also verify 3.3V power and GND connections");
    while (1) delay(10);
  }
  Serial.println("RFM69 initialized successfully");

  Serial.print("Setting frequency to ");
  Serial.print(RFM69_FREQ);
  Serial.println(" MHz");
  if (!rf69.setFrequency(RFM69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  Serial.println("Setting TX power to +20dBm");
  rf69.setTxPower(20, true);
  
  SPI.endTransaction();
  Serial.println("Setup complete");
}

void loop() {
  Serial.println("Loop running...");
  delay(1000);
}

SensorData readSensorData() {
  SensorData data;
  sensors_event_t temp_event, pressure_event;
  
  dps.getEvents(&temp_event, &pressure_event);
  
  data.pressure = pressure_event.pressure;
  data.temperature = temp_event.temperature;
  data.timestamp = millis();
  
  Serial.print("Pressure: "); Serial.print(data.pressure); Serial.println(" Pa");
  Serial.print("Temperature: "); Serial.print(data.temperature); Serial.println(" C");
  
  return data;
}