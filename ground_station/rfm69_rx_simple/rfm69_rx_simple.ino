/*
 * Simple RFM69 Receiver for Debug Bridge
 *
 * Target: Adafruit Feather M0 RFM69HCW (#3176)
 * Purpose: Receive packets and print to USB Serial
 *
 * NOTE: RFM69 is NOT LoRa - it's FSK modulation.
 *       Shorter range than LoRa but still useful for debug.
 *
 * Install: Arduino IDE -> Library Manager -> "RadioHead"
 */

#include <SPI.h>
#include <RH_RF69.h>

// Pin configuration for Feather M0 RFM69HCW
#define RFM69_CS   8
#define RFM69_RST  4
#define RFM69_INT  3

// Radio frequency - must match transmitter!
#define RF69_FREQ 915.0

// LED for status
#define LED_PIN 13

RH_RF69 rf69(RFM69_CS, RFM69_INT);

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);

  Serial.begin(115200);

  // Wait for serial with timeout
  uint32_t start = millis();
  while (!Serial && (millis() - start < 3000)) {
    digitalWrite(LED_PIN, (millis() / 100) % 2);
  }
  digitalWrite(LED_PIN, LOW);

  Serial.println("=== RFM69 RX Debug Bridge ===");
  Serial.println("Initializing radio...");

  // Reset radio
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);

  if (!rf69.init()) {
    Serial.println("ERROR: Radio init failed!");
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }

  Serial.println("Radio initialized.");

  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("ERROR: setFrequency failed!");
    while (1);
  }

  Serial.print("Frequency: ");
  Serial.print(RF69_FREQ);
  Serial.println(" MHz");

  // High power module (+20dBm)
  rf69.setTxPower(20, true);

  // Optional encryption (both sides must match)
  // uint8_t key[] = { 0x01, 0x02, ... 0x10 };  // 16 bytes
  // rf69.setEncryptionKey(key);

  Serial.println("Waiting for packets...");
  Serial.println();
}

uint32_t pkt_count = 0;

void loop() {
  if (rf69.available()) {
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf69.recv(buf, &len)) {
      digitalWrite(LED_PIN, HIGH);
      pkt_count++;

      Serial.print("[");
      Serial.print(pkt_count);
      Serial.print("] RSSI:");
      Serial.print(rf69.lastRssi(), DEC);
      Serial.print(" len:");
      Serial.print(len);
      Serial.print(" | ");

      // Print as ASCII if printable, else hex
      bool printable = true;
      for (uint8_t i = 0; i < len; i++) {
        if (buf[i] < 32 || buf[i] > 126) {
          printable = false;
          break;
        }
      }

      if (printable) {
        buf[len] = 0;
        Serial.println((char*)buf);
      } else {
        for (uint8_t i = 0; i < len; i++) {
          if (buf[i] < 0x10) Serial.print('0');
          Serial.print(buf[i], HEX);
          Serial.print(' ');
        }
        Serial.println();
      }

      digitalWrite(LED_PIN, LOW);
    }
  }
}
