/*
 * Simple LoRa Receiver for Debug Bridge
 *
 * Target: Adafruit Feather M0 with RFM9x LoRa
 * Purpose: Receive packets and print to USB Serial
 *
 * Wiring (for Feather M0 with integrated RFM9x):
 *   Built-in, no wiring needed
 *
 * Wiring (for Feather M0 Basic + RFM95W breakout):
 *   RFM95W VIN  -> 3V
 *   RFM95W GND  -> GND
 *   RFM95W SCK  -> SCK
 *   RFM95W MOSI -> MOSI
 *   RFM95W MISO -> MISO
 *   RFM95W CS   -> D10 (configurable below)
 *   RFM95W RST  -> D11 (configurable below)
 *   RFM95W G0   -> D6  (IRQ, configurable below)
 *
 * Install: Arduino IDE -> Library Manager -> "RadioHead"
 */

#include <SPI.h>
#include <RH_RF95.h>

// Pin configuration for Feather M0 with RFM9x
#define RFM95_CS   8   // Feather M0 LoRa: 8, Basic+breakout: 10
#define RFM95_RST  4   // Feather M0 LoRa: 4, Basic+breakout: 11
#define RFM95_INT  3   // Feather M0 LoRa: 3, Basic+breakout: 6

// Radio frequency - must match transmitter!
#define RF95_FREQ 915.0

// LED for status
#define LED_PIN 13

RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);

  // Wait for serial with timeout (don't block forever)
  uint32_t start = millis();
  while (!Serial && (millis() - start < 3000)) {
    digitalWrite(LED_PIN, (millis() / 100) % 2);  // Fast blink while waiting
  }
  digitalWrite(LED_PIN, LOW);

  Serial.println("=== LoRa RX Debug Bridge ===");
  Serial.println("Initializing radio...");

  // Reset radio
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("ERROR: Radio init failed!");
    Serial.println("Check wiring and pin configuration.");
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }

  Serial.println("Radio initialized.");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("ERROR: setFrequency failed!");
    while (1);
  }

  Serial.print("Frequency: ");
  Serial.print(RF95_FREQ);
  Serial.println(" MHz");

  // Match these settings with the transmitter
  rf95.setTxPower(23, false);  // Max power for RX doesn't matter much

  // Default modem config: Bw125Cr45Sf128 (medium range, medium speed)
  // For longer range: rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);

  Serial.println("Waiting for packets...");
  Serial.println();
}

uint32_t pkt_count = 0;

void loop() {
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      digitalWrite(LED_PIN, HIGH);
      pkt_count++;

      // Print packet header
      Serial.print("[");
      Serial.print(pkt_count);
      Serial.print("] RSSI:");
      Serial.print(rf95.lastRssi(), DEC);
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
        // Null-terminate and print as string
        buf[len] = 0;
        Serial.println((char*)buf);
      } else {
        // Print as hex
        for (uint8_t i = 0; i < len; i++) {
          if (buf[i] < 0x10) Serial.print('0');
          Serial.print(buf[i], HEX);
          Serial.print(' ');
        }
        Serial.println();
      }

      digitalWrite(LED_PIN, LOW);
    } else {
      Serial.println("Receive failed");
    }
  }
}
