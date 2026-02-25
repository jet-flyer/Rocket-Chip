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

// Pin configuration for Feather M0 Basic Proto (#2772) + RFM95W breakout (#3072)
// For Feather M0 with integrated RFM9x: CS=8, RST=4, INT=3
#define RFM95_CS   10
#define RFM95_RST  11
#define RFM95_INT  6

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

  // Must match RocketChip TX settings (rfm95w.cpp configure_modem)
  rf95.setTxPower(23, false);
  rf95.setSpreadingFactor(7);          // SF7
  rf95.setSignalBandwidth(125000);     // BW 125kHz
  rf95.setCodingRate4(5);              // CR 4/5
  rf95.setSyncWord(0x12);              // Private network (matches TX)

  Serial.println("Config: SF7 BW125 CR4/5 Sync=0x12");
  Serial.println("Waiting for packets...");
  Serial.println();
}

// Link quality statistics
uint32_t pkt_count = 0;
uint32_t rx_errors = 0;
int16_t rssi_min = 0;
int16_t rssi_max = -200;
int32_t rssi_sum = 0;
int8_t snr_min = 127;
int8_t snr_max = -128;
int32_t snr_sum = 0;
uint32_t last_status_ms = 0;
uint32_t last_pkt_ms = 0;

void printLinkStatus() {
  Serial.println();
  Serial.println("--- Link Quality ---");
  Serial.print("  Packets: ");
  Serial.print(pkt_count);
  Serial.print("  Errors: ");
  Serial.println(rx_errors);
  if (pkt_count > 0) {
    Serial.print("  RSSI: ");
    Serial.print(rssi_min);
    Serial.print(" / ");
    Serial.print((int16_t)(rssi_sum / (int32_t)pkt_count));
    Serial.print(" / ");
    Serial.print(rssi_max);
    Serial.println(" dBm (min/avg/max)");
    Serial.print("  SNR:  ");
    Serial.print(snr_min);
    Serial.print(" / ");
    Serial.print((int8_t)(snr_sum / (int32_t)pkt_count));
    Serial.print(" / ");
    Serial.print(snr_max);
    Serial.println(" dB (min/avg/max)");
    if (last_pkt_ms > 0) {
      uint32_t elapsed = millis() - last_pkt_ms;
      if (elapsed < 10000) {
        Serial.print("  Last pkt: ");
        Serial.print(elapsed);
        Serial.println(" ms ago");
      } else {
        Serial.print("  Last pkt: ");
        Serial.print(elapsed / 1000);
        Serial.println(" s ago");
      }
    }
  }
  Serial.println("--------------------");
  Serial.println();
}

void loop() {
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      digitalWrite(LED_PIN, HIGH);
      pkt_count++;
      last_pkt_ms = millis();

      int16_t rssi = rf95.lastRssi();
      int8_t snr = rf95.lastSNR();

      // Update link quality stats
      rssi_sum += rssi;
      if (rssi < rssi_min || rssi_min == 0) rssi_min = rssi;
      if (rssi > rssi_max) rssi_max = rssi;
      snr_sum += snr;
      if (snr < snr_min) snr_min = snr;
      if (snr > snr_max) snr_max = snr;

      // Print packet header with RSSI + SNR
      Serial.print("[");
      Serial.print(pkt_count);
      Serial.print("] RSSI:");
      Serial.print(rssi, DEC);
      Serial.print(" SNR:");
      Serial.print(snr, DEC);
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
      rx_errors++;
      Serial.println("[RX ERROR] Receive failed");
    }
  }

  // Print link quality summary every 30 seconds
  if (millis() - last_status_ms >= 30000) {
    last_status_ms = millis();
    printLinkStatus();
  }
}
