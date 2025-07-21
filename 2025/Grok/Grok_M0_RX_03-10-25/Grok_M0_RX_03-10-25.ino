#include <RH_RF69.h>
#include <MAVLink.h>

// Pin definitions for RFM69 on Feather M0
#define RH_RF69_HAVE_SERIAL
#define RFM69_CS  10   // "B"
#define RFM69_RST 11   // "A"
#define RFM69_INT  6   // "D"
#define RF69_FREQ 915.0 // Radio frequency in MHz (adjust as needed)
#define MAVLINK_BUF_SIZE 64
#define LED_PIN 13
#define SIGNAL_TIMEOUT 5000
#define SEA_LEVEL_PRESSURE_HPA 1013.25

RH_RF69 rf69(RFM69_CS, RFM69_INT); // Initialize RFM69 with correct pins
uint8_t mavlink_buf[MAVLINK_BUF_SIZE];
bool mavlink_bridge = false;
unsigned long last_led_update = 0;
unsigned long last_packet_time = 0;
bool signal_acquired = false;

// Function to calculate altitude from pressure
float calculate_altitude(float press_abs) {
  return 44330.0 * (1.0 - pow(press_abs / SEA_LEVEL_PRESSURE_HPA, 0.1903));
}

void setup() {
  Serial.begin(921600);
  while (!Serial) delay(1);
  Serial.println("Starting setup...");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize RFM69 radio with correct pins
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  Serial.println("Radio initialized");

  Serial.println("Before rf69.init()");
  bool init_result = rf69.init();
  Serial.println("After rf69.init()");
  if (!init_result) {
    Serial.println("RFM69 init failed");
    while (1)
      ;
  }
  Serial.println("RFM69 radio initialized");
  Serial.println("Setup complete");

  // Wait for initial signal acquisition
  Serial.println("Awaiting Telemetry Acquisition");
  unsigned long start_time = millis();
  while (!signal_acquired && (millis() - start_time < SIGNAL_TIMEOUT)) {
    if (rf69.available()) {
      uint8_t len = sizeof(mavlink_buf);
      if (rf69.recv(mavlink_buf, &len)) {
        signal_acquired = true;
        last_packet_time = millis();
        break;
      }
    }
    delay(100);
  }
  if (!signal_acquired) {
    Serial.println("No signal detected within timeout, retrying...");
    delay(1000);
    setup(); // Restart setup if no signal
    return;
  }
}

void loop() {
  // Handle user commands to toggle MAVLink bridge
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.equalsIgnoreCase("mavlink on")) {
      mavlink_bridge = true;
      Serial.println("MAVLink bridge enabled");
    } else if (command.equalsIgnoreCase("mavlink off")) {
      mavlink_bridge = false;
      Serial.println("MAVLink bridge disabled");
    }
  }

  // Check for incoming radio data
  if (rf69.available()) {
    uint8_t len = sizeof(mavlink_buf);
    if (rf69.recv(mavlink_buf, &len)) {
      last_packet_time = millis();
      signal_acquired = true;

      if (mavlink_bridge) {
        Serial.write(mavlink_buf, len); // Raw binary output
      } else {
        // Decode and format MAVLink messages
        mavlink_message_t msg;
        mavlink_status_t status;

        for (uint8_t i = 0; i < len; i++) {
          if (mavlink_parse_char(MAVLINK_COMM_0, mavlink_buf[i], &msg, &status)) {
            switch (msg.msgid) {
              case MAVLINK_MSG_ID_SCALED_PRESSURE: {
                mavlink_scaled_pressure_t sp;
                mavlink_msg_scaled_pressure_decode(&msg, &sp);
                uint32_t sp_time_ms = sp.time_boot_ms;
                uint32_t sp_hrs = sp_time_ms / 3600000;
                uint32_t sp_min = (sp_time_ms % 3600000) / 60000;
                float sp_sec = (sp_time_ms % 60000) / 1000.0;
                Serial.println();
                if (sp_hrs > 0) {
                  Serial.print(sp_hrs); Serial.print(":");
                  Serial.print(sp_min); Serial.print(":");
                  Serial.println(sp_sec, 2);
                } else {
                  Serial.print(sp_min); Serial.print(":");
                  Serial.println(sp_sec, 2);
                }
                Serial.print("  alt,");
                Serial.println(calculate_altitude(sp.press_abs), 2);
                break;
              }
              case MAVLINK_MSG_ID_SCALED_IMU: {
                mavlink_scaled_imu_t imu;
                mavlink_msg_scaled_imu_decode(&msg, &imu);
                uint32_t imu_time_ms = imu.time_boot_ms;
                uint32_t imu_hrs = imu_time_ms / 3600000;
                uint32_t imu_min = (imu_time_ms % 3600000) / 60000;
                float imu_sec = (imu_time_ms % 60000) / 1000.0;
                Serial.println();
                if (imu_hrs > 0) {
                  Serial.print(imu_hrs); Serial.print(":");
                  Serial.print(imu_min); Serial.print(":");
                  Serial.println(imu_sec, 2);
                } else {
                  Serial.print(imu_min); Serial.print(":");
                  Serial.println(imu_sec, 2);
                }
                Serial.print("  G,");
                Serial.print(imu.xacc / 9800.0, 2); Serial.print(",");
                Serial.print(imu.yacc / 9800.0, 2); Serial.print(",");
                Serial.println(imu.zacc / 9800.0, 2);
                break;
              }
              case MAVLINK_MSG_ID_ATTITUDE: {
                mavlink_attitude_t att;
                mavlink_msg_attitude_decode(&msg, &att);
                uint32_t att_time_ms = att.time_boot_ms;
                uint32_t att_hrs = att_time_ms / 3600000;
                uint32_t att_min = (att_time_ms % 3600000) / 60000;
                float att_sec = (att_time_ms % 60000) / 1000.0;
                Serial.println();
                if (att_hrs > 0) {
                  Serial.print(att_hrs); Serial.print(":");
                  Serial.print(att_min); Serial.print(":");
                  Serial.println(att_sec, 2);
                } else {
                  Serial.print(att_min); Serial.print(":");
                  Serial.println(att_sec, 2);
                }
                Serial.print("  att,");
                Serial.print(att.roll, 2); Serial.print(",");
                Serial.print(att.pitch, 2); Serial.print(",");
                Serial.println(att.yaw, 2);
                Serial.print("  RPM,");
                Serial.println(att.yawspeed, 2);
                break;
              }
              case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
                mavlink_local_position_ned_t pos;
                mavlink_msg_local_position_ned_decode(&msg, &pos);
                uint32_t pos_time_ms = pos.time_boot_ms;
                uint32_t pos_hrs = pos_time_ms / 3600000;
                uint32_t pos_min = (pos_time_ms % 3600000) / 60000;
                float pos_sec = (pos_time_ms % 60000) / 1000.0;
                Serial.println();
                if (pos_hrs > 0) {
                  Serial.print(pos_hrs); Serial.print(":");
                  Serial.print(pos_min); Serial.print(":");
                  Serial.println(pos_sec, 2);
                } else {
                  Serial.print(pos_min); Serial.print(":");
                  Serial.println(pos_sec, 2);
                }
                Serial.print("  pos,");
                Serial.print(pos.x, 2); Serial.print(",");
                Serial.print(pos.y, 2); Serial.print(",");
                Serial.print(pos.z, 2); Serial.print(",");
                Serial.print(pos.vx, 2); Serial.print(",");
                Serial.print(pos.vy, 2); Serial.print(",");
                Serial.println(pos.vz, 2);
                break;
              }
              case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
                mavlink_global_position_int_t gps;
                mavlink_msg_global_position_int_decode(&msg, &gps);
                uint32_t gps_time_ms = gps.time_boot_ms;
                uint32_t gps_hrs = gps_time_ms / 3600000;
                uint32_t gps_min = (gps_time_ms % 3600000) / 60000;
                float gps_sec = (gps_time_ms % 60000) / 1000.0;
                Serial.println();
                if (gps_hrs > 0) {
                  Serial.print(gps_hrs); Serial.print(":");
                  Serial.print(gps_min); Serial.print(":");
                  Serial.println(gps_sec, 2);
                } else {
                  Serial.print(gps_min); Serial.print(":");
                  Serial.println(gps_sec, 2);
                }
                Serial.print("  GPS,");
                Serial.print(gps.lat / 1E7, 4); Serial.print(",");
                Serial.print(gps.lon / 1E7, 4); Serial.print(",");
                Serial.print(gps.alt / 1000.0, 1); Serial.print(",");
                Serial.print(gps.vx / 100.0, 2); Serial.print(",");
                Serial.print(gps.vy / 100.0, 2); Serial.print(",");
                Serial.println(gps.vz / 100.0, 2);
                break;
              }
            }
          }
        }
      }
    }
  } else {
    // Check for signal loss
    if (signal_acquired && (millis() - last_packet_time >= SIGNAL_TIMEOUT)) {
      signal_acquired = false;
      Serial.println("Telemetry Signal Loss Detected, Initiating Reacquisition");
    }
  }

  // Toggle LED every second when no signal is acquired and bridge is off
  if (!mavlink_bridge && !signal_acquired && (millis() - last_led_update >= 1000)) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    last_led_update = millis();
  }
}