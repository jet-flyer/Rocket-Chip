# Rocket Chip - Arduino Flight Tracking System

A comprehensive Arduino-based flight tracking and data logging system for rockets and other aerial vehicles. This project includes various implementations for different hardware configurations and use cases.

## 🚀 Project Overview

Rocket Chip is designed to provide real-time flight data logging, attitude tracking, and telemetry for rocket launches and other aerial applications. The system uses high-precision sensors to capture acceleration, orientation, altitude, and other flight parameters.

## 📁 Project Structure

```
Rocket Chip/
├── versions/                     # Organized by functionality
│   ├── v1.0-deprecated/         # Deprecated implementations
│   │   ├── ahrs_10dof-RocketChip/
│   │   ├── bmp280test/
│   │   ├── Datalog/
│   │   ├── Datalogger/
│   │   └── OLD-Datalogger/
│   ├── v2.0-modular/            # Modular architecture versions
│   │   ├── RocketChip/          # Basic modular implementation
│   │   ├── RocketChip_AP/       # ArduPilot integration
│   │   ├── RocketChip_Multi-board/ # Multi-board support
│   │   └── psram-test/          # PSRAM testing
│   ├── v3.0-current/            # Latest stable version
│   └── ai-assisted/             # AI-assisted versions
│       ├── claude/              # Claude AI-assisted implementations
│       ├── grok/                # Grok AI-assisted implementations
│       └── gemini/              # Gemini AI-assisted implementations
├── development/                  # Active development
│   ├── experimental/            # Experimental features
│   └── consolidated/            # Consolidated versions
├── docs/                        # Documentation
├── libraries/                   # Custom libraries
├── tools/                       # Development tools
└── examples/                    # Example implementations
```

## 🛠️ Hardware Requirements

### Primary Target Hardware
- **Main Board**: Adafruit RP2350 Feather with 8MB PSRAM
- **Core**: Earle Philhower's Pico Core
- **Sensors**: 
  - ICM20948 (9-axis IMU)
  - DPS310 (Pressure/Altitude sensor)
- **Storage**: 8MB PSRAM + Flash storage
- **Communication**: Serial, I2C, SPI

### Alternative Hardware
- **10-DOF Board**: Adafruit 10-DOF sensor board
- **Pressure Sensor**: BMP280/BMP085
- **IMU**: LSM303 (accelerometer + magnetometer)

## 📋 Features

### Core Features
- **Real-time Data Logging**: High-frequency sensor data capture
- **Launch Detection**: Automatic launch detection with configurable thresholds
- **Attitude Tracking**: Roll, pitch, and yaw calculation using AHRS algorithms
- **Altitude Monitoring**: Barometric pressure-based altitude calculation
- **PSRAM Buffer**: Efficient memory management for high-speed logging
- **MAVLink Integration**: Compatible with ArduPilot and ground control stations

### Advanced Features
- **Hybrid Storage**: PSRAM buffering with flash storage
- **Pre-launch Buffer**: Circular buffer for pre-launch data
- **Sensor Calibration**: Automatic and manual sensor calibration
- **Status LED**: Visual status indication with NeoPixel
- **Serial Debug**: Comprehensive debugging and configuration interface
- **Modular Architecture**: Configurable for different hardware setups

## 🔧 Installation & Setup

### Prerequisites
1. **Arduino IDE** (1.8.x or 2.x)
2. **Required Libraries**:
   - Adafruit_ICM20X
   - Adafruit_ICM20948
   - Adafruit_DPS310
   - Adafruit_Sensor
   - Adafruit_AHRS
   - Adafruit_NeoPixel
   - LittleFS
   - MAVLink

### Setup Instructions
1. Clone or download this repository
2. Open Arduino IDE
3. Install required libraries via Library Manager
4. Select the appropriate board (Adafruit RP2350 Feather)
5. Choose the correct core (Earle Philhower's Pico Core)
6. Open the desired sketch from the project folders
7. Configure `config.h` with your specific settings
8. Upload to your board

## ⚙️ Configuration

### Main Configuration (`config.h`)
```cpp
// Hardware Configuration
#define SERIAL_BAUD 115200
#define NEOPIXEL_PIN 16
#define NEOPIXEL_BRIGHTNESS 50

// Sensor Configuration
#define LAUNCH_ACCEL_THRESHOLD 2.0  // G's to detect launch
#define SAMPLE_RATE_HZ 1000         // Data sampling rate
#define LOG_RATE_HZ 100             // Data logging rate

// Storage Configuration
#define PSRAM_BUFFER_SIZE (4 * 1024 * 1024)  // 4MB buffer
#define PRELAUNCH_BUFFER_SECONDS 5           // Pre-launch data retention
```

### Debug Modes
- `DEBUG_OFF`: No serial output
- `DEBUG_STATUS`: Status messages only
- `DEBUG_VERBOSE`: Include 10Hz data output
- `DEBUG_TEST`: Test mode with manual triggers

## 📊 Data Output

### Serial Commands
- `help`: Display available commands
- `status`: Show system status
- `calibrate`: Start sensor calibration
- `test`: Enter test mode
- `dump`: Dump stored data
- `reset`: Reset system

### Data Formats
- **MAVLink**: Standard MAVLink 2.0 protocol
- **CSV**: Comma-separated values for analysis
- **Binary**: Compact binary format for storage

## 🚀 Usage

### Basic Operation
1. Power on the board
2. Wait for initialization (yellow LED pulsing)
3. System will calibrate sensors (solid yellow LED)
4. Ready for launch (blue LED)
5. Launch detection triggers active logging (green LED blinking)
6. Data is automatically saved to flash storage

### Advanced Operation
- Use serial interface for configuration and debugging
- Monitor real-time data via serial output
- Configure launch detection thresholds
- Calibrate sensors for optimal performance

## 🔍 Troubleshooting

### Common Issues
1. **Sensors not detected**: Check I2C connections and addresses
2. **PSRAM not working**: Verify PSRAM is properly initialized
3. **Launch not detected**: Adjust `LAUNCH_ACCEL_THRESHOLD`
4. **Data corruption**: Check flash storage and PSRAM integrity

### Debug Tips
- Use `DEBUG_VERBOSE` mode for detailed output
- Monitor serial output for error messages
- Check LED status indicators
- Verify sensor readings with `status` command

## 📈 Performance

### Specifications
- **Sampling Rate**: Up to 1kHz
- **Storage Capacity**: 4MB PSRAM + Flash
- **Launch Detection**: < 10ms response time
- **Data Retention**: Configurable pre-launch buffer
- **Battery Life**: Depends on logging rate and hardware

## 🤝 Contributing

This project welcomes contributions! Please feel free to:
- Report bugs and issues
- Suggest new features
- Submit pull requests
- Improve documentation

## 📄 License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE](LICENSE) file for details.

**Note**: This project uses libraries from Adafruit and ArduPilot which may have their own licenses. Please ensure compliance with all applicable licenses when using this software.

## 🙏 Acknowledgments

- Adafruit for sensor libraries and hardware
- ArduPilot community for MAVLink integration
- Earle Philhower for Pico core development
- Various AI assistants (Claude, Gemini, Grok) for development assistance

## 📞 Support

For questions, issues, or contributions:
- Check the troubleshooting section above
- Review the configuration options
- Examine the example implementations
- Test with the provided test sketches

---

**Note**: This project is actively developed and may have multiple versions for different use cases. Choose the implementation that best fits your hardware and requirements. 