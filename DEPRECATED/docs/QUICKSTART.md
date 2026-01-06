# Quick Start Guide

## 🚀 Get Started in 5 Minutes

### 1. Choose Your Version
Based on your hardware and needs:

| Hardware | Recommended Version | Location |
|----------|-------------------|----------|
| **Basic Arduino + 10-DOF** | Deprecated | `versions/v1.0-deprecated/ahrs_10dof-RocketChip/` |
| **RP2350 + PSRAM** | Modular | `versions/v2.0-modular/RocketChip/` |
| **Modular Design** | Modular | `versions/v2.0-modular/RocketChip/` |
| **ArduPilot Integration** | AP | `versions/v2.0-modular/RocketChip_AP/` |
| **AI-Assisted Versions** | Various | `versions/ai-assisted/{claude,grok,gemini}/` |

### 2. Install Dependencies
```bash
# Required Arduino Libraries
Adafruit_ICM20X
Adafruit_ICM20948
Adafruit_DPS310
Adafruit_Sensor
Adafruit_AHRS
Adafruit_NeoPixel
LittleFS
MAVLink
```

### 3. Basic Setup
1. Open Arduino IDE
2. Select board: **Adafruit RP2350 Feather**
3. Select core: **Earle Philhower's Pico Core**
4. Open your chosen sketch
5. Upload to board

### 4. First Test
```cpp
// Open Serial Monitor at 115200 baud
// Type: help
// Expected output: Available commands list
```

### 5. Verify Operation
- **Yellow LED pulsing**: Initializing
- **Solid yellow**: Calibrating
- **Blue LED**: Ready for launch
- **Green blinking**: Active logging

## 🔧 Common Configurations

### Basic Configuration (`config.h`)
```cpp
#define SERIAL_BAUD 115200
#define LAUNCH_ACCEL_THRESHOLD 2.0
#define DEBUG_MODE DEBUG_STATUS
```

### Advanced Configuration
```cpp
#define PSRAM_BUFFER_SIZE (4 * 1024 * 1024)
#define SAMPLE_RATE_HZ 1000
#define LOG_RATE_HZ 100
```

## 📊 Data Output

### Serial Commands
- `status` - System status
- `calibrate` - Sensor calibration
- `test` - Test mode
- `dump` - Export data

### Data Formats
- **MAVLink**: For ground control stations
- **CSV**: For analysis in Excel/Python
- **Binary**: Compact storage format

## 🚨 Troubleshooting

### Quick Fixes
1. **No serial output**: Check baud rate (115200)
2. **Sensors not found**: Verify I2C connections
3. **Launch not detected**: Increase threshold in config.h
4. **Memory errors**: Reduce buffer size

### LED Status Guide
- **Red**: Error state
- **Yellow**: Initializing/Calibrating
- **Blue**: Ready
- **Green**: Active logging

## 📈 Next Steps

1. **Calibrate sensors** for your environment
2. **Test launch detection** with manual triggers
3. **Configure data logging** parameters
4. **Set up ground station** for real-time monitoring

## 🔗 Related Documentation

- [Full README](README.md) - Complete project documentation
- [Development Workflow](DEVELOPMENT.md) - Version management
- [Hardware Setup](docs/hardware/) - Wiring diagrams
- [Software Guide](docs/software/) - Advanced configuration

---

**Need help?** Check the troubleshooting section or open an issue on GitHub. 