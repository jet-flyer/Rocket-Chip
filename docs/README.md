Rocket Chip is an open-source, microcontroller-based tracking and telemetry system designed primarily for model rockets, but usfel to track any kind of movement that needs high percision or a live data feed. It enables real-time data logging, sensor monitoring, launch detection, and optional live telemetry transmission. Built on the Adafruit RP2350 HSTX Feather board with 8MB PSRAM, the project emphasizes modularity, high-performance data capture, and ease of use via the Arduino IDE for initial development.

The system supports multiple configurations:
- **Nano Version**: Lightweight, bare-bones setup focused on local data logging with minimal sensors.
- **Main Version**: Core features including sensors for local logging, with options to add live telemetry modules.
- **Pro Version**: Advanced capabilities such as high-performance sensors, pyro triggering for parachutes or staging, and enhanced data rates.

While the initial focus is on Arduino IDE compatibility, future iterations plan to incorporate Real-Time Operating Systems (RTOS) for low-latency operations and more advanced chipsets.

This repository contains deprecated older implementations, current AI-assisted development branches, documentation, libraries, and tools to support building and debugging the system.

## Key Features

- **High-Frequency Sensor Data Capture**: Up to 1kHz sampling rate for IMU (Inertial Measurement Unit) and pressure/altitude sensors.
- **Launch Detection**: Configurable thresholds for automatic detection of launch events based on acceleration or altitude changes.
- **Hybrid Storage System**: Utilizes PSRAM for buffering high-speed data and flash storage for persistent logging, with pre-launch data buffering.
- **Sensor Support**:
  - 9-axis IMU (e.g., ICM20948 for accelerometer, gyroscope, magnetometer).
  - Barometric pressure/altitude sensors (e.g., DPS310 or BMP280).
  - Calibration routines for accurate readings.
- **Data Output Formats**: Real-time streaming in MAVLink (for integration with ground control stations like QGroundControl), CSV, and binary formats.
- **Telemetry Integration**: Optional live transmission of telemetry data, with MAVLink protocol support.
- **Modular Architecture**: Separate modules for data logging, attitude tracking (AHRS - Attitude and Heading Reference System), and advanced features like pyro control.
- **Debugging Tools**: Serial commands for system management, debug modes, and compatibility with debug probes (e.g., Adafruit's SWD Debug Probe for troubleshooting hardware issues).
- **AI-Assisted Development**: Current development includes branches enhanced by AI agents (Claude, Grok, Gemini) for optimized code and features.

## Hardware Requirements

- **Microcontroller**: Adafruit RP2350 HSTX Feather board with 8MB onboard PSRAM (primary target for high-performance buffering).
- **Sensors**:
  - ICM20948 (9-DoF IMU).
  - DPS310 (Precision Pressure Sensor) or BMP280 as alternative.
- **Storage**: Onboard flash or external SD card for data logging.
- **Optional Add-ons**:
  - Telemetry radio modules for live data transmission.
  - Pyro channels for Pro version (e.g., triggering parachutes or staging).
  - NeoPixel for status indicators.
- **Debugging**: Adafruit SWD Debug Probe (Product ID: 5699) recommended for resolving hardware/firmware issues, especially during development.

Sourcing: Prioritize Adafruit components for simplicity. If alternatives offer significant cost or performance benefits (e.g., from Digi-Key or Mouser), they can be evaluated.

## Project Structure

- **`deprecated/`**: Older, non-AI implementations including:
  - `ahrs_10dof-RocketChip/`: Legacy attitude tracking with 10-DoF sensors.
  - `bmp280test/`: Altitude testing scripts.
  - `Datalog/`, `Datalogger/`, `OLD-Datalogger/`: Various data logging variants.
- **`current-dev/`**: Active development with AI-assisted code:
  - `ai-assisted/claude/`: Modular components, e.g., pro features like pyro and telemetry.
  - `ai-assisted/grok/`: Consolidated files with PSRAM enhancements.
  - `ai-assisted/gemini/`: Preserved implementations from Gemini AI.
- **`docs/`**: Additional documentation and resources.
- **`libraries/`**: Shared libraries for sensors and protocols (e.g., MAVLink).
- **`tools/`**: Utility scripts, including debug probe configurations.
- **`config.h`**: Central configuration file for hardware settings, thresholds, and modes.

## Installation and Setup

1. **Clone the Repository**:
   ```
   git clone https://github.com/jet-flyer/Rocket-Chip.git
   ```

2. **Arduino IDE Setup**:
   - Install the Arduino IDE.
   - Add support for RP2040/RP2350 boards via the Boards Manager (search for "Raspberry Pi Pico/RP2040").
   - Install required libraries (e.g., Adafruit_ICM20X, Adafruit_DPS310, MAVLink) via the Library Manager.

3. **Hardware Assembly**:
   - Connect sensors to the RP2350 Feather (refer to pin mappings in `config.h`).
   - For PSRAM usage, ensure the board has the 8MB variant.

4. **Upload Code**:
   - Open the desired `.ino` file (e.g., from `current-dev/ai-assisted/grok/main/`).
   - Configure settings in `config.h`.
   - Compile and upload via Arduino IDE.

If using the debug probe:
- Connect via SWD pins.
- Use it for stepping through code or inspecting memory during issues like sensor initialization failures.

## Usage

- **Basic Operation**:
  - Power on the board; it will initialize sensors and start logging.
  - Launch detection triggers high-rate logging.
  - Data is buffered in PSRAM and flushed to flash.

- **Serial Commands** (via USB Serial):
  - `calibrate`: Run sensor calibration.
  - `start_log`: Manually start logging.
  - `stop_log`: Stop and save data.
  - `telemetry_on`: Enable MAVLink output.

- **Data Retrieval**:
  - Connect to a ground station via telemetry for live views.
  - Post-flight, download logs from flash storage.

For Pro features (in development):
- Configure pyro pins in `config.h` for event-based triggering.

## Configuration

Edit `config.h` for customizations:
- Serial baud rate, NeoPixel pin.
- Launch thresholds (e.g., acceleration > 2g).
- Sample rates (IMU: 100-1000Hz, Pressure: 10-100Hz).
- Storage options (PSRAM buffer size, flash chip select).
- Debug modes (verbose output).

## Future Plans

- Integrate RTOS for low-latency, high-data-rate operations.
- Expand to advanced chipsets beyond RP2350.
- Add wireless telemetry protocols.
- Develop a user-friendly GUI for configuration and data visualization.
- Optimize for broader applications (e.g., drone racing, environmental monitoring).

## Contributing

Contributions are welcome! Fork the repo, create a branch for your feature/bugfix, and submit a pull request. Focus on modular code, Adafruit compatibility, and clear documentation.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details. (Add a LICENSE file if not present.)

For questions or support, open an issue on GitHub.
