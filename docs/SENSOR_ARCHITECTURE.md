# Modular Sensor Architecture

**Status:** Active
**Last Updated:** 2026-02-13

## Overview

RocketChip sensors use a **transport-neutral data type + transport-specific backend** pattern. Each sensor category (GPS, IMU, barometer) defines a shared data struct with zero transport dependencies. Transport-specific backends (I2C, UART, SPI) implement the same API contract and produce the shared data type. The main firmware selects the backend at boot via auto-detection and dispatches through function pointers.

This architecture enables:
- **Plug-and-play transport migration** — swap an I2C sensor for SPI without touching fusion code
- **Auto-detection at boot** — firmware tries each transport and picks whichever responds
- **Zero virtual dispatch overhead** — C function pointers, no vtable, no heap allocation
- **Independent driver development** — each backend is a self-contained .cpp file with no cross-dependencies

## Pattern

```
src/drivers/
  <sensor>.h           ← Transport-neutral data types (shared contract)
  <sensor>_<transport>.h   ← Transport-specific API
  <sensor>_<transport>.cpp ← Transport-specific implementation
```

### Transport-Neutral Header (`<sensor>.h`)

Defines the data struct all backends produce. Zero SDK includes — pure C/C++ types.

```cpp
// gps.h — example
typedef struct {
    double latitude;
    double longitude;
    float altitudeM;
    // ... all fields that consumers need
} gps_data_t;

typedef enum {
    GPS_TRANSPORT_NONE = 0,
    GPS_TRANSPORT_I2C  = 1,
    GPS_TRANSPORT_UART = 2,
} gps_transport_t;
```

### Transport-Specific Backend

Each backend implements the same function signature contract:

```cpp
bool <sensor>_<transport>_init(void);
bool <sensor>_<transport>_ready(void);
bool <sensor>_<transport>_update(void);
bool <sensor>_<transport>_get_data(<sensor>_data_t* data);
```

Backends include only their transport-specific SDK headers (`hardware/i2c.h`, `hardware/uart.h`, `hardware/spi.h`). They share no code with other backends — duplication of small helper functions (like `update_data_from_lwgps()`) is intentional to avoid coupling.

### Dispatch in main.cpp

Function pointers set at boot based on auto-detection:

```cpp
static bool (*gps_fn_update)(void) = nullptr;
static bool (*gps_fn_get_data)(gps_data_t*) = nullptr;

// In init_sensors():
if (gps_uart_init()) {
    gps_fn_update = gps_uart_update;
    gps_fn_get_data = gps_uart_get_data;
} else if (gps_i2c_init()) {
    gps_fn_update = gps_pa1010d_update;
    gps_fn_get_data = gps_pa1010d_get_data;
}

// In sensor loop:
gps_fn_update();
```

## Current Implementation

### GPS (Implemented)

| File | Transport | Status |
|------|-----------|--------|
| `src/drivers/gps.h` | Neutral | Active — `gps_data_t`, `gps_fix_t`, `gps_transport_t` |
| `src/drivers/gps_pa1010d.h/cpp` | I2C | Active — PA1010D via I2C (Qwiic dev) |
| `src/drivers/gps_uart.h/cpp` | UART | Active — Adafruit Ultimate GPS FeatherWing (GPIO0/1) |

**Detection order:** UART first (2-second timeout), then I2C. UART is preferred in production because it eliminates I2C bus contention entirely (LL Entry 20/24).

### IMU (Future)

| File | Transport | Status |
|------|-----------|--------|
| `src/drivers/imu.h` | Neutral | Planned — extract from `icm20948.h` |
| `src/drivers/icm20948.h/cpp` | I2C | Active (current) — rename to `icm20948_i2c.*` |
| `src/drivers/icm20948_spi.h/cpp` | SPI | Planned — same ICM-20948 via SPI |
| `src/drivers/adxl375_spi.h/cpp` | SPI | Planned — high-G accelerometer |

### Barometer (Future)

| File | Transport | Status |
|------|-----------|--------|
| `src/drivers/baro.h` | Neutral | Planned — extract from `baro_dps310.h` |
| `src/drivers/baro_dps310.h/cpp` | I2C | Active (current) — rename to `baro_dps310_i2c.*` |
| `src/drivers/baro_dps310_spi.h/cpp` | SPI | Planned — same DPS310 via SPI |
| `src/drivers/baro_bmp580_spi.h/cpp` | SPI | Planned — BMP580 (higher performance) |

## Design Decisions

### Why function pointers, not virtual dispatch?

Virtual dispatch requires heap allocation for polymorphic objects and adds vtable indirection overhead. For a bare-metal system with exactly one instance of each sensor backend selected at boot, function pointers are simpler, use zero heap, and have identical runtime performance.

### Why duplicate `update_data_from_lwgps()` across GPS backends?

Both I2C and UART GPS backends have ~40 lines of identical field copies from lwGPS to `gps_data_t`. Factoring this into a shared helper would create a coupling between two otherwise-independent translation units. The duplication is intentional — it keeps each backend fully self-contained and independently modifiable.

### Why auto-detect at boot, not compile-time selection?

The "same binary" principle: one firmware binary supports all transport configurations. No `#ifdef GPS_UART` conditionals. The firmware probes for available hardware at boot and configures itself. This simplifies production (one binary per board family) and supports development hardware that may have different sensor configurations.

### Detection order

Within each sensor category, try the production transport first (UART/SPI), fall back to development transport (I2C). This minimizes boot delay on production hardware — the preferred transport is tried first, and the I2C fallback only runs if the production path fails.

## Consumer Independence

Sensor consumers (ESKF fusion, seqlock, CLI status) depend only on transport-neutral types. They never include transport-specific headers:

```
  Consumers              Transport-Neutral        Transport-Specific
  ─────────              ─────────────────        ──────────────────
  eskf.cpp        ──→    gps.h (gps_data_t)   ←── gps_pa1010d.cpp
  main.cpp seqlock ──→                         ←── gps_uart.cpp
  rc_os.cpp CLI   ──→
```

This means adding a new GPS transport (e.g., SPI u-blox) requires zero changes to fusion, display, or calibration code.
