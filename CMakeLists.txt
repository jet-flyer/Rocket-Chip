# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

# Set board and platform BEFORE including pico_sdk_import.cmake
set(PICO_BOARD "adafruit_feather_rp2350")
set(PICO_PLATFORM "rp2350")

# Include the Pico SDK import
include(pico_sdk_import.cmake)

# Project definition
project(rocket_chip_freertos_validation C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# Set FreeRTOS config directory BEFORE including FreeRTOS_Kernel_import.cmake
set(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_SOURCE_DIR}" CACHE STRING "" FORCE)

# Include FreeRTOS Kernel
include(FreeRTOS_Kernel_import.cmake)

# ============================================================================
# Common Include Directory and Debug Configuration
# ============================================================================

# Add common include path for debug.h and other shared headers
set(ROCKETCHIP_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)

# Enable debug output by default (disable for release builds)
# To disable: cmake -DCONFIG_DEBUG=OFF ..
option(CONFIG_DEBUG "Enable debug output macros" ON)

# ============================================================================
# ArduPilot Board Configuration (Global)
# ============================================================================
# These MUST be defined globally before any ArduPilot headers are included.
# They satisfy ArduPilot's board detection in AP_HAL_Boards.h.
add_compile_definitions(
    # Board identification
    HAL_BOARD_ROCKETCHIP=99
    CONFIG_HAL_BOARD=99
    CONFIG_HAL_BOARD_SUBTYPE=0
    # Memory limits
    HAL_PROGRAM_SIZE_LIMIT_KB=4096
    # Serial port config
    HAL_NUM_CAN_IFACES=0
    BOARD_FLASH_SIZE=4096
    # Core feature flags
    HAL_HAVE_BOARD_VOLTAGE=0
    HAL_HAVE_SERVO_VOLTAGE=0
    HAL_WITH_IO_MCU=0
    HAL_WITH_DRONECAN=0
    HAL_MAX_CAN_PROTOCOL_DRIVERS=0
    HAL_HAVE_IMU_HEATER=0
    HAL_WITH_EKF_DOUBLE=0
    # Param system enabled - use real AP_Param
    AP_PARAM_ENABLED=1
    # GCS enabled for AP_AccelCal and status messages
    HAL_GCS_ENABLED=1
    # Disable Pico SDK's operator new/delete - we provide our own that zeros memory
    # ArduPilot expects zeroed allocations (like ChibiOS). See PD11.
    PICO_CXX_DISABLE_ALLOCATION_OVERRIDES=1
    # Bypass DTR check in stdio_usb_connected() - some terminals don't set DTR
    # Without this, USB wait loops hang forever waiting for DTR signal
    PICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1
    # Disable ScopedTimer debug output
    ROCKETCHIP_DISABLE_TIMING_DEBUG=1
)

# ============================================================================
# HAL Library
# ============================================================================

add_library(hal STATIC
    src/hal/Bus.cpp
    src/hal/GPIO.cpp
    src/hal/ADC.cpp
    src/hal/PWM.cpp
    src/hal/UART.cpp
    src/hal/Timing.cpp
    src/hal/PIO.cpp
    src/hal/HAL.cpp
    src/hal/Radio_RFM95W.cpp
)

target_include_directories(hal PUBLIC
    ${CMAKE_SOURCE_DIR}/src/hal
    ${ROCKETCHIP_INCLUDE_DIR}
)

if(CONFIG_DEBUG)
    target_compile_definitions(hal PUBLIC CONFIG_DEBUG)
endif()

target_link_libraries(hal PUBLIC
    pico_stdlib
    hardware_i2c
    hardware_spi
    hardware_gpio
    hardware_adc
    hardware_uart
    hardware_timer
    hardware_pio
    hardware_pwm
    hardware_watchdog
    hardware_clocks
    FreeRTOS-Kernel
)

# Note: USBSerial support requires project to link tinyusb_device
# and provide tusb_config.h. Enable with pico_enable_stdio_usb().

# ============================================================================
# ST Platform-Independent Sensor Drivers
# ============================================================================

# ISM330DHCX IMU driver (STMicroelectronics)
add_library(st_ism330dhcx INTERFACE)
target_include_directories(st_ism330dhcx INTERFACE
    ${CMAKE_SOURCE_DIR}/lib/st_drivers/ism330dhcx-pid
)
target_sources(st_ism330dhcx INTERFACE
    ${CMAKE_SOURCE_DIR}/lib/st_drivers/ism330dhcx-pid/ism330dhcx_reg.c
)

# LIS3MDL Magnetometer driver (STMicroelectronics)
add_library(st_lis3mdl INTERFACE)
target_include_directories(st_lis3mdl INTERFACE
    ${CMAKE_SOURCE_DIR}/lib/st_drivers/lis3mdl-pid
)
target_sources(st_lis3mdl INTERFACE
    ${CMAKE_SOURCE_DIR}/lib/st_drivers/lis3mdl-pid/lis3mdl_reg.c
)

# DPS310 Barometer driver (Ruuvi) - source files are in src/ subdirectory
add_library(ruuvi_dps310 INTERFACE)
target_include_directories(ruuvi_dps310 INTERFACE
    ${CMAKE_SOURCE_DIR}/lib/st_drivers/ruuvi-dps310/src
)
target_sources(ruuvi_dps310 INTERFACE
    ${CMAKE_SOURCE_DIR}/lib/st_drivers/ruuvi-dps310/src/dps310.c
)

# ============================================================================
# HAL Sensors Library (C++ wrappers around ST drivers)
# ============================================================================

add_library(hal_sensors STATIC
    src/hal/IMU_ISM330DHCX.cpp
    src/hal/IMU_ICM20948.cpp
    src/hal/Mag_LIS3MDL.cpp
    src/hal/Baro_DPS310.cpp
    src/hal/GPS_PA1010D.cpp
)

target_include_directories(hal_sensors PUBLIC
    ${CMAKE_SOURCE_DIR}/src/hal
)

target_link_libraries(hal_sensors PUBLIC
    hal
    st_ism330dhcx
    st_lis3mdl
    ruuvi_dps310
)

# ============================================================================
# AP_HAL_RP2350 - ArduPilot HAL Implementation for RP2350
# ============================================================================

# AP_FlashStorage - ArduPilot wear-leveled flash storage
add_library(ap_flash_storage STATIC
    lib/ardupilot/libraries/AP_FlashStorage/AP_FlashStorage.cpp
)

# Note: lib/ap_compat MUST come before lib/ardupilot/libraries
# so our AP_HAL shim takes priority over ArduPilot's full HAL
target_include_directories(ap_flash_storage PUBLIC
    ${CMAKE_SOURCE_DIR}/lib/ap_compat
    ${CMAKE_SOURCE_DIR}/lib/ardupilot/libraries
    ${CMAKE_SOURCE_DIR}/src/hal
    ${ROCKETCHIP_INCLUDE_DIR}
)

target_link_libraries(ap_flash_storage PUBLIC
    pico_stdlib
    FreeRTOS-Kernel  # Needed because board/empty.h includes Semaphores.h
)

add_library(ap_hal_rp2350 STATIC
    # RP2350 HAL implementations
    lib/ap_compat/AP_HAL_RP2350/Semaphores.cpp
    lib/ap_compat/AP_HAL_RP2350/Storage.cpp
    lib/ap_compat/AP_HAL_RP2350/Scheduler.cpp
    lib/ap_compat/AP_HAL_RP2350/Util.cpp
    lib/ap_compat/AP_HAL_RP2350/HAL_RP2350_Class.cpp
    lib/ap_compat/AP_HAL_RP2350/GPIO.cpp
    lib/ap_compat/AP_HAL_RP2350/AnalogIn.cpp
    lib/ap_compat/AP_HAL_RP2350/UARTDriver.cpp
    lib/ap_compat/AP_HAL_RP2350/I2CDevice.cpp
    lib/ap_compat/AP_HAL_RP2350/DeviceBus.cpp
    lib/ap_compat/AP_HAL_RP2350/SPIDevice.cpp
    lib/ap_compat/AP_HAL_RP2350/system.cpp
    lib/ap_compat/AP_HAL_RP2350/malloc_wrapper.cpp
    # ArduPilot AP_HAL base class implementations
    lib/ardupilot/libraries/AP_HAL/Device.cpp
    lib/ardupilot/libraries/AP_HAL/Storage.cpp
    lib/ardupilot/libraries/AP_HAL/Util.cpp
    lib/ardupilot/libraries/AP_HAL/Scheduler.cpp
    lib/ardupilot/libraries/AP_HAL/system.cpp
    lib/ardupilot/libraries/AP_HAL/UARTDriver.cpp
    lib/ardupilot/libraries/AP_HAL/utility/BetterStream.cpp
    lib/ardupilot/libraries/AP_HAL/utility/print_vprintf.cpp
    lib/ardupilot/libraries/AP_HAL/utility/utoa_invert.cpp
    lib/ardupilot/libraries/AP_HAL/utility/ftoa_engine.cpp
    # Dependencies for AP_Param and AP_InertialSensor
    lib/ardupilot/libraries/AP_HAL/Semaphores.cpp
    lib/ardupilot/libraries/AP_HAL/utility/RingBuffer.cpp
)

target_include_directories(ap_hal_rp2350 PUBLIC
    ${CMAKE_SOURCE_DIR}/lib/ap_compat/AP_HAL_RP2350
    ${CMAKE_SOURCE_DIR}/lib/ap_compat
    ${CMAKE_SOURCE_DIR}/lib/ardupilot/libraries
    ${CMAKE_SOURCE_DIR}/lib
    ${CMAKE_SOURCE_DIR}/src              # For "hal/GPIO.h" to reach RocketChip HAL
    ${CMAKE_SOURCE_DIR}/src/hal
    ${ROCKETCHIP_INCLUDE_DIR}
)

if(CONFIG_DEBUG)
    target_compile_definitions(ap_hal_rp2350 PUBLIC CONFIG_DEBUG)
endif()

target_link_libraries(ap_hal_rp2350 PUBLIC
    ap_flash_storage
    ap_compat           # AP_InternalError, AP_Notify stubs
    hal                 # For RocketChip UART/I2C/SPI wrappers
    pico_stdlib
    pico_unique_id
    hardware_timer
    hardware_watchdog
    hardware_sync
    hardware_flash
    hardware_adc
    hardware_uart
    hardware_i2c
    hardware_spi
    hardware_gpio
    FreeRTOS-Kernel
)

# Note: malloc_wrapper.cpp overrides operator new to use calloc (zero-init).
# This is needed because ArduPilot code assumes zeroed memory (like ChibiOS).
# See RP2350_FULL_AP_PORT.md PD11 for background.

# ============================================================================
# ArduPilot Compatibility Layer (config, macros, internal error tracking)
# ============================================================================

add_library(ap_compat STATIC
    lib/ap_compat/AP_InternalError/AP_InternalError.cpp
    lib/ap_compat/AP_Notify/AP_Notify.cpp
    lib/ap_compat/AP_GPS/AP_GPS.cpp
)

target_include_directories(ap_compat PUBLIC
    ${CMAKE_SOURCE_DIR}/lib/ap_compat
    ${CMAKE_SOURCE_DIR}/lib/ap_compat/AP_HAL_RP2350
    ${CMAKE_SOURCE_DIR}/lib/ardupilot/libraries
    ${CMAKE_SOURCE_DIR}/lib  # For mavlink/common/mavlink.h
    ${CMAKE_SOURCE_DIR}/src/hal
    ${ROCKETCHIP_INCLUDE_DIR}
)

if(CONFIG_DEBUG)
    target_compile_definitions(ap_compat PUBLIC CONFIG_DEBUG)
endif()

target_link_libraries(ap_compat PUBLIC
    pico_stdlib
)

# ============================================================================
# CalibrationStore - Persistent calibration data storage
# ============================================================================

add_library(calibration_store STATIC
    src/calibration/CalibrationStore.cpp
)

target_include_directories(calibration_store PUBLIC
    ${CMAKE_SOURCE_DIR}/src/calibration
    ${ROCKETCHIP_INCLUDE_DIR}
)

target_link_libraries(calibration_store PUBLIC
    ap_hal_rp2350
)

# ============================================================================
# ArduPilot Math Library (AP_Math)
# ============================================================================

add_library(ap_math STATIC
    lib/ardupilot/libraries/AP_Math/AP_Math.cpp
    lib/ardupilot/libraries/AP_Math/vector2.cpp
    lib/ardupilot/libraries/AP_Math/vector3.cpp
    lib/ardupilot/libraries/AP_Math/matrix3.cpp
    lib/ardupilot/libraries/AP_Math/matrixN.cpp
    lib/ardupilot/libraries/AP_Math/matrix_alg.cpp
    lib/ardupilot/libraries/AP_Math/quaternion.cpp
    lib/ardupilot/libraries/AP_Math/location.cpp
    lib/ardupilot/libraries/AP_Math/polygon.cpp
    lib/ardupilot/libraries/AP_Math/crc.cpp
    lib/ardupilot/libraries/AP_Math/AP_GeodesicGrid.cpp
)

target_include_directories(ap_math PUBLIC
    ${CMAKE_SOURCE_DIR}/lib/ap_compat
    ${CMAKE_SOURCE_DIR}/lib/ardupilot/libraries
)

target_link_libraries(ap_math PUBLIC
    ap_compat
)

target_compile_definitions(ap_math PUBLIC
    AP_PARAM_ENABLED=1
    HAL_GCS_ENABLED=1
    HAL_WITH_EKF_DOUBLE=0
)

# ============================================================================
# ArduPilot Filter Library
# ============================================================================

add_library(ap_filter STATIC
    lib/ardupilot/libraries/Filter/LowPassFilter.cpp
    lib/ardupilot/libraries/Filter/LowPassFilter2p.cpp
    lib/ardupilot/libraries/Filter/NotchFilter.cpp
    lib/ardupilot/libraries/Filter/HarmonicNotchFilter.cpp
    lib/ardupilot/libraries/Filter/DerivativeFilter.cpp
    lib/ardupilot/libraries/Filter/ModeFilter.cpp
    lib/ardupilot/libraries/Filter/SlewLimiter.cpp
)

target_include_directories(ap_filter PUBLIC
    ${CMAKE_SOURCE_DIR}/lib/ap_compat
    ${CMAKE_SOURCE_DIR}/lib/ardupilot/libraries
)

target_link_libraries(ap_filter PUBLIC
    ap_math
)

# ============================================================================
# ArduPilot AHRS (Attitude and Heading Reference System)
# ============================================================================

add_library(ap_ahrs STATIC
    lib/ap_compat/AP_AHRS/AP_AHRS.cpp
)

target_include_directories(ap_ahrs PUBLIC
    ${CMAKE_SOURCE_DIR}/lib/ap_compat
    ${CMAKE_SOURCE_DIR}/lib/ap_compat/AP_HAL_RP2350
    ${CMAKE_SOURCE_DIR}/lib/ardupilot/libraries
    ${CMAKE_SOURCE_DIR}/src/hal
    ${ROCKETCHIP_INCLUDE_DIR}
)

if(CONFIG_DEBUG)
    target_compile_definitions(ap_ahrs PUBLIC CONFIG_DEBUG)
endif()

target_link_libraries(ap_ahrs PUBLIC
    ap_math
    ap_hal_rp2350
    pico_stdlib
)

# ============================================================================
# GCS MAVLink (Ground Control Station Communication)
# ============================================================================

add_library(ap_gcs STATIC
    lib/ap_compat/GCS_MAVLink/GCS.cpp
)

target_include_directories(ap_gcs PUBLIC
    ${CMAKE_SOURCE_DIR}/lib/ap_compat
    ${CMAKE_SOURCE_DIR}/lib/ap_compat/AP_HAL_RP2350
    ${CMAKE_SOURCE_DIR}/lib  # For mavlink/common/mavlink.h
    ${CMAKE_SOURCE_DIR}/lib/ardupilot/libraries
    ${CMAKE_SOURCE_DIR}/src/hal
    ${ROCKETCHIP_INCLUDE_DIR}
)

if(CONFIG_DEBUG)
    target_compile_definitions(ap_gcs PUBLIC CONFIG_DEBUG)
endif()

target_link_libraries(ap_gcs PUBLIC
    ap_hal_rp2350
    pico_stdlib
)

# ============================================================================
# ArduPilot Calibrators (Accelerometer and Compass)
# ============================================================================

add_library(ap_calibrators STATIC
    lib/ardupilot/libraries/AP_AccelCal/AccelCalibrator.cpp
    lib/ardupilot/libraries/AP_AccelCal/AP_AccelCal.cpp
    # NOTE: CompassCalibrator integration paused pending dependency reevaluation
    # Use ArduPilot's CompassCalibrator.cpp directly when resuming, NOT a custom copy
    # See standards/AP_DEPENDENCY_POLICY.md
)

# Enable compass calibration - these must be defined before AP_Compass_config.h is included
target_compile_definitions(ap_calibrators PUBLIC
    AP_COMPASS_ENABLED=1
    AP_AHRS_DCM_ENABLED=1
    COMPASS_CAL_ENABLED=1
    HAL_GCS_ENABLED=1
)

target_include_directories(ap_calibrators PUBLIC
    ${CMAKE_SOURCE_DIR}/lib/ap_compat
    ${CMAKE_SOURCE_DIR}/lib/ap_compat/AP_HAL_RP2350
    ${CMAKE_SOURCE_DIR}/lib/ardupilot/libraries
    ${CMAKE_SOURCE_DIR}/lib  # For mavlink/common/mavlink.h
    ${CMAKE_SOURCE_DIR}/src/hal
    ${CMAKE_SOURCE_DIR}/src/calibration
    ${ROCKETCHIP_INCLUDE_DIR}
)

target_link_libraries(ap_calibrators PUBLIC
    ap_math
    ap_hal_rp2350
    ap_ahrs
    ap_gcs
    calibration_store
)

# ============================================================================
# ArduPilot StorageManager (required by AP_Param)
# ============================================================================

add_library(ap_storage_manager STATIC
    lib/ardupilot/libraries/StorageManager/StorageManager.cpp
)

target_include_directories(ap_storage_manager PUBLIC
    ${CMAKE_SOURCE_DIR}/lib/ap_compat
    ${CMAKE_SOURCE_DIR}/lib/ap_compat/AP_HAL_RP2350
    ${CMAKE_SOURCE_DIR}/lib/ardupilot/libraries
    ${CMAKE_SOURCE_DIR}/src/hal
    ${ROCKETCHIP_INCLUDE_DIR}
)

target_compile_definitions(ap_storage_manager PUBLIC
    AP_PARAM_ENABLED=1
)

target_link_libraries(ap_storage_manager PUBLIC
    ap_hal_rp2350
)

# ============================================================================
# ArduPilot Parameter System (Real implementation)
# ============================================================================

add_library(ap_param STATIC
    lib/ardupilot/libraries/AP_Param/AP_Param.cpp
    lib/ardupilot/libraries/AP_Param/AP_ParamT.cpp
)

target_include_directories(ap_param PUBLIC
    ${CMAKE_SOURCE_DIR}/lib/ap_compat
    ${CMAKE_SOURCE_DIR}/lib/ap_compat/AP_HAL_RP2350
    ${CMAKE_SOURCE_DIR}/lib/ardupilot/libraries
    ${CMAKE_SOURCE_DIR}/src/hal
    ${ROCKETCHIP_INCLUDE_DIR}
)

target_compile_definitions(ap_param PUBLIC
    AP_PARAM_ENABLED=1
)

target_link_libraries(ap_param PUBLIC
    ap_math
    ap_storage_manager
    ap_hal_rp2350
)

# ============================================================================
# ArduPilot Serial Manager
# ============================================================================
# DISABLED: AP_SerialManager not in sparse checkout
# TODO: Add to sparse checkout when needed
#
# add_library(ap_serial_manager STATIC
#     lib/ardupilot/libraries/AP_SerialManager/AP_SerialManager.cpp
# )
#
# target_include_directories(ap_serial_manager PUBLIC
#     ${CMAKE_SOURCE_DIR}/lib/ap_compat
#     ${CMAKE_SOURCE_DIR}/lib/ap_compat/AP_HAL_RP2350
#     ${CMAKE_SOURCE_DIR}/lib/ardupilot/libraries
#     ${CMAKE_SOURCE_DIR}/src/hal
#     ${ROCKETCHIP_INCLUDE_DIR}
# )
#
# if(CONFIG_DEBUG)
#     target_compile_definitions(ap_serial_manager PUBLIC CONFIG_DEBUG)
# endif()
#
# target_link_libraries(ap_serial_manager PUBLIC
#     ap_param
#     ap_hal_rp2350
#     pico_stdlib
# )

# ============================================================================
# ArduPilot Logger (minimal for AP_InertialSensor dependency)
# ============================================================================
# DISABLED: AP_Logger not in sparse checkout
# TODO: Add to sparse checkout when needed
#
# add_library(ap_logger STATIC
#     lib/ardupilot/libraries/AP_Logger/AP_Logger.cpp
#     lib/ardupilot/libraries/AP_Logger/AP_Logger_Backend.cpp
#     lib/ardupilot/libraries/AP_Logger/LogFile.cpp
#     lib/ardupilot/libraries/AP_Logger/LoggerMessageWriter.cpp
# )
#
# target_include_directories(ap_logger PUBLIC
#     ${CMAKE_SOURCE_DIR}/lib/ap_compat
#     ${CMAKE_SOURCE_DIR}/lib/ap_compat/AP_HAL_RP2350
#     ${CMAKE_SOURCE_DIR}/lib/ardupilot/libraries
#     ${CMAKE_SOURCE_DIR}/src/hal
#     ${ROCKETCHIP_INCLUDE_DIR}
# )
#
# if(CONFIG_DEBUG)
#     target_compile_definitions(ap_logger PUBLIC CONFIG_DEBUG)
# endif()
#
# target_link_libraries(ap_logger PUBLIC
#     ap_param
#     ap_hal_rp2350
#     pico_stdlib
# )

# ============================================================================
# ArduPilot Inertial Sensor (Full library with ICM-20948 support)
# ============================================================================
# ICM-20948 is supported by AP_InertialSensor_Invensensev2

add_library(ap_inertial_sensor STATIC
    # Core framework - USE AP_COMPAT VERSION for std::atomic fix (PD12)
    # The ap_compat versions have _new_gyro_data/_new_accel_data as std::atomic<bool>
    # for cross-core memory visibility on FreeRTOS SMP dual-core RP2350
    lib/ap_compat/AP_InertialSensor/AP_InertialSensor.cpp
    lib/ap_compat/AP_InertialSensor/AP_InertialSensor_Backend.cpp
    # Rest from ArduPilot (no cross-core flag issues)
    lib/ardupilot/libraries/AP_InertialSensor/AP_InertialSensor_Params.cpp
    lib/ardupilot/libraries/AP_InertialSensor/AP_InertialSensor_Logging.cpp
    lib/ardupilot/libraries/AP_InertialSensor/AP_InertialSensor_tempcal.cpp
    lib/ardupilot/libraries/AP_InertialSensor/BatchSampler.cpp
    lib/ardupilot/libraries/AP_InertialSensor/FastRateBuffer.cpp
    lib/ardupilot/libraries/AP_InertialSensor/AuxiliaryBus.cpp
    # ICM-20948 driver (via Invensensev2) - custom copy with debug disabled
    lib/ap_compat/AP_InertialSensor_Invensensev2/AP_InertialSensor_Invensensev2.cpp
)

target_include_directories(ap_inertial_sensor PUBLIC
    # CRITICAL: ap_compat/AP_InertialSensor MUST come BEFORE ardupilot/libraries
    # so that the std::atomic fixed headers are used instead of ArduPilot's originals
    ${CMAKE_SOURCE_DIR}/lib/ap_compat/AP_InertialSensor
    ${CMAKE_SOURCE_DIR}/lib/ap_compat
    ${CMAKE_SOURCE_DIR}/lib/ap_compat/AP_HAL_RP2350
    ${CMAKE_SOURCE_DIR}/lib/ardupilot/libraries
    ${CMAKE_SOURCE_DIR}/src/hal
    ${ROCKETCHIP_INCLUDE_DIR}
)

# Enable only ICM-20948 backend, disable others
target_compile_definitions(ap_inertial_sensor PUBLIC
    AP_INERTIALSENSOR_ENABLED=1
    HAL_GCS_ENABLED=1  # Required for AP_AccelCal
    HAL_INS_DEFAULT=HAL_INS_INVENSENSEV2_I2C
    AP_INERTIALSENSOR_INVENSENSEV2_ENABLED=1
    # Disable unused backends
    AP_INERTIALSENSOR_BMI055_ENABLED=0
    AP_INERTIALSENSOR_BMI088_ENABLED=0
    AP_INERTIALSENSOR_BMI160_ENABLED=0
    AP_INERTIALSENSOR_BMI270_ENABLED=0
    AP_INERTIALSENSOR_INVENSENSE_ENABLED=0
    AP_INERTIALSENSOR_INVENSENSEV3_ENABLED=0
    AP_INERTIALSENSOR_LSM9DS0_ENABLED=0
    AP_INERTIALSENSOR_LSM9DS1_ENABLED=0
    AP_INERTIALSENSOR_L3G4200D_ENABLED=0
    AP_INERTIALSENSOR_ADIS1647X_ENABLED=0
    AP_INERTIALSENSOR_SCHA63T_ENABLED=0
    AP_INERTIALSENSOR_SITL_ENABLED=0
    AP_INERTIALSENSOR_EXTERNALAHRS_ENABLED=0
)

if(CONFIG_DEBUG)
    target_compile_definitions(ap_inertial_sensor PUBLIC CONFIG_DEBUG)
endif()

target_link_libraries(ap_inertial_sensor PUBLIC
    ap_param
    ap_calibrators
    ap_filter
    # ap_logger  # Disabled - not yet implemented
    ap_hal_rp2350
    pico_stdlib
)

# ============================================================================
# ArduPilot Compass (AK09916 via ICM-20948)
# ============================================================================

add_library(ap_compass STATIC
    # Core framework
    lib/ardupilot/libraries/AP_Compass/AP_Compass.cpp
    lib/ardupilot/libraries/AP_Compass/AP_Compass_Backend.cpp
    # Compass_PerMotor stubbed in compass_stubs.cpp (motor compensation not for rockets)
    # Calibration methods (cal_update, start_calibration_all, cancel_calibration_all, etc.)
    lib/ardupilot/libraries/AP_Compass/AP_Compass_Calibration.cpp
    # Calibration algorithm is in CompassCalibrator.cpp
    lib/ardupilot/libraries/AP_Compass/CompassCalibrator.cpp
    # AK09916 driver (inside ICM-20948)
    lib/ardupilot/libraries/AP_Compass/AP_Compass_AK09916.cpp
    # Declination tables for magnetic variation
    lib/ardupilot/libraries/AP_Declination/AP_Declination.cpp
    lib/ardupilot/libraries/AP_Declination/tables.cpp
    # Stubs for complex dependencies (Compass_PerMotor, Location::zero)
    lib/ap_compat/stubs/compass_stubs.cpp
)

target_include_directories(ap_compass PUBLIC
    ${CMAKE_SOURCE_DIR}/lib/ap_compat
    ${CMAKE_SOURCE_DIR}/lib/ap_compat/AP_HAL_RP2350
    ${CMAKE_SOURCE_DIR}/lib/ardupilot/libraries
    ${ROCKETCHIP_INCLUDE_DIR}
)

target_compile_definitions(ap_compass PUBLIC
    AP_COMPASS_ENABLED=1
    COMPASS_CAL_ENABLED=1
    AP_COMPASS_AK09916_ENABLED=1
    AP_COMPASS_ICM20948_ENABLED=1
    # Disable unused backends
    AP_COMPASS_SITL_ENABLED=0
    AP_COMPASS_DRONECAN_ENABLED=0
    AP_COMPASS_EXTERNALAHRS_ENABLED=0
    AP_COMPASS_LIS3MDL_ENABLED=0
    AP_COMPASS_HMC5843_ENABLED=0
    AP_COMPASS_AK8963_ENABLED=0
    AP_COMPASS_BMM150_ENABLED=0
    AP_COMPASS_BMM350_ENABLED=0
    AP_COMPASS_MMC3416_ENABLED=0
    AP_COMPASS_MMC5XX3_ENABLED=0
    AP_COMPASS_QMC5883L_ENABLED=0
    AP_COMPASS_QMC5883P_ENABLED=0
    AP_COMPASS_IST8308_ENABLED=0
    AP_COMPASS_IST8310_ENABLED=0
    AP_COMPASS_LIS2MDL_ENABLED=0
    AP_COMPASS_IIS2MDC_ENABLED=0
    AP_COMPASS_LSM303D_ENABLED=0
    AP_COMPASS_LSM9DS1_ENABLED=0
    AP_COMPASS_MAG3110_ENABLED=0
    AP_COMPASS_RM3100_ENABLED=0
    AP_COMPASS_MSP_ENABLED=0
)

if(CONFIG_DEBUG)
    target_compile_definitions(ap_compass PUBLIC CONFIG_DEBUG)
endif()

target_link_libraries(ap_compass PUBLIC
    ap_inertial_sensor  # For auxiliary bus access
    ap_param
    ap_hal_rp2350
    pico_stdlib
)

# ============================================================================
# ArduPilot Barometer (DPS310)
# ============================================================================

add_library(ap_baro STATIC
    # Use ap_compat versions (customized for RocketChip, not ArduPilot originals)
    lib/ap_compat/AP_Baro/AP_Baro.cpp
    lib/ap_compat/AP_Baro/AP_Baro_Backend.cpp
    lib/ap_compat/AP_Baro/AP_Baro_DPS280.cpp
    lib/ap_compat/AP_Baro/AP_Baro_atmosphere.cpp
    # Filter library (DerivativeFilter used by AP_Baro)
    lib/ardupilot/libraries/Filter/DerivativeFilter.cpp
)

target_include_directories(ap_baro PUBLIC
    ${CMAKE_SOURCE_DIR}/lib/ap_compat
    ${CMAKE_SOURCE_DIR}/lib/ap_compat/AP_HAL_RP2350
    ${CMAKE_SOURCE_DIR}/lib/ardupilot/libraries
    ${ROCKETCHIP_INCLUDE_DIR}
)

target_compile_definitions(ap_baro PUBLIC
    AP_BARO_ENABLED=1
    AP_BARO_DPS280_ENABLED=1
    # Disable unused backends
    AP_BARO_SITL_ENABLED=0
    AP_BARO_DRONECAN_ENABLED=0
    AP_BARO_EXTERNALAHRS_ENABLED=0
    AP_BARO_MSP_ENABLED=0
    AP_BARO_BMP085_ENABLED=0
    AP_BARO_BMP280_ENABLED=0
    AP_BARO_BMP388_ENABLED=0
    AP_BARO_BMP581_ENABLED=0
    AP_BARO_FBM320_ENABLED=0
    AP_BARO_ICM20789_ENABLED=0
    AP_BARO_ICP101XX_ENABLED=0
    AP_BARO_ICP201XX_ENABLED=0
    AP_BARO_KELLERLD_ENABLED=0
    AP_BARO_LPS2XH_ENABLED=0
    AP_BARO_MS5611_ENABLED=0
    AP_BARO_SPL06_ENABLED=0
)

if(CONFIG_DEBUG)
    target_compile_definitions(ap_baro PUBLIC CONFIG_DEBUG)
endif()

target_link_libraries(ap_baro PUBLIC
    ap_param
    ap_hal_rp2350
    pico_stdlib
)

# ============================================================================
# Services Library (FreeRTOS Tasks)
# ============================================================================

add_library(services STATIC
    src/services/SensorTask.cpp
)

target_include_directories(services PUBLIC
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/services
    ${ROCKETCHIP_INCLUDE_DIR}
)

if(CONFIG_DEBUG)
    target_compile_definitions(services PUBLIC CONFIG_DEBUG)
endif()

target_link_libraries(services PUBLIC
    hal
    hal_sensors
    ap_inertial_sensor
    ap_compass
    ap_baro
    FreeRTOS-Kernel
)

# ============================================================================
# Production Target: rocketchip
# ============================================================================

add_executable(rocketchip
    src/main.cpp
)

target_include_directories(rocketchip PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${ROCKETCHIP_INCLUDE_DIR}
)

target_link_libraries(rocketchip
    services
    hal
    hal_sensors
    ap_compat
    pico_stdlib
    pico_multicore
    hardware_i2c
    hardware_spi
    hardware_dma
    hardware_timer
    hardware_clocks
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(rocketchip 1)
pico_enable_stdio_uart(rocketchip 0)

pico_add_extra_outputs(rocketchip)

# ============================================================================
# Validation Target: freertos_validation (preserved for reference/testing)
# ============================================================================

add_executable(freertos_validation
    tests/validation/freertos_validation/main.c
    tests/validation/freertos_validation/hooks.c
)

target_link_libraries(freertos_validation
    pico_stdlib
    pico_multicore
    hardware_i2c
    hardware_spi
    hardware_dma
    hardware_timer
    hardware_clocks
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(freertos_validation 1)
pico_enable_stdio_uart(freertos_validation 0)

pico_add_extra_outputs(freertos_validation)

# Simple test executable (no FreeRTOS)
add_executable(simple_test
    tests/smoke_tests/simple_test.c
)

target_link_libraries(simple_test
    pico_stdlib
)

pico_enable_stdio_usb(simple_test 1)
pico_enable_stdio_uart(simple_test 0)

pico_add_extra_outputs(simple_test)

# ============================================================================
# Smoke Tests
# ============================================================================

# IMU Qwiic Port Test - verifies ISM330DHCX + LIS3MDL FeatherWing connectivity
add_executable(smoke_imu_qwiic
    tests/smoke_tests/imu_qwiic_test.c
)

target_link_libraries(smoke_imu_qwiic
    pico_stdlib
    hardware_i2c
)

pico_enable_stdio_usb(smoke_imu_qwiic 1)
pico_enable_stdio_uart(smoke_imu_qwiic 0)

pico_add_extra_outputs(smoke_imu_qwiic)

# HAL Validation Test - comprehensive HAL smoke test
add_executable(smoke_hal_validation
    tests/smoke_tests/hal_validation.cpp
)

target_link_libraries(smoke_hal_validation
    hal
    pico_stdlib
    hardware_i2c
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(smoke_hal_validation 1)
pico_enable_stdio_uart(smoke_hal_validation 0)

pico_add_extra_outputs(smoke_hal_validation)

# ST Driver Sensor Test - validates all three ST driver wrappers
add_executable(smoke_st_sensors
    tests/smoke_tests/st_sensors_test.cpp
)

target_link_libraries(smoke_st_sensors
    hal_sensors
    pico_stdlib
    hardware_i2c
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(smoke_st_sensors 1)
pico_enable_stdio_uart(smoke_st_sensors 0)

pico_add_extra_outputs(smoke_st_sensors)

# GPS Module Test - validates PA1010D GPS driver
add_executable(smoke_gps
    tests/smoke_tests/gps_test.cpp
)

target_link_libraries(smoke_gps
    hal_sensors
    pico_stdlib
    hardware_i2c
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(smoke_gps 1)
pico_enable_stdio_uart(smoke_gps 0)

pico_add_extra_outputs(smoke_gps)

# I2C Bus Scanner - debug tool to find devices
add_executable(i2c_scan
    tests/smoke_tests/i2c_scan.c
)

target_link_libraries(i2c_scan
    pico_stdlib
    hardware_i2c
)

pico_enable_stdio_usb(i2c_scan 1)
pico_enable_stdio_uart(i2c_scan 0)

pico_add_extra_outputs(i2c_scan)

# ============================================================================
# Radio Tests
# ============================================================================

# Radio TX Test - transmits debug packets
add_executable(smoke_radio_tx
    tests/smoke_tests/radio_tx_test.cpp
)

target_link_libraries(smoke_radio_tx
    hal
    pico_stdlib
    hardware_spi
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(smoke_radio_tx 1)
pico_enable_stdio_uart(smoke_radio_tx 0)

pico_add_extra_outputs(smoke_radio_tx)

# ============================================================================
# Ground Station
# ============================================================================

# Radio RX - Ground station receiver
add_executable(radio_rx
    ground_station/radio_rx.cpp
)

target_link_libraries(radio_rx
    hal
    pico_stdlib
    hardware_spi
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(radio_rx 1)
pico_enable_stdio_uart(radio_rx 0)

pico_add_extra_outputs(radio_rx)

# ============================================================================
# Calibration Smoke Test (DISABLED - needs FreeRTOS hooks update)
# ============================================================================
# TODO: Update calibration_test.cpp to include FreeRTOS hooks or
#       integrate with main firmware for calibration testing via CLI

# ============================================================================
# INS Probe Test - Full AP_InertialSensor integration
# ============================================================================

add_executable(smoke_ins_probe
    tests/smoke_tests/ins_probe_test.cpp
)

target_link_libraries(smoke_ins_probe
    ap_hal_rp2350
    ap_inertial_sensor
    ap_math
    pico_stdlib
    hardware_i2c
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

target_include_directories(smoke_ins_probe PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/ap_compat/AP_HAL_RP2350
    ${CMAKE_SOURCE_DIR}/lib/ap_compat
    ${CMAKE_SOURCE_DIR}/lib/ardupilot/libraries
)

pico_enable_stdio_usb(smoke_ins_probe 1)
pico_enable_stdio_uart(smoke_ins_probe 0)

pico_add_extra_outputs(smoke_ins_probe)

# ============================================================================
# Compass Calibration Synthetic Test (DISABLED - needs FreeRTOS hooks + API update)
# ============================================================================
# TODO: Update compass_cal_synthetic_test.cpp with current CompassCalibrator API

# ============================================================================
# Compass Minimal Test (DISABLED - needs FreeRTOS hooks + API update)
# ============================================================================
# TODO: Update compass_minimal_test.cpp with current Compass API

# ============================================================================
# USB Input Test (minimal, no flash operations)
# ============================================================================

add_executable(smoke_usb_input
    tests/smoke_tests/usb_input_test.cpp
)

target_link_libraries(smoke_usb_input
    pico_stdlib
)

pico_enable_stdio_usb(smoke_usb_input 1)
pico_enable_stdio_uart(smoke_usb_input 0)

pico_add_extra_outputs(smoke_usb_input)

# ============================================================================
# USB + NeoPixel Test (tests if NeoPixel PIO before stdio_init_all breaks USB)
# ============================================================================

add_executable(smoke_usb_neopixel
    tests/smoke_tests/usb_neopixel_test.cpp
)

target_link_libraries(smoke_usb_neopixel
    pico_stdlib
    hal
)

pico_enable_stdio_usb(smoke_usb_neopixel 1)
pico_enable_stdio_uart(smoke_usb_neopixel 0)

pico_add_extra_outputs(smoke_usb_neopixel)

# ============================================================================
# USB + HAL Test (tests if hal.init() before stdio_init_all breaks USB)
# ============================================================================

add_executable(smoke_usb_hal
    tests/smoke_tests/usb_hal_test.cpp
)

target_link_libraries(smoke_usb_hal
    pico_stdlib
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    ap_hal_rp2350
)

pico_enable_stdio_usb(smoke_usb_hal 1)
pico_enable_stdio_uart(smoke_usb_hal 0)

pico_add_extra_outputs(smoke_usb_hal)

# ============================================================================
# AP_HAL_RP2350 Smoke Test
# ============================================================================

add_executable(smoke_ap_hal
    tests/smoke_tests/ap_hal_test.cpp
)

target_link_libraries(smoke_ap_hal
    pico_stdlib
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    ap_hal_rp2350
)

pico_enable_stdio_usb(smoke_ap_hal 1)
pico_enable_stdio_uart(smoke_ap_hal 0)

pico_add_extra_outputs(smoke_ap_hal)

# ============================================================================
# Storage Smoke Test
# ============================================================================

add_executable(smoke_storage
    tests/smoke_tests/storage_test.cpp
    # hooks defined inline in storage_test.cpp (matching ap_hal_test.cpp pattern)
)

target_link_libraries(smoke_storage
    ap_hal_rp2350
    pico_stdlib
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(smoke_storage 1)
pico_enable_stdio_uart(smoke_storage 0)

pico_add_extra_outputs(smoke_storage)

# ============================================================================
# AP_HAL_RP2350 Phase 2 Smoke Test
# ============================================================================

add_executable(smoke_phase2
    tests/smoke_tests/smoke_phase2.cpp
)

target_link_libraries(smoke_phase2
    ap_hal_rp2350
    hal
    pico_stdlib
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(smoke_phase2 1)
pico_enable_stdio_uart(smoke_phase2 0)

pico_add_extra_outputs(smoke_phase2)

# ============================================================================
# Minimal Blink Test (no FreeRTOS - for debugging boot issues)
# ============================================================================
# DISABLED: Test file does not exist
#
# add_executable(minimal_blink
#     tests/smoke_tests/minimal_blink.cpp
# )
#
# target_link_libraries(minimal_blink
#     pico_stdlib
# )
#
# pico_add_extra_outputs(minimal_blink)

# Flash Test - minimal test to verify flash operations work
add_executable(flash_test
    tests/smoke_tests/flash_test.cpp
)

target_link_libraries(flash_test
    pico_stdlib
    hardware_flash
    hardware_sync
)

pico_add_extra_outputs(flash_test)

# Baro Test - simple DPS310 test to isolate restart hang (no FreeRTOS/ArduPilot)
add_executable(smoke_baro
    tests/smoke_tests/baro_test.cpp
)

target_link_libraries(smoke_baro
    pico_stdlib
    hardware_i2c
    hardware_gpio
)

pico_enable_stdio_usb(smoke_baro 1)
pico_enable_stdio_uart(smoke_baro 0)

pico_add_extra_outputs(smoke_baro)

# FreeRTOS Static Allocation Test - minimal test without AP_HAL
# DISABLED: Test file does not exist
#
# add_executable(freertos_static_test
#     tests/smoke_tests/freertos_static_test.cpp
# )
#
# target_link_libraries(freertos_static_test
#     pico_stdlib
#     FreeRTOS-Kernel
#     FreeRTOS-Kernel-Heap4
# )
#
# pico_add_extra_outputs(freertos_static_test)

# FreeRTOS with AP_HAL include (but no HAL usage) - tests global object
# DISABLED: Test file does not exist
#
# add_executable(freertos_with_hal_include
#     tests/smoke_tests/freertos_with_hal_include.cpp
# )
#
# target_link_libraries(freertos_with_hal_include
#     pico_stdlib
#     FreeRTOS-Kernel
#     FreeRTOS-Kernel-Heap4
#     ap_hal_rp2350
# )
#
# pico_enable_stdio_usb(freertos_with_hal_include 1)
# pico_enable_stdio_uart(freertos_with_hal_include 0)
#
# pico_add_extra_outputs(freertos_with_hal_include)
