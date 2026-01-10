#!/bin/bash
# RocketChip FreeRTOS SMP Build Script
# Builds validation firmware for RP2350 (Pico 2)

set -e  # Exit on error

echo "======================================"
echo "RocketChip FreeRTOS SMP Build"
echo "======================================"
echo ""

# Check for ARM toolchain
if ! command -v arm-none-eabi-gcc &> /dev/null; then
    echo "ERROR: ARM toolchain not found!"
    echo ""
    echo "Install with:"
    echo "  Ubuntu/Debian: sudo apt install gcc-arm-none-eabi cmake build-essential"
    echo "  macOS: brew install --cask gcc-arm-embedded"
    echo "  Windows: Download from https://developer.arm.com/downloads/-/gnu-rm"
    echo ""
    exit 1
fi

echo "ARM GCC version:"
arm-none-eabi-gcc --version | head -1
echo ""

# Create build directory
echo "Creating build directory..."
mkdir -p build
cd build

# Configure with CMake
echo "Configuring with CMake..."
cmake -DPICO_SDK_PATH=../pico-sdk -DPICO_NO_PICOTOOL=1 ..

# Build
echo ""
echo "Building..."
make -j$(nproc 2>/dev/null || echo 4)

# Success
echo ""
echo "======================================"
echo "Build complete!"
echo "======================================"
echo ""
echo "Output files:"
ls -lh freertos_validation.uf2 freertos_validation.elf 2>/dev/null || true
echo ""
echo "To flash:"
echo "1. Hold BOOTSEL button on Pico 2"
echo "2. Connect USB cable"
echo "3. Copy freertos_validation.uf2 to RPI-RP2 drive"
echo ""
