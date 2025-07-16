#!/bin/bash

# Franka Drake build script

set -e  # Exit on any error

echo "Building Franka Drake..."

# Create build directory if it doesn't exist
mkdir -p build
cd build

# Configure with CMake
echo "Configuring with CMake..."
cmake ..

# Build
echo "Building..."
make -j$(nproc)

echo "Build complete!"
echo "Executable location: build/bin/franka-fci-sim-server" 