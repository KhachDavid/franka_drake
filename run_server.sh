#!/bin/bash

# Franka Drake FCI Server runner

set -e

# Check if build exists
if [ ! -f "build/bin/franka-fci-sim-server" ]; then
    echo "Server not built. Running build first..."
    ./build.sh
fi

echo "Starting Franka FCI Simulation Server..."
echo "Press Ctrl+C to stop"

# Run the server
./build/bin/franka-fci-sim-server ~/ws/franka/src/franka_description/package.xml models/urdf/fer_drake_fingerless.urdf 0.001 false