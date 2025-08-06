#!/bin/bash

# Franka Drake FCI Server runner with End Effector Configuration Support

set -e

# Function to display usage
show_usage() {
    echo "Usage: $0 [CONFIG] [MODE]"
    echo ""
    echo "CONFIG options:"
    echo "  fingerless  - Franka arm without gripper (non-EE, fer_link7 end effector)"
    echo "  gripper     - Franka arm with gripper (EE, fer_hand_tcp end effector)"
    echo "  auto        - Interactive selection (default)"
    echo ""
    echo "MODE options:"
    echo "  headless    - Run without visualization, continuous logging"
    echo "  visual      - Run with Meshcat visualization (default)"
    echo "  turbo       - Run faster than real-time (headless + turbo)"
    echo ""
    echo "Examples:"
    echo "  $0                          # Interactive selection with visualization"
    echo "  $0 fingerless headless      # Fingerless robot, headless mode"
    echo "  $0 gripper visual           # Gripper robot with visualization"
    echo "  $0 gripper turbo            # Gripper robot, fast simulation"
}

# Parse command line arguments
CONFIG=${1:-auto}
MODE=${2:-visual}

# Validate config argument
if [[ "$CONFIG" != "fingerless" && "$CONFIG" != "gripper" && "$CONFIG" != "auto" ]]; then
    echo "Error: Invalid CONFIG '$CONFIG'"
    show_usage
    exit 1
fi

# Validate mode argument
if [[ "$MODE" != "headless" && "$MODE" != "visual" && "$MODE" != "turbo" ]]; then
    echo "Error: Invalid MODE '$MODE'"
    show_usage
    exit 1
fi

# Check if build exists
if [ ! -f "build/bin/franka-fci-sim-server" ]; then
    echo "Server not built. Running build first..."
    echo "Building Franka Drake..."

    # Create build directory if it doesn't exist
    mkdir -p build
fi

# Build
cd build
cmake ..
echo "Building..."
make -j$(nproc)
echo "Build complete!"
echo "Executable location: build/bin/franka-fci-sim-server"
cd ..

# Interactive configuration selection
if [ "$CONFIG" = "auto" ]; then
    echo ""
    echo "=== Franka Robot Configuration Selection ==="
    echo "1) Fingerless Robot (Non-EE)"
    echo "   - End effector: fer_link7 (wrist)"
    echo "   - Mass: 0.36kg, 7 joints"
    echo "   - Use case: Basic arm movements, tool mounting"
    echo ""
    echo "2) Gripper Robot (EE)"
    echo "   - End effector: fer_hand_tcp (tool center point)"
    echo "   - Mass: 0.73kg, 9 joints (7 arm + 2 gripper)"
    echo "   - Use case: Grasping, manipulation tasks"
    echo ""
    read -p "Select configuration (1 or 2): " choice
    
    case $choice in
        1)
            CONFIG="fingerless"
            ;;
        2)
            CONFIG="gripper"
            ;;
        *)
            echo "Invalid choice. Defaulting to fingerless configuration."
            CONFIG="fingerless"
            ;;
    esac
fi

# Set URDF file based on configuration
if [ "$CONFIG" = "fingerless" ]; then
    URDF_FILE="models/urdf/fer_drake_fingerless.urdf"
    CONFIG_DESC="Fingerless Robot (Non-EE)"
else
    URDF_FILE="models/urdf/fer_drake_gripper_fixed.urdf"
    CONFIG_DESC="Gripper Robot (EE)"
fi

# Set mode parameters
case $MODE in
    "headless")
        HEADLESS_FLAG="true"
        EXTRA_ARGS=""
        MODE_DESC="Headless (no visualization)"
        ;;
    "visual")
        HEADLESS_FLAG="false"
        EXTRA_ARGS=""
        MODE_DESC="Visual (Meshcat visualization)"
        ;;
    "turbo")
        HEADLESS_FLAG="true"
        EXTRA_ARGS="turbo"
        MODE_DESC="Turbo (faster than real-time)"
        ;;
esac

echo ""
echo "=== Starting Franka FCI Simulation Server ==="
echo "Configuration: $CONFIG_DESC"
echo "Mode: $MODE_DESC"
echo "URDF: $URDF_FILE"
echo ""
echo "The server will auto-detect and configure:"
if [ "$CONFIG" = "fingerless" ]; then
    echo "  • End Effector Frame: fer_link7"
    echo "  • Joint Count: 7 (arm only)"
    echo "  • Mass: 0.36kg"
    echo "  • Gripper: None"
else
    echo "  • End Effector Frame: fer_hand_tcp"
    echo "  • Joint Count: 9 (7 arm + 2 gripper)"
    echo "  • Mass: 0.73kg"
    echo "  • Gripper: 2 finger joints"
fi
echo ""
if [ "$MODE" = "visual" ]; then
    echo "Visualization available at: http://localhost:7000"
fi
echo "FCI Server listening on: TCP port 1337, UDP port 1338"
echo "Press Ctrl+C to stop"
echo ""

# Run the server
./build/bin/franka-fci-sim-server models/urdf/package.xml "$URDF_FILE" 0.001 "$HEADLESS_FLAG" $EXTRA_ARGS