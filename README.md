# Franka Drake Integration

A Franka Control Interface (FCI) simulation server using Drake for robot simulation and libfranka for control.

## Project Structure

```
franka_drake/
├── apps/                    # Applications
│   ├── CMakeLists.txt
│   └── franka_fci_sim_server_main.cpp  # Main FCI server
├── include/                 # Header files
│   └── franka_drake/
├── models/                  # Robot models
│   └── urdf/               # URDF files
├── scripts/                 # Utility scripts
├── src/                     # Core library source
│   ├── CMakeLists.txt
│   ├── franka_drake_module.cpp
│   ├── simulator_server.cpp
│   └── fci_sim_server.cpp
├── build/                   # Build directory (created)
├── CMakeLists.txt          # Main CMake configuration
└── README.md
```

## Dependencies

- Drake (for robot simulation)
- Poco (for networking)
- libfranka (for Franka robot control)

## Building

```bash
# Create build directory
mkdir build && cd build

# Configure with CMake
cmake ..

# Build
make -j$(nproc)

# Install (optional)
sudo make install
```

## Usage

Run the FCI simulation server:

```bash
./bin/franka-fci-sim-server
```

The server will start and listen for libfranka connections on the default FCI port.

## Development

- All build artifacts are contained in the `build/` directory
- Source code is organized in `src/` for the core library
- Applications are in `apps/`
- Models and URDF files are in `models/`
