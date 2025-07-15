# franka_drake

This package provides a modular, production-quality interface for simulating the Franka Emika Panda robot in Drake, and for drop-in simulation with libfranka-based applications.

## Directory Structure

- `include/franka_drake/` — Public headers for the Drake module and simulator server
- `src/` — Implementation of the Drake module and simulator server
- `examples/` — Example binaries for running the simulator server and using the Drake module
- `urdf/` — Robot model files (URDF)
- `scripts/` — (Optional) Helper scripts

## Usage

- To run the simulator server (libfranka drop-in): see `examples/run_simulator_server.cpp`
- To use the Franka Drake module in your own simulation: see `examples/drake_simple_example.cpp`

## Building

Standard CMake build. See CMakeLists.txt for details.
