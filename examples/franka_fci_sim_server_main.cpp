#include "franka_drake/fci_sim_server.h"
#include <iostream>

// --- Drake integration stubs ---
// TODO: Include your Drake headers and types here
// #include <drake/...>

// Example: Convert Drake's robot state to protocol::RobotState
franka_fci_sim::protocol::RobotState drake_state_to_protocol_state(/*const DrakePlant& plant, ...*/) {
  franka_fci_sim::protocol::RobotState state{};
  // TODO: Fill in state fields from your Drake simulation
  // Example:
  // state.q = ...; // joint positions
  // state.dq = ...; // joint velocities
  // state.tau_J = ...; // measured torques
  // state.O_T_EE = ...; // end-effector pose
  // ...
  return state;
}

// Example: Apply protocol::RobotCommand to Drake simulation
void apply_protocol_command_to_drake(const franka_fci_sim::protocol::RobotCommand& cmd/*, DrakePlant& plant, ...*/) {
  // TODO: Extract desired torques/positions from cmd and apply to Drake
  // Example:
  // plant.SetActuation(cmd.control.tau_J_d);
  // ...
}

int main(int argc, char** argv) {
  using namespace franka_fci_sim;
  FrankaFciSimServer server(30200);

  // TODO: Create or access your Drake simulation objects here
  // DrakePlant plant = ...;

  // State provider: returns the current simulated robot state
  server.set_state_provider([/*&plant*/]() {
    return drake_state_to_protocol_state(/*plant*/);
  });

  // Command handler: applies received commands to the simulation
  server.set_command_handler([/*&plant*/](const protocol::RobotCommand& cmd) {
    apply_protocol_command_to_drake(cmd/*, plant*/);
  });

  std::cout << "Starting Franka FCI Simulator Server..." << std::endl;
  server.run();
  return 0;
} 