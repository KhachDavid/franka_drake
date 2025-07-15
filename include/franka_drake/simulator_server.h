#pragma once

#include <string>
#include <memory>
#include <drake/systems/framework/diagram_builder.h>
#include "franka_drake/franka_drake_module.h"

namespace franka_drake {

/**
 * @brief Simulator server for libfranka-compatible simulation.
 *
 * Listens on a TCP port, accepts connections, and simulates a Franka arm using Drake.
 * Implements the FCI protocol (or a compatible subset).
 */
class SimulatorServer {
 public:
  /**
   * @param port TCP port to listen on (default: 12345)
   * @param package_xml Path to the package.xml for resource resolution
   * @param urdf_path Path to the Franka URDF file
   * @param time_step Discrete update period for the plant
   */
  SimulatorServer(int port, const std::string& package_xml, const std::string& urdf_path, double time_step = 0.001);

  // Start the server (blocking call)
  void Run();

 private:
  int port_;
  std::string package_xml_;
  std::string urdf_path_;
  double time_step_;
  std::unique_ptr<drake::systems::DiagramBuilder<double>> builder_;
  std::unique_ptr<FrankaDrakeModule> module_;
  // Add more members as needed (e.g., Poco server, connection handlers)
};

} // namespace franka_drake 