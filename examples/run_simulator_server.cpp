#include "franka_drake/simulator_server.h"
#include <iostream>

int main(int argc, char* argv[]) {
  if (argc < 4) {
    std::cerr << "Usage: " << argv[0] << " <package.xml> <urdf_path> <port> [time_step]\n";
    return 1;
  }
  std::string package_xml = argv[1];
  std::string urdf_path = argv[2];
  int port = std::stoi(argv[3]);
  double time_step = (argc > 4) ? std::stod(argv[4]) : 0.001;

  franka_drake::SimulatorServer server(port, package_xml, urdf_path, time_step);
  server.Run();
  return 0;
} 