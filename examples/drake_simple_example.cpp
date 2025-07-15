#include "franka_drake/franka_drake_module.h"
#include <drake/systems/framework/diagram_builder.h>
#include <iostream>

int main(int argc, char* argv[]) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <package.xml> <urdf_path> [time_step]\n";
    return 1;
  }
  std::string package_xml = argv[1];
  std::string urdf_path = argv[2];
  double time_step = (argc > 3) ? std::stod(argv[3]) : 0.001;

  drake::systems::DiagramBuilder<double> builder;
  franka_drake::FrankaDrakeModule module(&builder, package_xml, urdf_path, time_step);

  std::cout << "FrankaDrakeModule created and ready for integration!\n";
  return 0;
} 