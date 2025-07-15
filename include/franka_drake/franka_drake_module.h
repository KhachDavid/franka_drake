#pragma once

#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/geometry/scene_graph.h>
#include <memory>
#include <string>

namespace franka_drake {

/**
 * @brief Drake System for simulating a Franka arm.
 *
 * Loads a URDF, adds actuators, sets up damping, disables collisions, and exposes
 * input/output ports for integration into any Drake Diagram.
 */
class FrankaDrakeModule {
 public:
  /**
   * @param builder Pointer to the DiagramBuilder to register systems with.
   * @param package_xml Path to the package.xml for resource resolution.
   * @param urdf_path Path to the Franka URDF file.
   * @param time_step Discrete update period for the plant.
   */
  FrankaDrakeModule(drake::systems::DiagramBuilder<double>* builder,
                    const std::string& package_xml,
                    const std::string& urdf_path,
                    double time_step = 0.001);

  // Returns a pointer to the underlying MultibodyPlant.
  drake::multibody::MultibodyPlant<double>* get_plant();
  drake::geometry::SceneGraph<double>* get_scene_graph();

 private:
  drake::multibody::MultibodyPlant<double>* plant_ = nullptr;
  drake::geometry::SceneGraph<double>* scene_graph_ = nullptr;
};

} // namespace franka_drake 