// Minimal example: demonstrate hooking the FCI embedder into an existing
// Drake builder/plant. Keep everything else (controllers, Meshcat) up to the user.

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/visualization/visualization_config_functions.h>
#include <drake/geometry/meshcat.h>
#include <iostream>
#include <thread>
#include <chrono>
#include "franka_drake/fci_sim_embed.h"

using drake::multibody::AddMultibodyPlantSceneGraph;

int main() {
  // Assume user owns Meshcat and controllers; we only provide the FCI hookup.
  const double dt = 0.001;
  drake::systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, dt);

  // One-liner: let the library add the default Franka model (fingerless by default),
  // weld base, add actuators/damping, finalize, and attach the FCI server.
  franka_fci_sim::FciSimEmbedder::AutoAttachOptions auto_opts;
  auto_opts.prefer_gripper = false; // set true to auto-load gripper variant
  auto_opts.disable_collisions = true; // let embedder remove proximity roles
  auto embed = franka_fci_sim::FciSimEmbedder::AutoAttach(&plant, &scene_graph, &builder, auto_opts, {});

  // Add Meshcat visualization.
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();
  drake::visualization::AddDefaultVisualization(&builder, meshcat);

  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(1.0);

  // Initialize to default configuration via helper.
  auto& root = simulator.get_mutable_context();
  auto& plant_ctx = plant.GetMyMutableContextFromRoot(&root);
  franka_fci_sim::SetDefaultFrankaInitialState(plant, plant_ctx);

  simulator.Initialize();

  embed->StartServer(1337, 1338);
  std::cout << "Franka FCI server started on 1337/1338.\n";
  std::cout << "Meshcat visualizer: http://localhost:7000\n";

  // Simple real-time loop; users typically run their own loop.
  for (;;) {
    simulator.AdvanceTo(simulator.get_context().get_time() + dt);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  // Never reached in this minimal example
  // embed->StopServer();
  // return 0;
}


