// Warehouse scene example: load a static warehouse SDF into SceneGraph,
// embed a Franka robot using FciSimEmbedder, add Meshcat viz, and run.

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/geometry/scene_graph.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/visualization/visualization_config_functions.h>
#include <drake/geometry/meshcat.h>
#include <iostream>
#include <thread>
#include <chrono>

#include "franka_drake/fci_sim_embed.h"

using drake::multibody::AddMultibodyPlantSceneGraph;

int main() {
  const double dt = 0.001; // 1 kHz
  drake::systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, dt);

  // Load warehouse scene (as a model to SceneGraph through the plant's parser).
  // We parse it using the Multibody Parser so visual geometries register in SceneGraph.
  {
    drake::multibody::Parser parser(&plant, &scene_graph);
    const std::string sdf_path = franka_fci_sim::ResolveModelPath("models/warehouse_scene.sdf");
    try {
      parser.AddModels(sdf_path);
    } catch (const std::exception& e) {
      std::cerr << "Failed to load warehouse SDF: " << sdf_path << "\n" << e.what() << std::endl;
    }
  }

  // Auto-attach the Franka robot (fingerless by default), disable collisions.
  franka_fci_sim::FciSimEmbedder::AutoAttachOptions auto_opts;
  auto_opts.prefer_gripper = true;
  auto_opts.disable_collisions = true;
  franka_fci_sim::FciSimOptions opts;
  opts.headless = false;
  opts.turbo = false;
  auto embed = franka_fci_sim::FciSimEmbedder::AutoAttach(&plant, &scene_graph, &builder, auto_opts, opts);

  // Meshcat visualization
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();
  drake::visualization::AddDefaultVisualization(&builder, meshcat);

  // Build and simulate
  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(opts.turbo ? 0.0 : 1.0);

  auto& root = simulator.get_mutable_context();
  auto& plant_ctx = plant.GetMyMutableContextFromRoot(&root);
  franka_fci_sim::SetDefaultFrankaInitialState(plant, plant_ctx);

  simulator.Initialize();
  embed->StartServer(1337, 1338);

  std::cout << "Warehouse example running. Meshcat: http://localhost:7000\n";
  std::cout << "Franka FCI server at 127.0.0.1:1337/1338\n";

  for (;;) {
    simulator.AdvanceTo(simulator.get_context().get_time() + dt);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  return 0;
}


