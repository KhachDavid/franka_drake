// Minimal example: demonstrate hooking the FCI embedder into an existing
// Drake builder/plant. Keep everything else (controllers, Meshcat) up to the user.

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/analysis/runge_kutta5_integrator.h>
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
  auto embed = franka_fci_sim::FciSimEmbedder::AutoAttach(&plant, &builder, auto_opts, {});

  // Strip proximity roles (disable collisions) to mirror server main behavior.
  {
    const drake::geometry::SourceId sid = plant.get_source_id().value();
    for (drake::multibody::BodyIndex b(0); b < plant.num_bodies(); ++b) {
      const auto& body = plant.get_body(b);
      for (const drake::geometry::GeometryId gid : plant.GetCollisionGeometriesForBody(body)) {
        scene_graph.RemoveRole(sid, gid, drake::geometry::Role::kProximity);
      }
    }
  }

  // Add a Meshcat visualizer (users can reuse their own Meshcat instance instead).
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();
  drake::visualization::AddDefaultVisualization(&builder, meshcat);

  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(1.0);
  auto& rk = simulator.reset_integrator<drake::systems::RungeKutta5Integrator<double>>();
  rk.set_target_accuracy(1e-12);
  rk.set_maximum_step_size(1e-4);

  // Initialize to the same default configuration as server main.
  auto& root = simulator.get_mutable_context();
  auto& plant_ctx = plant.GetMyMutableContextFromRoot(&root);
  const int n = plant.num_positions();
  const int nv = plant.num_velocities();
  Eigen::VectorXd q_des = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd v_des = Eigen::VectorXd::Zero(nv);
  if (n >= 7) {
    q_des[0] = 0.0;
    q_des[1] = -M_PI / 4.0;
    q_des[2] = 0.0;
    q_des[3] = -3.0 * M_PI / 4.0;
    q_des[4] = 0.0;
    q_des[5] = M_PI / 2.0;
    q_des[6] = M_PI / 4.0;
  }
  if (n > 7) {
    for (int i = 7; i < n; ++i) q_des[i] = 0.04; // open gripper if present
  }
  plant.SetPositions(&plant_ctx, q_des);
  plant.SetVelocities(&plant_ctx, v_des);

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


