// Pick and place scene example: load a static pick and place scene SDF into SceneGraph,
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
#include <iomanip>

#include "franka_drake/fci_sim_embed.h"

using drake::multibody::AddMultibodyPlantSceneGraph;

// No HTML/server handling here; monitoring is handled by the embedded
// LeafSystem reporter added by FciSimEmbedder.

int main() {
  const double dt = 0.001; // 1 kHz
  drake::systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, dt);

  // Load pick and place scene (as a model to SceneGraph through the plant's parser).
  // We parse it using the Multibody Parser so visual geometries register in SceneGraph.
  {
    drake::multibody::Parser parser(&plant, &scene_graph);
    const std::string sdf_path = franka_fci_sim::ResolveModelPath("models/pick_and_place_scene.sdf");
    try {
      parser.AddModels(sdf_path);
    } catch (const std::exception& e) {
      std::cerr << "Failed to load pick and place scene SDF: " << sdf_path << "\n" << e.what() << std::endl;
    }
  }

  // Auto-attach the Franka robot (fingerless by default), disable collisions.
  franka_fci_sim::FciSimEmbedder::AutoAttachOptions auto_opts;
  auto_opts.prefer_gripper = true;
  auto_opts.disable_collisions = true;
  franka_fci_sim::FciSimOptions opts;
  opts.headless = false;
  opts.turbo = true;  // Enable turbo mode for faster-than-real-time simulation
  auto embed = franka_fci_sim::FciSimEmbedder::AutoAttach(&plant, &scene_graph, &builder, auto_opts, opts);

  // Register the gripper and set an initial opening width
  franka_fci_sim::FciSimEmbedder::GripperSpec gripper_spec;
  gripper_spec.left_joint_name = "fer_finger_joint1";
  gripper_spec.right_joint_name = "fer_finger_joint2";
  gripper_spec.min_width_m = 0.0;
  gripper_spec.max_width_m = 0.08;
  gripper_spec.kp = 200.0;
  gripper_spec.kd = 5.0;
  gripper_spec.left_sign = +1.0;
  gripper_spec.right_sign = -1.0;  // Right finger prismatic axis is -Y
  embed->RegisterGripper(gripper_spec);
  embed->SetGripperWidth(0.005);

  // Meshcat visualization
  drake::geometry::MeshcatParams params;
  params.host = "0.0.0.0";      // listen on all interfaces
  params.port = std::nullopt;   // keep previous behavior; set a port if you want a fixed one
  auto meshcat = std::make_shared<drake::geometry::Meshcat>(params);
  drake::visualization::AddDefaultVisualization(&builder, meshcat);

  // Build and simulate
  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(opts.turbo ? 0.0 : 1.0);

  auto& root = simulator.get_mutable_context();
  auto& plant_ctx = plant.GetMyMutableContextFromRoot(&root);
  franka_fci_sim::SetDefaultFrankaInitialState(plant, plant_ctx);

  simulator.Initialize();
  
  // Configure ports based on environment or use defaults
  const char* tcp_port_env = std::getenv("FRANKA_TCP_PORT");
  const char* udp_port_env = std::getenv("FRANKA_UDP_PORT");
  const char* meshcat_port_env = std::getenv("MESHCAT_PORT");
  
  uint16_t tcp_port = tcp_port_env ? std::stoi(tcp_port_env) : 1337;
  uint16_t udp_port = udp_port_env ? std::stoi(udp_port_env) : 1340;
  uint16_t meshcat_port = meshcat_port_env ? std::stoi(meshcat_port_env) : 7000;
  
  // Update Meshcat port if environment variable is set
  if (meshcat_port_env) {
    params.port = meshcat_port;
    meshcat = std::make_shared<drake::geometry::Meshcat>(params);
    drake::visualization::AddDefaultVisualization(&builder, meshcat);
  }
  
  // Use configured ports for FCI server
  embed->StartServer(tcp_port, udp_port);
  
  std::cout << "Franka FCI server running on TCP " << tcp_port << ", UDP " << udp_port << std::endl;
  std::cout << "Meshcat available at http://localhost:" << meshcat_port << std::endl;

  for (;;) {
    simulator.AdvanceTo(simulator.get_context().get_time() + dt);
    if (!opts.turbo) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  }

  return 0;
}


