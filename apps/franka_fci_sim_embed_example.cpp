#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/systems/analysis/runge_kutta5_integrator.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>

#include "franka_drake/fci_sim_embed.h"

using drake::geometry::Role;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::Parser;
using drake::multibody::RevoluteJoint;
using drake::systems::DiagramBuilder;

static std::atomic<bool> g_running{true};

static void SignalHandler(int) {
  g_running.store(false);
}

// Helper: decide URDF based on a simple CONFIG string similar to run_server.sh
static std::string SelectUrdf(const std::string& config) {
  if (config == "gripper") {
    return "models/urdf/fer_drake_gripper_fixed.urdf";
  }
  return "models/urdf/fer_drake_fingerless.urdf"; // default fingerless
}

int main(int argc, char** argv) {
  // Visual, real-time defaults (no headless) with minimal CLI: [CONFIG]
  const std::string package_xml = "models/urdf/package.xml";
  std::string config = (argc >= 1 && argv[0]) ? "fingerless" : "fingerless";
  if (argc >= 2) config = argv[1];
  const bool headless = false;  // Visual example by default
  const bool turbo = false;     // Real-time by default
  const double dt = 0.001;

  const std::string urdf = SelectUrdf(config);

  std::signal(SIGINT, SignalHandler);

  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, dt);
  Parser parser(&plant, &scene_graph);
  parser.package_map().AddPackageXml(package_xml);
  const ModelInstanceIndex robot = parser.AddModels(urdf)[0];
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base", robot));

  // Add actuators (match main)
  struct J { const char* n; double eff; };
  const std::vector<J> act = {
      {"fer_joint1", 87}, {"fer_joint2", 87}, {"fer_joint3", 87},
      {"fer_joint4", 87}, {"fer_joint5", 12}, {"fer_joint6", 87}, {"fer_joint7", 87},
  };
  for (const auto& a : act) {
    const auto& joint = plant.GetJointByName<RevoluteJoint>(a.n, robot);
    plant.AddJointActuator(std::string(a.n) + "_act", joint, a.eff);
  }

  // Viscous damping (match main)
  const std::vector<std::pair<const char*, double>> damp = {
      {"fer_joint1", 1.0}, {"fer_joint2", 1.0}, {"fer_joint3", 1.0},
      {"fer_joint4", 1.0}, {"fer_joint5", 1.0}, {"fer_joint6", 1.0}, {"fer_joint7", 1.0},
  };
  for (const auto& [name, d] : damp) {
    plant.GetMutableJointByName<RevoluteJoint>(name, robot).set_default_damping(d);
  }

  // Disable proximity roles (like main)
  {
    const drake::geometry::SourceId sid = plant.get_source_id().value();
    for (drake::multibody::BodyIndex b(0); b < plant.num_bodies(); ++b) {
      const auto& body = plant.get_body(b);
      for (const drake::geometry::GeometryId gid : plant.GetCollisionGeometriesForBody(body)) {
        scene_graph.RemoveRole(sid, gid, Role::kProximity);
      }
    }
  }

  plant.Finalize();

  // Attach full FCI embedding
  franka_fci_sim::FciSimOptions opts;
  opts.headless = headless;
  opts.turbo = turbo;
  auto embed = franka_fci_sim::FciSimEmbedder::Attach(&plant, robot, &builder, opts);

  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(1.0);  // real-time
  auto& rk = simulator.reset_integrator<drake::systems::RungeKutta5Integrator<double>>();
  rk.set_target_accuracy(1e-12);
  rk.set_maximum_step_size(1e-4);

  // Initial joint configuration (match main)
  auto& root = simulator.get_mutable_context();
  auto& plant_ctx = plant.GetMyMutableContextFromRoot(&root);
  const int n = plant.num_positions(robot);
  const int nv = plant.num_velocities(robot);
  Eigen::VectorXd q_des = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd v_des = Eigen::VectorXd::Zero(nv);
  q_des[0] = 0.0;
  q_des[1] = -M_PI / 4.0;
  q_des[2] = 0.0;
  q_des[3] = -3.0 * M_PI / 4.0;
  q_des[4] = 0.0;
  q_des[5] = M_PI / 2.0;
  q_des[6] = M_PI / 4.0;
  if (n > 7) {
    for (int i = 7; i < n; ++i) q_des[i] = 0.04;
  }
  plant.SetPositions(&plant_ctx, robot, q_des);
  plant.SetVelocities(&plant_ctx, robot, v_des);

  simulator.Initialize();

  embed->StartServer(1337, 1338);
  std::cout << "Starting FCI embedded simulation server on port 1337...\n";
  if (headless) {
    std::cout << "Running in HEADLESS mode (no visualization)\n";
  }
  if (turbo) {
    std::cout << "TURBO mode enabled - running faster than real-time when possible\n";
  } else {
    std::cout << "Real-time mode enabled (RTF=1.0)\n";
  }
  std::cout << "Press Ctrl+C to stop.\n" << std::flush;

  // Real-time loop with light sleep
  const double sim_step = 0.001;
  while (g_running.load()) {
    simulator.AdvanceTo(simulator.get_context().get_time() + sim_step);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  embed->StopServer();
  return 0;
}


