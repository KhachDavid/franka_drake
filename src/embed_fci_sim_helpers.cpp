#include "franka_drake/fci_sim_embed.h"

#include <thread>
#include <vector>

#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/systems/analysis/runge_kutta5_integrator.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/visualization/visualization_config_functions.h>
#include <drake/geometry/meshcat.h>

namespace franka_fci_sim {

namespace {
std::string SelectUrdf(RobotConfig cfg) {
  if (cfg == RobotConfig::Gripper) return "models/urdf/fer_drake.urdf";
  return "models/urdf/fer_drake_fingerless.urdf";
}
}

EmbeddedApp BuildEmbeddedApp(const BuildArgs& args) {
  using drake::systems::DiagramBuilder;
  using drake::multibody::AddMultibodyPlantSceneGraph;
  using drake::multibody::Parser;
  using drake::multibody::RevoluteJoint;

  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, args.time_step);
  Parser parser(&plant, &scene_graph);
  const std::string pkg_xml = ResolveModelPath(args.package_xml_path);
  const std::string urdf_path = ResolveModelPath(SelectUrdf(args.config));
  parser.package_map().AddPackageXml(pkg_xml);
  const auto robot = parser.AddModels(urdf_path)[0];
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base", robot));

  struct J { const char* n; double eff; };
  const std::vector<J> act = {{"fer_joint1",87},{"fer_joint2",87},{"fer_joint3",87},
                              {"fer_joint4",87},{"fer_joint5",12},{"fer_joint6",87},{"fer_joint7",87}};
  for (const auto& a : act) {
    const auto& joint = plant.GetJointByName<RevoluteJoint>(a.n, robot);
    plant.AddJointActuator(std::string(a.n) + "_act", joint, a.eff);
  }

  const std::vector<std::pair<const char*, double>> damp = {{"fer_joint1",1.0},{"fer_joint2",1.0},{"fer_joint3",1.0},
                                                            {"fer_joint4",1.0},{"fer_joint5",1.0},{"fer_joint6",1.0},{"fer_joint7",1.0}};
  for (const auto& [name, d] : damp) {
    plant.GetMutableJointByName<RevoluteJoint>(name, robot).set_default_damping(d);
  }

  // Let the embedder handle disabling collisions using SceneGraph.

  plant.Finalize();

  // Optional visualization (Meshcat) in Visual mode
  std::shared_ptr<drake::geometry::Meshcat> meshcat;
  if (args.mode == RunMode::Visual) {
    drake::geometry::MeshcatParams params;
    params.host = "0.0.0.0";      // listen on all interfaces
    params.port = std::nullopt;   // keep previous behavior; set a port if you want a fixed one
    meshcat = std::make_shared<drake::geometry::Meshcat>(params);
    drake::visualization::AddDefaultVisualization(&builder, meshcat);
  }

  FciSimOptions opts;
  opts.headless = (args.mode != RunMode::Visual);
  opts.turbo = (args.mode == RunMode::Turbo);
  // Disable collisions via Attach overload with SceneGraph (default true)
  auto embed = FciSimEmbedder::Attach(&plant, robot, &scene_graph, &builder, opts, true);

  auto diagram = builder.Build();
  auto simulator = std::make_unique<drake::systems::Simulator<double>>(*diagram);
  simulator->set_target_realtime_rate(opts.turbo ? 0.0 : 1.0);
  auto& rk = simulator->reset_integrator<drake::systems::RungeKutta5Integrator<double>>();
  rk.set_target_accuracy(1e-12);
  rk.set_maximum_step_size(1e-4);

  auto& root = simulator->get_mutable_context();
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
  if (n > 7) for (int i = 7; i < n; ++i) q_des[i] = 0.04;
  plant.SetPositions(&plant_ctx, robot, q_des);
  plant.SetVelocities(&plant_ctx, robot, v_des);

  simulator->Initialize();
  embed->StartServer(1337, 1338);

  EmbeddedApp app;
  app.diagram = std::move(diagram);
  app.simulator = std::move(simulator);
  app.embedder = std::move(embed);
  return app;
}

void RunRealtimeLoop(drake::systems::Simulator<double>& simulator, bool turbo) {
  const double sim_step = 0.001;
  while (true) {
    simulator.AdvanceTo(simulator.get_context().get_time() + sim_step);
    if (!turbo) std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}

}  // namespace franka_fci_sim


