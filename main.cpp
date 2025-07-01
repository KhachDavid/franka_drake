#include <chrono>
#include <iostream>
#include <thread>

// Drake core / geometry / multibody
#include <drake/common/find_resource.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/revolute_joint.h>

// Drake systems
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/analysis/runge_kutta5_integrator.h>
#include <drake/systems/analysis/simulator_config.h>
#include <drake/systems/controllers/inverse_dynamics_controller.h>
#include <drake/systems/controllers/pid_controller.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/adder.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/systems/framework/leaf_system.h>

// Drake visualization
#include <drake/visualization/visualization_config_functions.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/rgba.h>
#include <drake/geometry/shape_specification.h>

// Eigen helpers
#include <Eigen/Core>

using namespace drake;
using Eigen::VectorXd;

// Leaf system that outputs a torque vector whose last entry is set by a Meshcat slider.
class Joint7TorqueSource final : public systems::LeafSystem<double> {
 public:
  Joint7TorqueSource(int num_act, std::shared_ptr<geometry::Meshcat> meshcat)
      : num_act_(num_act), meshcat_(std::move(meshcat)) {
    this->DeclareVectorOutputPort("tau7", num_act_,
                                  &Joint7TorqueSource::CalcOutput);
    if (num_act_ >= 7) {
      meshcat_->AddSlider("tau7_cmd[Nm]", -1.0, 1.0, 0.01, 0.0);
    }
  }

 private:
  void CalcOutput(const systems::Context<double>&,
                  systems::BasicVector<double>* output) const {
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(num_act_);
    if (num_act_ >= 7) {
      tau(num_act_ - 1) = meshcat_->GetSliderValue("tau7_cmd[Nm]");
    }
    output->SetFromVector(tau);
  }
  const int num_act_;
  std::shared_ptr<geometry::Meshcat> meshcat_;
};

int main(int argc, char* argv[]) {
  if (argc < 4) {
    std::cerr << "Usage: " << argv[0]
              << " <package.xml> <arm.urdf> <time_step>\n";
    return 1;
  }
  const std::string  package_xml = argv[1];
  const std::string  urdf        = argv[2];
  const double       dt          = std::stod(argv[3]);

  systems::DiagramBuilder<double> builder;

  // -------------------------------------------------------------
  // 1) Construct the MultibodyPlant + SceneGraph
  // -------------------------------------------------------------
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, dt);

  multibody::Parser parser(&plant, &scene_graph);
  parser.package_map().AddPackageXml(package_xml);
  const auto robot = parser.AddModels(urdf)[0];

  // Weld the "base" frame to the world.
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("base", robot));

  // -------------------------------------------------------------
  // 2) Add joint actuators
  // -------------------------------------------------------------
  struct J { std::string n; double eff; };
  const std::vector<J> act = {
      {"fer_joint1", 87},
      {"fer_joint2", 87},
      {"fer_joint3", 87},
      {"fer_joint4", 87},
      {"fer_joint5", 12},
      {"fer_joint6", 87},
      {"fer_joint7", 12},
  };
  for (const auto& a : act) {
    const auto& joint =
        plant.GetJointByName<multibody::RevoluteJoint>(a.n, robot);
    plant.AddJointActuator(a.n + "_act", joint, a.eff);
  }

  // -------------------------------------------------------------
  // 3) Set per-joint viscous damping to zero (pure gravity compensation)
  // -------------------------------------------------------------
  const std::vector<std::pair<std::string,double>> damp = {
      {"fer_joint1", 0.0},
      {"fer_joint2", 0.0},
      {"fer_joint3", 0.0},
      {"fer_joint4", 0.0},
      {"fer_joint5", 0.0},
      {"fer_joint6", 0.0},
      {"fer_joint7", 0.0},
  };
  for (const auto& [name, d] : damp) {
    plant.GetMutableJointByName<multibody::RevoluteJoint>(name, robot)
         .set_default_damping(d);
  }

  plant.Finalize();

  // -------------------------------------------------------------
  // 4) Build a "desired pose" VectorXd (q_des, v_des)
  // -------------------------------------------------------------
  const int n = plant.num_positions(robot);
  VectorXd q_des(n), v_des(plant.num_velocities(robot));
  if (n >= 7) {
    q_des << 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, 0.0;
  } else {
    q_des << 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2;
  }
  v_des.setZero();  // all velocities = 0

  // -------------------------------------------------------------
  // 5) Add the InverseDynamics (GravityComp) block
  // -------------------------------------------------------------
  using systems::controllers::InverseDynamics;
  auto* g_comp = builder.AddSystem<InverseDynamics<double>>(
      &plant,
      InverseDynamics<double>::InverseDynamicsMode::kGravityCompensation);

  builder.Connect(plant.get_state_output_port(),
                g_comp->get_input_port_estimated_state());

  // -------------------------------------------------------------
  // Prepare Meshcat (visualization + controls) *before* wiring torque.
  // -------------------------------------------------------------
  auto meshcat = std::make_shared<geometry::Meshcat>();

  // 6) Feed gravity-comp torques plus user torque into the plant
  // -------------------------------------------------------------
  const int num_act = plant.num_actuators(robot);
  auto* tau7_src = builder.AddSystem<Joint7TorqueSource>(num_act, meshcat);
  auto* adder = builder.AddSystem<systems::Adder<double>>(2, num_act);

  builder.Connect(g_comp->get_output_port(), adder->get_input_port(0));
  builder.Connect(tau7_src->get_output_port(), adder->get_input_port(1));
  builder.Connect(adder->get_output_port(), plant.get_actuation_input_port());

  // -------------------------------------------------------------
  // 9) (Optional) Add Drake Visualizer & LogSinks
  // -------------------------------------------------------------
  visualization::AddDefaultVisualization(&builder, meshcat);

  // ------------------------------------------------------------------
  // Add RGB triads to visualize the axes of link6 and link7 frames.
  // ------------------------------------------------------------------
  const double axis_L = 0.08;  // length [m]
  const double r_ax = 0.002;  // radius [m]

  auto add_triad = [&](const multibody::Frame<double>& F,
                       const std::string& path) {
    using geometry::Cylinder;
    using geometry::Rgba;
    using math::RigidTransformd;
    using Eigen::Vector3d;

    // X-axis (red)
    meshcat->SetObject(path + "/x", Cylinder(r_ax, axis_L), Rgba(1, 0, 0, 1));
    meshcat->SetTransform(path + "/x",
        RigidTransformd(Eigen::AngleAxisd(M_PI / 2, Vector3d::UnitY()),
                        Vector3d(axis_L / 2, 0, 0)));

    // Y-axis (green)
    meshcat->SetObject(path + "/y", Cylinder(r_ax, axis_L), Rgba(0, 1, 0, 1));
    meshcat->SetTransform(path + "/y",
        RigidTransformd(Eigen::AngleAxisd(-M_PI / 2, Vector3d::UnitX()),
                        Vector3d(0, axis_L / 2, 0)));

    // Z-axis (blue)
    meshcat->SetObject(path + "/z", Cylinder(r_ax, axis_L), Rgba(0, 0, 1, 1));
    meshcat->SetTransform(path + "/z",
        RigidTransformd(Eigen::Quaterniond::Identity(), Vector3d(0, 0, axis_L / 2)));
  };

  add_triad(plant.GetFrameByName("fer_link6", robot), "link6_triad");
  add_triad(plant.GetFrameByName("fer_link7", robot), "link7_triad");

  const int n_state = plant.num_positions() + plant.num_velocities();
  auto* logger =
      builder.AddSystem<systems::VectorLogSink<double>>(n_state);
  builder.Connect(plant.get_state_output_port(), logger->get_input_port());

  // Log the *actual* commanded torques that are sent to the plant, i.e.
  // gravity-compensation plus any user-slider input.
  auto* torque_logger =
      builder.AddSystem<systems::VectorLogSink<double>>(num_act);
  builder.Connect(adder->get_output_port(), torque_logger->get_input_port());

  // -------------------------------------------------------------
  // 10) Build the diagram + simulate
  // -------------------------------------------------------------
  auto diagram = builder.Build();
  systems::Simulator<double> sim(*diagram);
  //sim.set_target_realtime_rate(1.0);

  auto& rk = sim.reset_integrator<systems::RungeKutta5Integrator<double>>();
  rk.set_target_accuracy(1e-12);
  rk.set_maximum_step_size(1e-4);

  // Set the initial joint state to exactly (q_des, v_des)
  auto& root = sim.get_mutable_context();
  auto& plant_ctx = plant.GetMyMutableContextFromRoot(&root);

  plant.SetPositions(&plant_ctx, robot, q_des);

  plant.SetPositions(&plant_ctx, robot, q_des);
  plant.SetVelocities(&plant_ctx, robot, v_des);

  sim.Initialize();
  std::cout << "Running … press <Enter> to begin" << std::flush;
  std::cin.get();
  sim.AdvanceTo(30.0);

  // Dump a few logged samples
  const auto& log = logger->GetLog(logger->GetMyContextFromRoot(sim.get_context()));
  for (int i = 0; i < log.num_samples(); i += 200) {
    std::cout << log.sample_times()[i] << "  "
              << log.data().col(i).head(n).transpose() << "\n";
    for (int j = 0; j < num_act; ++j) {
      std::cout << "  tau " << (j + 1) << " = "
                << torque_logger->GetLog(
                       torque_logger->GetMyContextFromRoot(sim.get_context()))
                       .data()(j, i)
                << "  [Nm]\n";
    }
  }

  return 0;
}
