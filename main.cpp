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

// Drake visualization
#include <drake/visualization/visualization_config_functions.h>

// Eigen helpers
#include <Eigen/Core>

// Our custom “mask out joint 7” system:
#include "zero_last_joint.h"

using namespace drake;
using Eigen::VectorXd;

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

  // Weld the “base” frame to the world.
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
  };
  for (const auto& a : act) {
    const auto& joint =
        plant.GetJointByName<multibody::RevoluteJoint>(a.n, robot);
    plant.AddJointActuator(a.n + "_act", joint, a.eff);
  }

  // -------------------------------------------------------------
  // 3) Set per-joint viscous damping to a nonzero value
  // -------------------------------------------------------------
  const std::vector<std::pair<std::string,double>> damp = {
      {"fer_joint1", 22.0},
      {"fer_joint2", 22.0},
      {"fer_joint3", 22.0},
      {"fer_joint4", 22.0},
      {"fer_joint5", 11.0},
      {"fer_joint6", 11.0},
  };
  for (const auto& [name, d] : damp) {
    plant.GetMutableJointByName<multibody::RevoluteJoint>(name, robot)
         .set_default_damping(d);
  }

  plant.Finalize();

  // -------------------------------------------------------------
  // 4) Build a “desired pose” VectorXd (q_des, v_des)
  // -------------------------------------------------------------
  const int n = plant.num_positions(robot);
  VectorXd q_des(n), v_des(plant.num_velocities(robot));
  q_des << 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2;
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
  // 6) Add our “ZeroLastJoint” mask block
  // -------------------------------------------------------------
  // This block will take the 7×1 gravity‐comp output, copy indices 0–5 as-is,
  // and force index 6 to zero.  In other words: τ_out(i)=τ_gc(i) for i=0..5, but
  // τ_out(6)=0.0 ⇒ effectively removing gravity compensation on joint 7.
  auto* zero_last = builder.AddSystem<drake::systems::ZeroLastJoint>();

  // Wire: g_comp (output) → zero_last (input)
  builder.Connect(g_comp->get_output_port(), zero_last->get_input_port(0));
  // -------------------------------------------------------------
  // 7) Add the PID controller
  // -------------------------------------------------------------
  // We need a constant desired‐state source to feed the PID.
  auto* desired_source = builder.AddSystem<systems::ConstantVectorSource<double>>(
      (VectorXd(q_des.size() + v_des.size()) << q_des, v_des).finished());

  VectorXd kp = VectorXd::Zero(n), kd = VectorXd::Zero(n), ki = VectorXd::Zero(n);

  kp << 0, 0,0,0,  0, 0;
  kd << 0,  0,  0,  0,   0,  0;
  ki << 0, 0, 0, 0, 0, 0;

  auto* pid = builder.AddSystem<systems::controllers::PidController>(kp, ki, kd);

  // Wire: plant.state_output → PID’s estimated_state input
  builder.Connect(plant.get_state_output_port(),
                  pid->get_input_port_estimated_state());

  // Wire: desired_source output → PID’s desired_state input
  builder.Connect(desired_source->get_output_port(),
                  pid->get_input_port_desired_state());

  // -------------------------------------------------------------
  // 8) Add the Adder (τ_total = τ_gc_masked + τ_pd)
  // -------------------------------------------------------------
  auto* adder = builder.AddSystem<systems::Adder<double>>(2, n);

  // Wire: zero_last output → adder input port 0
  builder.Connect(zero_last->get_output_port(0), adder->get_input_port(0));

  // Wire: pid output → adder input port 1
  builder.Connect(pid->get_output_port(), adder->get_input_port(1));

  // Finally: Wire adder’s sum into the plant’s actuation input port
  builder.Connect(adder->get_output_port(), plant.get_actuation_input_port());

  // -------------------------------------------------------------
  // 9) (Optional) Add Drake Visualizer & LogSinks
  // -------------------------------------------------------------
  visualization::AddDefaultVisualization(&builder);

  const int n_state = plant.num_positions() + plant.num_velocities();
  auto* logger =
      builder.AddSystem<systems::VectorLogSink<double>>(n_state);
  builder.Connect(plant.get_state_output_port(), logger->get_input_port());

  auto* torque_logger =
      builder.AddSystem<systems::VectorLogSink<double>>(plant.num_actuators(robot));
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
  plant.SetVelocities(&plant_ctx, robot, v_des);

  sim.Initialize();
  std::cout << "Running … press <Enter> to begin"; //std::cin.get();
  sim.AdvanceTo(30.0);

  // Dump a few logged samples
  const auto& L = logger->GetLog(logger->GetMyContextFromRoot(sim.get_context()));
  for (int i = 0; i < L.num_samples(); i += 200) {
    std::cout << L.sample_times()[i] << "  "
              << L.data().col(i).head(n).transpose() << "\n";
    for (int j = 0; j < 6; ++j) {
      std::cout << "  tau " << (j + 1) << " = "
                << torque_logger->GetLog(
                       torque_logger->GetMyContextFromRoot(sim.get_context()))
                       .data()(j, i)
                << "  [Nm]\n";
    }
  }

  return 0;
}
