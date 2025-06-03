// Standard C++
#include <chrono>
#include <iostream>
#include <thread>

// Drake core / geometry / multibody
#include <drake/common/find_resource.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/multibody/tree/prismatic_joint.h>

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

// Drake visualisation
#include <drake/visualization/visualization_config_functions.h>

// Eigen helpers (needed for Eigen::VectorXd literals)
#include <Eigen/Core>

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

  // ------------------------------------------------------------- plant
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, dt);
  multibody::Parser parser(&plant, &scene_graph);
  parser.package_map().AddPackageXml(package_xml);
  const auto robot = parser.AddModels(urdf)[0];
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("base", robot));

  // actuators (same table as before)
  struct J { std::string n; double eff; };
  const std::vector<J> act = {
      {"fer_joint1",87},{"fer_joint2",87},{"fer_joint3",87},
      {"fer_joint4",87},{"fer_joint5",12},{"fer_joint6",87},{"fer_joint7",87},
      };
  for (const auto& a: act) {
    if (a.n.find("finger")==std::string::npos) {
      const auto& j = plant.GetJointByName<multibody::RevoluteJoint>(a.n,robot);
      plant.AddJointActuator(a.n+"_act", j, a.eff);
    } else {
      const auto& j = plant.GetJointByName<multibody::PrismaticJoint>(a.n,robot);
      plant.AddJointActuator(a.n+"_act", j, a.eff);
    }
  }

  // ---------------- ***higher viscous damping***
  const std::vector<std::pair<std::string,double>> damp = {
      {"fer_joint1",22.0}, {"fer_joint2",22.0}, {"fer_joint3",22.0},
      {"fer_joint4",22.0}, {"fer_joint5",11.0}, {"fer_joint6",11.0},
      {"fer_joint7",11.0},
      };
  for (const auto& [name,d] : damp) {
    if (name.find("finger")==std::string::npos)
      plant.GetMutableJointByName<multibody::RevoluteJoint>(name,robot)
           .set_default_damping(0.0);
    else
      plant.GetMutableJointByName<multibody::PrismaticJoint>(name,robot)
           .set_default_damping(0.0);
  }

  plant.Finalize();

  // ------------------------------------------------------------- desired pose
  const int n = plant.num_positions(robot);
  VectorXd q_des(n);
  q_des << 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4;
  const VectorXd v_des = VectorXd::Zero(plant.num_velocities(robot));

  // temporary context → gravity torque
  auto ctx_tmp = plant.CreateDefaultContext();
  plant.SetPositions(ctx_tmp.get(), robot, q_des);
  plant.SetVelocities(ctx_tmp.get(), robot, v_des);

  //VectorXd tau_gc = -plant.CalcGravityGeneralizedForces(*ctx_tmp);   // NEGATIVE
  //std::cout << "Gravity-hold torque:  " << tau_gc.transpose() << '\n';

  // constant torque source wired straight to plant
  //auto* g_src =
  //    builder.AddSystem<systems::ConstantVectorSource<double>>(tau_gc);
  //builder.Connect(g_src->get_output_port(), plant.get_actuation_input_port());

  // ── Gravity-comp system that recomputes −g(q) every time step 
  // ---------------------------------------------------------------------------
// Gravity compensation (already in your code)
// ---------------------------------------------------------------------------
using systems::controllers::InverseDynamics;
auto* g_comp = builder.AddSystem<InverseDynamics<double>>(
    &plant,
    InverseDynamics<double>::InverseDynamicsMode::kGravityCompensation);
builder.Connect(plant.get_state_output_port(),
                g_comp->get_input_port_estimated_state());



// ---------------------------------------------------------------------------
// Desired state source  (positions ‖ velocities)
// ---------------------------------------------------------------------------
auto* desired_source =
    builder.AddSystem<systems::ConstantVectorSource<double>>(
        (VectorXd(q_des.size() + v_des.size()) << q_des, v_des).finished());

// ---------------------------------------------------------------------------
// Very-low-gain PD
// ---------------------------------------------------------------------------
VectorXd kp = VectorXd::Zero(n);
VectorXd kd = VectorXd::Zero(n);
VectorXd ki = VectorXd::Zero(n);

// *** Use the template *PidController* (no `<double>` here) ***
auto* pid =
    builder.AddSystem<systems::controllers::PidController>(kp, ki, kd);

builder.Connect(plant.get_state_output_port(),
                pid->get_input_port_estimated_state());
builder.Connect(desired_source->get_output_port(),
                pid->get_input_port_desired_state());

// ---------------------------------------------------------------------------
// Sum  τ_total = τ_gc  +  τ_pd
// ---------------------------------------------------------------------------
auto* adder = builder.AddSystem<systems::Adder<double>>(2, n);
builder.Connect(g_comp->get_output_port(), adder->get_input_port(0));
builder.Connect(pid->get_output_port(),    adder->get_input_port(1));
builder.Connect(adder->get_output_port(),  plant.get_actuation_input_port());


  // ------------------------------------------------------------- viz + log
  visualization::AddDefaultVisualization(&builder);

  const int n_state = plant.num_positions()+plant.num_velocities();
  auto* logger =
      builder.AddSystem<systems::VectorLogSink<double>>(n_state);
  builder.Connect(plant.get_state_output_port(), logger->get_input_port());

  auto* torque_logger =
      builder.AddSystem<systems::VectorLogSink<double>>(plant.num_actuators(robot));
  builder.Connect(adder->get_output_port(), torque_logger->get_input_port());

  // ------------------------------------------------------------- build + sim
  auto diagram = builder.Build();
  systems::Simulator<double> sim(*diagram);
  sim.set_target_realtime_rate(1.0);
  auto& rk = sim.reset_integrator<systems::RungeKutta5Integrator<double>>();
  rk.set_target_accuracy(1e-12);
  rk.set_maximum_step_size(1e-4);          //

  auto& root = sim.get_mutable_context();
  auto& pctx = plant.GetMyMutableContextFromRoot(&root);
  plant.SetPositions(&pctx, robot, q_des);
  plant.SetVelocities(&pctx, robot, v_des);

  sim.Initialize();
  std::cout << "Running … press <Enter> to begin"; std::cin.get();
  sim.AdvanceTo(20.0);

  // ------------------------------------------------------------- simple dump
  const auto& L =
      logger->GetLog(logger->GetMyContextFromRoot(sim.get_context()));
  for (int i = 0; i < L.num_samples(); i += 200) {
    std::cout << L.sample_times()[i] << "  "
              << L.data().col(i).head(n).transpose() << '\n';
    for (int j = 0; j != 7; ++j) {
        std::cout << "tau " << j+1 << " = "
              << torque_logger->GetLog(torque_logger->GetMyContextFromRoot(sim.get_context()))
                     .data()(j, i) << "  [Nm]\n";

    }

  }
  return 0;
}
