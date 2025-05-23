#include <chrono>
#include <iostream>
#include <thread>

#include <drake/common/find_resource.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/multibody/tree/prismatic_joint.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/controllers/inverse_dynamics_controller.h>
#include <drake/systems/controllers/pid_controller.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/adder.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/visualization/visualization_config_functions.h>
#include <drake/systems/analysis/runge_kutta5_integrator.h> 
#include <drake/systems/analysis/simulator_config.h>          // ResetIntegratorFromFlags

using namespace drake;
using Eigen::VectorXd;

int main(int argc, char* argv[]) {
  if (argc < 4) {
    std::cerr << "Usage: " << argv[0]
              << " <path_to_package.xml> <path_to_urdf> <time_step>" << std::endl;
    return 1;
  }

  const std::string package_xml_path{argv[1]};
  const std::string urdf_path{argv[2]};
  const double time_step = std::stod(argv[3]);

  systems::DiagramBuilder<double> builder;

  //---------------------------------------------------------------------------
  // Plant + SceneGraph
  //---------------------------------------------------------------------------
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, time_step);

  multibody::Parser parser(&plant, &scene_graph);
  parser.package_map().AddPackageXml(package_xml_path);
  const auto robot = parser.AddModels(urdf_path)[0];

  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("base", robot));

  //---------------------------------------------------------------------------
  // Actuators
  //---------------------------------------------------------------------------
  struct JointSpec { std::string name; double effort; };
  const std::vector<JointSpec> kActuators = {
      {"fer_joint1", 87.0}, {"fer_joint2", 87.0}, {"fer_joint3", 87.0},
      {"fer_joint4", 87.0}, {"fer_joint5", 12.0}, {"fer_joint6", 12.0},
      {"fer_joint7", 12.0},
      {"fer_finger_joint1", 100.0}, {"fer_finger_joint2", 100.0}};

  for (const auto& spec : kActuators) {
    if (spec.name.find("finger") == std::string::npos) {
      const auto& j =
          plant.GetJointByName<multibody::RevoluteJoint>(spec.name, robot);
      plant.AddJointActuator(spec.name + "_act", j, spec.effort);
    } else {
      const auto& j =
          plant.GetJointByName<multibody::PrismaticJoint>(spec.name, robot);
      plant.AddJointActuator(spec.name + "_act", j, spec.effort);
    }
  }

  //---------------------------------------------------------------------------
  // Light viscous joint damping (helps discrete stepping)
  //---------------------------------------------------------------------------
  const std::vector<std::pair<std::string, double>> kDamp = {
      {"fer_joint1", 0.05}, {"fer_joint2", 0.05}, {"fer_joint3", 0.05},
      {"fer_joint4", 0.05}, {"fer_joint5", 0.35}, {"fer_joint6", 0.35},
      {"fer_joint7", 0.35},
      {"fer_finger_joint1", 1.0}, {"fer_finger_joint2", 1.0}};

  for (const auto& [name, d] : kDamp) {
    if (name.find("finger") == std::string::npos) {
      auto& j =
          plant.GetMutableJointByName<multibody::RevoluteJoint>(name, robot);
      j.set_default_damping(d);
    } else {
      auto& j =
          plant.GetMutableJointByName<multibody::PrismaticJoint>(name, robot);
      j.set_default_damping(d);
    }
  }

  plant.Finalize();

  //---------------------------------------------------------------------------
  // Desired posture (initial and reference)
  //---------------------------------------------------------------------------
  const int n = plant.num_positions(robot);               // 9
  VectorXd q_des(n);
  q_des << 0.0, -M_PI / 4, 0.0, -3 * M_PI / 4, 0.0, M_PI / 2, M_PI / 4, 0.0,
      0.0;
  const VectorXd v_des = VectorXd::Zero(plant.num_velocities(robot));

  auto* desired_source = builder.AddSystem<systems::ConstantVectorSource<double>>(
      (VectorXd(q_des.size() + v_des.size()) << q_des, v_des).finished());

  //---------------------------------------------------------------------------
  // Gravity compensation
  //---------------------------------------------------------------------------
  using systems::controllers::InverseDynamics;
  auto* g_comp = builder.AddSystem<InverseDynamics<double>>(
      &plant, InverseDynamics<double>::InverseDynamicsMode::kGravityCompensation);
  builder.Connect(plant.get_state_output_port(),
                  g_comp->get_input_port_estimated_state());

  //---------------------------------------------------------------------------
  // PD controller
  //---------------------------------------------------------------------------
  VectorXd kp = VectorXd::Constant(n, 16000.0);
  VectorXd kd = VectorXd::Zero(n);
  VectorXd ki = VectorXd::Zero(n);

  auto* pid =
      builder.AddSystem<systems::controllers::PidController<double>>(kp, ki, kd);
  builder.Connect(plant.get_state_output_port(),
                  pid->get_input_port_estimated_state());
  builder.Connect(desired_source->get_output_port(),
                  pid->get_input_port_desired_state());

  //---------------------------------------------------------------------------
  // Sum τ = τ_gc + τ_pd
  //---------------------------------------------------------------------------
  auto* sum = builder.AddSystem<systems::Adder<double>>(2, n);
  builder.Connect(g_comp->get_output_port(), sum->get_input_port(0));
  builder.Connect(pid->get_output_port(),    sum->get_input_port(1));
  builder.Connect(sum->get_output_port(),    plant.get_actuation_input_port());

  //---------------------------------------------------------------------------
  // Visualisation + logging
  //---------------------------------------------------------------------------
  visualization::AddDefaultVisualization(&builder);
  auto* sink = builder.AddSystem<systems::VectorLogSink<double>>(
      plant.get_state_output_port().size());
  builder.Connect(plant.get_state_output_port(), sink->get_input_port());

  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(1.0);
  auto& rk5 = simulator.reset_integrator<
    drake::systems::RungeKutta5Integrator<double>>();
  rk5.set_target_accuracy(1e-10);
  rk5.set_maximum_step_size(0.001);

  // Initial state
  auto& root_context  = simulator.get_mutable_context();
  auto& plant_context = plant.GetMyMutableContextFromRoot(&root_context);
  plant.SetPositions(&plant_context, robot, q_des);
  plant.SetVelocities(&plant_context, robot, v_des);

  simulator.Initialize();
  std::cout << "Press enter to start simulation..." << std::endl;
  std::cin.get();

  simulator.AdvanceTo(20.0);

  //---------------------------------------------------------------------------
  // Report
  //---------------------------------------------------------------------------
  const auto& plant_ctx = plant.GetMyContextFromRoot(root_context);
  const auto& tau       = plant.get_actuation_input_port().Eval(plant_ctx);

  std::cout << "Final tau vector: " << tau.transpose() << "\n";
  std::cout << "Final q error: "
            << (plant.GetPositions(plant_context, robot) - q_des).transpose()
            << "\n";

  const auto& log  = sink->GetLog(sink->GetMyContextFromRoot(root_context));
  std::cout << "Logged " << log.num_samples() << " samples of dimension "
            << log.data().rows() << ".\n";

  // Print every 100-th sample (avoid spamming)
  for (int i = 0; i < log.num_samples(); i += 100) {
    std::cout << log.data().col(i).transpose() << "\n";
  }
  return 0;
}
