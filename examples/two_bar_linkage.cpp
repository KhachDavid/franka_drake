#include <iostream>
#include <drake/common/find_resource.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/controllers/inverse_dynamics_controller.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/visualization/visualization_config_functions.h>

using namespace drake;

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <path_to_urdf>" << std::endl;
    return 1;
  }

  const std::string urdf_path = argv[1];
  const double time_step = 0.001;

  systems::DiagramBuilder<double> builder;

  // Plant + SceneGraph
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, time_step);
  multibody::Parser parser(&plant, &scene_graph);
  const auto model_instance = parser.AddModels(urdf_path)[0];

  // Add actuator for gravity compensation
  const auto& joint = plant.GetJointByName<multibody::RevoluteJoint>("joint1", model_instance);
  plant.AddJointActuator("joint1_actuator", joint);

  plant.Finalize();

  // Gravity compensation
  auto* gcomp = builder.AddSystem<systems::controllers::InverseDynamics<double>>(
      &plant,
      systems::controllers::InverseDynamics<double>::InverseDynamicsMode::kGravityCompensation);
  builder.Connect(plant.get_state_output_port(),
                  gcomp->get_input_port_estimated_state());
  builder.Connect(gcomp->get_output_port(), plant.get_actuation_input_port());

  // Visualization
  visualization::AddDefaultVisualization(&builder);

  // Log joint positions
  auto* pos_logger = builder.AddSystem<systems::VectorLogSink<double>>(
      plant.get_state_output_port(model_instance).size());
  builder.Connect(plant.get_state_output_port(model_instance),
                  pos_logger->get_input_port());

  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(1.0);

  // Set initial joint angle
  auto& root_context = simulator.get_mutable_context();
  auto& plant_context = plant.GetMyMutableContextFromRoot(&root_context);

  Eigen::VectorXd q0(1);
  q0 << M_PI / 4;  // 45 degrees
  plant.SetPositions(&plant_context, model_instance, q0);
  plant.SetVelocities(&plant_context, model_instance, Eigen::VectorXd::Zero(1));

  simulator.Initialize();

  std::cout << "Running gravity compensation on 2-bar linkage..." << std::endl;
  simulator.AdvanceTo(10.0);

  // Output logged joint angles
  const auto& log = pos_logger->GetLog(pos_logger->GetMyContextFromRoot(root_context));
  std::cout << "Logged " << log.num_samples() << " samples of joint angles:\n";
  for (int i = 0; i < log.num_samples(); i += 100) {
    std::cout << "t = " << log.sample_times()[i]
              << " | angle = " << log.data()(0, i) << " rad\n";
  }

  // Also print final torque
  const auto& final_plant_context = plant.GetMyContextFromRoot(simulator.get_context());
  const auto& tau = plant.get_actuation_input_port().Eval(final_plant_context);
  std::cout << "Final gravity torque: " << tau.transpose() << std::endl;

  return 0;
}
