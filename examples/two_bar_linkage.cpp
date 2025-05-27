#include <iostream>
#include <drake/common/find_resource.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/visualization/visualization_config_functions.h>

using namespace drake;

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <path_to_urdf>\n";
    return 1;
  }
  const std::string urdf = argv[1];
  const double dt = 0.001;

  systems::DiagramBuilder<double> builder;

  // --------------------------  Plant + SceneGraph
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, dt);
  multibody::Parser(&plant, &scene_graph).AddModels(urdf);
  plant.AddJointActuator("joint1_act",
      plant.GetJointByName<multibody::RevoluteJoint>("joint1"));
  plant.Finalize();

  // --------------------------  Desired pose
  const double desired_angle = M_PI / 4;  // 45 °
  auto ctx_tmp = plant.CreateDefaultContext();
  plant.SetPositions(ctx_tmp.get(), Eigen::VectorXd::Constant(1, desired_angle));
  plant.SetVelocities(ctx_tmp.get(), Eigen::VectorXd::Zero(1));

  // gravity-hold torque (note the minus sign!)
  Eigen::VectorXd tau_hold = -plant.CalcGravityGeneralizedForces(*ctx_tmp);
  std::cout << "Gravity-hold torque  =  " << tau_hold.transpose() << "  [Nm]\n";

  // change tau to some other value if desired
  tau_hold(0) += 0.001;  // flip sign for testing

  // --------------------------  constant torque source
  auto* torque_src =
      builder.AddSystem<systems::ConstantVectorSource<double>>(tau_hold);
  builder.Connect(torque_src->get_output_port(),
                  plant.get_actuation_input_port());

  // --------------------------  logging + visualisation
  visualization::AddDefaultVisualization(&builder);

  const int n_state = plant.num_positions() + plant.num_velocities();
  auto* log = builder.AddSystem<systems::VectorLogSink<double>>(n_state);
  builder.Connect(plant.get_state_output_port(), log->get_input_port());

  // --------------------------  build & simulate
  auto diagram = builder.Build();
  systems::Simulator<double> sim(*diagram);
  auto& plant_ctx = plant.GetMyMutableContextFromRoot(
      &sim.get_mutable_context());

  plant.SetPositions(&plant_ctx, Eigen::VectorXd::Constant(1, desired_angle));
  plant.SetVelocities(&plant_ctx, Eigen::VectorXd::Zero(1));

  sim.set_target_realtime_rate(1.0);  // run in real-time
  sim.Initialize();
  std::cout << "Running 10 s with manual gravity compensation …\n";
  sim.AdvanceTo(10.0);

  // --------------------------  print log (joint angle only)
  const auto& L = log->GetLog(log->GetMyContextFromRoot(sim.get_context()));
  const int nq = plant.num_positions();          // =1 for this linkage
  for (int i = 0; i < L.num_samples(); i += 100) {
    double t  = L.sample_times()[i];
    double q1 = L.data().col(i).head(nq)(0);     // first position element
    std::cout << "t = " << t << " s  |  angle = " << q1 << " rad\n";
  }
  return 0;
}
