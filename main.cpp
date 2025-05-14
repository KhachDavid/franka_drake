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
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/visualization/visualization_config_functions.h>
#include <drake/systems/controllers/pid_controller.h>
#include <drake/systems/primitives/adder.h>
#include <drake/systems/primitives/vector_log_sink.h>

using namespace drake;
using Eigen::VectorXd;

int main(int argc, char* argv[]) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0]
              << " <path_to_package.xml> <path_to_urdf>" << std::endl;
    return 1;
  }

  const std::string package_xml_path{argv[1]};
  const std::string urdf_path{argv[2]};

  systems::DiagramBuilder<double> builder;

  // --------------------------------------------------------------------------
  // Plant and SceneGraph
  // --------------------------------------------------------------------------
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, /*time_step=*/0.001); // originally set to 0.001
      // when set to 0, we let the integrator choose its own time step

  multibody::Parser parser(&plant, &scene_graph);
  parser.package_map().AddPackageXml(package_xml_path);

  const multibody::ModelInstanceIndex robot =
      parser.AddModels(urdf_path)[0];

  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("base", robot));

  // --------------------------------------------------------------------------
  // Add torque actuators for the seven arm joints BEFORE finalizing
  // --------------------------------------------------------------------------
  // Torque limits from the URDF <limit effort="…"> tag
  struct JointSpec { std::string name; double effort; };

  const std::vector<JointSpec> kActuators = {
    {"fer_joint1", 87.0}, {"fer_joint2", 87.0}, {"fer_joint3", 87.0},
    {"fer_joint4", 87.0},
    {"fer_joint5", 12.0}, {"fer_joint6", 12.0}, {"fer_joint7", 12.0},
    {"fer_finger_joint1", 100.0}, {"fer_finger_joint2", 100.0}
  };

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

  plant.Finalize();

  // --------------------------------------------------------------------------
  // Desired posture source (q, v=0)  --------
  // --------------------------------------------------------------------------
  const int n = plant.num_positions(robot);      // n == 9
  VectorXd q_des = VectorXd::Zero(n);
  q_des << 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4 - 0.1, 0.0, 0.0;

  VectorXd v_des = VectorXd::Zero(plant.num_velocities(robot));   // also 9


  auto* desired_source = builder.AddSystem<systems::ConstantVectorSource<double>>(
      (VectorXd(q_des.size() + v_des.size()) << q_des, v_des).finished());

  // --------------------------------------------------------------------------
  // Inverse-Dynamics Controller (PD + gravity)
  // P error goes to zero due to Krasovski-LaSalle invariance principle
  // --------------------------------------------------------------------------
  // ----------------------------------------------------------------------
  //  PD block  (vector-form constructor: Kp, Ki, Kd)
  // ----------------------------------------------------------------------
  VectorXd kp = VectorXd::Zero(n);
  VectorXd kd = VectorXd::Zero(n);
  VectorXd ki = VectorXd::Zero(n);

  kp.head<7>().setConstant(40.0);
  kd.head<7>() = 2.0 * kp.head<7>().cwiseSqrt();

  auto* pd =
      builder.AddSystem<systems::controllers::PidController<double>>(kp, ki, kd);

  builder.Connect(plant.get_state_output_port(),          // x = [q; v]
                  pd->get_input_port_estimated_state());
  
  builder.Connect(desired_source->get_output_port(),      // [q_d; v_d]
                  pd->get_input_port_desired_state());

  // ----------------------------------------------------------------------
  //  Gravity-compensation block      g̃(q)
  // ----------------------------------------------------------------------
  using systems::controllers::InverseDynamics;
  auto* g_comp =
      builder.AddSystem<InverseDynamics<double>>(
          &plant,
          InverseDynamics<double>::InverseDynamicsMode::kGravityCompensation);

  builder.Connect(plant.get_state_output_port(),
                  g_comp->get_input_port_estimated_state());

  // ----------------------------------------------------------------------
  //  Sum:   τ = τ_PD  +  g̃(q)
  // ----------------------------------------------------------------------
  auto* sum = builder.AddSystem<systems::Adder<double>>(2, n);

  builder.Connect(pd   ->get_output_port(), sum->get_input_port(0));
  builder.Connect(g_comp->get_output_port(), sum->get_input_port(1));
  builder.Connect(sum->get_output_port(), plant.get_actuation_input_port());
  // --------------------------------------------------------------------------
  // Visualisation and simulation
  // --------------------------------------------------------------------------
  visualization::AddDefaultVisualization(&builder);

  const double kLogPeriod = 0.01;           // 10 ms
  auto* sink = builder.AddSystem<
                drake::systems::VectorLogSink<double>>(
                plant.get_state_output_port().size());
  // wire port → sink
  builder.Connect(plant.get_state_output_port(), sink->get_input_port());

  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(1.0);

  // Set initial state to something far from the goal
  auto& root_context   = simulator.get_mutable_context();
  auto& plant_context  = plant.GetMyMutableContextFromRoot(&root_context);
  VectorXd q0 = q_des;

  q0 << 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4, 0.0, 0.0;
  

  plant.SetPositions(&plant_context, robot, q0);
  plant.SetVelocities(&plant_context, robot,
                      VectorXd::Zero(plant.num_velocities(robot)));

  simulator.Initialize();
  // start when enter is pressed
  std::cout << "Press enter to start simulation..." << std::endl;
  std::cin.get();
  simulator.AdvanceTo(20.0);

  // Get the plant’s sub-context from the diagram context you already have
  const auto& plant_ctx =
      plant.GetMyContextFromRoot(root_context);

  const auto& tau =
      plant.get_actuation_input_port().Eval(plant_ctx);
  
  std::cout << "Final tau vector: " << tau.transpose() << std::endl;

  std::cout << "Final q error: "
          << (plant.GetPositions(plant_context, robot) - q_des).transpose()
          << std::endl;

  const auto& sink_context =
    sink->GetMyContextFromRoot(root_context);      // sink’s sub-context

const auto& log = sink->GetLog(sink_context);     // note the argument

std::cout << "Logged " << log.num_samples()
          << " samples of dimension " << log.data().rows() << ".\n";

          // print all 
for (int i = 0; i < log.num_samples(); ++i) {
    std::cout << log.data().col(i).transpose() << std::endl;
}

  return 0;
}
