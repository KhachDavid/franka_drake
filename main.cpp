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
      multibody::AddMultibodyPlantSceneGraph(&builder, /*time_step=*/0.001);

  multibody::Parser parser(&plant, &scene_graph);
  parser.package_map().AddPackageXml(package_xml_path);

  const multibody::ModelInstanceIndex robot =
      parser.AddModels(urdf_path)[0];

  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("base", robot));

  // --------------------------------------------------------------------------
  // Add torque actuators for the seven arm joints BEFORE finalizing
  // --------------------------------------------------------------------------
  // Torque limits from the URDF <limit effort="…"> tags
  const std::unordered_map<std::string, double> kEffort = {
    {"fer_joint1", 87.0}, {"fer_joint2", 87.0}, {"fer_joint3", 87.0},
    {"fer_joint4", 87.0},                         // shoulder + elbow
    {"fer_joint5", 12.0}, {"fer_joint6", 12.0}, {"fer_joint7", 12.0}  // wrist
  };

  for (const auto& [name, limit] : kEffort) {
    const auto& joint =
      plant.GetJointByName<multibody::RevoluteJoint>(name, robot);

    // create an actuator named "<joint>_act" with the specified torque limit
    plant.AddJointActuator(name + "_act", joint, limit);
  }

  // Add a fixed joint for the gripper
  const std::vector<std::string> kGripperJoints{
      "fer_finger_joint1", "fer_finger_joint2"};
    
  for (const std::string& name : kGripperJoints) {
      const auto& joint =
          plant.GetJointByName<multibody::PrismaticJoint>(name, robot);
  
      plant.AddJointActuator(name + "_act", joint);   // ← change here
  }

  plant.Finalize();

  // --------------------------------------------------------------------------
  // Desired posture source (q*, v*=0)  --------
  // --------------------------------------------------------------------------
  const int n = plant.num_positions(robot);      // n == 9
  VectorXd q_des = VectorXd::Zero(n);
  q_des << 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4, 0.0, 0.0;

  VectorXd v_des = VectorXd::Zero(plant.num_velocities(robot));   // also 9


  auto* desired_source = builder.AddSystem<systems::ConstantVectorSource<double>>(
      (VectorXd(q_des.size() + v_des.size()) << q_des, v_des).finished());

  // --------------------------------------------------------------------------
  // Inverse-Dynamics Controller (PD + gravity)
  // P error goes to zero due to Krasovski-LaSalle invariance principle
  // --------------------------------------------------------------------------
  VectorXd kp   = VectorXd::Zero(n);
  VectorXd kd   = VectorXd::Zero(n);
  VectorXd ki   = VectorXd::Zero(n);

  kp.setConstant(10.0);     // arm joints
  kd = 2.0 * kp.cwiseSqrt(); // critical damping

  auto* id_controller =
      builder.AddSystem<systems::controllers::InverseDynamicsController<double>>(
          plant, kp, kd, ki, robot);

  const int nv = plant.num_velocities(robot);
  auto* zero_accel = builder.AddSystem<systems::ConstantVectorSource<double>>(
                      VectorXd::Zero(nv));

  // Wire state -> controller
  builder.Connect(plant.get_state_output_port(),
                  id_controller->get_input_port_estimated_state());

  // Wire desired state -> controller
  builder.Connect(desired_source->get_output_port(),
                  id_controller->get_input_port_desired_state());

  builder.Connect(zero_accel->get_output_port(),
                id_controller->get_input_port_desired_acceleration());

  // Controller output tau -> plant actuation
  builder.Connect(id_controller->get_output_port(0), // 0 is generalized forces 
                  plant.get_actuation_input_port());

  // --------------------------------------------------------------------------
  // Visualisation and simulation
  // --------------------------------------------------------------------------
  visualization::AddDefaultVisualization(&builder);

  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(1.0);

  // Set initial state to something far from the goal (optional)
  auto& root_context   = simulator.get_mutable_context();
  auto& plant_context  = plant.GetMyMutableContextFromRoot(&root_context);
  VectorXd q0 = q_des;
  q0.head<7>().array() += 0.5;           // 0.5 rad offset for demo
  plant.SetPositions(&plant_context, robot, q0);
  plant.SetVelocities(&plant_context, robot,
                      VectorXd::Zero(plant.num_velocities(robot)));

  simulator.Initialize();
  simulator.AdvanceTo(10.0);             // run 10 s

  return 0;
}
