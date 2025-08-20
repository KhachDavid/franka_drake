// Drake-based pick-and-place with collision awareness
// This integrates directly with the Drake simulation and physics,
// using proper IK, collision detection, and object manipulation.

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/geometry/scene_graph.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/visualization/visualization_config_functions.h>
#include <drake/geometry/meshcat.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/solvers/solve.h>
#include <drake/multibody/tree/weld_joint.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <optional>

#include "franka_drake/fci_sim_embed.h"

using drake::multibody::AddMultibodyPlantSceneGraph;

#include "franka_drake/manipulation_helpers.h"
using franka_fci_sim::manipulation::AttachObject;
using franka_fci_sim::manipulation::DetachObject;
using franka_fci_sim::manipulation::ExecuteJointTrajectory;
using franka_fci_sim::manipulation::FindObjectByName;
using franka_fci_sim::manipulation::SolveCollisionFreeIK;
using franka_fci_sim::manipulation::WaitForGripper;

int main() {
  const double dt = 0.001; // 1 kHz
  drake::systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, dt);

  // Load pick and place scene
  {
    drake::multibody::Parser parser(&plant, &scene_graph);
    const std::string sdf_path = franka_fci_sim::ResolveModelPath("models/pick_and_place_scene.sdf");
    try {
      parser.AddModels(sdf_path);
      std::cout << "Loaded pick and place scene from: " << sdf_path << std::endl;
    } catch (const std::exception& e) {
      std::cerr << "Failed to load pick and place scene SDF: " << sdf_path << "\n" << e.what() << std::endl;
      return -1;
    }
  }

  // Auto-attach the Franka robot with collision detection enabled
  franka_fci_sim::FciSimEmbedder::AutoAttachOptions auto_opts;
  auto_opts.prefer_gripper = true;  // This will use gripper_fixed for now
  auto_opts.disable_collisions = true;  // Disable self-collisions for robot
  
  franka_fci_sim::FciSimOptions opts;
  opts.headless = false;
  opts.turbo = false;
  
  auto embed = franka_fci_sim::FciSimEmbedder::AutoAttach(&plant, &scene_graph, &builder, auto_opts, opts);

  // Register gripper for control
  franka_fci_sim::FciSimEmbedder::GripperSpec gripper_spec;
  gripper_spec.left_joint_name = "fer_finger_joint1";
  gripper_spec.right_joint_name = "fer_finger_joint2";
  gripper_spec.min_width_m = 0.0;
  gripper_spec.max_width_m = 0.08;
  // Increase PD gains for a firmer grasp in simulation
  gripper_spec.kp = 450.0;
  gripper_spec.kd = 12.0;
  gripper_spec.left_sign = +1.0;
  gripper_spec.right_sign = -1.0;  // Right finger moves in opposite direction (negative Y axis)
  embed->RegisterGripper(gripper_spec);

  // Meshcat visualization
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();
  drake::visualization::AddDefaultVisualization(&builder, meshcat);

  // Build and simulate
  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(opts.turbo ? 0.0 : 1.0);

  auto& root = simulator.get_mutable_context();
  auto& plant_ctx = plant.GetMyMutableContextFromRoot(&root);
  
  // Set initial state
  franka_fci_sim::SetDefaultFrankaInitialState(plant, plant_ctx);

  simulator.Initialize();

  embed->SetGripperWidth(0.08);  // Open
  WaitForGripper(simulator, 1.0);
  
  embed->SetGripperWidth(0.02);  // Close
  WaitForGripper(simulator, 1.0);

  // Find the robot and objects
  const auto& robot_instance = plant.GetBodyByName("fer_link1").model_instance();
  
  // Find the red cube (try different possible names)
  auto red_cube_body = FindObjectByName(plant, "box_link");
  if (!red_cube_body) {
    std::cerr << "Could not find red cube in scene!" << std::endl;
    return -1;
  }

  // Get current positions
  const auto& ee_frame = plant.GetFrameByName("fer_hand_tcp", robot_instance);
  const auto& world_frame = plant.world_frame();
  
  // Define pick-and-place targets (updated for new box positions)
  const auto red_cube_pose = plant.EvalBodyPoseInWorld(plant_ctx, plant.get_body(*red_cube_body));
  // Define waypoints with better planning and return home
  struct Waypoint {
    drake::math::RigidTransformd pose;
    std::string description;
    bool relax_constraints;
    double gripper_width;  // -1 means no change
    bool check_grasp;      // Check if gripper made contact
  };
  
  std::vector<Waypoint> waypoints;
  
  // Store home configuration for return
  const Eigen::VectorXd q_home_config = plant.GetPositions(plant_ctx, robot_instance);
  
  // Initial approach position (safely above cube, gripper fully open)
  drake::math::RigidTransformd X_W_approach = red_cube_pose;
  X_W_approach.set_translation(red_cube_pose.translation() + Eigen::Vector3d(0, 0, 0.15));  // 15cm above cube center
  X_W_approach.set_rotation(drake::math::RotationMatrixd::MakeXRotation(M_PI));
  waypoints.push_back({X_W_approach, "Approach red cube (fully open gripper)", false, 0.08, false});  // 8cm = fully open
  
  // Intermediate approach - get closer but still safe distance
  drake::math::RigidTransformd X_W_intermediate = red_cube_pose;
  X_W_intermediate.set_translation(red_cube_pose.translation() + Eigen::Vector3d(0, 0, 0.08));  // 8cm above cube
  X_W_intermediate.set_rotation(drake::math::RotationMatrixd::MakeXRotation(M_PI));
  waypoints.push_back({X_W_intermediate, "Move closer to cube", false, -1, false});
  
  // Pre-grasp - position gripper slightly above cube to avoid collision during approach
  // The gripper has complex collision geometry that extends beyond the visual mesh
  // Position TCP slightly above cube center to account for this
  drake::math::RigidTransformd X_W_pregrasp = red_cube_pose;
  X_W_pregrasp.set_translation(red_cube_pose.translation() + Eigen::Vector3d(0, 0, 0.01));  // TCP 1cm above cube center
  X_W_pregrasp.set_rotation(drake::math::RotationMatrixd::MakeXRotation(M_PI));
  waypoints.push_back({X_W_pregrasp, "Pre-grasp position (slightly above cube)", false, -1, false});
  
  // Move down to grasp height while closing gripper partially
  // This avoids collision issues by closing gripper as we approach
  drake::math::RigidTransformd X_W_grasp = red_cube_pose;
  X_W_grasp.set_translation(red_cube_pose.translation());  // TCP at cube center
  X_W_grasp.set_rotation(drake::math::RotationMatrixd::MakeXRotation(M_PI));
  waypoints.push_back({X_W_grasp, "Move to grasp position", false, 0.05, false});  // Close to 5cm first
  
  // Final close on cube - fingers will close around cube center
  // Cube is 4cm wide, so close to 4.2cm for firm grasp without excessive force
  waypoints.push_back({X_W_grasp, "Close gripper on cube", false, 0.042, true});
  
  // Lift position (straight up from grasp position)
  drake::math::RigidTransformd X_W_lift = X_W_pregrasp;
  X_W_lift.set_translation(X_W_pregrasp.translation() + Eigen::Vector3d(0, 0, 0.15));
  waypoints.push_back({X_W_lift, "Lift cube", true, -1, false});
  
  // Move to place position
  drake::math::RigidTransformd X_W_place_above;
  X_W_place_above.set_translation(Eigen::Vector3d(0.5, 0.2, 0.25));
  X_W_place_above.set_rotation(drake::math::RotationMatrixd::MakeXRotation(M_PI));
  waypoints.push_back({X_W_place_above, "Move to place position", true, -1, false});
  
  // Lower to place
  drake::math::RigidTransformd X_W_place;
  X_W_place.set_translation(Eigen::Vector3d(0.5, 0.2, 0.08));
  X_W_place.set_rotation(drake::math::RotationMatrixd::MakeXRotation(M_PI));
  waypoints.push_back({X_W_place, "Lower to place", true, -1, false});
  
  // Open gripper to release
  waypoints.push_back({X_W_place, "Release cube", true, 0.07, false});
  
  // Lift away after placing
  drake::math::RigidTransformd X_W_retreat = X_W_place;
  X_W_retreat.set_translation(X_W_place.translation() + Eigen::Vector3d(0, 0, 0.15));
  waypoints.push_back({X_W_retreat, "Retreat after placing", false, -1, false});

  bool object_attached = false;
  drake::math::RigidTransformd attachment_transform;
  
  // Execute waypoints with proper gripper control
  for (size_t i = 0; i < waypoints.size(); ++i) {
    const auto& waypoint = waypoints[i];
    
    std::cout << "\n=== Step " << (i+1) << "/" << waypoints.size() << ": " << waypoint.description << " ===";
    if (waypoint.relax_constraints) std::cout << " [RELAXED CONSTRAINTS]";
    std::cout << std::endl;
    
    // Special case: For "Release cube", detach FIRST before opening gripper
    if (waypoint.description == "Release cube" && object_attached) {
      // Calculate where cube should be placed based on gripper position
      const auto& ee_frame = plant.GetFrameByName("fer_hand_tcp", robot_instance);
      const auto X_W_EE = plant.CalcRelativeTransform(plant_ctx, world_frame, ee_frame);
      
      // Place cube at its attached position (2cm below TCP when gripper points down)
      const auto X_W_Cube_final = X_W_EE * attachment_transform;
      
      DetachObject(plant, plant_ctx, *red_cube_body, X_W_Cube_final);
      object_attached = false;
      std::cout << "  Cube released at position: " << X_W_Cube_final.translation().transpose() << std::endl;
      
      // Small delay to let cube settle before opening gripper
      WaitForGripper(simulator, 0.2);  // 200ms delay
    }
    
    // Handle gripper width changes
    if (waypoint.gripper_width >= 0) {
      embed->SetGripperWidth(waypoint.gripper_width);
      WaitForGripper(simulator, 2.0);  // Give gripper 2 seconds to reach position
    }
    
    // Get current robot configuration
    const Eigen::VectorXd q_current_robot = plant.GetPositions(plant_ctx, robot_instance);
    const Eigen::VectorXd q_current_full = plant.GetPositions(plant_ctx);
    
    // Solve IK for this waypoint
    auto q_target_full = SolveCollisionFreeIK(plant, plant_ctx, robot_instance, 
                                             waypoint.pose, q_current_full, 
                                             waypoint.relax_constraints);
    
    if (!q_target_full) {
      std::cout << "Could not find collision-free path!" << std::endl;
      if (!waypoint.relax_constraints) {
        std::cout << "Retrying with relaxed constraints..." << std::endl;
        q_target_full = SolveCollisionFreeIK(plant, plant_ctx, robot_instance, 
                                            waypoint.pose, q_current_full, true);
      }
      if (!q_target_full) {
        std::cout << "Failed - skipping waypoint" << std::endl;
        continue;
      }
    }
    
    // Extract robot configuration
    auto temp_ctx = plant.CreateDefaultContext();
    plant.SetPositions(temp_ctx.get(), *q_target_full);
    const Eigen::VectorXd q_target_robot = plant.GetPositions(*temp_ctx, robot_instance);
    
    // Execute trajectory to this waypoint
    if (object_attached) {
      ExecuteJointTrajectory(plant, plant_ctx, simulator, robot_instance, 
                           q_current_robot, q_target_robot, 3.0, waypoint.description,
                           true, &(*red_cube_body), &attachment_transform);
    } else {
      ExecuteJointTrajectory(plant, plant_ctx, simulator, robot_instance, 
                           q_current_robot, q_target_robot, 3.0, waypoint.description);
    }
    
    // Check grasp AFTER closing gripper
    if (waypoint.check_grasp && !object_attached) {
      // Check if cube is properly positioned relative to gripper
      const auto& ee_frame = plant.GetFrameByName("fer_hand_tcp", robot_instance);
      const auto X_W_EE = plant.CalcRelativeTransform(plant_ctx, world_frame, ee_frame);
      const auto X_W_Cube = plant.EvalBodyPoseInWorld(plant_ctx, plant.get_body(*red_cube_body));
      
      // Expected: TCP should be very close to cube center when properly positioned for grasp
      const Eigen::Vector3d actual_offset = X_W_EE.translation() - X_W_Cube.translation();
      const double distance = actual_offset.norm();
      
      // Also check gripper width to ensure it actually closed on something
      const double current_gripper_width = embed->GetGripperWidth();

      // Primary success condition: a bit looser to improve reliability in sim
      const bool good_pose = distance < 0.035;               // 3.5cm
      const bool good_width = current_gripper_width < 0.048; // < 48mm

      // Sticky fallback: when near and closing, snap-attach for robustness
      const bool near_pose = distance < 0.05;                 // 5cm
      const bool closing_enough = current_gripper_width < 0.055; // < 55mm

      if ((good_pose && good_width) || (near_pose && closing_enough)) {
        if (!(good_pose && good_width)) {
          std::cout << "  Sticky grasp fallback engaged (distance=" << distance*1000
                    << "mm, width=" << current_gripper_width*1000 << "mm)" << std::endl;
        }
        std::cout << "  Successful grasp! TCP-to-cube offset: " << actual_offset.transpose() << std::endl;
        std::cout << "     Gripper width: " << current_gripper_width*1000 << "mm (cube is 40mm)" << std::endl;
        attachment_transform = AttachObject(plant, plant_ctx, *red_cube_body, robot_instance);
        object_attached = true;
      } else {
        std::cout << "  Failed to grasp - ";
        if (!good_pose) {
          std::cout << "position error too large (" << distance*100 << "cm)" << std::endl;
          std::cout << "     Actual offset from cube: " << actual_offset.transpose() << std::endl;
        } else {
          std::cout << "gripper not properly closed (width: " << current_gripper_width*1000 << "mm)" << std::endl;
        }
      }
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // Return to home position
  std::cout << "\nReturning to home position..." << std::endl;
  
  const Eigen::VectorXd q_current = plant.GetPositions(plant_ctx, robot_instance);
  ExecuteJointTrajectory(plant, plant_ctx, simulator, robot_instance, 
                       q_current, q_home_config, 4.0, "Return to home");
  
  // Keep simulation running
  while (true) {
    simulator.AdvanceTo(simulator.get_context().get_time() + dt);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  return 0;
}
