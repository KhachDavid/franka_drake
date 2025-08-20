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
#include <drake/geometry/scene_graph_inspector.h>
#include <drake/geometry/geometry_roles.h>
#include <drake/geometry/collision_filter_declaration.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/solvers/solve.h>
#include <drake/multibody/tree/weld_joint.h>
#include <drake/geometry/query_object.h>
#include <drake/geometry/shape_specification.h>
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

  // Meshcat visualization (added before build)
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();
  drake::visualization::AddDefaultVisualization(&builder, meshcat);

  // Find the robot (no context yet)
  const auto& robot_instance = plant.GetBodyByName("fer_link1").model_instance();
  
  // Find the red cube (try different possible names)
  auto red_cube_body = FindObjectByName(plant, "box_link");
  if (!red_cube_body) {
    std::cerr << "Could not find red cube in scene!" << std::endl;
    return -1;
  }

  // Targeted collision filtering: exclude palm (fer_hand) vs boxes to avoid overly conservative mesh blocking grasp
  try {
    const auto& inspector = scene_graph.model_inspector();
    drake::geometry::GeometrySet hand_set;
    drake::geometry::GeometrySet box_set;
    size_t hand_count = 0;
    size_t box_count = 0;
    const auto hand_frame = plant.GetBodyFrameIdOrThrow(plant.GetBodyByName("fer_hand", robot_instance).index());
    for (const auto& gid : inspector.GetGeometries(hand_frame, drake::geometry::Role::kProximity)) { hand_set.Add(gid); ++hand_count; }
    // Collect all proximity geoms for any body named "box_link" (all colored boxes in scene)
    for (drake::multibody::BodyIndex i(0); i < plant.num_bodies(); ++i) {
      const auto& body = plant.get_body(i);
      if (body.name() == std::string("box_link")) {
        const auto frame = plant.GetBodyFrameIdOrThrow(body.index());
        for (const auto& gid : inspector.GetGeometries(frame, drake::geometry::Role::kProximity)) { box_set.Add(gid); ++box_count; }
      }
    }
    if (hand_count > 0 && box_count > 0) {
      scene_graph.collision_filter_manager().Apply(
          drake::geometry::CollisionFilterDeclaration().ExcludeBetween(hand_set, box_set));
      std::cout << "Applied collision filter: exclude fer_hand vs all box_* proximity." << std::endl;
    }
  } catch (...) {
    std::cout << "Warning: could not apply palm-vs-box collision filter." << std::endl;
  }

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

  // Infer object size (prefer proximity geometry box); fallback to 0.04m
  double object_min_span_m = 0.04;
  try {
    const auto frame_id = plant.GetBodyFrameIdOrThrow(plant.get_body(*red_cube_body).index());
    const auto& inspector = scene_graph.model_inspector();
    std::vector<drake::geometry::GeometryId> geoms = inspector.GetGeometries(frame_id, drake::geometry::Role::kProximity);
    if (geoms.empty()) {
      geoms = inspector.GetGeometries(frame_id, drake::geometry::Role::kIllustration);
    }
    for (const auto& gid : geoms) {
      const auto& shape = inspector.GetShape(gid);
      if (const auto* box = dynamic_cast<const drake::geometry::Box*>(&shape)) {
        const Eigen::Vector3d size = box->size();
        object_min_span_m = size.minCoeff();
        break;
      }
    }
  } catch (...) {
    // Keep fallback value
  }

  // Compute gripper setpoints from object size
  const double kClearance = 0.002;  // 2 mm clearance
  const double final_close_width = std::clamp(object_min_span_m + kClearance, 0.0, 0.08);
  const double preclose_width = std::min(0.07, object_min_span_m + 0.02);

  // Get current positions
  const auto& ee_frame = plant.GetFrameByName("fer_hand_tcp", robot_instance);
  const auto& world_frame = plant.world_frame();

  // Capture starting home configuration for final return (now that context exists)
  const Eigen::VectorXd q_home_config = plant.GetPositions(plant_ctx, robot_instance);
  
  // Helper to solve IK and execute a move to the target pose
  auto move_to_pose = [&](const drake::math::RigidTransformd& X_W_target,
                          const std::string& desc,
                          bool relax_constraints = false) -> bool {
    const Eigen::VectorXd q_current_robot = plant.GetPositions(plant_ctx, robot_instance);
    const Eigen::VectorXd q_current_full = plant.GetPositions(plant_ctx);
    auto q_target_full = SolveCollisionFreeIK(plant, plant_ctx, robot_instance,
                                              X_W_target, q_current_full, relax_constraints);
    if (!q_target_full && !relax_constraints) {
      q_target_full = SolveCollisionFreeIK(plant, plant_ctx, robot_instance,
                                           X_W_target, q_current_full, true);
    }
    if (!q_target_full) return false;
    auto tmp = plant.CreateDefaultContext();
    plant.SetPositions(tmp.get(), *q_target_full);
    const Eigen::VectorXd q_target_robot = plant.GetPositions(*tmp, robot_instance);
    ExecuteJointTrajectory(plant, plant_ctx, simulator, robot_instance,
                           q_current_robot, q_target_robot, 2.5, desc);
    return true;
  };

  // Robust grasp: open → approach → pregrasp → grasp → close → evaluate, with XY correction retries
  bool object_attached = false;
  drake::math::RigidTransformd attachment_transform;
  Eigen::Vector2d xy_correction(0.0, 0.0);
  const int kMaxAttempts = 3;
  for (int attempt = 1; attempt <= kMaxAttempts && !object_attached; ++attempt) {
    const auto cube_pose = plant.EvalBodyPoseInWorld(plant_ctx, plant.get_body(*red_cube_body));

    // Build poses with current XY correction
    drake::math::RigidTransformd X_W_approach = cube_pose;
    auto tA = cube_pose.translation();
    tA.x() += xy_correction.x();
    tA.y() += xy_correction.y();
    tA.z() += 0.15;
    X_W_approach.set_translation(tA);
    X_W_approach.set_rotation(drake::math::RotationMatrixd::MakeXRotation(M_PI));

    drake::math::RigidTransformd X_W_pregrasp = cube_pose;
    auto tP = cube_pose.translation();
    tP.x() += xy_correction.x();
    tP.y() += xy_correction.y();
    tP.z() += 0.015;  // 1.5cm above
    X_W_pregrasp.set_translation(tP);
    X_W_pregrasp.set_rotation(drake::math::RotationMatrixd::MakeXRotation(M_PI));

    drake::math::RigidTransformd X_W_grasp = cube_pose;
    auto tG = cube_pose.translation();
    tG.x() += xy_correction.x();
    tG.y() += xy_correction.y();
    X_W_grasp.set_translation(tG);
    X_W_grasp.set_rotation(drake::math::RotationMatrixd::MakeXRotation(M_PI));

    std::cout << "\n=== Grasp attempt " << attempt << " ===" << std::endl;
    embed->SetGripperWidth(0.07);
    WaitForGripper(simulator, 2);

    if (!move_to_pose(X_W_approach, "Approach above cube")) continue;
    if (!move_to_pose(X_W_pregrasp, "Pregrasp above cube")) continue;

    embed->SetGripperWidth(preclose_width);
    WaitForGripper(simulator, 2);

    if (!move_to_pose(X_W_grasp, "Move to grasp")) continue;

    embed->SetGripperWidth(final_close_width);
    WaitForGripper(simulator, 2);

    // Evaluate grasp
    const auto X_W_EE = plant.CalcRelativeTransform(plant_ctx, world_frame, ee_frame);
    const auto X_W_Cube = plant.EvalBodyPoseInWorld(plant_ctx, plant.get_body(*red_cube_body));
    const Eigen::Vector3d offset = X_W_EE.translation() - X_W_Cube.translation();
    const double dist = offset.norm();
    const double width_now = embed->GetGripperWidth();
    const bool good_pose = dist < 0.03;
    const bool good_width = width_now <= final_close_width + 0.004;   // within +4mm of target
    const bool near_pose = dist < 0.05;
    const bool closing_enough = width_now <= final_close_width + 0.010; // within +10mm
    if ((good_pose && good_width) || (near_pose && closing_enough)) {
      std::cout << "Grasp success (dist=" << dist << ", width=" << width_now << ")" << std::endl;
      attachment_transform = AttachObject(plant, plant_ctx, *red_cube_body, robot_instance);
      object_attached = true;
      break;
    }

    // Compute XY correction for next attempt (clamped to ±1.5cm)
    Eigen::Vector2d delta_xy(offset.x(), offset.y());
    for (int i = 0; i < 2; ++i) {
      if (delta_xy[i] > 0.015) delta_xy[i] = 0.015;
      if (delta_xy[i] < -0.015) delta_xy[i] = -0.015;
    }
    xy_correction -= delta_xy;  // move TCP toward cube center next time
    std::cout << "Grasp failed (dist=" << dist << ", width=" << width_now
              << ") — applying XY correction: " << delta_xy.transpose() << std::endl;
  }

  if (!object_attached) {
    std::cerr << "Failed to grasp object after retries." << std::endl;
    return -1;
  }

  // From here on, plan only the post-grasp sequence: lift → place → release → retreat
  struct Waypoint {
    drake::math::RigidTransformd pose;
    std::string description;
    bool relax_constraints;
    double gripper_width;  // -1 means no change
  };

  std::vector<Waypoint> waypoints;

  // Lift straight up from current grasp location
  const auto cube_pose_after = plant.EvalBodyPoseInWorld(plant_ctx, plant.get_body(*red_cube_body));
  drake::math::RigidTransformd X_W_lift = cube_pose_after;
  X_W_lift.set_translation(cube_pose_after.translation() + Eigen::Vector3d(0, 0, 0.20));
  X_W_lift.set_rotation(drake::math::RotationMatrixd::MakeXRotation(M_PI));
  waypoints.push_back({X_W_lift, "Lift cube", true, -1});

  // Transit to place-above pose
  drake::math::RigidTransformd X_W_place_above;
  X_W_place_above.set_translation(Eigen::Vector3d(0.5, 0.2, 0.25));
  X_W_place_above.set_rotation(drake::math::RotationMatrixd::MakeXRotation(M_PI));
  waypoints.push_back({X_W_place_above, "Move to place position", true, -1});

  // Lower to place
  drake::math::RigidTransformd X_W_place;
  X_W_place.set_translation(Eigen::Vector3d(0.5, 0.2, 0.08));
  X_W_place.set_rotation(drake::math::RotationMatrixd::MakeXRotation(M_PI));
  waypoints.push_back({X_W_place, "Lower to place", true, -1});

  // Release at place pose
  waypoints.push_back({X_W_place, "Release cube", true, 0.07});

  // Retreat
  drake::math::RigidTransformd X_W_retreat = X_W_place;
  X_W_retreat.set_translation(X_W_place.translation() + Eigen::Vector3d(0, 0, 0.15));
  waypoints.push_back({X_W_retreat, "Retreat after placing", false, -1});

  // Execute the post-grasp sequence
  bool carrying = true;
  for (size_t i = 0; i < waypoints.size(); ++i) {
    const auto& waypoint = waypoints[i];
    std::cout << "\n=== Post-grasp step " << (i+1) << "/" << waypoints.size() << ": "
              << waypoint.description << (waypoint.relax_constraints ? " [RELAXED]" : "")
              << std::endl;

    if (waypoint.description == "Release cube") {
      const auto X_W_EE = plant.CalcRelativeTransform(plant_ctx, world_frame, ee_frame);
      const auto X_W_Cube_final = X_W_EE * attachment_transform;
      DetachObject(plant, plant_ctx, *red_cube_body, X_W_Cube_final);
      std::cout << "  Cube released at: " << X_W_Cube_final.translation().transpose() << std::endl;
      WaitForGripper(simulator, 0.2);
      carrying = false;
    }

    if (waypoint.gripper_width >= 0) {
      embed->SetGripperWidth(waypoint.gripper_width);
      WaitForGripper(simulator, 0.5);
    }

    const Eigen::VectorXd q_current_robot = plant.GetPositions(plant_ctx, robot_instance);
    const Eigen::VectorXd q_current_full = plant.GetPositions(plant_ctx);
    auto q_target_full = SolveCollisionFreeIK(plant, plant_ctx, robot_instance,
                                              waypoint.pose, q_current_full,
                                              waypoint.relax_constraints);
    if (!q_target_full) continue;
    auto tmp = plant.CreateDefaultContext();
    plant.SetPositions(tmp.get(), *q_target_full);
    const Eigen::VectorXd q_target_robot = plant.GetPositions(*tmp, robot_instance);
    ExecuteJointTrajectory(plant, plant_ctx, simulator, robot_instance,
                           q_current_robot, q_target_robot, 3.0, waypoint.description,
                           carrying, &(*red_cube_body), &attachment_transform);
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
