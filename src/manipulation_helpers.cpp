#include "franka_drake/manipulation_helpers.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <thread>

#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/solvers/solve.h>

namespace franka_fci_sim {
namespace manipulation {

std::optional<Eigen::VectorXd> SolveCollisionFreeIK(
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>& /*context*/,
    const drake::multibody::ModelInstanceIndex& robot,
    const drake::math::RigidTransformd& X_W_EE_target,
    const Eigen::VectorXd& q_current,
    bool relax_constraints) {
  using drake::multibody::InverseKinematics;
  try {
    InverseKinematics ik(plant);
    const auto& world = plant.world_frame();
    const auto& ee = plant.GetFrameByName("fer_hand_tcp", robot);

    const double pos_tol = relax_constraints ? 0.02 : 0.005;
    ik.AddPositionConstraint(
        ee, Eigen::Vector3d::Zero(), world,
        X_W_EE_target.translation() - Eigen::Vector3d(pos_tol, pos_tol, pos_tol),
        X_W_EE_target.translation() + Eigen::Vector3d(pos_tol, pos_tol, pos_tol));

    const drake::math::RotationMatrixd R = X_W_EE_target.rotation();
    const double rot_tol = relax_constraints ? 30.0 * M_PI / 180.0 : 10.0 * M_PI / 180.0;
    ik.AddOrientationConstraint(ee, drake::math::RotationMatrixd(), world, R, rot_tol);

    auto* prog = ik.get_mutable_prog();
    const auto& q_vars = ik.q();

    const Eigen::VectorXd q_nominal = q_current;
    const double regularization_weight = relax_constraints ? 0.1 : 1.0;
    prog->AddQuadraticCost(
        regularization_weight * (q_vars - q_nominal).transpose() * (q_vars - q_nominal));

    prog->SetInitialGuess(q_vars, q_current);
    const auto result = drake::solvers::Solve(*prog);
    if (result.is_success()) {
      return result.GetSolution(q_vars);
    }
    std::cout << "IK failed: " << result.get_solution_result() << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "IK exception: " << e.what() << std::endl;
  }
  return std::nullopt;
}

std::optional<drake::multibody::BodyIndex> FindObjectByName(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::string& object_name_substring) {
  for (drake::multibody::BodyIndex b(0); b < plant.num_bodies(); ++b) {
    const auto& body = plant.get_body(b);
    if (body.name().find(object_name_substring) != std::string::npos) {
      return b;
    }
  }
  return std::nullopt;
}

void WaitForGripper(drake::systems::Simulator<double>& simulator, double wait_time_seconds) {
  const double start_time = simulator.get_context().get_time();
  while (simulator.get_context().get_time() < start_time + wait_time_seconds) {
    simulator.AdvanceTo(simulator.get_context().get_time() + 0.001);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}

void ExecuteJointTrajectory(
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>& plant_ctx,
    drake::systems::Simulator<double>& simulator,
    const drake::multibody::ModelInstanceIndex& robot,
    const Eigen::VectorXd& q_start,
    const Eigen::VectorXd& q_end,
    double duration_seconds,
    const std::string& description,
    bool has_payload,
    const drake::multibody::BodyIndex* attached_body,
    const drake::math::RigidTransformd* attachment_transform) {
  std::cout << "Robot: " << description << " (duration: " << duration_seconds << "s)";
  if (has_payload) std::cout << " [WITH PAYLOAD]";
  std::cout << std::endl;

  const double dt = 0.001;
  const double t_start = simulator.get_context().get_time();
  // Enforce a conservative joint velocity limit to avoid aggressive motions.
  // Use a uniform cap for simplicity; tuned for Panda-like dynamics.
  const double max_joint_vel = has_payload ? 0.4 : 0.6;  // rad/s equivalent
  double required = 0.0;
  const Eigen::VectorXd q_delta = (q_end - q_start).cwiseAbs();
  for (int i = 0; i < q_delta.size(); ++i) {
    required = std::max(required, q_delta[i] / max_joint_vel);
  }
  // Add a safety margin and respect the requested minimum duration.
  double effective_duration = std::max(duration_seconds, required * 1.25);
  if (has_payload) effective_duration *= 1.25;  // a bit slower when carrying

  while (simulator.get_context().get_time() < t_start + effective_duration) {
    const double t_rel = simulator.get_context().get_time() - t_start;
    const double alpha = std::clamp(t_rel / effective_duration, 0.0, 1.0);
    const double s = 0.5 * (1.0 - std::cos(M_PI * alpha));
    const Eigen::VectorXd q_interp = (1.0 - s) * q_start + s * q_end;

    plant.SetPositions(&plant_ctx, robot, q_interp);
    // Set a small nominal velocity toward the goal for stability (not used by Drake physics, but
    // keeps articulated dynamics calm when other systems peek at v).
    const double vel_scale = has_payload ? 0.25 : 0.1;
    plant.SetVelocities(&plant_ctx, robot, vel_scale * (q_end - q_start) / effective_duration);

    if (has_payload && attached_body && attachment_transform) {
      const auto& ee_frame = plant.GetFrameByName("fer_hand_tcp", robot);
      const auto& world_frame = plant.world_frame();
      const auto X_W_EE = plant.CalcRelativeTransform(plant_ctx, world_frame, ee_frame);
      const auto X_W_Obj = X_W_EE * (*attachment_transform);
      const auto& body = plant.get_body(*attached_body);
      plant.SetFreeBodyPose(&plant_ctx, body, X_W_Obj);
      plant.SetFreeBodySpatialVelocity(&plant_ctx, body, drake::multibody::SpatialVelocity<double>::Zero());
    }

    simulator.AdvanceTo(simulator.get_context().get_time() + dt);
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }

  std::cout << "  Movement completed" << std::endl;
}

drake::math::RigidTransformd AttachObject(
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>& plant_ctx,
    const drake::multibody::BodyIndex& object_body,
    const drake::multibody::ModelInstanceIndex& robot) {
  const auto& ee_frame = plant.GetFrameByName("fer_hand_tcp", robot);
  const auto& world_frame = plant.world_frame();

  // Compute the attachment transform from the object's current pose to avoid any jump/drop.
  const auto X_W_EE = plant.CalcRelativeTransform(plant_ctx, world_frame, ee_frame);
  const auto& body = plant.get_body(object_body);
  const auto X_W_Obj_current = plant.EvalBodyPoseInWorld(plant_ctx, body);
  const drake::math::RigidTransformd X_EE_Obj = X_W_EE.inverse() * X_W_Obj_current;

  // Re-set the pose to the exact same value (idempotent) and zero its velocity.
  plant.SetFreeBodyPose(&plant_ctx, body, X_W_Obj_current);
  plant.SetFreeBodySpatialVelocity(&plant_ctx, body, drake::multibody::SpatialVelocity<double>::Zero());

  std::cout << "  Grasping " << plant.get_body(object_body).name() << " (frozen relative to TCP)" << std::endl;
  return X_EE_Obj;
}

void DetachObject(
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>& plant_ctx,
    const drake::multibody::BodyIndex& object_body,
    const drake::math::RigidTransformd& X_W_final) {
  const auto& body = plant.get_body(object_body);
  plant.SetFreeBodyPose(&plant_ctx, body, X_W_final);
  plant.SetFreeBodySpatialVelocity(&plant_ctx, body, drake::multibody::SpatialVelocity<double>::Zero());
  std::cout << "  Placing " << body.name() << " at final position" << std::endl;
}

}  // namespace manipulation
}  // namespace franka_fci_sim


