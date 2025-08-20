#pragma once

#include <optional>
#include <string>

#include <Eigen/Dense>

#include <drake/geometry/scene_graph.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/model_instance.h>
#include <drake/systems/analysis/simulator.h>

namespace franka_fci_sim {
namespace manipulation {

/** \brief Solve collision-aware inverse kinematics for an end-effector pose.
 *
 * Attempts to find joint positions that achieve a target end-effector pose using Drake's
 * InverseKinematics for the frame named "fer_hand_tcp" in the provided robot instance.
 * The solver uses a position and orientation constraint and regularizes toward q_current.
 *
 * \param plant MultibodyPlant to solve against (must be finalized).
 * \param context Plant context to read current kinematics.
 * \param robot Robot model instance index containing the end-effector frame.
 * \param X_W_EE_target Desired end-effector pose in world.
 * \param q_current Current full-plant position vector used as initial guess and nominal.
 * \param relax_constraints When true, loosens tolerances to improve robustness.
 * \return Full-plant q solution on success; std::nullopt if infeasible or solver failure.
 */
std::optional<Eigen::VectorXd> SolveCollisionFreeIK(
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>& context,
    const drake::multibody::ModelInstanceIndex& robot,
    const drake::math::RigidTransformd& X_W_EE_target,
    const Eigen::VectorXd& q_current,
    bool relax_constraints = false);

/** \brief Find a free body by name substring.
 *
 * Searches all bodies in the plant and returns the first BodyIndex whose name contains
 * the given substring.
 */
std::optional<drake::multibody::BodyIndex> FindObjectByName(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::string& object_name_substring);

/** \brief Advance simulation for a fixed time to allow gripper motion to settle. */
void WaitForGripper(drake::systems::Simulator<double>& simulator, double wait_time_seconds = 1.5);

/** \brief Execute a smooth cosine-blend joint trajectory for a single robot instance.
 *
 * Optionally updates an attached object's pose rigidly relative to the frame "fer_hand_tcp".
 */
void ExecuteJointTrajectory(
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>& plant_ctx,
    drake::systems::Simulator<double>& simulator,
    const drake::multibody::ModelInstanceIndex& robot,
    const Eigen::VectorXd& q_start,
    const Eigen::VectorXd& q_end,
    double duration_seconds,
    const std::string& description,
    bool has_payload = false,
    const drake::multibody::BodyIndex* attached_body = nullptr,
    const drake::math::RigidTransformd* attachment_transform = nullptr);

/** \brief Rigidly "attach" an object body to the end-effector for simulated grasping.
 *
 * Places the object a fixed offset in front of the TCP so it remains visible between
 * the gripper fingers.
 * \return The fixed transform X_EE_Obj used for the attachment.
 */
drake::math::RigidTransformd AttachObject(
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>& plant_ctx,
    const drake::multibody::BodyIndex& object_body,
    const drake::multibody::ModelInstanceIndex& robot);

/** \brief "Detach" an object by placing it at the provided world pose and zeroing velocity. */
void DetachObject(
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>& plant_ctx,
    const drake::multibody::BodyIndex& object_body,
    const drake::math::RigidTransformd& X_W_final);

}  // namespace manipulation
}  // namespace franka_fci_sim


