Manipulation helpers API

Overview

Reusable helpers for Drake-based pick-and-place demos. These live in `include/franka_drake/manipulation_helpers.h` and are implemented in `src/manipulation_helpers.cpp`.

Functions

- SolveCollisionFreeIK
  - Purpose: Find a joint configuration reaching a target TCP pose (`fer_hand_tcp`) using Drake IK with position/orientation constraints and a nominal toward the current `q`.
  - Signature:
    ```cpp
    std::optional<Eigen::VectorXd> SolveCollisionFreeIK(
        drake::multibody::MultibodyPlant<double>& plant,
        drake::systems::Context<double>& context,
        const drake::multibody::ModelInstanceIndex& robot,
        const drake::math::RigidTransformd& X_W_EE_target,
        const Eigen::VectorXd& q_current,
        bool relax_constraints = false);
    ```
  - Notes: When `relax_constraints` is true, tolerances are loosened to improve robustness.

- FindObjectByName
  - Purpose: Return the first body whose name contains a substring.
  - Signature:
    ```cpp
    std::optional<drake::multibody::BodyIndex> FindObjectByName(
        const drake::multibody::MultibodyPlant<double>& plant,
        const std::string& object_name_substring);
    ```

- WaitForGripper
  - Purpose: Advance the simulator for a fixed time, useful to let gripper motion settle.
  - Signature:
    ```cpp
    void WaitForGripper(drake::systems::Simulator<double>& simulator, double wait_time_seconds = 1.5);
    ```

- ExecuteJointTrajectory
  - Purpose: Cosine-blend joint-space motion from `q_start` to `q_end`. Optionally keeps an attached object rigidly in front of the TCP during motion.
  - Signature:
    ```cpp
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
    ```

- AttachObject
  - Purpose: Rigidly place an object a small offset in front of the TCP to simulate grasp.
  - Signature:
    ```cpp
    drake::math::RigidTransformd AttachObject(
        drake::multibody::MultibodyPlant<double>& plant,
        drake::systems::Context<double>& plant_ctx,
        const drake::multibody::BodyIndex& object_body,
        const drake::multibody::ModelInstanceIndex& robot);
    ```

- DetachObject
  - Purpose: Place the object at a final pose and zero its spatial velocity.
  - Signature:
    ```cpp
    void DetachObject(
        drake::multibody::MultibodyPlant<double>& plant,
        drake::systems::Context<double>& plant_ctx,
        const drake::multibody::BodyIndex& object_body,
        const drake::math::RigidTransformd& X_W_final);
    ```

Usage example

```cpp
#include "franka_drake/manipulation_helpers.h"

using franka_fci_sim::manipulation::SolveCollisionFreeIK;
using franka_fci_sim::manipulation::ExecuteJointTrajectory;

// ... build plant/scene_graph, AutoAttach robot, build diagram, initialize sim ...

auto q_full = plant.GetPositions(plant_ctx);
const auto robot = plant.GetBodyByName("fer_link1").model_instance();
const auto& ee = plant.GetFrameByName("fer_hand_tcp", robot);
const auto& world = plant.world_frame();
const auto X_W_EE = plant.CalcRelativeTransform(plant_ctx, world, ee);

auto q_target_full = SolveCollisionFreeIK(plant, plant_ctx, robot, X_W_EE, q_full);
if (q_target_full) {
  auto tmp = plant.CreateDefaultContext();
  plant.SetPositions(tmp.get(), *q_target_full);
  const auto q_target = plant.GetPositions(*tmp, robot);
  const auto q_start = plant.GetPositions(plant_ctx, robot);
  ExecuteJointTrajectory(plant, plant_ctx, simulator, robot, q_start, q_target, 2.0, "move");
}
```


