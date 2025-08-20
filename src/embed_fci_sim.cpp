#include "franka_drake/fci_sim_embed.h"

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include <drake/geometry/scene_graph.h>
#include <drake/geometry/collision_filter_manager.h>
#include <drake/geometry/geometry_set.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/multibody/tree/prismatic_joint.h>
#include <drake/systems/controllers/inverse_dynamics.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/primitives/adder.h>
#include <drake/systems/primitives/demultiplexer.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/geometry/scene_graph.h>
#include <drake/systems/framework/diagram_builder.h>
#include <cstdlib>
#include <filesystem>

#include "franka_drake/fci_sim_server.h"
#include "franka_drake/gripper_sim_server.h"
// Using in-file impedance wrapper for parity with server main

namespace franka_fci_sim {

namespace {

// Internal shared state identical to the app; kept here to avoid exposing this
// detail in the public header while maintaining behavior consistency.
struct SharedRobotState {
  std::mutex mutex;
  std::array<double, 7> q{};
  std::array<double, 7> dq{};
  std::array<double, 7> dq_filtered{};
  std::array<double, 7> tau_J{};
  std::array<double, 16> O_T_EE{};
  std::array<double, 7> tau_cmd{};
  std::array<double, 7> q_cmd{};
  std::array<double, 7> dq_cmd{};
  std::array<double, 16> O_T_EE_cmd{};
  std::array<double, 6> O_dP_EE_cmd{};
  std::array<double, 2> elbow{};
  std::array<double, 2> elbow_cmd{};
  std::array<double, 7> tau_ext_hat_filtered{};
  std::array<double, 6> O_F_ext_hat_K{};
  std::array<double, 6> K_F_ext_hat_K{};
  bool has_position_command = false;
  bool has_velocity_command = false;
  bool has_torque_command = false;
  bool has_cartesian_command = false;
  bool has_cartesian_velocity_command = false;
  protocol::Move::ControllerMode current_controller_mode = protocol::Move::ControllerMode::kExternalController;
  protocol::Move::MotionGeneratorMode current_motion_mode = protocol::Move::MotionGeneratorMode::kJointPosition;
  std::chrono::steady_clock::time_point last_command_time = std::chrono::steady_clock::now();
  std::string ee_frame_name = "fer_link7";
  bool has_gripper = false;
  double ee_mass = 0.35973;
  std::array<double, 3> ee_com{0.00089, -0.00044, 0.05491};
  std::array<double, 9> ee_inertia{0.00019541063, 1.65231e-06, 1.48826e-06,
                                   1.65231e-06, 0.00019210361, -1.31132e-06,
                                   1.48826e-06, -1.31132e-06, 0.00017936256};
  std::array<double, 2> gripper_q{0.04, 0.04};
  std::array<double, 2> gripper_dq{0.0, 0.0};
  std::array<std::string, 2> gripper_joint_names{"", ""};
  int num_gripper_joints = 0;
  // Gripper control
  bool gripper_registered = false;
  bool gripper_target_user_set = false;
  double gripper_min_width = 0.0;
  double gripper_max_width = 0.08;
  double gripper_kp = 200.0;
  double gripper_kd = 5.0;
  double gripper_left_sign = 1.0;
  double gripper_right_sign = 1.0;
  double gripper_target_width = 0.08;
};

SharedRobotState& GetSharedState() {
  static SharedRobotState* state = new SharedRobotState();
  return *state;
}

}  // namespace

// Apply a collision filter that disables robot self-collisions while keeping
// collisions with the environment enabled. This constructs a GeometrySet of all
// collision geometries belonging to the given robot model instance and excludes
// collisions within that set.
static void ApplyRobotSelfCollisionFilter(const drake::multibody::MultibodyPlant<double>* plant,
                                          const drake::multibody::ModelInstanceIndex& robot,
                                          drake::geometry::SceneGraph<double>* scene_graph) {
  if (plant == nullptr || scene_graph == nullptr) return;
  drake::geometry::GeometrySet robot_geometries;
  for (drake::multibody::BodyIndex b(0); b < plant->num_bodies(); ++b) {
    const auto& body = plant->get_body(b);
    if (body.model_instance() != robot) continue;
    for (const drake::geometry::GeometryId gid : plant->GetCollisionGeometriesForBody(body)) {
      robot_geometries.Add(gid);
    }
  }
  auto manager = scene_graph->collision_filter_manager();
  drake::geometry::CollisionFilterDeclaration decl;
  decl.ExcludeWithin(robot_geometries);
  manager.Apply(decl);
}

struct FciSimEmbedder::Impl {
  Impl(const drake::multibody::MultibodyPlant<double>* p,
       const drake::multibody::ModelInstanceIndex& r,
       drake::systems::DiagramBuilder<double>* b,
       const FciSimOptions& o)
      : plant(p), robot(r), builder(b), options(o) {}

  const drake::multibody::MultibodyPlant<double>* plant;
  const drake::multibody::ModelInstanceIndex robot;
  drake::systems::DiagramBuilder<double>* builder;
  FciSimOptions options;

  // Systems created during ConfigureSystems()
  drake::systems::controllers::InverseDynamics<double>* g_comp = nullptr;
  drake::systems::Adder<double>* torque_adder = nullptr;
  int state_input_port_index = -1;
  int num_arm_joints_cached = 7;
  int num_total_joints_cached = 7;
  // Cached indices for gripper joints within the robot instance position vector
  int gripper_left_q_index = -1;
  int gripper_right_q_index = -1;
  int gripper_left_act_index = -1;
  int gripper_right_act_index = -1;

  // Networking server
  std::unique_ptr<FrankaFciSimServer> server;
  std::unique_ptr<FrankaGripperSimServer> gripper_server;

  // Wiring helpers
  std::pair<int, int> DetectEndEffectorConfiguration();
  void ConfigureControllerSystems(int num_arm_joints, int num_total_joints);
  protocol::RobotState MakeRobotStateSnapshot();
  void HandleRobotCommand(const protocol::RobotCommand& cmd);
};

std::pair<int, int> FciSimEmbedder::Impl::DetectEndEffectorConfiguration() {
  auto& state = GetSharedState();
  std::lock_guard<std::mutex> lock(state.mutex);
  int num_arm_joints = 7;
  int num_total_joints = plant->num_positions(robot);
  const std::vector<std::string> candidates = options.ee_frame_hint.empty()
                                                  ? std::vector<std::string>{"fer_hand_tcp", "fer_link8", "fer_hand", "fer_link7"}
                                                  : std::vector<std::string>{options.ee_frame_hint, "fer_hand_tcp", "fer_link8", "fer_hand", "fer_link7"};
  for (const auto& frame_name : candidates) {
    try {
      plant->GetFrameByName(frame_name, robot);
      state.ee_frame_name = frame_name;
      break;
    } catch (const std::exception&) {
      continue;
    }
  }
  try {
    plant->GetFrameByName("fer_hand", robot);
    state.has_gripper = true;
    std::vector<std::string> gripper_candidates = {"fer_finger_joint1", "fer_finger_joint2"};
    state.num_gripper_joints = 0;
    for (const auto& jn : gripper_candidates) {
      try {
        const auto& joint = plant->GetJointByName(jn, robot);
        // Resolve q indices inside the model instance
        const int nq_before = plant->num_positions(robot);
        (void)nq_before; // quiet unused variable in some Drake versions
        // Fallback: query via tree API if available
        if (state.num_gripper_joints < 2) {
          state.gripper_joint_names[state.num_gripper_joints++] = jn;
        }
      } catch (const std::exception&) {
      }
    }
    // Try to query position indices using MultibodyPlant APIs
    try {
      const auto& jl = plant->GetJointByName("fer_finger_joint1", robot);
      const auto& jr = plant->GetJointByName("fer_finger_joint2", robot);
      gripper_left_q_index = jl.position_start();
      gripper_right_q_index = jr.position_start();
    } catch (const std::exception&) {
      gripper_left_q_index = 7;
      gripper_right_q_index = 8;
    }
    // Compute within-instance actuation indices using actuator order (matches plant actuation port).
    try {
      // Collect actuator order for this instance
      struct Entry { int ordinal; std::string joint_name; };
      std::vector<Entry> entries;
      for (drake::multibody::JointActuatorIndex a(0); a < plant->num_actuators(); ++a) {
        const auto& act = plant->get_joint_actuator(a);
        if (act.model_instance() != robot) continue;
        entries.push_back({static_cast<int>(a), act.joint().name()});
      }
      std::sort(entries.begin(), entries.end(), [](const Entry& x, const Entry& y){ return x.ordinal < y.ordinal; });
      auto find_local_index_by_joint_name = [&](const std::string& jn)->int{
        for (int i = 0; i < static_cast<int>(entries.size()); ++i) if (entries[i].joint_name == jn) return i; return -1; };
      gripper_left_act_index = find_local_index_by_joint_name("fer_finger_joint1");
      gripper_right_act_index = find_local_index_by_joint_name("fer_finger_joint2");
      if (gripper_left_act_index < 0) gripper_left_act_index = num_arm_joints;
      if (gripper_right_act_index < 0) gripper_right_act_index = num_arm_joints + 1;
    } catch (const std::exception&) {
      gripper_left_act_index = num_arm_joints;
      gripper_right_act_index = num_arm_joints + 1;
    }
    // EE parameters for gripper variants
    if (state.ee_frame_name == "fer_hand_tcp") {
      state.ee_mass = 0.73;
      state.ee_com = {0.0, 0.0, 0.058};
      state.ee_inertia = {0.001, 0.0, 0.0,
                          0.0, 0.0025, 0.0,
                          0.0, 0.0, 0.0017};
    } else if (state.ee_frame_name == "fer_hand") {
      state.ee_mass = 0.73;
      state.ee_com = {-0.01, 0.0, 0.03};
      state.ee_inertia = {0.001, 0.0, 0.0,
                          0.0, 0.0025, 0.0,
                          0.0, 0.0, 0.0017};
    }
  } catch (const std::exception&) {
    state.has_gripper = false;
    state.num_gripper_joints = 0;
    // Fingerless defaults
    state.ee_mass = 0.35973;
    state.ee_com = {0.00089, -0.00044, 0.05491};
    state.ee_inertia = {0.00019541063, 1.65231e-06, 1.48826e-06,
                        1.65231e-06, 0.00019210361, -1.31132e-06,
                        1.48826e-06, -1.31132e-06, 0.00017936256};
  }
  return {num_arm_joints, num_total_joints};
}

void FciSimEmbedder::Impl::ConfigureControllerSystems(int num_arm_joints, int num_total_joints) {
  using drake::systems::controllers::InverseDynamics;
  g_comp = builder->AddSystem<InverseDynamics<double>>(plant, InverseDynamics<double>::InverseDynamicsMode::kGravityCompensation);
  // Gravity comp needs full plant state (includes environment), keep global state connection.
  builder->Connect(plant->get_state_output_port(), g_comp->get_input_port_estimated_state());

  // Full-featured impedance/velocity/cartesian/torque controller (parity with app).
  // Restore in-file impedance controller wrapper
  class ImpedanceWrapper final : public drake::systems::LeafSystem<double> {
   public:
    explicit ImpedanceWrapper(const drake::multibody::MultibodyPlant<double>* plant,
                              drake::multibody::ModelInstanceIndex robot,
                              int arm_joints, int total_joints, int nu_robot,
                              int gripper_left_q_index, int gripper_right_q_index,
                              int gripper_left_act_index, int gripper_right_act_index)
        : plant_(plant), robot_(robot), arm_joints_(arm_joints), total_joints_(total_joints), nu_robot_(nu_robot),
          gripper_left_q_index_(gripper_left_q_index), gripper_right_q_index_(gripper_right_q_index),
          gripper_left_act_index_(gripper_left_act_index), gripper_right_act_index_(gripper_right_act_index) {
      state_in_ = this->DeclareVectorInputPort("robot_state", drake::systems::BasicVector<double>(2 * total_joints_)).get_index();
      this->DeclareVectorOutputPort("control_torques", drake::systems::BasicVector<double>(nu_robot_),
                                    &ImpedanceWrapper::CalcTorques);
      plant_context_ = plant_->CreateDefaultContext();
      integral_errors_ = Eigen::VectorXd::Zero(arm_joints_);
      velocity_integral_errors_ = Eigen::VectorXd::Zero(arm_joints_);
      // Build mapping from arm joints to full-plant velocity indices for gravity selection.
      arm_velocity_indices_.clear();
      arm_velocity_indices_.reserve(arm_joints_);
      // Try by known names first
      bool names_ok = true;
      for (int i = 1; i <= arm_joints_; ++i) {
        std::string name = std::string("fer_joint") + std::to_string(i);
        try {
          const auto& joint = plant_->GetJointByName(name, robot_);
          arm_velocity_indices_.push_back(joint.velocity_start());
        } catch (const std::exception&) {
          names_ok = false; break;
        }
      }
      if (!names_ok || static_cast<int>(arm_velocity_indices_.size()) != arm_joints_) {
        arm_velocity_indices_.clear();
        // Fallback: collect single-dof non-gripper joints of this instance, sort by velocity_start
        std::vector<std::pair<int, std::string>> vstart_name;
        for (drake::multibody::JointIndex j(0); j < plant_->num_joints(); ++j) {
          const auto& joint = plant_->get_joint(j);
          if (joint.model_instance() != robot_) continue;
          if (joint.num_velocities() != 1) continue;
          const std::string& jn = joint.name();
          if (jn == std::string("fer_finger_joint1") || jn == std::string("fer_finger_joint2")) continue;
          vstart_name.emplace_back(joint.velocity_start(), jn);
        }
        std::sort(vstart_name.begin(), vstart_name.end(), [](const auto& a, const auto& b){return a.first < b.first;});
        for (int i = 0; i < arm_joints_ && i < static_cast<int>(vstart_name.size()); ++i) {
          arm_velocity_indices_.push_back(vstart_name[i].first);
        }
      }
    }
    int state_in() const { return state_in_; }
   private:
    void CalcTorques(const drake::systems::Context<double>& ctx, drake::systems::BasicVector<double>* out) const {
      auto& st = GetSharedState();
      std::lock_guard<std::mutex> lock(st.mutex);
      Eigen::VectorXd tau(arm_joints_);
      const auto& x = this->get_input_port(state_in_).Eval(ctx);
      Eigen::VectorXd q = x.head(total_joints_);
      Eigen::VectorXd dq = x.tail(total_joints_);
      // Seed the internal setpoint filter with the current joint positions on first use
      // to avoid a large transient on the first command cycle.
      static bool q_filter_seeded = false;
      if (!q_filter_seeded) {
        for (int i = 0; i < arm_joints_ && i < q.size(); ++i) {
          q_cmd_filtered_[i] = q[i];
        }
        q_filter_seeded = true;
      }
      plant_->SetPositions(plant_context_.get(), robot_, q.head(total_joints_));
      plant_->SetVelocities(plant_context_.get(), robot_, dq.head(total_joints_));
      // Gravity is provided by external g_comp; controller outputs pure control torques.
      Eigen::VectorXd tau_g_arm = Eigen::VectorXd::Zero(arm_joints_);

      static const std::array<double, 7> torque_limits{87.0,87.0,87.0,87.0,50.0,50.0,40.0};
      static const double max_tau_rate = 20000.0; // Nm/s (more relaxed to track fast trajectories)
      static Eigen::VectorXd last_tau = Eigen::VectorXd::Zero(7);

      const bool commands_active = (std::chrono::duration<double>(std::chrono::steady_clock::now() - st.last_command_time).count() < 0.1);

      // TORQUE CONTROL: pass-through minus gravity (matches app high limits, no rate limit)
      if (st.has_torque_command) {
        // External torque control: pass user torques only; gravity is added by g_comp path.
        static const std::array<double, 7> high_limits{87.0,87.0,87.0,87.0,50.0,50.0,40.0};
        for (int i = 0; i < arm_joints_; ++i) {
          double t = st.tau_cmd[i];
          t = std::clamp(t, -high_limits[i], high_limits[i]);
          tau[i] = t;
        }
        integral_errors_.setZero();
        velocity_integral_errors_.setZero();
        // Compose output vector sized to actuation inputs (arm + optional gripper).
        Eigen::VectorXd tau_out = Eigen::VectorXd::Zero(nu_robot_);
        for (int i = 0; i < std::min(arm_joints_, nu_robot_); ++i) {
          tau_out[i] = tau[i];
        }
        // Leave any extra actuators (e.g., gripper) at zero torque during external torque control.
        out->SetFromVector(tau_out);
        return;
      }
      // VELOCITY CONTROL (slightly lower gains to reduce chatter with client PI)
      else if (st.has_velocity_command && commands_active) {
        const double Kp[7] = {40,40,40,40,26,22,18};
        const double Ki[7] = {4,4,4,4,2.6,2.2,1.8};
        for (int i = 0; i < arm_joints_; ++i) {
          double e = st.dq_cmd[i] - dq[i];
          velocity_integral_errors_[i] = std::clamp(velocity_integral_errors_[i] + e * 0.001, -0.5, 0.5);
          tau[i] = Kp[i] * e + Ki[i] * velocity_integral_errors_[i];
        }
      }
      // CARTESIAN CONTROL (position or velocity)
      else if ((st.has_cartesian_command || st.has_cartesian_velocity_command) && commands_active) {
        Eigen::MatrixXd J(6, arm_joints_);
        plant_->CalcJacobianSpatialVelocity(*plant_context_, drake::multibody::JacobianWrtVariable::kQDot,
                                            plant_->GetFrameByName(st.ee_frame_name, robot_), Eigen::Vector3d::Zero(),
                                            plant_->world_frame(), plant_->world_frame(), &J);
        Eigen::VectorXd F(6);
        if (st.has_cartesian_command) {
          Eigen::Matrix4d T_cmd;
          for (int c = 0; c < 4; ++c) for (int r = 0; r < 4; ++r) T_cmd(r,c) = st.O_T_EE_cmd[c*4 + r];
          drake::math::RigidTransformd X_W_EE_cmd(T_cmd);
          const drake::math::RigidTransformd X_W_EE = plant_->CalcRelativeTransform(*plant_context_, plant_->world_frame(), plant_->GetFrameByName(st.ee_frame_name, robot_));
          Eigen::Vector3d p_err = X_W_EE_cmd.translation() - X_W_EE.translation();
          drake::math::RotationMatrixd R_err = X_W_EE_cmd.rotation() * X_W_EE.rotation().inverse();
          Eigen::AngleAxisd aa(R_err.matrix());
          Eigen::Vector3d o_err = aa.angle() * aa.axis();
          const double Kt = 100.0, Kr = 30.0, Dt = 20.0, Dr = 10.0;
          Eigen::VectorXd v_ee = J * dq.head(arm_joints_);
          F.head<3>() = Kt * p_err - Dt * v_ee.head<3>();
          F.tail<3>() = Kr * o_err - Dr * v_ee.tail<3>();
        } else {
          Eigen::VectorXd v_ee = J * dq.head(arm_joints_);
          const double Kpt = 80.0, Kpr = 20.0;
          Eigen::VectorXd v_ref(6);
          for (int i = 0; i < 6; ++i) v_ref[i] = st.O_dP_EE_cmd[i];
          Eigen::VectorXd v_err = v_ref - v_ee;
          F.head<3>() = Kpt * v_err.head<3>();
          F.tail<3>() = Kpr * v_err.tail<3>();
        }
        tau = J.transpose() * F;
      }
      // POSITION CONTROL (only when we have active position commands)
      else if (st.has_position_command && commands_active) {
        // Motion generator approach: smooth trajectory generation with rate limiting like libfranka-sim
        // Moderate gains with emphasis on trajectory smoothing rather than high stiffness
        const double K[7]  = {150,150,150,150,100,80,60};    // Moderate stiffness
        const double D[7]  = {15,15,15,15,12,10,8};          // Moderate damping
        const double Ki[7] = {0.5,0.5,0.5,0.5,0.3,0.2,0.1}; // Small integral for steady-state
        
        // Joint limits for rate limiting (rad/s and rad/s²)
        static const double max_vel[7] = {2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61};
        static const double max_acc[7] = {15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0};
        
        static const std::array<double, 7> plant_to_franka_sign{1.0,1.0,1.0,1.0,1.0,1.0,1.0};
        
        // Trajectory generation with smooth acceleration-limited motion
        for (int i = 0; i < arm_joints_; ++i) {
          double q_target = plant_to_franka_sign[i] * st.q_cmd[i];
          double q_current = q[i];
          double dq_current = dq[i];
          
          // Compute desired trajectory point with rate limiting
          double q_error = q_target - q_cmd_filtered_[i];
          
          // Velocity-limited approach to target
          double max_step_vel = max_vel[i] * 0.001; // max velocity step per 1ms
          if (q_error > max_step_vel) {
            q_cmd_filtered_[i] += max_step_vel;
          } else if (q_error < -max_step_vel) {
            q_cmd_filtered_[i] -= max_step_vel;
          } else {
            q_cmd_filtered_[i] = q_target;
          }
          
          // Compute desired velocity (derivative of filtered position)
          static std::array<double, 7> q_cmd_prev{{0,0,0,0,0,0,0}};
          double dq_desired = (q_cmd_filtered_[i] - q_cmd_prev[i]) / 0.001;
          q_cmd_prev[i] = q_cmd_filtered_[i];
          
          // Clamp desired velocity
          dq_desired = std::clamp(dq_desired, -max_vel[i], max_vel[i]);
          
          // PD+I control with velocity feed-forward
          double pos_error = q_cmd_filtered_[i] - q_current;
          double vel_error = dq_desired - dq_current;
          
          // Integrate position error with anti-windup
          double integ = integral_errors_[i] + pos_error * 0.001;
          integ = std::clamp(integ, -0.1, 0.1); // tight integral bounds
          integral_errors_[i] = integ;
          
          // Control law: PD with velocity feed-forward + small integral
          tau[i] = K[i] * pos_error + D[i] * vel_error + Ki[i] * integ;
          
          // Acceleration limiting on torque output
          static std::array<double, 7> tau_prev{{0,0,0,0,0,0,0}};
          double tau_rate = (tau[i] - tau_prev[i]) / 0.001;
          double max_tau_rate = max_acc[i] * 10.0; // rough torque rate from acceleration
          if (tau_rate > max_tau_rate) tau[i] = tau_prev[i] + max_tau_rate * 0.001;
          if (tau_rate < -max_tau_rate) tau[i] = tau_prev[i] - max_tau_rate * 0.001;
          tau_prev[i] = tau[i];
        }
      } else {
        // Idle: zero from controller; g_comp holds posture.
        tau.setZero();
        integral_errors_.setZero();
        velocity_integral_errors_.setZero();
      }

      for (int i = 0; i < arm_joints_; ++i) {
        double t = std::clamp(tau[i], -torque_limits[i], torque_limits[i]);
        double delta = t - last_tau[i];
        double max_delta = max_tau_rate * 0.001;
        if (delta > max_delta) t = last_tau[i] + max_delta;
        if (delta < -max_delta) t = last_tau[i] - max_delta;
        tau[i] = t;
        last_tau[i] = t;
      }

      // Compose output vector of size nu_robot_: arm torques + optional gripper torques
      Eigen::VectorXd tau_out = Eigen::VectorXd::Zero(nu_robot_);
      for (int i = 0; i < std::min(arm_joints_, nu_robot_); ++i) tau_out[i] = tau[i];

      if (nu_robot_ > arm_joints_ && st.gripper_registered && st.num_gripper_joints == 2 && gripper_left_q_index_ >= 0 && gripper_right_q_index_ >= 0) {
        // Simple P-D control for prismatic fingers to track desired width.
        // For prismatic with non-negative limits, both joints target +w/2; axis orientation handles direction.
        double w = std::clamp(st.gripper_target_width, st.gripper_min_width, st.gripper_max_width);
        // Desired opening split equally; both prismatic joint coordinates are positive (0..0.04).
        double qL_des = (w * 0.5);
        double qR_des = (w * 0.5);
        const int iL = gripper_left_act_index_;
        const int iR = gripper_right_act_index_;
        if (iL >= 0 && iR >= 0 && iL < nu_robot_ && iR < nu_robot_) {
          // Use position indices to read joint states; actuation indices only for torque output slots.
          const int qL_idx = gripper_left_q_index_;
          const int qR_idx = gripper_right_q_index_;
          if (qL_idx >= 0 && qR_idx >= 0 && qL_idx < q.size() && qR_idx < q.size()) {
            double qL = q[qL_idx];
            double qR = q[qR_idx];
            double vL = dq[qL_idx];
            double vR = dq[qR_idx];
            double eL = qL_des - qL;
            double eR = qR_des - qR;
            tau_out[iL] = st.gripper_kp * eL - st.gripper_kd * vL;
            tau_out[iR] = st.gripper_kp * eR - st.gripper_kd * vR;
          }
        }
      }

      out->SetFromVector(tau_out);
    }
    const drake::multibody::MultibodyPlant<double>* plant_;
    const drake::multibody::ModelInstanceIndex robot_;
    int arm_joints_;
    int total_joints_;
    int nu_robot_;
    int gripper_left_q_index_;
    int gripper_right_q_index_;
    int gripper_left_act_index_;
    int gripper_right_act_index_;
    int state_in_;
    mutable std::unique_ptr<drake::systems::Context<double>> plant_context_;
    mutable Eigen::VectorXd integral_errors_;
    mutable Eigen::VectorXd velocity_integral_errors_;
    mutable std::array<double,7> q_cmd_filtered_{{0,0,0,0,0,0,0}};
    std::vector<int> arm_velocity_indices_;
  };

  num_arm_joints_cached = num_arm_joints;
  num_total_joints_cached = num_total_joints;
  const int nu_robot = plant->get_actuation_input_port(robot).size();
  auto* ctrl = builder->AddSystem<ImpedanceWrapper>(plant, robot, num_arm_joints, num_total_joints, nu_robot,
                                                    gripper_left_q_index, gripper_right_q_index,
                                                    gripper_left_act_index, gripper_right_act_index);
  state_input_port_index = ctrl->state_in();
  // Connect only the robot model instance state (positions+velocities) to the controller.
  builder->Connect(plant->get_state_output_port(robot), ctrl->get_input_port(state_input_port_index));

  // Add torque adder and connect to plant. Feed gravity torques mapped to the robot's actuation order.
  // Size: actuation inputs for the robot instance (arm + fingers if actuated).
  torque_adder = builder->AddSystem<drake::systems::Adder<double>>(2, nu_robot);

  // Map full-plant generalized forces to the robot instance's actuation input ordering.
  class GravityToActuationSelector final : public drake::systems::LeafSystem<double> {
   public:
    GravityToActuationSelector(const drake::multibody::MultibodyPlant<double>* plant,
                               drake::multibody::ModelInstanceIndex robot,
                               int nu_robot)
        : plant_(plant), robot_(robot), nu_robot_(nu_robot) {
      // Input: full generalized forces (size = plant->num_velocities())
      in_ = this->DeclareVectorInputPort("tau_g_full", drake::systems::BasicVector<double>(plant_->num_velocities())).get_index();
      // Output: actuation-ordered forces for this robot instance (size = nu_robot)
      this->DeclareVectorOutputPort("tau_g_inst", drake::systems::BasicVector<double>(nu_robot_),
                                    &GravityToActuationSelector::Calc);
      BuildMapping();
    }
   private:
    void BuildMapping() {
      // Collect all joint actuators belonging to this instance with their global actuator order
      // and the velocity index of the underlying joint.
      struct MapEntry { int actuator_order; int vstart; };
      std::vector<MapEntry> entries;
      entries.reserve(nu_robot_);
      for (drake::multibody::JointActuatorIndex a(0); a < plant_->num_actuators(); ++a) {
        const auto& act = plant_->get_joint_actuator(a);
        if (act.model_instance() != robot_) continue;
        const auto& joint = act.joint();
        if (joint.num_velocities() != 1) continue;
        entries.push_back({static_cast<int>(a), joint.velocity_start()});
      }
      std::sort(entries.begin(), entries.end(), [](const MapEntry& x, const MapEntry& y){ return x.actuator_order < y.actuator_order; });
      mapping_.clear();
      for (const auto& e : entries) mapping_.push_back(e.vstart);
      // If fewer than nu_robot actuators were discovered (shouldn't happen), pad with -1
      while (static_cast<int>(mapping_.size()) < nu_robot_) mapping_.push_back(-1);
    }
    void Calc(const drake::systems::Context<double>& ctx, drake::systems::BasicVector<double>* out) const {
      const auto& tau_full = this->get_input_port(in_).Eval(ctx);
      Eigen::VectorXd tau = Eigen::VectorXd::Zero(nu_robot_);
      for (int i = 0; i < nu_robot_ && i < static_cast<int>(mapping_.size()); ++i) {
        int idx = mapping_[i];
        if (idx >= 0 && idx < tau_full.size()) tau[i] = tau_full[idx];
      }
      out->SetFromVector(tau);
    }
    const drake::multibody::MultibodyPlant<double>* plant_;
    const drake::multibody::ModelInstanceIndex robot_;
    int nu_robot_;
    int in_;
    std::vector<int> mapping_;
  };

  auto* selector = builder->AddSystem<GravityToActuationSelector>(plant, robot, nu_robot);
  builder->Connect(g_comp->get_output_port(), selector->get_input_port(0));
  builder->Connect(selector->get_output_port(), torque_adder->get_input_port(0));

  builder->Connect(ctrl->get_output_port(), torque_adder->get_input_port(1));

  // Connect actuation to the robot model instance only.
  builder->Connect(torque_adder->get_output_port(), plant->get_actuation_input_port(robot));

  // State mirror system: periodically mirrors plant state into shared state for server.
  class StateMirror final : public drake::systems::LeafSystem<double> {
   public:
    StateMirror(const drake::multibody::MultibodyPlant<double>* plant,
                drake::multibody::ModelInstanceIndex robot,
                int total_joints,
                int gripper_left_q_index,
                int gripper_right_q_index)
        : plant_(plant), robot_(robot), total_joints_(total_joints),
          gripper_left_q_index_(gripper_left_q_index), gripper_right_q_index_(gripper_right_q_index) {
      in_ = this->DeclareVectorInputPort("robot_state", drake::systems::BasicVector<double>(2 * total_joints_)).get_index();
      this->DeclarePeriodicUnrestrictedUpdateEvent(0.001, 0.0, &StateMirror::DoUpdate);
      plant_context_ = plant_->CreateDefaultContext();
    }
   private:
    void DoUpdate(const drake::systems::Context<double>& ctx, drake::systems::State<double>*) const {
      auto& st = GetSharedState();
      std::lock_guard<std::mutex> lock(st.mutex);
      const auto& x = this->get_input_port(in_).Eval(ctx);
      Eigen::VectorXd q = x.head(total_joints_);
      Eigen::VectorXd dq = x.tail(total_joints_);
      // Update arm joint state
      for (int i = 0; i < 7 && i < q.size(); ++i) {
        st.q[i] = q[i];
        st.dq[i] = dq[i];
        st.dq_filtered[i] = 0.8 * st.dq_filtered[i] + 0.2 * st.dq[i];
        st.tau_J[i] = 0.1 * std::sin(q[i]);
      }
      // Update gripper state
      if (gripper_left_q_index_ >= 0 && gripper_right_q_index_ >= 0 && q.size() > gripper_right_q_index_) {
        st.gripper_q[0] = std::max(0.0, q[gripper_left_q_index_]);
        st.gripper_q[1] = std::max(0.0, q[gripper_right_q_index_]);
        st.gripper_dq[0] = dq[gripper_left_q_index_];
        st.gripper_dq[1] = dq[gripper_right_q_index_];
      }
      // EE pose
      plant_->SetPositions(plant_context_.get(), robot_, q.head(total_joints_));
      const auto& ee = plant_->GetFrameByName(st.ee_frame_name, robot_);
      const auto& world = plant_->world_frame();
      const drake::math::RigidTransformd X_W_EE = plant_->CalcRelativeTransform(*plant_context_, world, ee);
      const Eigen::Matrix4d T = X_W_EE.GetAsMatrix4();
      for (int c = 0; c < 4; ++c) for (int r = 0; r < 4; ++r) st.O_T_EE[c*4 + r] = T(r,c);
      // Elbow
      st.elbow[0] = st.q[2];
      st.elbow[1] = (st.q[3] < 0) ? 1.0 : -1.0;
      // Seed commands once
      static bool seeded = false;
      if (!seeded) {
        st.q_cmd = st.q;
        st.O_T_EE_cmd = st.O_T_EE;
        // Initialize gripper target width from current state if present,
        // but only if the user hasn't set a custom width yet.
        if (!st.gripper_target_user_set && st.num_gripper_joints == 2) {
          st.gripper_target_width = std::clamp(st.gripper_q[0] + st.gripper_q[1], st.gripper_min_width, st.gripper_max_width);
        }
        seeded = true;
      }
    }
    const drake::multibody::MultibodyPlant<double>* plant_;
    const drake::multibody::ModelInstanceIndex robot_;
    int total_joints_;
    int in_;
    mutable std::unique_ptr<drake::systems::Context<double>> plant_context_;
    int gripper_left_q_index_;
    int gripper_right_q_index_;
  };

  auto* mirror = builder->AddSystem<StateMirror>(plant, robot, num_total_joints, gripper_left_q_index, gripper_right_q_index);
  builder->Connect(plant->get_state_output_port(robot), mirror->get_input_port(0));
}

protocol::RobotState FciSimEmbedder::Impl::MakeRobotStateSnapshot() {
  auto& st = GetSharedState();
  std::lock_guard<std::mutex> lock(st.mutex);
  protocol::RobotState rs{};
  rs.q = st.q;
  rs.dq = st.dq;  // Use raw velocities to reduce double-filter lag with client-side filters
  rs.tau_J = st.tau_J;
  rs.O_T_EE = st.O_T_EE;
  rs.O_T_EE_d = st.O_T_EE;
  rs.O_T_EE_c = st.O_T_EE_cmd;
  rs.q_d = st.has_position_command ? st.q_cmd : st.q;
  rs.dq_d.fill(0.0);
  for (int i = 0; i < 7; ++i) rs.tau_J_d[i] = st.tau_cmd[i];
  rs.F_T_EE[0] = rs.F_T_EE[5] = rs.F_T_EE[10] = rs.F_T_EE[15] = 1.0;
  rs.EE_T_K[0] = rs.EE_T_K[5] = rs.EE_T_K[10] = rs.EE_T_K[15] = 1.0;
  rs.F_T_NE[0] = rs.F_T_NE[5] = rs.F_T_NE[10] = rs.F_T_NE[15] = 1.0;
  rs.NE_T_EE[0] = rs.NE_T_EE[5] = rs.NE_T_EE[10] = rs.NE_T_EE[15] = 1.0;
  rs.m_ee = st.ee_mass;
  rs.m_load = 0.0;
  rs.I_ee[0] = st.ee_inertia[0];
  rs.I_ee[1] = st.ee_inertia[4];
  rs.I_ee[2] = st.ee_inertia[8];
  rs.I_ee[3] = st.ee_inertia[1];
  rs.I_ee[4] = st.ee_inertia[2];
  rs.I_ee[5] = st.ee_inertia[5];
  rs.F_x_Cee[0] = st.ee_com[0];
  rs.F_x_Cee[1] = st.ee_com[1];
  rs.F_x_Cee[2] = st.ee_com[2];
  rs.tau_ext_hat_filtered = st.tau_ext_hat_filtered;
  rs.O_F_ext_hat_K = st.O_F_ext_hat_K;
  rs.K_F_ext_hat_K = st.K_F_ext_hat_K;
  rs.theta = st.q;
  rs.dtheta = st.dq;
  rs.control_command_success_rate = 1.0;
  return rs;
}

void FciSimEmbedder::Impl::HandleRobotCommand(const protocol::RobotCommand& cmd) {
  auto& st = GetSharedState();
  std::lock_guard<std::mutex> lock(st.mutex);
  st.tau_cmd = cmd.control.tau_J_d;
  st.q_cmd = cmd.motion.q_c;
  st.dq_cmd = cmd.motion.dq_c;
  st.O_T_EE_cmd = cmd.motion.O_T_EE_c;
  st.O_dP_EE_cmd = cmd.motion.O_dP_EE_c;
  st.elbow_cmd = cmd.motion.elbow_c;
  st.last_command_time = std::chrono::steady_clock::now();
  // Engage the appropriate controller branch purely based on requested motion/controller modes.
  st.has_torque_command = (st.current_controller_mode == protocol::Move::ControllerMode::kExternalController);
  st.has_position_command = (st.current_motion_mode == protocol::Move::MotionGeneratorMode::kJointPosition);
  st.has_velocity_command = (st.current_motion_mode == protocol::Move::MotionGeneratorMode::kJointVelocity);
  st.has_cartesian_command = (st.current_motion_mode == protocol::Move::MotionGeneratorMode::kCartesianPosition);
  st.has_cartesian_velocity_command = (st.current_motion_mode == protocol::Move::MotionGeneratorMode::kCartesianVelocity);
  // When motion_generation_finished, keep holding last joint position to mimic real robot behavior.
  if (cmd.motion.motion_generation_finished) {
    st.has_position_command = true;  // keep holding the last q_cmd
    st.has_velocity_command = false;
    st.has_cartesian_command = false;
    st.has_cartesian_velocity_command = false;
  }
}

// Public FciSimEmbedder
std::unique_ptr<FciSimEmbedder> FciSimEmbedder::Attach(
    const drake::multibody::MultibodyPlant<double>* plant,
    const drake::multibody::ModelInstanceIndex& robot_instance,
    drake::systems::DiagramBuilder<double>* builder,
    const FciSimOptions& options) {
  auto handle = std::unique_ptr<FciSimEmbedder>(new FciSimEmbedder(plant, robot_instance, builder, options));
  handle->ConfigureSystems();
  return handle;
}

std::unique_ptr<FciSimEmbedder> FciSimEmbedder::Attach(
    const drake::multibody::MultibodyPlant<double>* plant,
    const drake::multibody::ModelInstanceIndex& robot_instance,
    drake::geometry::SceneGraph<double>* scene_graph,
    drake::systems::DiagramBuilder<double>* builder,
    const FciSimOptions& options,
    bool disable_collisions) {
  auto handle = Attach(plant, robot_instance, builder, options);
  if (scene_graph != nullptr && disable_collisions) {
    // Replace full proximity removal with self-collision filtering only.
    ApplyRobotSelfCollisionFilter(plant, robot_instance, scene_graph);
  }
  return handle;
}

FciSimEmbedder::FciSimEmbedder(const drake::multibody::MultibodyPlant<double>* plant,
                               const drake::multibody::ModelInstanceIndex& robot_instance,
                               drake::systems::DiagramBuilder<double>* builder,
                               const FciSimOptions& options)
    : impl_(new Impl(plant, robot_instance, builder, options)) {}

FciSimEmbedder::~FciSimEmbedder() = default;

void FciSimEmbedder::ConfigureSystems() {
  auto [num_arm, num_total] = impl_->DetectEndEffectorConfiguration();
  impl_->ConfigureControllerSystems(num_arm, num_total);
}

void FciSimEmbedder::StartServer(uint16_t tcp_port, uint16_t udp_port) {
  if (impl_->server) return;
  impl_->server = std::make_unique<FrankaFciSimServer>(tcp_port, udp_port);
  impl_->server->set_state_provider([this]() { return impl_->MakeRobotStateSnapshot(); });
  impl_->server->set_command_handler([this](const protocol::RobotCommand& cmd) { impl_->HandleRobotCommand(cmd); });
  impl_->server->set_mode_change_handler([](protocol::Move::ControllerMode c, protocol::Move::MotionGeneratorMode m) {
    auto& st = GetSharedState();
    std::lock_guard<std::mutex> lock(st.mutex);
    st.current_controller_mode = c;
    st.current_motion_mode = m;
  });
  // Run in background thread
  std::thread([srv = impl_->server.get()](){ srv->run(); }).detach();

  // Start libfranka-compatible gripper server on its port
  using research_interface::gripper::kCommandPort;
  impl_->gripper_server = std::make_unique<FrankaGripperSimServer>(kCommandPort,
      [](){ return GetSharedState().gripper_q[0] + GetSharedState().gripper_q[1]; },
      [this](double width, double speed){ this->SetGripperWidth(width); /* speed handled internally */ },
      [this](){ this->SetGripperWidth(GetSharedState().gripper_max_width); },
      [this](double width, double speed, double force, double eps_out){ (void)eps_out; this->SetGripperWidth(width); /* force could scale gains if needed */ });
  std::thread([srv = impl_->gripper_server.get()](){ srv->run(); }).detach();
}

void FciSimEmbedder::StopServer() {
  if (impl_->server) {
    impl_->server->stop();
    impl_->server.reset();
  }
}

void FciSimEmbedder::SetModeChangeHandler(FrankaFciSimServer::ModeChangeHandler handler) {
  if (impl_->server) {
    impl_->server->set_mode_change_handler(std::move(handler));
  }
}

void FciSimEmbedder::RegisterGripper(const GripperSpec& spec) {
  auto& st = GetSharedState();
  std::lock_guard<std::mutex> lock(st.mutex);
  st.gripper_registered = true;
  st.gripper_joint_names[0] = spec.left_joint_name;
  st.gripper_joint_names[1] = spec.right_joint_name;
  st.gripper_min_width = spec.min_width_m;
  st.gripper_max_width = spec.max_width_m;
  st.gripper_kp = spec.kp;
  st.gripper_kd = spec.kd;
  st.gripper_left_sign = spec.left_sign;
  st.gripper_right_sign = spec.right_sign;
}

void FciSimEmbedder::SetGripperWidth(double width_m, double speed_mps) {
  auto& st = GetSharedState();
  std::lock_guard<std::mutex> lock(st.mutex);
  (void)speed_mps; // speed is not used in this immediate-set implementation
  const double w_des = std::clamp(width_m, st.gripper_min_width, st.gripper_max_width);
  st.gripper_target_width = w_des;  // set immediately so single calls take effect
  st.gripper_target_user_set = true;
}

double FciSimEmbedder::GetGripperWidth() const {
  auto& st = GetSharedState();
  std::lock_guard<std::mutex> lock(st.mutex);
  return st.gripper_q[0] + st.gripper_q[1];
}

std::unique_ptr<FciSimEmbedder> FciSimEmbedder::AutoAttach(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::systems::DiagramBuilder<double>* builder,
    AutoAttachOptions auto_options,
    FciSimOptions options) {
  using drake::multibody::ModelInstanceIndex;
  using drake::multibody::RevoluteJoint;

  // 1) Try to find an existing Franka model instance by probing for a known joint.
  ModelInstanceIndex robot_instance;
  bool found = false;
  // Drake doesn't expose a single call for all instance indices in older versions;
  // iterate over all joints and record the instance when we first see a known joint.
  try {
    for (drake::multibody::JointIndex j(0); j < plant->num_joints(); ++j) {
      const auto& joint = plant->get_joint(j);
      if (joint.name() == std::string("fer_joint1")) {
        robot_instance = joint.model_instance();
        found = true;
        break;
      }
    }
  } catch (const std::exception&) {
  }

  // 2) If not found, add a default model to the plant.
  if (!found) {
    if (plant->is_finalized()) {
      throw std::runtime_error("FciSimEmbedder::AutoAttach: plant is already finalized and no Franka model was found. Provide a mutable, non-finalized plant or pre-load the model.");
    }
    const std::string pkg_xml_in = auto_options.package_xml_path.empty() ? std::string("models/urdf/package.xml") : auto_options.package_xml_path;
    const std::string urdf_in = !auto_options.urdf_path_override.empty() ? auto_options.urdf_path_override
                                : (auto_options.prefer_gripper ? std::string("models/urdf/fer_drake_gripper_fixed.urdf")
                                                               : std::string("models/urdf/fer_drake_fingerless.urdf"));
    const std::string pkg_xml = ResolveModelPath(pkg_xml_in);
    const std::string urdf = ResolveModelPath(urdf_in);
    drake::multibody::Parser parser(plant);
    parser.package_map().AddPackageXml(pkg_xml);
    const auto added = parser.AddModels(urdf);
    if (added.empty()) {
      throw std::runtime_error("FciSimEmbedder::AutoAttach: failed to add default Franka model from URDF: " + urdf);
    }
    robot_instance = added[0];

    // Weld base to world if present.
    try {
      plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base", robot_instance));
    } catch (const std::exception&) {
      // If base frame isn't found, leave as-is; user may have already constrained it.
    }

    // Add simple actuators for the 7 arm joints so we can apply torques.
    struct J { const char* n; double eff; };
    const std::vector<J> act = {{"fer_joint1",87},{"fer_joint2",87},{"fer_joint3",87},
                                {"fer_joint4",87},{"fer_joint5",12},{"fer_joint6",87},{"fer_joint7",87}};
    for (const auto& a : act) {
      try {
        const auto& joint = plant->GetJointByName<RevoluteJoint>(a.n, robot_instance);
        plant->AddJointActuator(std::string(a.n) + "_act", joint, a.eff);
      } catch (const std::exception&) {
        // Skip if joint missing (e.g., variant models) — minimal actuation may already exist.
      }
    }

    // Add actuators for gripper prismatic joints if present
    try {
      const auto& jl = plant->GetJointByName<drake::multibody::PrismaticJoint>("fer_finger_joint1", robot_instance);
      plant->AddJointActuator(std::string("fer_finger_joint1_act"), jl, 100.0);
    } catch (const std::exception&) {}
    try {
      const auto& jr = plant->GetJointByName<drake::multibody::PrismaticJoint>("fer_finger_joint2", robot_instance);
      plant->AddJointActuator(std::string("fer_finger_joint2_act"), jr, 100.0);
    } catch (const std::exception&) {}

    // Set light default damping to all known arm joints.
    const std::vector<std::pair<const char*, double>> damp = {{"fer_joint1",1.0},{"fer_joint2",1.0},{"fer_joint3",1.0},
                                                              {"fer_joint4",1.0},{"fer_joint5",1.0},{"fer_joint6",1.0},{"fer_joint7",1.0}};
    for (const auto& [name, d] : damp) {
      try {
        plant->GetMutableJointByName<RevoluteJoint>(name, robot_instance).set_default_damping(d);
      } catch (const std::exception&) {
      }
    }

    // Finalize plant after modifications.
    plant->Finalize();
  } else {
    // If found and not finalized, ensure required actuators exist; otherwise leave as-is.
    if (!plant->is_finalized()) {
      struct J { const char* n; double eff; };
      const std::vector<J> act = {{"fer_joint1",87},{"fer_joint2",87},{"fer_joint3",87},
                                  {"fer_joint4",87},{"fer_joint5",12},{"fer_joint6",87},{"fer_joint7",87}};
      for (const auto& a : act) {
        try {
          const auto& joint = plant->GetJointByName<RevoluteJoint>(a.n, robot_instance);
          plant->AddJointActuator(std::string(a.n) + "_act", joint, a.eff);
        } catch (const std::exception&) {
        }
      }
      // Gripper actuators if present
      try {
        const auto& jl = plant->GetJointByName<drake::multibody::PrismaticJoint>("fer_finger_joint1", robot_instance);
        plant->AddJointActuator(std::string("fer_finger_joint1_act"), jl, 100.0);
      } catch (const std::exception&) {}
      try {
        const auto& jr = plant->GetJointByName<drake::multibody::PrismaticJoint>("fer_finger_joint2", robot_instance);
        plant->AddJointActuator(std::string("fer_finger_joint2_act"), jr, 100.0);
      } catch (const std::exception&) {}
      const std::vector<std::pair<const char*, double>> damp = {{"fer_joint1",1.0},{"fer_joint2",1.0},{"fer_joint3",1.0},
                                                                {"fer_joint4",1.0},{"fer_joint5",1.0},{"fer_joint6",1.0},{"fer_joint7",1.0}};
      for (const auto& [name, d] : damp) {
        try {
          plant->GetMutableJointByName<RevoluteJoint>(name, robot_instance).set_default_damping(d);
        } catch (const std::exception&) {
        }
      }
      // Weld base if not already constrained.
      try {
        plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base", robot_instance));
      } catch (const std::exception&) {}
      plant->Finalize();
    }
  }

  return FciSimEmbedder::Attach(plant, robot_instance, builder, options);
}

std::unique_ptr<FciSimEmbedder> FciSimEmbedder::AutoAttach(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::systems::DiagramBuilder<double>* builder) {
  return AutoAttach(plant, builder, AutoAttachOptions{}, FciSimOptions{});
}

std::unique_ptr<FciSimEmbedder> FciSimEmbedder::AutoAttach(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    drake::systems::DiagramBuilder<double>* builder,
    AutoAttachOptions auto_options,
    FciSimOptions options) {
  // Reuse core attach logic first.
  auto handle = AutoAttach(plant, builder, auto_options, options);

  // Optionally apply self-collision filtering (keep environment collisions).
  if (scene_graph != nullptr && auto_options.disable_collisions) {
    ApplyRobotSelfCollisionFilter(plant, handle->impl_->robot, scene_graph);
  }

  return handle;
}

void SetDefaultFrankaInitialState(drake::multibody::MultibodyPlant<double>& plant,
                                  drake::systems::Context<double>& plant_context) {
  // Determine model instance if single-model plant; otherwise apply to all instances that
  // expose the expected Franka joint names. We use the plant-wide APIs that accept instance.
  const int n_positions = plant.num_positions();
  const int n_velocities = plant.num_velocities();
  Eigen::VectorXd q_des = Eigen::VectorXd::Zero(n_positions);
  Eigen::VectorXd v_des = Eigen::VectorXd::Zero(n_velocities);

  // Best-effort: locate a model instance by probing a known joint name
  drake::multibody::ModelInstanceIndex instance;
  bool found = false;
  try {
    for (drake::multibody::JointIndex j(0); j < plant.num_joints(); ++j) {
      const auto& joint = plant.get_joint(j);
      if (joint.name() == std::string("fer_joint1")) {
        instance = joint.model_instance();
        found = true;
        break;
      }
    }
  } catch (const std::exception&) {
  }

  if (found) {
    const int n = plant.num_positions(instance);
    const int nv = plant.num_velocities(instance);
    Eigen::VectorXd qi = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd vi = Eigen::VectorXd::Zero(nv);
    if (n >= 7) {
      qi[0] = 0.0;
      qi[1] = -M_PI / 4.0;
      qi[2] = 0.0;
      qi[3] = -3.0 * M_PI / 4.0;
      qi[4] = 0.0;
      qi[5] = M_PI / 2.0;
      qi[6] = M_PI / 4.0;
    }
    if (n > 7) {
      for (int i = 7; i < n; ++i) qi[i] = 0.04; // open gripper if present
    }
    plant.SetPositions(&plant_context, instance, qi);
    plant.SetVelocities(&plant_context, instance, vi);
  } else {
    // Fallback: apply to whole-plant vectors if instance not found
    if (n_positions >= 7) {
      q_des[0] = 0.0;
      q_des[1] = -M_PI / 4.0;
      q_des[2] = 0.0;
      q_des[3] = -3.0 * M_PI / 4.0;
      q_des[4] = 0.0;
      q_des[5] = M_PI / 2.0;
      q_des[6] = M_PI / 4.0;
    }
    if (n_positions > 7) {
      for (int i = 7; i < n_positions; ++i) q_des[i] = 0.04;
    }
    plant.SetPositions(&plant_context, q_des);
    plant.SetVelocities(&plant_context, v_des);
  }
}

auto AutoBuildAndAttach(drake::systems::DiagramBuilder<double>* builder,
                        double time_step,
                        FciSimEmbedder::AutoAttachOptions auto_options,
                        FciSimOptions options) -> AutoBuildResult {
  using drake::multibody::AddMultibodyPlantSceneGraph;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(builder, time_step);
  auto embedder = FciSimEmbedder::AutoAttach(&plant, &scene_graph, builder, auto_options, options);
  return AutoBuildResult{&plant, &scene_graph, std::move(embedder)};
}

std::string ResolveModelPath(const std::string& relative_or_absolute) {
  namespace fs = std::filesystem;
  // Absolute path and exists
  if (fs::path(relative_or_absolute).is_absolute() && fs::exists(relative_or_absolute)) {
    return relative_or_absolute;
  }

  // Env overrides
  const char* models_env = std::getenv("FRANKA_DRAKE_MODELS_DIR");
  const char* root_env = std::getenv("FRANKA_DRAKE_ROOT");

  auto try_path = [](const fs::path& p) -> std::optional<std::string> {
    if (!p.empty() && fs::exists(p)) return p.string();
    return std::nullopt;
  };

  // If starts with models/, try $FRANKA_DRAKE_MODELS_DIR first
  if (relative_or_absolute.rfind("models/", 0) == 0 && models_env) {
    if (auto ok = try_path(fs::path(models_env) / fs::path(relative_or_absolute).lexically_relative("models")); ok) {
      return *ok;
    }
  }

  // Try FRANKA_DRAKE_ROOT
  if (root_env) {
    if (auto ok = try_path(fs::path(root_env) / relative_or_absolute); ok) return *ok;
  }

  // Try CWD
  if (auto ok = try_path(fs::current_path() / relative_or_absolute); ok) return *ok;

  // Try alongside executable
  // Get /proc/self/exe on Linux
  char exe_buf[4096] = {0};
  ssize_t len = ::readlink("/proc/self/exe", exe_buf, sizeof(exe_buf)-1);
  fs::path exe_dir;
  if (len > 0) {
    exe_buf[len] = '\0';
    exe_dir = fs::path(exe_buf).parent_path();
  }
  if (!exe_dir.empty()) {
    if (auto ok = try_path(exe_dir / relative_or_absolute); ok) return *ok;
    if (auto ok = try_path(exe_dir / ".." / relative_or_absolute); ok) return *ok;
    if (auto ok = try_path(exe_dir / ".." / ".." / relative_or_absolute); ok) return *ok;
  }

  return relative_or_absolute;  // best effort fallback
}

}  // namespace franka_fci_sim

