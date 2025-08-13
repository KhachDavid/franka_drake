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
#include <drake/math/rigid_transform.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/systems/controllers/inverse_dynamics.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/primitives/adder.h>
#include <drake/systems/primitives/demultiplexer.h>
#include <drake/systems/primitives/vector_log_sink.h>

#include "franka_drake/fci_sim_server.h"

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
};

SharedRobotState& GetSharedState() {
  static SharedRobotState* state = new SharedRobotState();
  return *state;
}

}  // namespace

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

  // Networking server
  std::unique_ptr<FrankaFciSimServer> server;

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
        plant->GetJointByName(jn, robot);
        if (state.num_gripper_joints < 2) {
          state.gripper_joint_names[state.num_gripper_joints++] = jn;
        }
      } catch (const std::exception&) {
      }
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
  builder->Connect(plant->get_state_output_port(), g_comp->get_input_port_estimated_state());

  // Full-featured impedance/velocity/cartesian/torque controller (parity with app).
  class ImpedanceWrapper final : public drake::systems::LeafSystem<double> {
   public:
    explicit ImpedanceWrapper(const drake::multibody::MultibodyPlant<double>* plant,
                              drake::multibody::ModelInstanceIndex robot,
                              int arm_joints, int total_joints)
        : plant_(plant), robot_(robot), arm_joints_(arm_joints), total_joints_(total_joints) {
      state_in_ = this->DeclareVectorInputPort("robot_state", drake::systems::BasicVector<double>(2 * total_joints_)).get_index();
      this->DeclareVectorOutputPort("control_torques", drake::systems::BasicVector<double>(arm_joints_),
                                    &ImpedanceWrapper::CalcTorques);
      plant_context_ = plant_->CreateDefaultContext();
      integral_errors_ = Eigen::VectorXd::Zero(arm_joints_);
      velocity_integral_errors_ = Eigen::VectorXd::Zero(arm_joints_);
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
      plant_->SetPositions(plant_context_.get(), robot_, q.head(total_joints_));
      plant_->SetVelocities(plant_context_.get(), robot_, dq.head(total_joints_));

      static const std::array<double, 7> torque_limits{60.0,60.0,60.0,60.0,35.0,35.0,30.0};
      static const double max_tau_rate = 500.0; // Nm/s
      static Eigen::VectorXd last_tau = Eigen::VectorXd::Zero(7);

      // TORQUE CONTROL: pass-through minus gravity
      if (st.has_torque_command) {
        Eigen::VectorXd tau_g = plant_->CalcGravityGeneralizedForces(*plant_context_);
        for (int i = 0; i < arm_joints_; ++i) {
          tau[i] = st.tau_cmd[i] - tau_g[i];
        }
        integral_errors_.setZero();
        velocity_integral_errors_.setZero();
      }
      // VELOCITY CONTROL
      else if (st.has_velocity_command && (std::chrono::duration<double>(std::chrono::steady_clock::now() - st.last_command_time).count() < 0.1)) {
        const double Kp[7] = {40,40,40,40,25,20,18};
        const double Ki[7] = {4,4,4,4,2.5,2.0,1.8};
        for (int i = 0; i < arm_joints_; ++i) {
          double e = st.dq_cmd[i] - dq[i];
          velocity_integral_errors_[i] = std::clamp(velocity_integral_errors_[i] + e * 0.001, -0.5, 0.5);
          tau[i] = Kp[i] * e + Ki[i] * velocity_integral_errors_[i];
        }
      }
      // CARTESIAN CONTROL (position or velocity)
      else if ((st.has_cartesian_command || st.has_cartesian_velocity_command) && (std::chrono::duration<double>(std::chrono::steady_clock::now() - st.last_command_time).count() < 0.1)) {
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
          const double Kpt = 50.0, Kpr = 15.0;
          Eigen::VectorXd v_ref(6);
          for (int i = 0; i < 6; ++i) v_ref[i] = st.O_dP_EE_cmd[i];
          Eigen::VectorXd v_err = v_ref - v_ee;
          F.head<3>() = Kpt * v_err.head<3>();
          F.tail<3>() = Kpr * v_err.tail<3>();
        }
        tau = J.transpose() * F;
      }
      // POSITION CONTROL (default hold/track)
      else {
        const double K[7] = {60,60,60,60,40,30,25};
        const double D[7] = {18,18,18,18,12,10,8};
        for (int i = 0; i < arm_joints_; ++i) {
          double e = st.q_cmd[i] - q[i];
          double v = dq[i];
          // No integral by default for stability
          tau[i] = K[i] * e - D[i] * v;
        }
      }

      // Torque clamp and rate limit
      for (int i = 0; i < arm_joints_; ++i) {
        double t = std::clamp(tau[i], -torque_limits[i], torque_limits[i]);
        double delta = t - last_tau[i];
        double max_delta = max_tau_rate * 0.001;
        if (delta > max_delta) t = last_tau[i] + max_delta;
        if (delta < -max_delta) t = last_tau[i] - max_delta;
        tau[i] = t;
        last_tau[i] = t;
      }

      out->SetFromVector(tau);
    }
    const drake::multibody::MultibodyPlant<double>* plant_;
    const drake::multibody::ModelInstanceIndex robot_;
    int arm_joints_;
    int total_joints_;
    int state_in_;
    mutable std::unique_ptr<drake::systems::Context<double>> plant_context_;
    mutable Eigen::VectorXd integral_errors_;
    mutable Eigen::VectorXd velocity_integral_errors_;
  };

  auto* ctrl = builder->AddSystem<ImpedanceWrapper>(plant, robot, num_arm_joints, num_total_joints);
  state_input_port_index = ctrl->state_in();
  builder->Connect(plant->get_state_output_port(), ctrl->get_input_port(state_input_port_index));

  // Add torque adder and connect to plant
  torque_adder = builder->AddSystem<drake::systems::Adder<double>>(2, num_arm_joints);
  if (num_total_joints > num_arm_joints) {
    auto* selector = builder->AddSystem<drake::systems::Demultiplexer<double>>(std::vector<int>{num_arm_joints, num_total_joints - num_arm_joints});
    builder->Connect(g_comp->get_output_port(), selector->get_input_port());
    builder->Connect(selector->get_output_port(0), torque_adder->get_input_port(0));
  } else {
    builder->Connect(g_comp->get_output_port(), torque_adder->get_input_port(0));
  }
  builder->Connect(ctrl->get_output_port(), torque_adder->get_input_port(1));
  builder->Connect(torque_adder->get_output_port(), plant->get_actuation_input_port());

  // State mirror system: periodically mirrors plant state into shared state for server.
  class StateMirror final : public drake::systems::LeafSystem<double> {
   public:
    StateMirror(const drake::multibody::MultibodyPlant<double>* plant,
                drake::multibody::ModelInstanceIndex robot,
                int total_joints)
        : plant_(plant), robot_(robot), total_joints_(total_joints) {
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
      // Update joint state
      for (int i = 0; i < 7 && i < q.size(); ++i) {
        st.q[i] = q[i];
        st.dq[i] = dq[i];
        st.dq_filtered[i] = 0.8 * st.dq_filtered[i] + 0.2 * st.dq[i];
        st.tau_J[i] = 0.1 * std::sin(q[i]);
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
        seeded = true;
      }
    }
    const drake::multibody::MultibodyPlant<double>* plant_;
    const drake::multibody::ModelInstanceIndex robot_;
    int total_joints_;
    int in_;
    mutable std::unique_ptr<drake::systems::Context<double>> plant_context_;
  };

  auto* mirror = builder->AddSystem<StateMirror>(plant, robot, num_total_joints);
  builder->Connect(plant->get_state_output_port(), mirror->get_input_port(0));
}

protocol::RobotState FciSimEmbedder::Impl::MakeRobotStateSnapshot() {
  auto& st = GetSharedState();
  std::lock_guard<std::mutex> lock(st.mutex);
  protocol::RobotState rs{};
  rs.q = st.q;
  rs.dq = st.dq_filtered;
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
  bool has_pos = false;
  for (int i = 0; i < 7; ++i) {
    if (std::abs(cmd.motion.q_c[i]) > 0.0) { has_pos = true; break; }
  }
  st.has_position_command = has_pos && st.current_motion_mode == protocol::Move::MotionGeneratorMode::kJointPosition;
  st.has_torque_command = (st.current_controller_mode == protocol::Move::ControllerMode::kExternalController);
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

}  // namespace franka_fci_sim

