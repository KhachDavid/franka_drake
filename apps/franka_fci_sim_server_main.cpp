#include "franka_drake/fci_sim_server.h"
#include <drake/common/find_resource.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/visualization/visualization_config_functions.h>
#include <drake/geometry/meshcat.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/systems/controllers/inverse_dynamics.h>
#include <drake/systems/primitives/adder.h>
#include <drake/systems/primitives/demultiplexer.h>
#include <drake/systems/analysis/runge_kutta5_integrator.h>
#include <drake/math/rigid_transform.h>
#include <drake/math/rotation_matrix.h>
#include <drake/multibody/math/spatial_algebra.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <array>
#include <vector>
#include <string>
#include <mutex>
#include <iomanip> // Required for std::fixed and std::setprecision
#include <cmath>   // For sin function in gravity torque calculation

// Shared state between Drake simulation and FCI server
struct SharedRobotState {
  std::mutex mutex;
  std::array<double, 7> q{};      // Joint positions
  std::array<double, 7> dq{};     // Joint velocities  
  std::array<double, 7> dq_filtered{};  // Filtered joint velocities
  std::array<double, 7> tau_J{};  // Joint torques
  std::array<double, 16> O_T_EE{}; // End-effector pose
  std::array<double, 7> tau_cmd{}; // Commanded torques from FCI
  std::array<double, 7> q_cmd{};   // Commanded positions from FCI  
  std::array<double, 7> dq_cmd{}; // Commanded velocities from FCI
  std::array<double, 16> O_T_EE_cmd{}; // Commanded EE pose from FCI
  std::array<double, 6> O_dP_EE_cmd{};  // Commanded EE velocity (twist) from FCI
  std::array<double, 2> elbow{};  // Current elbow configuration [position, sign]
  std::array<double, 2> elbow_cmd{};  // Commanded elbow configuration
  std::array<double, 7> tau_ext_hat_filtered{};  // Filtered external torques
  std::array<double, 6> O_F_ext_hat_K{};  // External wrench in base frame
  std::array<double, 6> K_F_ext_hat_K{};  // External wrench in stiffness frame
  bool has_position_command = false;
  bool has_velocity_command = false;
  bool has_torque_command = false;
  bool has_cartesian_command = false;
  bool has_cartesian_velocity_command = false;
  // Track current controller mode for proper torque detection
  franka_fci_sim::protocol::Move::ControllerMode current_controller_mode = franka_fci_sim::protocol::Move::ControllerMode::kExternalController;
  franka_fci_sim::protocol::Move::MotionGeneratorMode current_motion_mode = franka_fci_sim::protocol::Move::MotionGeneratorMode::kJointPosition;
  
  // Command timeout tracking
  std::chrono::steady_clock::time_point last_command_time = std::chrono::steady_clock::now();
  
  // END EFFECTOR CONFIGURATION
  std::string ee_frame_name = "fer_link7";  // Default to fingerless
  bool has_gripper = false;                 // Whether gripper is present
  double ee_mass = 0.35973;                 // Mass of end effector
  std::array<double, 3> ee_com{0.00089, -0.00044, 0.05491}; // Center of mass offset
  std::array<double, 9> ee_inertia{0.00019541063, 1.65231e-06, 1.48826e-06,
                                   1.65231e-06, 0.00019210361, -1.31132e-06,
                                   1.48826e-06, -1.31132e-06, 0.00017936256}; // Inertia tensor
  
  // GRIPPER STATE (if present)
  std::array<double, 2> gripper_q{0.04, 0.04};    // Gripper joint positions (open)
  std::array<double, 2> gripper_dq{0.0, 0.0};     // Gripper joint velocities
  std::array<std::string, 2> gripper_joint_names{"", ""}; // Gripper joint names
  int num_gripper_joints = 0;                     // Number of gripper joints found
};

// Global shared state
SharedRobotState g_robot_state;
std::atomic<bool> g_sim_running{true};

// Simulation Runner class for cleaner, modular simulation loop
class SimulationRunner {
public:
  SimulationRunner(drake::systems::Simulator<double>& sim, 
                   const drake::multibody::MultibodyPlant<double>& plant,
                   const drake::multibody::ModelInstanceIndex& robot,
                   bool headless = false)
      : simulator_(sim), plant_(plant), robot_(robot), headless_(headless) {
    sim_start_time_ = std::chrono::steady_clock::now();
    last_log_time_ = sim_start_time_;
    
    // Default to real-time simulation (RTF = 1.0)
    simulator_.set_target_realtime_rate(1.0);
  }
  
  void EnableTurboMode() {
    turbo_mode_ = true;
    simulator_.set_target_realtime_rate(0.0);  // No real-time constraint
    std::cout << "TURBO MODE ENABLED - Running faster than real-time!" << std::endl;
  }
  
  void RunStep() {
    const double sim_step = 0.001;  // 1ms simulation step
    
    if (turbo_mode_) {
      // TURBO MODE: Run as fast as possible
      simulator_.AdvanceTo(simulator_.get_context().get_time() + sim_step);
    } else {
      // REAL-TIME MODE: Maintain RTF = 1.0
      auto current_time = std::chrono::steady_clock::now();
      double elapsed = std::chrono::duration<double>(current_time - sim_start_time_).count();
      double target_sim_time = elapsed;
      
      if (simulator_.get_context().get_time() < target_sim_time) {
        simulator_.AdvanceTo(std::min(simulator_.get_context().get_time() + sim_step, target_sim_time));
      }
    }
    
    // Update shared state periodically (every 10ms to reduce overhead)
    if (++step_count_ % 10 == 0) {
      UpdateSharedState();
    }
    
    // Log periodically
    auto now = std::chrono::steady_clock::now();
    double log_interval = headless_ ? 0.5 : 2.0;  // 500ms for headless, 2s for visual
    if (std::chrono::duration<double>(now - last_log_time_).count() >= log_interval) {
      LogStatus();
      last_log_time_ = now;
    }
  }
  
  bool ShouldContinue() const {
    return g_sim_running.load();
  }
  
private:
  void UpdateSharedState() {
    std::lock_guard<std::mutex> lock(g_robot_state.mutex);
    auto& context = simulator_.get_context();
    auto& plant_context = plant_.GetMyContextFromRoot(context);
    
    const std::vector<std::string> joint_names = {
        "fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4",
        "fer_joint5", "fer_joint6", "fer_joint7"
    };
    
    // Sign mapping between Drake plant joint convention and libfranka convention
    static const std::array<double, 7> plant_to_franka_sign{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

    // Update arm joint states
    for (int i = 0; i < 7; ++i) {
      const auto& joint = plant_.GetJointByName<drake::multibody::RevoluteJoint>(joint_names[i], robot_);
      const double q_pl = joint.get_angle(plant_context);
      const double dq_pl = joint.get_angular_rate(plant_context);
      g_robot_state.q[i] = plant_to_franka_sign[i] * q_pl;
      g_robot_state.dq[i] = plant_to_franka_sign[i] * dq_pl;
      
      // Apply simple first-order low-pass filter for velocity
      const double alpha = 0.8;  // Filter coefficient (0 = no filtering, 1 = no update)
      g_robot_state.dq_filtered[i] = alpha * g_robot_state.dq_filtered[i] + (1 - alpha) * g_robot_state.dq[i];
      
      // Very rough torque placeholder (kept); align sign to franka space
      g_robot_state.tau_J[i] = plant_to_franka_sign[i] * (0.1 * sin(q_pl));
    }
    
    // Update gripper states if present
    if (g_robot_state.has_gripper && g_robot_state.num_gripper_joints > 0) {
      for (int i = 0; i < g_robot_state.num_gripper_joints; ++i) {
        try {
          const auto& joint = plant_.GetJointByName(g_robot_state.gripper_joint_names[i], robot_);
          if (joint.num_positions() == 1) {
            // Map Drake finger joints to Franka order (1: left, 2: right)
            const int idx = (g_robot_state.gripper_joint_names[i].find("finger_joint1") != std::string::npos) ? 0 : 1;
            g_robot_state.gripper_q[idx] = plant_.GetPositions(plant_context, robot_)[7 + i];
            g_robot_state.gripper_dq[idx] = plant_.GetVelocities(plant_context, robot_)[7 + i];
          }
        } catch (const std::exception&) {
          // Keep previous values if read fails
        }
      }
    }
    
    // Update end-effector pose
    const auto& ee_frame = plant_.GetFrameByName(g_robot_state.ee_frame_name, robot_);
    const auto& world_frame = plant_.world_frame();
    const drake::math::RigidTransformd X_W_EE = plant_.CalcRelativeTransform(plant_context, world_frame, ee_frame);
    const Eigen::Matrix4d T_matrix = X_W_EE.GetAsMatrix4();
    
    // Store as column-major array
    for (int col = 0; col < 4; ++col) {
      for (int row = 0; row < 4; ++row) {
        g_robot_state.O_T_EE[col * 4 + row] = T_matrix(row, col);
      }
    }
    
    // Calculate elbow configuration
    // The elbow position is defined as the angle of joint 3
    // The elbow sign indicates whether the elbow is "up" (+1) or "down" (-1)
    // This is a simplified calculation based on the joint configuration
    g_robot_state.elbow[0] = g_robot_state.q[2];  // Joint 3 position
    
    // Determine elbow sign based on joint 4 position
    // When joint 4 is negative (typical pose), elbow is considered "up" (+1)
    // When joint 4 is positive, elbow is "down" (-1)
    g_robot_state.elbow[1] = (g_robot_state.q[3] < 0) ? 1.0 : -1.0;
  }
  
  void LogStatus() {
    std::lock_guard<std::mutex> lock(g_robot_state.mutex);
    
    double sim_time = simulator_.get_context().get_time();
    double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - sim_start_time_).count();
    double rtf = sim_time / elapsed;
    
    if (headless_) {
      // Compact headless logging
      std::cout << "[HEADLESS] t=" << std::fixed << std::setprecision(3) << sim_time << "s ";
      std::cout << "q=[";
      for (int i = 0; i < 7; ++i) {
        std::cout << std::fixed << std::setprecision(4) << g_robot_state.q[i];
        if (i < 6) std::cout << ", ";
      }
      std::cout << "] RTF=" << std::fixed << std::setprecision(2) << rtf << "x";
      if (turbo_mode_) std::cout << " [TURBO]";
      std::cout << std::endl;
    } else {
      // Visual mode status
      std::cout << "[Drake] Sim time: " << std::fixed << std::setprecision(3) << sim_time << "s";
      std::cout << " RTF: " << std::fixed << std::setprecision(2) << rtf << "x";
      if (turbo_mode_) std::cout << " [TURBO]";
      std::cout << " | Commands: pos=" << g_robot_state.has_position_command 
                << " tau=" << g_robot_state.has_torque_command;
      std::cout << " | EE_z=" << std::fixed << std::setprecision(4) << g_robot_state.O_T_EE[14];
      std::cout << std::endl;
    }
  }
  
  drake::systems::Simulator<double>& simulator_;
  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::ModelInstanceIndex& robot_;
  bool headless_;
  bool turbo_mode_ = false;
  int step_count_ = 0;
  std::chrono::steady_clock::time_point sim_start_time_;
  std::chrono::steady_clock::time_point last_log_time_;
};

// DETECT END EFFECTOR CONFIGURATION
std::pair<int, int> DetectEndEffectorConfiguration(const drake::multibody::MultibodyPlant<double>& plant, 
                                                  const drake::multibody::ModelInstanceIndex& robot) {
  std::lock_guard<std::mutex> lock(g_robot_state.mutex);
  
  // Count total joints in the robot
  int num_arm_joints = 7;  // Standard Franka arm joints
  int num_total_joints = plant.num_positions(robot);
  
  // Try to find different end effector frames in order of preference
  std::vector<std::string> frame_candidates = {
    "fer_hand_tcp",    // Tool center point (preferred for full gripper)
    "fer_link8",       // Flange frame (alternative for full gripper)
    "fer_hand",        // Hand frame (basic gripper)
    "fer_link7"        // Wrist frame (fingerless configuration)
  };
  
  for (const auto& frame_name : frame_candidates) {
    try {
      plant.GetFrameByName(frame_name, robot);
      g_robot_state.ee_frame_name = frame_name;
      std::cout << "[EE CONFIG] Found end effector frame: " << frame_name << std::endl;
      break;
    } catch (const std::exception&) {
      // Frame not found, continue searching
      continue;
    }
  }
  
  // Detect if we have a gripper by checking for gripper frames
  try {
    plant.GetFrameByName("fer_hand", robot);
    g_robot_state.has_gripper = true;
    std::cout << "[EE CONFIG] Gripper detected!" << std::endl;
    
    // Detect gripper joints
    std::vector<std::string> gripper_candidates = {"fer_finger_joint1", "fer_finger_joint2"};
    g_robot_state.num_gripper_joints = 0;
    
    for (const auto& joint_name : gripper_candidates) {
      try {
        plant.GetJointByName(joint_name, robot);
        if (g_robot_state.num_gripper_joints < 2) {
          g_robot_state.gripper_joint_names[g_robot_state.num_gripper_joints] = joint_name;
          g_robot_state.num_gripper_joints++;
          std::cout << "[EE CONFIG] Found gripper joint: " << joint_name << std::endl;
        }
      } catch (const std::exception&) {
        // Joint not found, continue
        continue;
      }
    }
    
    // Set gripper-specific parameters
    if (g_robot_state.ee_frame_name == "fer_hand_tcp") {
      // Franka Hand + TCP parameters (Hand mass + offset to TCP)
      g_robot_state.ee_mass = 0.73;  // Franka Hand mass
      g_robot_state.ee_com = {0.0, 0.0, 0.058};  // TCP is ~58mm above hand
      g_robot_state.ee_inertia = {0.001, 0.0, 0.0,
                                  0.0, 0.0025, 0.0,
                                  0.0, 0.0, 0.0017}; // Hand inertia
    } else if (g_robot_state.ee_frame_name == "fer_hand") {
      // Pure Franka Hand parameters
      g_robot_state.ee_mass = 0.73;
      g_robot_state.ee_com = {-0.01, 0.0, 0.03};  // Hand COM offset
      g_robot_state.ee_inertia = {0.001, 0.0, 0.0,
                                  0.0, 0.0025, 0.0,
                                  0.0, 0.0, 0.0017};
    }
  } catch (const std::exception&) {
    g_robot_state.has_gripper = false;
    g_robot_state.num_gripper_joints = 0;
    std::cout << "[EE CONFIG] No gripper detected - fingerless configuration" << std::endl;
    
    // Fingerless configuration - use link7 parameters
    g_robot_state.ee_mass = 0.35973;  // Just the wrist
    g_robot_state.ee_com = {0.00089, -0.00044, 0.05491};
    g_robot_state.ee_inertia = {0.00019541063, 1.65231e-06, 1.48826e-06,
                                1.65231e-06, 0.00019210361, -1.31132e-06,
                                1.48826e-06, -1.31132e-06, 0.00017936256};
  }
  
  std::cout << "[EE CONFIG] Final configuration:" << std::endl;
  std::cout << "  Frame: " << g_robot_state.ee_frame_name << std::endl;
  std::cout << "  Has gripper: " << (g_robot_state.has_gripper ? "YES" : "NO") << std::endl;
  std::cout << "  Gripper joints: " << g_robot_state.num_gripper_joints << std::endl;
  std::cout << "  Total joints: " << num_total_joints << " (arm: " << num_arm_joints << ")" << std::endl;
  std::cout << "  Mass: " << g_robot_state.ee_mass << " kg" << std::endl;
  std::cout << "  COM: [" << g_robot_state.ee_com[0] << ", " << g_robot_state.ee_com[1] << ", " << g_robot_state.ee_com[2] << "]" << std::endl;
  
  return {num_arm_joints, num_total_joints};
}

// Drake system to provide FCI commands as input - SIMPLE IMPEDANCE CONTROLLER
class FciImpedanceController final : public drake::systems::LeafSystem<double> {
 public:
  FciImpedanceController(const drake::multibody::MultibodyPlant<double>* plant,
                        const drake::multibody::ModelInstanceIndex& robot_instance)
      : plant_(plant), robot_instance_(robot_instance) {
    // We'll declare ports after we know the robot configuration
    // This will be updated after robot detection
    expected_num_arm_joints_ = 7;  // Default to arm joints only
    expected_num_total_joints_ = 7;
    
    // Initialize integral errors
    integral_errors_.setZero(7);
    velocity_integral_errors_.setZero(7);
    
    std::cout << "[Impedance Controller] Initialized" << std::endl;
  }
  
  // Configure the controller for the detected robot configuration
  void ConfigureForRobot(int num_arm_joints, int num_total_joints) {
    expected_num_arm_joints_ = num_arm_joints;
    expected_num_total_joints_ = num_total_joints;
    
    // Declare input ports for current robot state (positions + velocities)
    int state_size = 2 * num_total_joints;  // positions + velocities
    state_input_port_ = this->DeclareVectorInputPort("robot_state", drake::systems::BasicVector<double>(state_size)).get_index();
    
    // Declare output port for computed control torques (only for arm joints)
    this->DeclareVectorOutputPort("control_torques", drake::systems::BasicVector<double>(num_arm_joints),
                                 &FciImpedanceController::CalcControlTorques);
    
    // Create a context for the plant that we'll update each time
    plant_context_ = plant_->CreateDefaultContext();
    
    std::cout << "[Impedance Controller] Configured for " << num_total_joints 
              << " total joints (" << num_arm_joints << " arm joints)" << std::endl;
  }

   // Public accessor for state input port
   int get_state_input_port() const { return state_input_port_; }
   


 private:
   void CalcControlTorques(const drake::systems::Context<double>& context,
                          drake::systems::BasicVector<double>* output) const {
     std::lock_guard<std::mutex> lock(g_robot_state.mutex);
     
     Eigen::VectorXd control_torques(expected_num_arm_joints_); // Use expected_num_arm_joints_
     
      // IMPORTANT: Check torque control mode FIRST - no PID calculations at all!
      static const std::array<double, 7> plant_to_franka_sign{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
      if (g_robot_state.has_torque_command) {
        // PURE TORQUE CONTROL: Pass-through, but compensate for outer gravity add
        // Evaluate current state and compute gravity torques for the arm, then subtract them
        const auto& state_input = this->get_input_port(state_input_port_).Eval(context);
        Eigen::VectorXd current_q = state_input.head(expected_num_total_joints_);
        Eigen::VectorXd current_dq = state_input.tail(expected_num_total_joints_);

        plant_->SetPositions(plant_context_.get(), robot_instance_, current_q.head(expected_num_total_joints_));
        plant_->SetVelocities(plant_context_.get(), robot_instance_, current_dq.head(expected_num_total_joints_));

        Eigen::VectorXd tau_g = plant_->CalcGravityGeneralizedForces(*plant_context_);

         for (int i = 0; i < expected_num_arm_joints_; ++i) {
          // Convert commanded torques from Franka convention to plant, then subtract gravity
          const double tau_cmd_pl = plant_to_franka_sign[i] * g_robot_state.tau_cmd[i];
          control_torques[i] = tau_cmd_pl - tau_g[i];
        }

        // Reset integral errors when in torque mode
        integral_errors_.setZero();
        velocity_integral_errors_.setZero();

        // Apply torque limits for safety and output immediately
        const double torque_limits[7] = {87.0, 87.0, 87.0, 87.0, 50.0, 50.0, 40.0};
        for (int i = 0; i < expected_num_arm_joints_; ++i) {
          control_torques[i] = std::max(-torque_limits[i], std::min(torque_limits[i], control_torques[i]));
        }

        output->SetFromVector(control_torques);
        return;  // EARLY RETURN - No PID calculations in torque mode!
      }
     
     // Check if commands have timed out (no new commands for 100ms)
     auto now = std::chrono::steady_clock::now();
     double time_since_last_cmd = std::chrono::duration<double>(now - g_robot_state.last_command_time).count();
     bool commands_active = time_since_last_cmd < 0.1; // 100ms timeout
     
      // VELOCITY CONTROL: PI control on velocity (more damping, clamp output)
      if (g_robot_state.has_velocity_command && commands_active) {
       // Get current robot state from input
       const auto& state_input = this->get_input_port(state_input_port_).Eval(context);
       Eigen::VectorXd current_q = state_input.head(expected_num_total_joints_);
       Eigen::VectorXd current_dq = state_input.tail(expected_num_total_joints_);
       
       // Target velocities from FCI commands
       Eigen::VectorXd target_dq(expected_num_arm_joints_);
        for (int i = 0; i < expected_num_arm_joints_; ++i) {
          // Convert target velocities to plant convention
          target_dq[i] = plant_to_franka_sign[i] * g_robot_state.dq_cmd[i];
        }
       
       // Velocity control gains
        const double velocity_p_gains[7] = {40, 40, 40, 40, 25, 20, 18};  // N⋅m⋅s/rad
        const double velocity_i_gains[7] = {4, 4, 4, 4, 2.5, 2.0, 1.8};    // N⋅m⋅s²/rad
       
       // Calculate control torques: τ = Kp*(dq_d - dq) + Ki*∫(dq_d - dq)dt
        for (int i = 0; i < expected_num_arm_joints_; ++i) {
         double velocity_error = target_dq[i] - current_dq[i];
         
         // Update integral error with anti-windup
         velocity_integral_errors_[i] += velocity_error * 0.001; // dt = 0.001s
         const double max_integral = 0.5; // Limit integral contribution
         velocity_integral_errors_[i] = std::max(-max_integral, std::min(max_integral, velocity_integral_errors_[i]));
         
         control_torques[i] = velocity_p_gains[i] * velocity_error + 
                              velocity_i_gains[i] * velocity_integral_errors_[i];
       }
       
       // Debug output
       static int vel_ctrl_count = 0;
       if (++vel_ctrl_count <= 10 || vel_ctrl_count % 1000 == 0) {
         std::cout << "[Velocity Control] Iteration " << vel_ctrl_count << ":" << std::endl;
         std::cout << "  Target dq[6]: " << std::fixed << std::setprecision(4) << target_dq[6] << " rad/s" << std::endl;
         std::cout << "  Current dq[6]: " << std::fixed << std::setprecision(4) << current_dq[6] << " rad/s" << std::endl;
         std::cout << "  Error[6]: " << std::fixed << std::setprecision(4) << (target_dq[6] - current_dq[6]) << " rad/s" << std::endl;
         std::cout << "  Torque[6]: " << std::fixed << std::setprecision(4) << control_torques[6] << " Nm" << std::endl;
       }
     }
     // CARTESIAN CONTROL: Control in task space (position or velocity)
     else if ((g_robot_state.has_cartesian_command || g_robot_state.has_cartesian_velocity_command) && commands_active) {
       // Get current robot state from input
       const auto& state_input = this->get_input_port(state_input_port_).Eval(context);
       Eigen::VectorXd current_q = state_input.head(expected_num_total_joints_);
       Eigen::VectorXd current_dq = state_input.tail(expected_num_total_joints_);
       
       // Update plant context with current state
        plant_->SetPositions(plant_context_.get(), robot_instance_, current_q.head(expected_num_total_joints_));
        plant_->SetVelocities(plant_context_.get(), robot_instance_, current_dq.head(expected_num_total_joints_));
       
       // Get current end-effector pose
       const auto& ee_frame = plant_->GetFrameByName(g_robot_state.ee_frame_name, robot_instance_);
       const auto& world_frame = plant_->world_frame();
       const drake::math::RigidTransformd X_W_EE = plant_->CalcRelativeTransform(*plant_context_, world_frame, ee_frame);
       
       // Get Jacobian for the end-effector
       Eigen::MatrixXd J_ee(6, expected_num_arm_joints_);
       plant_->CalcJacobianSpatialVelocity(*plant_context_, drake::multibody::JacobianWrtVariable::kQDot,
                                           ee_frame, Eigen::Vector3d::Zero(), world_frame, world_frame, &J_ee);
       
       Eigen::VectorXd task_space_error(6);
       
       if (g_robot_state.has_cartesian_command) {
         // CARTESIAN POSITION CONTROL
         // Convert commanded pose from array to transformation matrix
         Eigen::Matrix4d T_cmd;
         for (int i = 0; i < 4; ++i) {
           for (int j = 0; j < 4; ++j) {
             T_cmd(j, i) = g_robot_state.O_T_EE_cmd[i * 4 + j];  // Column-major
           }
         }
         drake::math::RigidTransformd X_W_EE_cmd(T_cmd);
         
         // Compute position error
         Eigen::Vector3d position_error = X_W_EE_cmd.translation() - X_W_EE.translation();
         
         // Compute orientation error using angle-axis
         drake::math::RotationMatrixd R_error = X_W_EE_cmd.rotation() * X_W_EE.rotation().inverse();
         Eigen::AngleAxisd angle_axis(R_error.matrix());
         Eigen::Vector3d orientation_error = angle_axis.angle() * angle_axis.axis();
         
         // Combine errors
         task_space_error.head<3>() = position_error;
         task_space_error.tail<3>() = orientation_error;
         
         // Cartesian impedance gains
         const double cart_stiffness_trans = 100.0;  // N/m
         const double cart_stiffness_rot = 30.0;     // Nm/rad
         const double cart_damping_trans = 20.0;     // N⋅s/m
         const double cart_damping_rot = 10.0;       // Nm⋅s/rad
         
         // Task space force/torque
         Eigen::VectorXd F_task(6);
         F_task.head<3>() = cart_stiffness_trans * position_error;
         F_task.tail<3>() = cart_stiffness_rot * orientation_error;
         
         // Add damping in task space
         Eigen::VectorXd v_ee = J_ee * current_dq.head(expected_num_arm_joints_);
         F_task.head<3>() -= cart_damping_trans * v_ee.head<3>();
         F_task.tail<3>() -= cart_damping_rot * v_ee.tail<3>();
         
         // Convert to joint torques using Jacobian transpose
         control_torques = J_ee.transpose() * F_task;
         
         // Debug output
         static int cart_pos_ctrl_count = 0;
         if (++cart_pos_ctrl_count <= 10 || cart_pos_ctrl_count % 1000 == 0) {
           std::cout << "[Cartesian Position Control] Iteration " << cart_pos_ctrl_count << ":" << std::endl;
           std::cout << "  Position error: [" << position_error.transpose() << "] m" << std::endl;
           std::cout << "  Orientation error: [" << orientation_error.transpose() << "] rad" << std::endl;
           std::cout << "  Task force norm: " << F_task.head<3>().norm() << " N" << std::endl;
         }
       } else if (g_robot_state.has_cartesian_velocity_command) {
         // CARTESIAN VELOCITY CONTROL
         // Target velocity (twist) from FCI commands
         Eigen::VectorXd target_twist(6);
         for (int i = 0; i < 6; ++i) {
           target_twist[i] = g_robot_state.O_dP_EE_cmd[i];
         }
         
         // Current end-effector velocity
         Eigen::VectorXd current_twist = J_ee * current_dq.head(expected_num_arm_joints_);
         
         // Velocity error
         Eigen::VectorXd velocity_error = target_twist - current_twist;
         
         // Cartesian velocity gains
         const double cart_vel_p_trans = 50.0;  // N⋅s/m
         const double cart_vel_p_rot = 15.0;    // Nm⋅s/rad
         
         // Task space force/torque
         Eigen::VectorXd F_task(6);
         F_task.head<3>() = cart_vel_p_trans * velocity_error.head<3>();
         F_task.tail<3>() = cart_vel_p_rot * velocity_error.tail<3>();
         
         // Convert to joint torques
         control_torques = J_ee.transpose() * F_task;
         
         // Debug output
         static int cart_vel_ctrl_count = 0;
         if (++cart_vel_ctrl_count <= 10 || cart_vel_ctrl_count % 1000 == 0) {
           std::cout << "[Cartesian Velocity Control] Iteration " << cart_vel_ctrl_count << ":" << std::endl;
           std::cout << "  Target twist: [" << target_twist.transpose() << "]" << std::endl;
           std::cout << "  Velocity error norm: " << velocity_error.norm() << std::endl;
         }
       }
     }
      // POSITION CONTROL: PID control on position  
      else if (g_robot_state.has_position_command && commands_active) {
       // POSITION CONTROL: Simple joint impedance control
       // Get current robot state from input
       const auto& state_input = this->get_input_port(state_input_port_).Eval(context);
       Eigen::VectorXd current_q = state_input.head(expected_num_total_joints_);
       Eigen::VectorXd current_dq = state_input.tail(expected_num_total_joints_);
      
      // Target positions from FCI commands
      Eigen::VectorXd target_q(expected_num_arm_joints_);
      for (int i = 0; i < expected_num_arm_joints_; ++i) {
        // Convert target positions to plant convention
        target_q[i] = plant_to_franka_sign[i] * g_robot_state.q_cmd[i];
      }
      
       // Safer default impedance values to avoid oscillations under outer planners
       const double stiffness[7] = {60, 60, 60, 60, 40, 30, 25};   // N⋅m/rad
       const double damping[7]   = {18, 18, 18, 18, 12, 10, 8};    // N⋅m⋅s/rad

       // Disable integral by default for stability with MoveIt position commands
       const double integral_gains[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      
      // Calculate control torques: τ = K*(q_d - q) + Ki*∫(q_d - q)dt - D*dq
      for (int i = 0; i < expected_num_arm_joints_; ++i) {
        double position_error = target_q[i] - current_q[i];
        
        // Update integral error with anti-windup
        integral_errors_[i] += position_error * 0.001; // dt = 0.001s
         const double max_integral = 0.02; // Tighter anti-windup
        integral_errors_[i] = std::max(-max_integral, std::min(max_integral, integral_errors_[i]));
        
        // Limit velocity to reasonable values (safety check)
        double limited_velocity = current_dq[i];
        const double max_velocity = 1.5; // rad/s
        if (std::abs(limited_velocity) > max_velocity) {
          limited_velocity = (limited_velocity > 0) ? max_velocity : -max_velocity;
        }

        control_torques[i] = stiffness[i] * position_error +
                              integral_gains[i] * integral_errors_[i] -
                              damping[i] * limited_velocity;
      }
      
      // Debug output
      static int pos_ctrl_count = 0;
      if (++pos_ctrl_count <= 10 || pos_ctrl_count % 1000 == 0) {
        std::cout << "[Position Control] Iteration " << pos_ctrl_count << ":" << std::endl;
        
        // Print ALL joint targets and errors
        std::cout << "  Target q: [";
        for (int i = 0; i < expected_num_arm_joints_; ++i) {
          std::cout << std::fixed << std::setprecision(3) << target_q[i] << (i < 6 ? ", " : "");
        }
        std::cout << "]" << std::endl;
        
        std::cout << "  Current q: [";
        for (int i = 0; i < expected_num_arm_joints_; ++i) {
          std::cout << std::fixed << std::setprecision(3) << current_q[i] << (i < 6 ? ", " : "");
        }
        std::cout << "]" << std::endl;
        
        std::cout << "  Errors: [";
        for (int i = 0; i < expected_num_arm_joints_; ++i) {
          std::cout << std::fixed << std::setprecision(3) << (target_q[i] - current_q[i]) << (i < 6 ? ", " : "");
        }
        std::cout << "]" << std::endl;
        
        std::cout << "  Torques: [";
        for (int i = 0; i < expected_num_arm_joints_; ++i) {
          std::cout << std::fixed << std::setprecision(2) << control_torques[i] << (i < 6 ? ", " : "");
        }
        std::cout << "]" << std::endl;
      }
       
     } else {
       // NO COMMANDS OR TIMED OUT: Zero torques (gravity compensation will handle this)
       control_torques.setZero();
       
       // Reset integral errors when idle
       integral_errors_.setZero();
       velocity_integral_errors_.setZero();
       
       // Log when commands stop
       static bool was_active = false;
       if (!commands_active && was_active) {
         std::cout << "[Controller] Commands stopped - holding position with gravity compensation only" << std::endl;
       }
       was_active = commands_active;
     }
     
      // Apply torque limits and rate limits for safety
      const double torque_limits[7] = {60.0, 60.0, 60.0, 60.0, 35.0, 35.0, 30.0};
      static Eigen::VectorXd last_tau = Eigen::VectorXd::Zero(7);
      const double max_tau_rate = 500.0; // Nm/s equivalent (dt=1ms => 0.5 Nm step)
      for (int i = 0; i < expected_num_arm_joints_; ++i) {
        // Clamp absolute torque
        double tau = std::max(-torque_limits[i], std::min(torque_limits[i], control_torques[i]));
        // Rate limit
        double delta = tau - last_tau[i];
        double max_delta = max_tau_rate * 0.001;
        if (delta > max_delta) tau = last_tau[i] + max_delta;
        if (delta < -max_delta) tau = last_tau[i] - max_delta;
        control_torques[i] = tau;
        last_tau[i] = tau;
      }
     
     // Log limited torque for debugging (check if we were in position control)
     static int debug_count = 0;
     if (g_robot_state.has_position_command && (++debug_count <= 10 || debug_count % 1000 == 0)) {
       std::cout << "  Limited torque[6]: " << std::fixed << std::setprecision(4) << control_torques[6] << " Nm" << std::endl;
     }
     
     output->SetFromVector(control_torques);
   }
   
   // Member variables
   int state_input_port_;
   
   // Robot configuration
   int expected_num_arm_joints_;    // Number of arm joints (usually 7)
   int expected_num_total_joints_;  // Total joints including gripper
   
   // Controller state
   mutable Eigen::VectorXd integral_errors_;  // Integral of position errors
   mutable Eigen::VectorXd velocity_integral_errors_;  // Integral of velocity errors
   
   // Drake plant reference for kinematics calculations
   const drake::multibody::MultibodyPlant<double>* plant_;
   const drake::multibody::ModelInstanceIndex robot_instance_;
   mutable std::unique_ptr<drake::systems::Context<double>> plant_context_;
};

// Convert shared state to protocol RobotState
franka_fci_sim::protocol::RobotState get_robot_state() {
  std::lock_guard<std::mutex> lock(g_robot_state.mutex);
  
  franka_fci_sim::protocol::RobotState state{};
  
  // Basic joint state
   // Ensure joint sign convention consistent with state publisher
   state.q = g_robot_state.q;
   state.dq = g_robot_state.dq_filtered;  // Use filtered velocities for better stability
   state.tau_J = g_robot_state.tau_J;
  
  // End-effector poses
  state.O_T_EE = g_robot_state.O_T_EE;
  state.O_T_EE_d = g_robot_state.O_T_EE;  // Use current as desired for now
  state.O_T_EE_c = g_robot_state.O_T_EE_cmd;
  
  // Joint commands - IMPORTANT: q_d should reflect commanded positions if available
   if (g_robot_state.has_position_command) {
    state.q_d = g_robot_state.q_cmd;  // Use commanded positions (assumed same convention)
  } else {
    state.q_d = g_robot_state.q;      // Use current as desired
  }
  state.dq_d.fill(0.0);               // Zero velocity command
   // Convert commanded torques back to Franka convention for state echo
   for (int i = 0; i < 7; ++i) {
     state.tau_J_d[i] = g_robot_state.tau_cmd[i];
   }
  
  // Set default values for other fields
  state.F_T_EE.fill(0.0);
  state.F_T_EE[0] = state.F_T_EE[5] = state.F_T_EE[10] = state.F_T_EE[15] = 1.0; // Identity
  
  state.EE_T_K.fill(0.0);
  state.EE_T_K[0] = state.EE_T_K[5] = state.EE_T_K[10] = state.EE_T_K[15] = 1.0; // Identity
  
  state.F_T_NE.fill(0.0);
  state.F_T_NE[0] = state.F_T_NE[5] = state.F_T_NE[10] = state.F_T_NE[15] = 1.0; // Identity
  
  state.NE_T_EE.fill(0.0);
  state.NE_T_EE[0] = state.NE_T_EE[5] = state.NE_T_EE[10] = state.NE_T_EE[15] = 1.0; // Identity
  
  // Realistic end-effector parameters (CRITICAL for libfranka!)
  state.m_ee = g_robot_state.ee_mass;  // Use detected EE mass
  state.m_load = 0.0; // No additional load
  
  // Realistic end-effector inertia tensor (6 values: Ixx, Iyy, Izz, Ixy, Ixz, Iyz)
  // Use detected end effector inertia parametersless
  state.I_ee[0] = g_robot_state.ee_inertia[0];  // Ixx
  state.I_ee[1] = g_robot_state.ee_inertia[4];  // Iyy  
  state.I_ee[2] = g_robot_state.ee_inertia[8];  // Izz
  state.I_ee[3] = g_robot_state.ee_inertia[1];  // Ixy
  state.I_ee[4] = g_robot_state.ee_inertia[2];  // Ixz
  state.I_ee[5] = g_robot_state.ee_inertia[5];  // Iyz
  state.I_ee[6] = 0.0;    // (padding)
  state.I_ee[7] = 0.0;    // (padding)
  state.I_ee[8] = 0.0;    // (padding)
  
  // No load inertia
  state.I_load.fill(0.0);
  
  // Center of mass position for end-effector (relative to flange)
  state.F_x_Cee[0] = g_robot_state.ee_com[0];   // x
  state.F_x_Cee[1] = g_robot_state.ee_com[1];   // y  
  state.F_x_Cee[2] = g_robot_state.ee_com[2];   // z
  
  state.F_x_Cload.fill(0.0); // No load
  
  // Elbow configuration
  state.elbow = g_robot_state.elbow;
  state.elbow_d = g_robot_state.elbow;  // Desired = current for now
  state.elbow_c = g_robot_state.elbow_cmd;
  state.delbow_c.fill(0.0);  // Zero velocity for now
  state.ddelbow_c.fill(0.0); // Zero acceleration for now
  
  state.ddq_d.fill(0.0);
  state.dtau_J.fill(0.0);
  state.joint_contact.fill(0.0);
  state.cartesian_contact.fill(0.0);
  state.joint_collision.fill(0.0);
  state.cartesian_collision.fill(0.0);
  
  // External forces (simulated as zero for now - could be enhanced with contact detection)
  state.tau_ext_hat_filtered = g_robot_state.tau_ext_hat_filtered;
  state.O_F_ext_hat_K = g_robot_state.O_F_ext_hat_K;
  state.K_F_ext_hat_K = g_robot_state.K_F_ext_hat_K;
  state.O_dP_EE_d.fill(0.0);
  state.O_ddP_O.fill(0.0);
  state.O_dP_EE_c.fill(0.0);
  state.O_ddP_EE_c.fill(0.0);
  state.theta = g_robot_state.q;  // Motor positions = joint positions for simplicity
  state.dtheta = g_robot_state.dq;
  
  state.errors.fill(false);
  state.reflex_reason.fill(false);
  
  // CRITICAL: This field determines if libfranka will send commands!
  // Must be > 0.9 for libfranka to consider the robot ready
  state.control_command_success_rate = 1.0;
  
  // Debug output for first few states
  static int state_count = 0;
  if (++state_count <= 5) {
    std::cout << "[DEBUG] Robot state #" << state_count << ":" << std::endl;
    std::cout << "  q: [";
    for (int i = 0; i < 7; ++i) {
      std::cout << std::fixed << std::setprecision(4) << state.q[i] << (i < 6 ? ", " : "");
    }
    std::cout << "]" << std::endl;
    std::cout << "  dq: [";
    for (int i = 0; i < 7; ++i) {
      std::cout << std::fixed << std::setprecision(4) << state.dq[i] << (i < 6 ? ", " : "");
    }
    std::cout << "]" << std::endl;
    std::cout << "  robot_mode: " << static_cast<int>(state.robot_mode) << std::endl;
    std::cout << "  control_command_success_rate: " << state.control_command_success_rate << std::endl;
  }
  
  return state;
}

// Handle incoming FCI commands
void handle_robot_command(const franka_fci_sim::protocol::RobotCommand& cmd) {
  std::lock_guard<std::mutex> lock(g_robot_state.mutex);
  
  // Store all commands
  g_robot_state.tau_cmd = cmd.control.tau_J_d;
  g_robot_state.q_cmd = cmd.motion.q_c;
  g_robot_state.dq_cmd = cmd.motion.dq_c;
  g_robot_state.O_T_EE_cmd = cmd.motion.O_T_EE_c;
  g_robot_state.O_dP_EE_cmd = cmd.motion.O_dP_EE_c;
  g_robot_state.elbow_cmd = cmd.motion.elbow_c;
  
  // Reset command timeout on each new command
  g_robot_state.last_command_time = std::chrono::steady_clock::now();
  
  // Check for meaningful commands
  bool has_position_cmd = false;
  bool has_velocity_cmd = false;
  bool has_cartesian_cmd = false; 
  bool has_cartesian_velocity_cmd = false;
  bool has_torque_cmd = false;
  
  // Check position commands
  // IMPORTANT: libfranka sends position commands continuously, even if they match current position
  // We need to detect if we're receiving valid position commands, not just if they differ from current
  
  // First check if we have a valid position command (non-zero positions in expected range)
  bool has_valid_positions = false;
  for (int i = 0; i < 7; ++i) {
    // Joint positions are typically in range [-2.9, 2.9] radians for Franka
    if (std::abs(cmd.motion.q_c[i]) > 0.001 || cmd.motion.q_c[i] != 0.0) {
      has_valid_positions = true;
      break;
    }
  }
  
  // Position control is active if:
  // 1. We're in joint impedance mode AND joint position motion generator OR
  // 2. We have valid position commands AND joint position motion generator
  if ((g_robot_state.current_controller_mode == franka_fci_sim::protocol::Move::ControllerMode::kJointImpedance ||
       has_valid_positions) && 
      g_robot_state.current_motion_mode == franka_fci_sim::protocol::Move::MotionGeneratorMode::kJointPosition) {
    has_position_cmd = true;
  }
  
  // Check velocity commands
  bool has_valid_velocities = false;
  for (int i = 0; i < 7; ++i) {
    if (std::abs(cmd.motion.dq_c[i]) > 1e-6) {
      has_valid_velocities = true;
      break;
    }
  }
  
  // Velocity control is active if using joint velocity motion generator
  if (has_valid_velocities && 
      g_robot_state.current_motion_mode == franka_fci_sim::protocol::Move::MotionGeneratorMode::kJointVelocity) {
    has_velocity_cmd = true;
  }
  
  // Check cartesian commands
  bool has_valid_cartesian = false;
  // Check if the transformation matrix is not identity (has meaningful pose)
  for (int i = 0; i < 16; ++i) {
    double expected = (i % 5 == 0) ? 1.0 : 0.0; // Diagonal elements should be 1, others 0 for identity
    if (std::abs(cmd.motion.O_T_EE_c[i] - expected) > 1e-6) {
      has_valid_cartesian = true;
      break;
    }
  }
  
  if (has_valid_cartesian && 
      g_robot_state.current_motion_mode == franka_fci_sim::protocol::Move::MotionGeneratorMode::kCartesianPosition) {
    has_cartesian_cmd = true;
  }
  
  // Check cartesian velocity commands
  bool has_valid_cartesian_vel = false;
  for (int i = 0; i < 6; ++i) {
    if (std::abs(cmd.motion.O_dP_EE_c[i]) > 1e-6) {
      has_valid_cartesian_vel = true;
      break;
    }
  }
  
  if (has_valid_cartesian_vel && 
      g_robot_state.current_motion_mode == franka_fci_sim::protocol::Move::MotionGeneratorMode::kCartesianVelocity) {
    has_cartesian_velocity_cmd = true;
  }
  
  // Check torque commands
  has_torque_cmd = (g_robot_state.current_controller_mode == franka_fci_sim::protocol::Move::ControllerMode::kExternalController);
  if (!has_torque_cmd) {
    for (int i = 0; i < 7; ++i) {
      if (std::abs(cmd.control.tau_J_d[i]) > 1e-6) {
        has_torque_cmd = true;
        break;
      }
    }
  }
  
  // Update command flags
  g_robot_state.has_position_command = has_position_cmd;
  g_robot_state.has_velocity_command = has_velocity_cmd;
  g_robot_state.has_cartesian_command = has_cartesian_cmd;
  g_robot_state.has_cartesian_velocity_command = has_cartesian_velocity_cmd;
  g_robot_state.has_torque_command = has_torque_cmd;
  
  // Enhanced logging for debugging position control
  static int cmd_count = 0;
  cmd_count++;
  
  // Log first 50 commands in detail to debug position control
  if (cmd_count <= 50) {
    std::cout << "[FCI] Command #" << cmd_count << " details:" << std::endl;
    std::cout << "  Controller mode: " << static_cast<int>(g_robot_state.current_controller_mode) 
              << " (0=JointImp, 1=CartImp, 2=ExtCtrl)" << std::endl;
    std::cout << "  q_c: [";
    for (int i = 0; i < 7; ++i) {
      std::cout << std::fixed << std::setprecision(4) << cmd.motion.q_c[i] << (i < 6 ? ", " : "");
    }
    std::cout << "]" << std::endl;
    std::cout << "  tau_J_d: [";
    for (int i = 0; i < 7; ++i) {
      std::cout << std::fixed << std::setprecision(4) << cmd.control.tau_J_d[i] << (i < 6 ? ", " : "");
    }
    std::cout << "]" << std::endl;
    std::cout << "  elbow_c: [" << std::fixed << std::setprecision(4) 
              << cmd.motion.elbow_c[0] << ", " << cmd.motion.elbow_c[1] << "]" << std::endl;
    std::cout << "  Flags: pos=" << has_position_cmd << " vel=" << has_velocity_cmd
              << " cart=" << has_cartesian_cmd << " cart_vel=" << has_cartesian_velocity_cmd
              << " tau=" << has_torque_cmd << " valid_pos=" << has_valid_positions << std::endl;
  }
  
  // Regular logging every 1000 commands
  if (cmd_count % 1000 == 0) {
    std::cout << "[FCI] Command #" << cmd_count << " - ";
    if (has_position_cmd) std::cout << "POS ";
    if (has_cartesian_cmd) std::cout << "CART ";
    if (has_torque_cmd) std::cout << "TAU ";
    if (!has_position_cmd && !has_cartesian_cmd && !has_torque_cmd) std::cout << "IDLE";
    std::cout << " | q_cmd[6]=" << std::fixed << std::setprecision(4) << cmd.motion.q_c[6];
    std::cout << std::endl;
  }
}

int main(int argc, char** argv) {
  using namespace drake::multibody;
  using namespace drake::systems;
  using namespace franka_fci_sim;

  if (argc < 4) {
    std::cerr << "Usage: " << argv[0] << " <package.xml> <arm.urdf> <time_step> [headless] [turbo]\n";
    std::cerr << "  headless: 'true' to run without visualization and log joint positions continuously\n";
    std::cerr << "  turbo: 'turbo' to run faster than real-time (no 1:1 real-time constraint)\n";
    return 1;
  }
  const std::string  package_xml = argv[1];
  const std::string  urdf        = argv[2];
  const double       dt          = std::stod(argv[3]);
  const bool         headless    = (argc >= 5 && std::string(argv[4]) == "true");

  if (headless) {
    std::cout << "Running in HEADLESS mode - logging joint positions continuously" << std::endl;
  }

  DiagramBuilder<double> builder;

  // --- Plant and SceneGraph setup ---
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, dt);
  Parser parser(&plant, &scene_graph);
  parser.package_map().AddPackageXml(package_xml);
  const auto robot = parser.AddModels(urdf)[0];
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base", robot));

  // Add actuators
  struct J { std::string n; double eff; };
  const std::vector<J> act = {
      {"fer_joint1", 87}, {"fer_joint2", 87}, {"fer_joint3", 87},
      {"fer_joint4", 87}, {"fer_joint5", 12}, {"fer_joint6", 87}, {"fer_joint7", 87},
  };
  for (const auto& a : act) {
    const auto& joint = plant.GetJointByName<RevoluteJoint>(a.n, robot);
    plant.AddJointActuator(a.n + "_act", joint, a.eff);
  }

  // Set damping
  const std::vector<std::pair<std::string,double>> damp = {
      {"fer_joint1", 1.0}, {"fer_joint2", 1.0}, {"fer_joint3", 1.0},
      {"fer_joint4", 1.0}, {"fer_joint5", 1.0}, {"fer_joint6", 1.0}, {"fer_joint7", 1.0},
  };
  for (const auto& [name, d] : damp) {
    plant.GetMutableJointByName<RevoluteJoint>(name, robot).set_default_damping(d);
  }

  // Add joint frame visualizations
  const double axis_len = 0.15, axis_rad = 0.003;
  const drake::geometry::Cylinder cyl_shape(axis_rad, axis_len);
  const drake::math::RigidTransformd X_FGx(drake::math::RotationMatrixd::MakeYRotation(M_PI / 2), Eigen::Vector3d(axis_len / 2, 0, 0));
  const drake::math::RigidTransformd X_FGy(drake::math::RotationMatrixd::MakeXRotation(-M_PI / 2), Eigen::Vector3d(0, axis_len / 2, 0));
  const drake::math::RigidTransformd X_FGz(drake::math::RotationMatrixd(), Eigen::Vector3d(0, 0, axis_len / 2));
  const Eigen::Vector4d red(1, 0, 0, 0.9), grn(0, 1, 0, 0.9), blu(0, 0, 1, 0.9);
  for (const auto& a : act) {
    const auto& joint = plant.GetJointByName<RevoluteJoint>(a.n, robot);
    const auto& body = joint.child_body();
    plant.RegisterVisualGeometry(body, X_FGx, cyl_shape, a.n + ":x", red);
    plant.RegisterVisualGeometry(body, X_FGy, cyl_shape, a.n + ":y", grn);
    plant.RegisterVisualGeometry(body, X_FGz, cyl_shape, a.n + ":z", blu);
  }

  // Disable collisions
  {
    const drake::geometry::SourceId sid = plant.get_source_id().value();
    for (drake::multibody::BodyIndex b(0); b < plant.num_bodies(); ++b) {
      const auto& body = plant.get_body(b);
      for (const drake::geometry::GeometryId gid : plant.GetCollisionGeometriesForBody(body)) {
        scene_graph.RemoveRole(sid, gid, drake::geometry::Role::kProximity);
      }
    }
  }

  plant.Finalize();

  // DETECT END EFFECTOR CONFIGURATION
  auto [num_arm_joints, num_total_joints] = DetectEndEffectorConfiguration(plant, robot);

  // Visualization (only if not headless)
  std::shared_ptr<drake::geometry::Meshcat> meshcat;
  if (!headless) {
    meshcat = std::make_shared<drake::geometry::Meshcat>();
    drake::visualization::AddDefaultVisualization(&builder, meshcat);
  }

  // Gravity compensation
  using drake::systems::controllers::InverseDynamics;
  auto* g_comp = builder.AddSystem<InverseDynamics<double>>(
      &plant, InverseDynamics<double>::InverseDynamicsMode::kGravityCompensation);
  builder.Connect(plant.get_state_output_port(), g_comp->get_input_port_estimated_state());

  // FCI impedance controller
  auto* fci_controller = builder.AddSystem<FciImpedanceController>(&plant, robot);
  fci_controller->ConfigureForRobot(num_arm_joints, num_total_joints);
  builder.Connect(plant.get_state_output_port(), fci_controller->get_input_port(fci_controller->get_state_input_port()));

  // Handle gravity compensation based on robot configuration
  drake::systems::Adder<double>* adder = nullptr;
  
  if (num_total_joints > num_arm_joints) {
    // Robot with gripper: select only arm joint torques from gravity compensation
    auto* gravity_selector = builder.AddSystem<drake::systems::Demultiplexer<double>>(
      std::vector<int>{num_arm_joints, num_total_joints - num_arm_joints});  // Split into arm joints + gripper joints
    builder.Connect(g_comp->get_output_port(), gravity_selector->get_input_port());
    
    // Torque adder: gravity compensation (arm joints only) + FCI commands
    adder = builder.AddSystem<drake::systems::Adder<double>>(2, num_arm_joints);
    builder.Connect(gravity_selector->get_output_port(0), adder->get_input_port(0));  // Use selected gravity torques
    builder.Connect(fci_controller->get_output_port(), adder->get_input_port(1));
    
    // Connect arm torques to the plant
    builder.Connect(adder->get_output_port(), plant.get_actuation_input_port());
  } else {
    // Fingerless robot: use gravity compensation directly
    adder = builder.AddSystem<drake::systems::Adder<double>>(2, num_arm_joints);
    builder.Connect(g_comp->get_output_port(), adder->get_input_port(0));  // Direct gravity torques
    builder.Connect(fci_controller->get_output_port(), adder->get_input_port(1));
    
    // Connect arm torques to the plant
    builder.Connect(adder->get_output_port(), plant.get_actuation_input_port());
  }

  // Logging
  const int n_state = plant.num_positions() + plant.num_velocities();
  auto* logger = builder.AddSystem<drake::systems::VectorLogSink<double>>(n_state);
  builder.Connect(plant.get_state_output_port(), logger->get_input_port());
  auto* torque_logger = builder.AddSystem<drake::systems::VectorLogSink<double>>(plant.num_actuators(robot));
  builder.Connect(adder->get_output_port(), torque_logger->get_input_port());

  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(1.0);
  auto& rk = simulator.reset_integrator<drake::systems::RungeKutta5Integrator<double>>();
  rk.set_target_accuracy(1e-12);
  rk.set_maximum_step_size(1e-4);
  
  // Set initial joint configuration
  auto& root = simulator.get_mutable_context();
  auto& plant_ctx = plant.GetMyMutableContextFromRoot(&root);
  const int n = plant.num_positions(robot);
  const int nv = plant.num_velocities(robot);
  Eigen::VectorXd q_des = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd v_des = Eigen::VectorXd::Zero(nv);
  q_des.setZero();
  // Arm initial configuration (match ROS 2 franka_description defaults)
  q_des[0] = 0.0;
  q_des[1] = -M_PI / 4.0;
  q_des[2] = 0.0;
  q_des[3] = -3.0 * M_PI / 4.0;
  q_des[4] = 0.0;
  q_des[5] = M_PI / 2.0;
  q_des[6] = M_PI / 4.0;  // Match ROS 2 default directly (no sign inversion)
  // If gripper joints are present, start opened
  if (n > 7) {
    for (int i = 7; i < n; ++i) {
      q_des[i] = 0.04;  // Open fingers (within [0.0, 0.04])
    }
  }
  plant.SetPositions(&plant_ctx, robot, q_des);
  plant.SetVelocities(&plant_ctx, robot, v_des);

  // Seed shared robot state once so initial read() reflects actual configuration
  {
    std::lock_guard<std::mutex> lock(g_robot_state.mutex);
    const std::vector<std::string> joint_names = {
        "fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4",
        "fer_joint5", "fer_joint6", "fer_joint7"};
    static const std::array<double, 7> plant_to_franka_sign{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, -1.0};

    for (int i = 0; i < 7; ++i) {
      const auto& joint = plant.GetJointByName<drake::multibody::RevoluteJoint>(joint_names[i], robot);
      const double q_pl = joint.get_angle(plant_ctx);
      g_robot_state.q[i] = plant_to_franka_sign[i] * q_pl;
      g_robot_state.dq[i] = 0.0;
      g_robot_state.dq_filtered[i] = 0.0;
      g_robot_state.tau_J[i] = 0.0;
    }

    // Gripper state (if any)
    if (g_robot_state.has_gripper && g_robot_state.num_gripper_joints > 0) {
      Eigen::VectorXd q_all = plant.GetPositions(plant_ctx, robot);
      for (int i = 0; i < g_robot_state.num_gripper_joints; ++i) {
        const int idx = (g_robot_state.gripper_joint_names[i].find("finger_joint1") != std::string::npos) ? 0 : 1;
        const int q_index = 7 + i;
        if (q_index < q_all.size()) {
          g_robot_state.gripper_q[idx] = q_all[q_index];
        }
        g_robot_state.gripper_dq[idx] = 0.0;
      }
    }

    // End-effector pose
    const auto& ee_frame = plant.GetFrameByName(g_robot_state.ee_frame_name, robot);
    const auto& world_frame = plant.world_frame();
    const drake::math::RigidTransformd X_W_EE = plant.CalcRelativeTransform(plant_ctx, world_frame, ee_frame);
    const Eigen::Matrix4d T_matrix = X_W_EE.GetAsMatrix4();
    for (int col = 0; col < 4; ++col) {
      for (int row = 0; row < 4; ++row) {
        g_robot_state.O_T_EE[col * 4 + row] = T_matrix(row, col);
      }
    }

    // Elbow configuration
    g_robot_state.elbow[0] = g_robot_state.q[2];
    g_robot_state.elbow[1] = (g_robot_state.q[3] < 0) ? 1.0 : -1.0;

    // Mirror commanded state to current for startup consistency
    g_robot_state.q_cmd = g_robot_state.q;
    g_robot_state.O_T_EE_cmd = g_robot_state.O_T_EE;
  }

  simulator.Initialize();

  // Start FCI server in background thread
  franka_fci_sim::FrankaFciSimServer server(1337, 1338); // TCP port 1337, UDP port 1338
  server.set_state_provider(get_robot_state);
  server.set_command_handler(handle_robot_command);
  
  // Set up mode change handler to update shared state
  server.set_mode_change_handler([](franka_fci_sim::protocol::Move::ControllerMode controller_mode, 
                                    franka_fci_sim::protocol::Move::MotionGeneratorMode motion_mode) {
    std::lock_guard<std::mutex> lock(g_robot_state.mutex);
    g_robot_state.current_controller_mode = controller_mode;
    g_robot_state.current_motion_mode = motion_mode;
    std::cout << "[FCI] Controller mode changed to: " << static_cast<int>(controller_mode) 
              << " (0=JointImpedance, 1=CartesianImpedance, 2=ExternalController)" << std::endl;
    std::cout << "[FCI] Motion generator mode changed to: " << static_cast<int>(motion_mode)
              << " (0=JointPos, 1=JointVel, 2=CartPos, 3=CartVel)" << std::endl;
  });
  
  std::thread fci_thread([&server]() {
    server.run();
  });
  
  std::cout << "Starting FCI simulation server on port 1337..." << std::endl;
  if (!headless) {
    std::cout << "Drake simulation running with Meshcat on http://localhost:7000" << std::endl;
  } else {
    std::cout << "Drake simulation running in HEADLESS mode - continuous joint logging enabled" << std::endl;
  }
  std::cout << "Press Ctrl+C to stop." << std::endl;
  
  // Create simulation runner
  SimulationRunner sim_runner(simulator, plant, robot, headless);
  
  // Enable turbo mode if requested
  bool turbo_mode = (argc >= 6 && std::string(argv[5]) == "turbo");
  if (turbo_mode) {
    sim_runner.EnableTurboMode();
  }
  
  // Main simulation loop - CLEAN AND SIMPLE!
  while (sim_runner.ShouldContinue()) {
    sim_runner.RunStep();
    
    // Sleep only in real-time mode to maintain 1kHz rate
    if (!turbo_mode) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));  // Small sleep to prevent CPU spinning
    }
  }

  // Cleanup
  server.stop();
  fci_thread.join();
  
  return 0;
} 