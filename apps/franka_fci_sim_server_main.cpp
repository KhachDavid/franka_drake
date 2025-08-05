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
#include <drake/systems/analysis/runge_kutta5_integrator.h>
#include <drake/math/rigid_transform.h>
#include <Eigen/Core>
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
  std::array<double, 7> tau_J{};  // Joint torques
  std::array<double, 16> O_T_EE{}; // End-effector pose
  std::array<double, 7> tau_cmd{}; // Commanded torques from FCI
  std::array<double, 7> q_cmd{};   // Commanded positions from FCI  
  std::array<double, 16> O_T_EE_cmd{}; // Commanded EE pose from FCI
  bool has_position_command = false;
  bool has_torque_command = false;
  bool has_cartesian_command = false;
  // Track current controller mode for proper torque detection
  franka_fci_sim::protocol::Move::ControllerMode current_controller_mode = franka_fci_sim::protocol::Move::ControllerMode::kExternalController;
};

// Global shared state
SharedRobotState g_robot_state;
std::atomic<bool> g_sim_running{true};

// Ziegler-Nichols Auto-Tuning Class
class ZieglerNicholsAutoTuner {
 public:
   struct TuningResult {
     Eigen::VectorXd ultimate_gain;      // Ku values
     Eigen::VectorXd oscillation_period; // Tu values
     bool success;
   };
   
   ZieglerNicholsAutoTuner() {
     // Initialize tuning parameters
     max_gain_test_ = 500.0;  // Maximum gain to test (safety limit)
     min_oscillations_ = 3;   // Minimum oscillations to measure period
     convergence_threshold_ = 0.02;  // 2% change to declare convergence
   }
   
   // Auto-tune a single joint by finding ultimate gain and period
   TuningResult AutoTuneJoint(int joint_index, 
                             std::function<void(double)> set_gain_callback,
                             std::function<double()> get_position_callback,
                             std::function<double()> get_time_callback) {
     TuningResult result;
     result.ultimate_gain = Eigen::VectorXd::Zero(7);
     result.oscillation_period = Eigen::VectorXd::Zero(7);
     result.success = false;
     
     std::cout << "[ZN Tuning] Starting auto-tune for joint " << joint_index << std::endl;
     
     // Step 1: Find ultimate gain (Ku) by increasing gain until sustained oscillation
     double ku = FindUltimateGain(joint_index, set_gain_callback, get_position_callback, get_time_callback);
     if (ku <= 0) {
       std::cout << "[ZN Tuning] Failed to find ultimate gain for joint " << joint_index << std::endl;
       return result;
     }
     
     // Step 2: Measure oscillation period (Tu) at ultimate gain
     double tu = MeasureOscillationPeriod(joint_index, ku, set_gain_callback, get_position_callback, get_time_callback);
     if (tu <= 0) {
       std::cout << "[ZN Tuning] Failed to measure oscillation period for joint " << joint_index << std::endl;
       return result;
     }
     
     result.ultimate_gain[joint_index] = ku;
     result.oscillation_period[joint_index] = tu;
     result.success = true;
     
     std::cout << "[ZN Tuning] Joint " << joint_index << " tuning complete:" << std::endl;
     std::cout << "  Ultimate Gain (Ku): " << ku << std::endl;
     std::cout << "  Oscillation Period (Tu): " << tu << " seconds" << std::endl;
     
     return result;
   }
   
 private:
   double FindUltimateGain(int joint_index,
                          std::function<void(double)> set_gain_callback,
                          std::function<double()> get_position_callback,
                          std::function<double()> get_time_callback) {
     
     double gain = 10.0;  // Start with low gain
     double gain_increment = 10.0;
     
     while (gain < max_gain_test_) {
       set_gain_callback(gain);
       
       // Wait for settling
       std::this_thread::sleep_for(std::chrono::seconds(1));
       
       // Apply small disturbance and measure response
       bool oscillating = TestForOscillation(get_position_callback, get_time_callback);
       
       if (oscillating) {
         std::cout << "[ZN Tuning] Found ultimate gain: " << gain << std::endl;
         return gain;
       }
       
       gain += gain_increment;
       
       // Adaptive increment (start small, increase if no oscillation)
       if (gain > 100.0) gain_increment = 25.0;
       if (gain > 200.0) gain_increment = 50.0;
     }
     
     return -1.0;  // Failed to find ultimate gain
   }
   
   double MeasureOscillationPeriod(int joint_index, double ultimate_gain,
                                  std::function<void(double)> set_gain_callback,
                                  std::function<double()> get_position_callback,
                                  std::function<double()> get_time_callback) {
     
     set_gain_callback(ultimate_gain);
     
     // Wait for steady oscillation
     std::this_thread::sleep_for(std::chrono::seconds(2));
     
     // Record position data for period measurement
     std::vector<double> positions;
     std::vector<double> times;
     
     double start_time = get_time_callback();
     double measurement_duration = 5.0;  // Measure for 5 seconds
     
     while ((get_time_callback() - start_time) < measurement_duration) {
       positions.push_back(get_position_callback());
       times.push_back(get_time_callback());
       std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 100Hz sampling
     }
     
     // Analyze oscillation period using zero-crossings
     return CalculateOscillationPeriod(positions, times);
   }
   
   bool TestForOscillation(std::function<double()> get_position_callback,
                          std::function<double()> get_time_callback) {
     
     std::vector<double> positions;
     double start_time = get_time_callback();
     double test_duration = 3.0;  // Test for 3 seconds
     
     while ((get_time_callback() - start_time) < test_duration) {
       positions.push_back(get_position_callback());
       std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 20Hz sampling
     }
     
     // Check for sustained oscillation (simple variance check)
     if (positions.size() < 10) return false;
     
     double mean_pos = 0.0;
     for (double pos : positions) mean_pos += pos;
     mean_pos /= positions.size();
     
     double variance = 0.0;
     for (double pos : positions) {
       variance += (pos - mean_pos) * (pos - mean_pos);
     }
     variance /= positions.size();
     
     // If variance is above threshold, consider it oscillating
     double oscillation_threshold = 0.001;  // 1 mrad variance threshold
     return variance > oscillation_threshold;
   }
   
   double CalculateOscillationPeriod(const std::vector<double>& positions, 
                                    const std::vector<double>& times) {
     if (positions.size() < 20) return -1.0;
     
     // Find zero crossings (relative to mean position)
     double mean_pos = 0.0;
     for (double pos : positions) mean_pos += pos;
     mean_pos /= positions.size();
     
     std::vector<double> zero_crossing_times;
     for (size_t i = 1; i < positions.size(); ++i) {
       double prev_diff = positions[i-1] - mean_pos;
       double curr_diff = positions[i] - mean_pos;
       
       // Check for sign change (zero crossing)
       if ((prev_diff > 0 && curr_diff < 0) || (prev_diff < 0 && curr_diff > 0)) {
         // Linear interpolation to find exact crossing time
         double t_cross = times[i-1] + (times[i] - times[i-1]) * 
                         std::abs(prev_diff) / (std::abs(prev_diff) + std::abs(curr_diff));
         zero_crossing_times.push_back(t_cross);
       }
     }
     
     if (zero_crossing_times.size() < 4) return -1.0;  // Need at least 2 complete cycles
     
     // Calculate average period (time between alternate zero crossings = full cycle)
     double total_period = 0.0;
     int period_count = 0;
     
     for (size_t i = 2; i < zero_crossing_times.size(); ++i) {
       double period = zero_crossing_times[i] - zero_crossing_times[i-2];
       total_period += period;
       period_count++;
     }
     
     if (period_count == 0) return -1.0;
     
     return total_period / period_count;
   }
   
   double max_gain_test_;
   int min_oscillations_;
   double convergence_threshold_;
};

// Global auto-tuner instance
ZieglerNicholsAutoTuner g_zn_tuner;

// Drake system to provide FCI commands as input - SIMPLE WORKING PID CONTROLLER
class FciPidController final : public drake::systems::LeafSystem<double> {
 public:
   FciPidController() {
     // Declare input ports for current robot state
     state_input_port_ = this->DeclareVectorInputPort("robot_state", drake::systems::BasicVector<double>(14)).get_index();
     
     // Declare output port for computed control torques
     this->DeclareVectorOutputPort("control_torques", drake::systems::BasicVector<double>(7),
                                  &FciPidController::CalcControlTorques);
     
     // Initialize PID gains with REASONABLE values
     // Start with the working PD gains and add modest integral terms
     kp_ << 150, 150, 120, 120, 80, 60, 40;   // Working proportional gains
     ki_ << 0, 0, 0, 0, 0, 0, 0;       // Modest integral gains  
     kd_ << 0, 0, 0, 0, 0, 0  , 0;          // Working derivative gains
     
     // Initialize state variables
     integral_error_.setZero();
     first_call_ = true;
     
     std::cout << "[PID Controller] Initialized with gains:" << std::endl;
     std::cout << "  Kp: [" << kp_.transpose() << "]" << std::endl;
     std::cout << "  Ki: [" << ki_.transpose() << "]" << std::endl;
     std::cout << "  Kd: [" << kd_.transpose() << "]" << std::endl;
   }

   // Public accessor for state input port
   int get_state_input_port() const { return state_input_port_; }
   
   // Manual gain setting interface
   void SetPidGains(const Eigen::VectorXd& kp, const Eigen::VectorXd& ki, const Eigen::VectorXd& kd) {
     kp_ = kp;
     ki_ = ki;
     kd_ = kd;
     std::cout << "[PID Controller] Updated gains:" << std::endl;
     std::cout << "  Kp: [" << kp_.transpose() << "]" << std::endl;
     std::cout << "  Ki: [" << ki_.transpose() << "]" << std::endl;
     std::cout << "  Kd: [" << kd_.transpose() << "]" << std::endl;
   }

 private:
   void CalcControlTorques(const drake::systems::Context<double>& context,
                          drake::systems::BasicVector<double>* output) const {
     std::lock_guard<std::mutex> lock(g_robot_state.mutex);
     
     Eigen::VectorXd control_torques(7);
     
     // IMPORTANT: Check torque control mode FIRST - no PID calculations at all!
     if (g_robot_state.has_torque_command) {
       // PURE TORQUE CONTROL: Direct torque pass-through, no PID at all
       for (int i = 0; i < 7; ++i) {
         control_torques[i] = g_robot_state.tau_cmd[i];
       }
       
       // Reset integrator when in torque control mode
       integral_error_.setZero();
       first_call_ = true;
       
       // Debug logging for torque mode
       static int torque_debug_count = 0;
       if (++torque_debug_count % 2000 == 0) {  // Every 2 seconds at 1kHz
         std::cout << "[TORQUE MODE] Direct torques: [" << control_torques.transpose() << "]" << std::endl;
       }
       
       // Apply torque limits for safety and output immediately
       const double max_torque = 50.0;
       for (int i = 0; i < 7; ++i) {
         control_torques[i] = std::max(-max_torque, std::min(max_torque, control_torques[i]));
       }
       
       output->SetFromVector(control_torques);
       return;  // EARLY RETURN - No PID calculations in torque mode!
     }
     
     // Only run PID if we're NOT in torque mode
     if (g_robot_state.has_position_command) {
       // Get current robot state from input for PID calculations
       const auto& state_input = this->get_input_port(state_input_port_).Eval(context);
       
       // Extract current positions and velocities
       Eigen::VectorXd current_q = state_input.head(7);   // positions
       Eigen::VectorXd current_dq = state_input.tail(7);  // velocities
       
       // POSITION CONTROL: Use PID controller to track commanded positions
       Eigen::VectorXd target_q(7);
       for (int i = 0; i < 7; ++i) {
         target_q[i] = g_robot_state.q_cmd[i];
       }
       
       // Calculate position error
       Eigen::VectorXd position_error = target_q - current_q;
       
       // Handle first call - initialize previous time properly
       double current_time = context.get_time();
       double dt = 0.001;  // Default 1ms timestep
       
       if (!first_call_) {
         dt = current_time - previous_time_;
         if (dt <= 0.0 || dt > 0.1) {  // Sanity check: 0 < dt < 100ms
           dt = 0.001;
         }
       } else {
         first_call_ = false;
       }
       
       // Update integral error with simple anti-windup
       for (int i = 0; i < 7; ++i) {
         // Only integrate if error is significant and we're not saturated
         if (std::abs(position_error[i]) > 1e-6) {
           double tentative_integral = integral_error_[i] + position_error[i] * dt;
           
           // Anti-windup: limit integral contribution to ±5 Nm
           double integral_torque = ki_[i] * tentative_integral;
           if (std::abs(integral_torque) < 5.0) {
             integral_error_[i] = tentative_integral;
           }
           // Don't update integral if saturated
         }
       }
       
       // PID control law: τ = Kp*e + Ki*∫e*dt + Kd*(-dq)
       Eigen::VectorXd proportional_term = kp_.cwiseProduct(position_error);
       Eigen::VectorXd integral_term = ki_.cwiseProduct(integral_error_);
       Eigen::VectorXd derivative_term = kd_.cwiseProduct(-current_dq);  // velocity feedback
       
       control_torques = proportional_term + integral_term + derivative_term;
       
       // Small bias for libfranka responsiveness (only if really needed)
       for (int i = 0; i < 7; ++i) {
         if (std::abs(position_error[i]) < 1e-8 && std::abs(control_torques[i]) < 0.1) {
           control_torques[i] += (i % 2 == 0 ? 0.05 : -0.05);
         }
       }
       
       // Debug logging for PID mode
       static int pid_debug_count = 0;
       if (++pid_debug_count % 2000 == 0) {  // Every 2 seconds at 1kHz
         std::cout << "[PID MODE] Target: [" << target_q.transpose() << "]" << std::endl;
         std::cout << "[PID MODE] Current: [" << current_q.transpose() << "]" << std::endl;
         std::cout << "[PID MODE] Error: [" << position_error.transpose() << "]" << std::endl;
         std::cout << "[PID MODE] P: [" << proportional_term.transpose() << "]" << std::endl;
         std::cout << "[PID MODE] I: [" << integral_term.transpose() << "]" << std::endl;
         std::cout << "[PID MODE] D: [" << derivative_term.transpose() << "]" << std::endl;
         std::cout << "[PID MODE] Total: [" << control_torques.transpose() << "]" << std::endl;
       }
       
       previous_time_ = current_time;
       
       // Add any additional direct torque commands from libfranka (if any)
       for (int i = 0; i < 7; ++i) {
         control_torques[i] += g_robot_state.tau_cmd[i];
       }
       
     } else {
       // NO COMMANDS: Zero torques (gravity compensation will handle this)
       control_torques.setZero();
       
       // Reset integrator when idle
       integral_error_.setZero();
       first_call_ = true;
       
       // Debug logging for idle mode
       static int idle_debug_count = 0;
       if (++idle_debug_count % 5000 == 0) {  // Every 5 seconds at 1kHz
         std::cout << "[IDLE MODE] No commands, zero torques" << std::endl;
       }
     }
     
     // Apply torque limits for safety
     const double max_torque = 50.0;
     for (int i = 0; i < 7; ++i) {
       control_torques[i] = std::max(-max_torque, std::min(max_torque, control_torques[i]));
     }
     
     output->SetFromVector(control_torques);
   }
   
   // Member variables
   int state_input_port_;
   
   // PID gains (fixed size, reasonable values)
   Eigen::VectorXd kp_{7};  // Proportional gains
   Eigen::VectorXd ki_{7};  // Integral gains  
   Eigen::VectorXd kd_{7};  // Derivative gains
   
   // PID state variables (mutable for const method)
   mutable Eigen::VectorXd integral_error_{7};  // Accumulated integral error
   mutable double previous_time_;               // Previous time for dt calculation
   mutable bool first_call_;                    // Flag for proper time initialization
};

// Convert shared state to protocol RobotState
franka_fci_sim::protocol::RobotState get_robot_state() {
  std::lock_guard<std::mutex> lock(g_robot_state.mutex);
  
  franka_fci_sim::protocol::RobotState state{};
  
  // Basic joint state
  state.q = g_robot_state.q;
  state.dq = g_robot_state.dq;
  state.tau_J = g_robot_state.tau_J;
  
  // End-effector poses
  state.O_T_EE = g_robot_state.O_T_EE;
  state.O_T_EE_d = g_robot_state.O_T_EE;  // Use current as desired for now
  state.O_T_EE_c = g_robot_state.O_T_EE_cmd;
  
  // Joint commands
  state.q_d = g_robot_state.q;     // Use current as desired
  state.dq_d.fill(0.0);            // Zero velocity command
  state.tau_J_d = g_robot_state.tau_cmd;
  
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
  state.m_ee = 0.73;  // Franka gripper mass in kg
  state.m_load = 0.0; // No additional load
  
  // Realistic end-effector inertia tensor (6 values: Ixx, Iyy, Izz, Ixy, Ixz, Iyz)
  // Based on Franka gripper specifications 
  state.I_ee[0] = 0.001;  // Ixx
  state.I_ee[1] = 0.0025; // Iyy  
  state.I_ee[2] = 0.0017; // Izz
  state.I_ee[3] = 0.0;    // Ixy
  state.I_ee[4] = 0.0;    // Ixz
  state.I_ee[5] = 0.0;    // Iyz
  state.I_ee[6] = 0.0;    // (padding)
  state.I_ee[7] = 0.0;    // (padding)
  state.I_ee[8] = 0.0;    // (padding)
  
  // No load inertia
  state.I_load.fill(0.0);
  
  // Center of mass position for end-effector (relative to flange)
  state.F_x_Cee[0] = 0.0;   // x
  state.F_x_Cee[1] = 0.0;   // y  
  state.F_x_Cee[2] = 0.058; // z (gripper center of mass ~58mm from flange)
  
  state.F_x_Cload.fill(0.0); // No load
  
  state.elbow.fill(0.0);
  state.elbow_d.fill(0.0);
  state.elbow_c.fill(0.0);
  state.delbow_c.fill(0.0);
  state.ddelbow_c.fill(0.0);
  
  state.ddq_d.fill(0.0);
  state.dtau_J.fill(0.0);
  state.joint_contact.fill(0.0);
  state.cartesian_contact.fill(0.0);
  state.joint_collision.fill(0.0);
  state.cartesian_collision.fill(0.0);
  state.tau_ext_hat_filtered.fill(0.0);
  state.O_F_ext_hat_K.fill(0.0);
  state.K_F_ext_hat_K.fill(0.0);
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
  
  return state;
}

// Handle incoming FCI commands
void handle_robot_command(const franka_fci_sim::protocol::RobotCommand& cmd) {
  // DEBUG: Add logging to verify this function is called
  std::cout << "[DEBUG] handle_robot_command called with msg_id=" << cmd.message_id << std::endl;
  
  std::lock_guard<std::mutex> lock(g_robot_state.mutex);
  
  // Store all commands
  g_robot_state.tau_cmd = cmd.control.tau_J_d;
  g_robot_state.q_cmd = cmd.motion.q_c;
  g_robot_state.O_T_EE_cmd = cmd.motion.O_T_EE_c;
  
  // Check for meaningful commands (not just zero/default values)
  bool has_position_cmd = false;
  bool has_cartesian_cmd = false; 
  bool has_torque_cmd = false;
  
  // libfranka expects the robot to respond to position commands immediately, even if they match current position
  if (g_robot_state.current_controller_mode == franka_fci_sim::protocol::Move::ControllerMode::kJointImpedance) {
    // In joint impedance mode, ANY non-zero position command is valid
    for (int i = 0; i < 7; ++i) {
      if (std::abs(cmd.motion.q_c[i]) > 1e-6) {
        has_position_cmd = true;
        break;
      }
    }
  } else {
    // For other modes, compare with current position
    for (int i = 0; i < 7; ++i) {
      double current_pos = g_robot_state.q[i];
      double commanded_pos = cmd.motion.q_c[i];
      if (std::abs(commanded_pos - current_pos) > 0.01) {  // 0.01 radians = ~0.57 degrees threshold
        has_position_cmd = true;
        break;
      }
    }
    
    // If no significant difference detected, but we have any non-zero commanded positions, still treat as position command
    if (!has_position_cmd) {
      for (int i = 0; i < 7; ++i) {
        if (std::abs(cmd.motion.q_c[i]) > 1e-6) {
          has_position_cmd = true;
          break;
        }
      }
    }
  }
  
  // Check cartesian commands
  for (int i = 0; i < 16; ++i) {
    if (std::abs(cmd.motion.O_T_EE_c[i]) > 1e-6) {
      has_cartesian_cmd = true;
      break;
    }
  }
  
  // This handles cases where torques start at zero and ramp up (like move_joint7_torque)
  has_torque_cmd = (g_robot_state.current_controller_mode == franka_fci_sim::protocol::Move::ControllerMode::kExternalController);
  
  // Also check if any torques are explicitly non-zero (for immediate torque commands)
  for (int i = 0; i < 7; ++i) {
    if (std::abs(cmd.control.tau_J_d[i]) > 1e-6) {
      has_torque_cmd = true;
      break;
    }
  }
  
  // Update command flags for controller
  g_robot_state.has_position_command = has_position_cmd;
  g_robot_state.has_cartesian_command = has_cartesian_cmd;
  g_robot_state.has_torque_command = has_torque_cmd;
  
  // Enhanced logging every 100 commands
  static int cmd_count = 0;
  if (++cmd_count % 100 == 0) {
    std::cout << "[FCI] Command #" << cmd_count << " - ";
    if (has_position_cmd) {
      std::cout << "JOINT_POS q_c=[" << std::fixed << std::setprecision(3) 
                << cmd.motion.q_c[0] << "," << cmd.motion.q_c[1] << "," << cmd.motion.q_c[2] << ",...] ";
    }
    if (has_cartesian_cmd) {
      std::cout << "CARTESIAN pos=[" << std::fixed << std::setprecision(3)
                << cmd.motion.O_T_EE_c[12] << "," << cmd.motion.O_T_EE_c[13] << "," << cmd.motion.O_T_EE_c[14] << "] ";
    }
    if (has_torque_cmd) {
      std::cout << "TORQUE tau=[" << std::fixed << std::setprecision(3)
                << cmd.control.tau_J_d[0] << "," << cmd.control.tau_J_d[1] << "," << cmd.control.tau_J_d[2] << "," 
                << cmd.control.tau_J_d[3] << "," << cmd.control.tau_J_d[4] << "," << cmd.control.tau_J_d[5] << "," 
                << cmd.control.tau_J_d[6] << "]";
    }
    if (!has_position_cmd && !has_cartesian_cmd && !has_torque_cmd) {
      std::cout << "NO_COMMANDS (all zeros)";
    }
    std::cout << std::endl;
  }
  
  // Log first few commands in detail
  if (cmd_count <= 200) {  // Increased from 50 to 200 to see more of the trajectory
    std::cout << "[FCI] *** DETAILED COMMAND #" << cmd_count << " ***" << std::endl;
    std::cout << "  Joint positions: [" << std::fixed << std::setprecision(4);
    for (int i = 0; i < 7; ++i) {
      std::cout << cmd.motion.q_c[i] << (i < 6 ? ", " : "");
    }
    std::cout << "]" << std::endl;
    std::cout << "  Torques: [";
    for (int i = 0; i < 7; ++i) {
      std::cout << std::fixed << std::setprecision(4) << cmd.control.tau_J_d[i] << (i < 6 ? ", " : "");
    }
    std::cout << "]" << std::endl;
    std::cout << "  Controller Mode: " << static_cast<int>(g_robot_state.current_controller_mode) << std::endl;
    std::cout << "  Flags: pos=" << has_position_cmd << " cart=" << has_cartesian_cmd << " tau=" << has_torque_cmd << std::endl;
    
    // SPECIAL ALERT: Check for non-zero torques
    bool has_nonzero_torque = false;
    double max_torque = 0.0;
    int max_torque_joint = -1;
    for (int i = 0; i < 7; ++i) {
      if (std::abs(cmd.control.tau_J_d[i]) > 1e-6) {
        has_nonzero_torque = true;
        if (std::abs(cmd.control.tau_J_d[i]) > std::abs(max_torque)) {
          max_torque = cmd.control.tau_J_d[i];
          max_torque_joint = i;
        }
      }
    }
    
    if (has_nonzero_torque) {
      std::cout << "  🔥 NON-ZERO TORQUE DETECTED! Max: " << std::fixed << std::setprecision(6) 
                << max_torque << " Nm on joint " << (max_torque_joint + 1) << " 🔥" << std::endl;
    }
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

  // FCI position controller
  auto* fci_position_controller = builder.AddSystem<FciPidController>();
  builder.Connect(plant.get_state_output_port(), fci_position_controller->get_input_port(fci_position_controller->get_state_input_port()));

  // Torque adder: gravity compensation + FCI commands
  auto* adder = builder.AddSystem<drake::systems::Adder<double>>(2, 7);
  builder.Connect(g_comp->get_output_port(), adder->get_input_port(0));
  builder.Connect(fci_position_controller->get_output_port(), adder->get_input_port(1));
  builder.Connect(adder->get_output_port(), plant.get_actuation_input_port());

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
  Eigen::VectorXd q_des(n), v_des(plant.num_velocities(robot));
  q_des << 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/2;
  v_des.setZero();
  plant.SetPositions(&plant_ctx, robot, q_des);
  plant.SetVelocities(&plant_ctx, robot, v_des);

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
    std::cout << "[FCI] Controller mode changed to: " << static_cast<int>(controller_mode) 
              << " (0=JointImpedance, 1=CartesianImpedance, 2=ExternalController)" << std::endl;
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
  
  // Run Drake simulation on main thread (for Meshcat compatibility)
  const double sim_step = 0.001; // 1ms steps
  const std::vector<std::string> joint_names = {
      "fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4",
      "fer_joint5", "fer_joint6", "fer_joint7"
  };
  
  // TURBO MODE: Allow faster-than-real-time simulation
  bool turbo_mode = (argc >= 6 && std::string(argv[5]) == "turbo");
  if (turbo_mode) {
    std::cout << "TURBO MODE ENABLED - Running faster than real-time!" << std::endl;
    simulator.set_target_realtime_rate(0.0);  // No real-time constraint
  }
  
  int sim_loop_count = 0;
  auto last_log_time = std::chrono::steady_clock::now();
  auto sim_start_time = std::chrono::steady_clock::now();
  std::array<double, 7> prev_q = {0, 0, 0, 0, 0, 0, 0};  // Previous joint positions for movement detection
  
  // Performance monitoring
  auto last_perf_time = sim_start_time;
  int perf_step_count = 0;
  double total_physics_time = 0.0;
  
  while (g_sim_running.load()) {
    auto loop_start = std::chrono::steady_clock::now();
    
    if (turbo_mode) {
      // TURBO MODE: Run as fast as possible
      simulator.AdvanceTo(simulator.get_context().get_time() + sim_step);
    } else {
      // REAL-TIME MODE: Calculate target time based on wall-clock time
      auto current_wall_time = std::chrono::steady_clock::now();
      double elapsed_wall_time = std::chrono::duration<double>(current_wall_time - sim_start_time).count();
      double target_sim_time = elapsed_wall_time; // 1:1 real-time ratio
      
      // Only advance simulation if we're behind target time
      if (simulator.get_context().get_time() < target_sim_time) {
        const double advance_to = simulator.get_context().get_time() + sim_step;
        simulator.AdvanceTo(std::min(advance_to, target_sim_time));
      }
    }
    
    auto physics_end = std::chrono::steady_clock::now();
    double physics_time = std::chrono::duration<double>(physics_end - loop_start).count();
    total_physics_time += physics_time;
    perf_step_count++;
    
    // Update shared state with current Drake state (do this less frequently)
    if (sim_loop_count % 10 == 0) {  // Update every 10ms instead of every 1ms
      std::lock_guard<std::mutex> lock(g_robot_state.mutex);
      auto& context = simulator.get_context();
      auto& plant_context = plant.GetMyContextFromRoot(context);
      
      // Extract joint positions and velocities
      for (int i = 0; i < 7; ++i) {
        const auto& joint = plant.GetJointByName<drake::multibody::RevoluteJoint>(joint_names[i], robot);
        g_robot_state.q[i] = joint.get_angle(plant_context);
        g_robot_state.dq[i] = joint.get_angular_rate(plant_context);
        
        // For now, use simple gravity compensation torque estimate
        // This gives libfranka realistic torque values to validate
        double gravity_torque = 0.1 * sin(g_robot_state.q[i]); // Simple gravity model
        g_robot_state.tau_J[i] = gravity_torque;
      }
      
      // Get end-effector pose
      const auto& ee_frame = plant.GetFrameByName("fer_link7", robot);
      const auto& world_frame = plant.world_frame();
      const drake::math::RigidTransformd X_W_EE = plant.CalcRelativeTransform(plant_context, world_frame, ee_frame);
      const Eigen::Matrix4d T_matrix = X_W_EE.GetAsMatrix4();
      
      // Store as column-major array (Drake/libfranka convention)
      for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 4; ++row) {
          g_robot_state.O_T_EE[col * 4 + row] = T_matrix(row, col);
        }
      }
    }
    
    // Logging logic - different frequencies for headless vs normal mode
    sim_loop_count++;
    auto now = std::chrono::steady_clock::now();
    double log_interval = headless ? 0.5 : 2.0;  // 500ms for headless (slower), 2s for normal
    
    // Performance monitoring every 5 seconds
    if (std::chrono::duration<double>(now - last_perf_time).count() >= 5.0) {
      double avg_physics_time = total_physics_time / perf_step_count;
      double real_time_factor = simulator.get_context().get_time() / std::chrono::duration<double>(now - sim_start_time).count();
      
      std::cout << "[PERFORMANCE] Avg physics time: " << std::fixed << std::setprecision(4) 
                << avg_physics_time * 1000.0 << "ms" << std::endl;
      std::cout << "[PERFORMANCE] Real-time factor: " << std::fixed << std::setprecision(2) 
                << real_time_factor << "x" << std::endl;
      std::cout << "[PERFORMANCE] Simulation frequency: " << std::fixed << std::setprecision(1) 
                << perf_step_count / 5.0 << " Hz (target: 1000 Hz)" << std::endl;
      
      // Reset counters
      last_perf_time = now;
      perf_step_count = 0;
      total_physics_time = 0.0;
    }
    
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count() >= log_interval * 1000) {
      std::lock_guard<std::mutex> lock(g_robot_state.mutex);
      
      if (headless) {
        // Headless mode: Log all joint positions and detect movement
        bool movement_detected = false;
        for (int i = 0; i < 7; ++i) {
          if (std::abs(g_robot_state.q[i] - prev_q[i]) > 1e-6) {
            movement_detected = true;
            break;
          }
        }
        
        double elapsed_wall_time = std::chrono::duration<double>(now - sim_start_time).count();
        double real_time_factor = simulator.get_context().get_time() / elapsed_wall_time;
        
        std::cout << "[HEADLESS] t=" << std::fixed << std::setprecision(3) << simulator.get_context().get_time() << "s ";
        std::cout << "q=[";
        for (int i = 0; i < 7; ++i) {
          std::cout << std::fixed << std::setprecision(4) << g_robot_state.q[i];
          if (i < 6) std::cout << ", ";
        }
        std::cout << "] ";
        std::cout << "cmds=[" << g_robot_state.has_position_command << g_robot_state.has_cartesian_command << g_robot_state.has_torque_command << "]";
        if (movement_detected) {
          std::cout << " *** MOVEMENT DETECTED! ***";
        }
        std::cout << " RTF=" << std::fixed << std::setprecision(2) << real_time_factor << "x";
        if (turbo_mode) std::cout << " [TURBO]";
        std::cout << std::endl;
        
        // Update previous positions
        for (int i = 0; i < 7; ++i) {
          prev_q[i] = g_robot_state.q[i];
        }
      } else {
        // Normal mode: Log summary every 2 seconds with ALL joint positions
        double elapsed_wall_time = std::chrono::duration<double>(now - sim_start_time).count();
        double real_time_factor = simulator.get_context().get_time() / elapsed_wall_time;
        
        std::cout << "[Drake] Sim time: " << std::fixed << std::setprecision(3) << simulator.get_context().get_time() << "s";
        std::cout << " RTF: " << std::fixed << std::setprecision(2) << real_time_factor << "x";
        if (turbo_mode) std::cout << " [TURBO MODE]";
        std::cout << std::endl;
        
        std::cout << "  Current q: [";
        for (int i = 0; i < 7; ++i) {
          std::cout << std::fixed << std::setprecision(4) << g_robot_state.q[i] << (i < 6 ? ", " : "");
        }
        std::cout << "]" << std::endl;
        
        if (g_robot_state.has_position_command) {
          std::cout << "  Target  q: [";
          for (int i = 0; i < 7; ++i) {
            std::cout << std::fixed << std::setprecision(4) << g_robot_state.q_cmd[i] << (i < 6 ? ", " : "");
          }
          std::cout << "]" << std::endl;
          
          std::cout << "  Error   q: [";
          for (int i = 0; i < 7; ++i) {
            double error = g_robot_state.q_cmd[i] - g_robot_state.q[i];
            std::cout << std::fixed << std::setprecision(4) << error << (i < 6 ? ", " : "");
          }
          std::cout << "]" << std::endl;
        }
        
        std::cout << "  Commands: pos=" << g_robot_state.has_position_command 
                  << " cart=" << g_robot_state.has_cartesian_command 
                  << " tau=" << g_robot_state.has_torque_command 
                  << ", EE_z=" << std::fixed << std::setprecision(4) << g_robot_state.O_T_EE[14] << std::endl;
      }
      last_log_time = now;
    }
    
    // Sleep logic: Only sleep in real-time mode
    if (!turbo_mode) {
      // CRITICAL: Sleep for exactly 1ms to maintain 1kHz loop rate in real-time mode
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    // In turbo mode, no sleep - run as fast as possible!
  }

  // Cleanup
  server.stop();
  fci_thread.join();
  
  return 0;
} 