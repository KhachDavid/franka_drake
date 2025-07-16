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

// Drake system to provide FCI commands as input - PROPER POSITION CONTROL
class FciPositionController final : public drake::systems::LeafSystem<double> {
 public:
   FciPositionController() {
     // Declare input ports for current robot state
     state_input_port_ = this->DeclareVectorInputPort("robot_state", drake::systems::BasicVector<double>(14)).get_index();
     
     // Declare output port for computed control torques
     this->DeclareVectorOutputPort("control_torques", drake::systems::BasicVector<double>(7),
                                  &FciPositionController::CalcControlTorques);
   }

   // Public accessor for state input port
   int get_state_input_port() const { return state_input_port_; }

 private:
   void CalcControlTorques(const drake::systems::Context<double>& context,
                          drake::systems::BasicVector<double>* output) const {
     std::lock_guard<std::mutex> lock(g_robot_state.mutex);
     
     // Get current robot state from input
     const auto& state_input = this->get_input_port(state_input_port_).Eval(context);
     
     // Extract current positions and velocities
     Eigen::VectorXd current_q = state_input.head(7);   // positions
     Eigen::VectorXd current_dq = state_input.tail(7);  // velocities
     
     Eigen::VectorXd control_torques(7);
     
     if (g_robot_state.has_position_command) {
       // POSITION CONTROL: Use PD controller to track commanded positions
       Eigen::VectorXd target_q(7);
       for (int i = 0; i < 7; ++i) {
         target_q[i] = g_robot_state.q_cmd[i];
       }
       
       // PD gains (tuned for smooth motion)
       Eigen::VectorXd kp(7);  kp << 200, 200, 200, 200, 100, 100, 50;  // Position gains
       Eigen::VectorXd kd(7);  kd << 20, 20, 20, 20, 10, 10, 5;         // Velocity gains
       
       // PD control law: tau = Kp*(q_desired - q_current) - Kd*dq_current
       control_torques = kp.cwiseProduct(target_q - current_q) - kd.cwiseProduct(current_dq);
       
       // Debug logging (every 1000 calls to avoid spam)
       static int debug_count = 0;
       if (++debug_count % 1000 == 0) {
         std::cout << "[PD Controller] Target: [" << target_q.transpose() << "]" << std::endl;
         std::cout << "[PD Controller] Current: [" << current_q.transpose() << "]" << std::endl;
         std::cout << "[PD Controller] Error: [" << (target_q - current_q).transpose() << "]" << std::endl;
         std::cout << "[PD Controller] Control torques: [" << control_torques.transpose() << "]" << std::endl;
       }
       
       // Add any direct torque commands from libfranka
       for (int i = 0; i < 7; ++i) {
         control_torques[i] += g_robot_state.tau_cmd[i];
       }
       
     } else if (g_robot_state.has_torque_command) {
       // TORQUE CONTROL: Direct torque application
       for (int i = 0; i < 7; ++i) {
         control_torques[i] = g_robot_state.tau_cmd[i];
       }
     } else {
       // NO COMMANDS: Zero torques (gravity compensation will handle this)
       control_torques.setZero();
     }
     
     // Apply torque limits for safety
     const double max_torque = 50.0;  // Conservative limit
     for (int i = 0; i < 7; ++i) {
       control_torques[i] = std::max(-max_torque, std::min(max_torque, control_torques[i]));
     }
     
     output->SetFromVector(control_torques);
   }
   
   int state_input_port_;
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
  std::lock_guard<std::mutex> lock(g_robot_state.mutex);
  
  // Store all commands
  g_robot_state.tau_cmd = cmd.control.tau_J_d;
  g_robot_state.q_cmd = cmd.motion.q_c;
  g_robot_state.O_T_EE_cmd = cmd.motion.O_T_EE_c;
  
  // Check for meaningful commands (not just zero/default values)
  bool has_position_cmd = false;
  bool has_cartesian_cmd = false; 
  bool has_torque_cmd = false;
  
  // FIXED: Check joint position commands by comparing with current position
  // A position command is valid if it's significantly different from current position
  for (int i = 0; i < 7; ++i) {
    double current_pos = g_robot_state.q[i];
    double commanded_pos = cmd.motion.q_c[i];
    if (std::abs(commanded_pos - current_pos) > 0.01) {  // 0.01 radians = ~0.57 degrees threshold
      has_position_cmd = true;
      break;
    }
  }
  
  // If no significant difference detected, but we have any non-zero commanded positions, still treat as position command
  // This handles initial commands or cases where current position isn't properly initialized
  if (!has_position_cmd) {
    for (int i = 0; i < 7; ++i) {
      if (std::abs(cmd.motion.q_c[i]) > 1e-6) {
        has_position_cmd = true;
        break;
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
  
  // FIXED: Determine torque command based on controller mode from Move request
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
  if (cmd_count <= 50) {  // Increased from 10 to 50 to catch torque ramp-up
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
    std::cerr << "Usage: " << argv[0] << " <package.xml> <arm.urdf> <time_step> [headless]\n";
    std::cerr << "  headless: 'true' to run without visualization and log joint positions continuously\n";
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
  auto* fci_position_controller = builder.AddSystem<FciPositionController>();
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
  
  int sim_loop_count = 0;
  auto last_log_time = std::chrono::steady_clock::now();
  auto sim_start_time = std::chrono::steady_clock::now();
  std::array<double, 7> prev_q = {0, 0, 0, 0, 0, 0, 0};  // Previous joint positions for movement detection
  
  while (g_sim_running.load()) {
    // REAL-TIME SYNCHRONIZATION: Calculate target time based on wall-clock time
    auto current_wall_time = std::chrono::steady_clock::now();
    double elapsed_wall_time = std::chrono::duration<double>(current_wall_time - sim_start_time).count();
    double target_sim_time = elapsed_wall_time; // 1:1 real-time ratio
    
    // Only advance simulation if we're behind target time
    if (simulator.get_context().get_time() < target_sim_time) {
      const double advance_to = simulator.get_context().get_time() + sim_step;
      simulator.AdvanceTo(std::min(advance_to, target_sim_time));
    }
    
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
        std::cout << " (wall_time_ratio=" << std::fixed << std::setprecision(2) << simulator.get_context().get_time() / elapsed_wall_time << ")";
        std::cout << std::endl;
        
        // Update previous positions
        for (int i = 0; i < 7; ++i) {
          prev_q[i] = g_robot_state.q[i];
        }
      } else {
        // Normal mode: Log summary every 2 seconds with ALL joint positions
        std::cout << "[Drake] Sim time: " << std::fixed << std::setprecision(3) << simulator.get_context().get_time() << "s" << std::endl;
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
    
    // CRITICAL: Sleep for exactly 1ms to maintain 1kHz loop rate
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // Cleanup
  server.stop();
  fci_thread.join();
  
  return 0;
} 