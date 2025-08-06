// Copyright (c) 2024 Franka Emika GmbH
// Cartesian trajectory following - demonstrates smooth path tracking in task space
#include <cmath>
#include <iostream>
#include <iomanip>
#include <vector>
#include <array>
#include <Eigen/Dense>

#include <franka/active_motion_generator.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include "examples_common.h"

// Trajectory parameters
constexpr double TRAJECTORY_DURATION = 15.0;  // Total duration in seconds
constexpr double SAMPLE_TIME = 0.001;         // 1 kHz control rate

// 3D Lissajous curve parameters
constexpr double A = 0.1;   // Amplitude in X (meters)
constexpr double B = 0.08;  // Amplitude in Y (meters)
constexpr double C = 0.05;  // Amplitude in Z (meters)
constexpr double OMEGA_X = 2.0 * M_PI / 5.0;   // X frequency
constexpr double OMEGA_Y = 3.0 * M_PI / 5.0;   // Y frequency
constexpr double OMEGA_Z = 4.0 * M_PI / 5.0;   // Z frequency

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    // Connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    
    // Load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    // First move to initial position
    std::array<double, 7> q_goal = {{0, -M_PI/4, 0, -3*M_PI/4, 0, M_PI/2, M_PI/4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "Moving to initial position..." << std::endl;
    robot.control(motion_generator);

    // Get initial pose
    auto initial_state = robot.readOnce();
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d initial_position(initial_transform.translation());
    Eigen::Quaterniond initial_orientation(initial_transform.rotation());

    std::cout << "\nStarting Cartesian trajectory following demo..." << std::endl;
    std::cout << "The end-effector will follow a 3D Lissajous curve." << std::endl;
    std::cout << "Initial position: [" 
              << initial_position[0] << ", " 
              << initial_position[1] << ", " 
              << initial_position[2] << "]" << std::endl;

    // Control callback
    double time = 0;
    bool trajectory_initialized = false;
    Eigen::Vector3d trajectory_offset;
    
    auto cartesian_trajectory_callback = [&](const franka::RobotState& robot_state,
                                            franka::Duration period) -> franka::CartesianPose {
      time += period.toSec();

      // Initialize trajectory offset on first call
      if (!trajectory_initialized) {
        Eigen::Affine3d current_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        trajectory_offset = current_transform.translation();
        trajectory_initialized = true;
      }

      // Generate Lissajous curve trajectory
      double x = A * std::sin(OMEGA_X * time);
      double y = B * std::sin(OMEGA_Y * time + M_PI/3);  // Phase shift for interesting pattern
      double z = C * std::sin(OMEGA_Z * time + M_PI/6);

      // Compute desired position (offset from initial position)
      Eigen::Vector3d desired_position = trajectory_offset + Eigen::Vector3d(x, y, z);

      // Create desired transformation matrix (keep orientation constant)
      Eigen::Affine3d desired_transform(initial_transform);
      desired_transform.translation() = desired_position;

      // Convert to array format
      std::array<double, 16> desired_pose;
      Eigen::Map<Eigen::Matrix4d>(&desired_pose[0], 4, 4) = desired_transform.matrix();

      // Print progress and trajectory info
      static int print_counter = 0;
      if (++print_counter >= 100) {  // Every 100ms
        print_counter = 0;
        
        // Calculate current error
        Eigen::Affine3d current_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Vector3d position_error = desired_position - current_transform.translation();
        double error_magnitude = position_error.norm();

        std::cout << "\rTime: " << std::fixed << std::setprecision(1) << time << "s"
                  << " | Progress: " << std::setprecision(1) << (time / TRAJECTORY_DURATION * 100) << "%"
                  << " | Error: " << std::setprecision(4) << error_magnitude * 1000 << "mm     " 
                  << std::flush;

        // Periodically print detailed position info
        if (static_cast<int>(time) % 3 == 0 && static_cast<int>(time * 10) % 10 == 0) {
          std::cout << "\n  Position: [" 
                    << std::setprecision(3) << desired_position[0] << ", "
                    << desired_position[1] << ", " 
                    << desired_position[2] << "] m"
                    << " | Actual: ["
                    << current_transform.translation()[0] << ", "
                    << current_transform.translation()[1] << ", "
                    << current_transform.translation()[2] << "] m" << std::endl;
        }
      }

      // Check if trajectory is complete
      if (time >= TRAJECTORY_DURATION) {
        std::cout << "\n\nTrajectory completed!" << std::endl;
        
        // Print final statistics
        Eigen::Affine3d current_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Vector3d final_error = trajectory_offset - current_transform.translation();
        std::cout << "Final position error: " << final_error.norm() * 1000 << " mm" << std::endl;
        
        return franka::MotionFinished(franka::CartesianPose(desired_pose, robot_state.elbow));
      }

      return franka::CartesianPose(desired_pose, robot_state.elbow);
    };

    // Execute trajectory
    robot.control(cartesian_trajectory_callback);

    std::cout << "\nCartesian trajectory execution finished successfully." << std::endl;
    std::cout << "Trajectory type: 3D Lissajous curve" << std::endl;
    std::cout << "Total duration: " << TRAJECTORY_DURATION << " seconds" << std::endl;
    std::cout << "Amplitudes: X=" << A*1000 << "mm, Y=" << B*1000 << "mm, Z=" << C*1000 << "mm" << std::endl;

  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
    return -1;
  }

  return 0;
}