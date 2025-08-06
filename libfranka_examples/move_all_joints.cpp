#include <cmath>
#include <iostream>
#include <array>
#include <franka/robot.h>
#include <franka/rate_limiting.h>
#include <franka/exception.h>

/**
 * @example move_all_joints.cpp
 * An example that moves all 7 joints of the robot to demonstrate full position control.
 * Each joint moves to a different target position using a smooth trajectory.
 */

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-ip>" << std::endl;
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);
    
    // Set default collision behavior
    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},  // Lower torque thresholds
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},  // Upper torque thresholds
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},  // Lower force thresholds
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},  // Upper force thresholds
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},         // Lower force thresholds (acc)
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},         // Upper force thresholds (acc)
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},         // Lower force thresholds (jerk)
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}});        // Upper force thresholds (jerk)

    // Set joint impedance
    robot.setJointImpedance({{300, 300, 300, 250, 250, 200, 200}});

    // Move to initial position
    std::array<double, 7> q_start{{0, -M_PI/4, 0, -3*M_PI/4, 0, M_PI/2, M_PI/2}};
    std::array<double, 7> q_goal{{0, -M_PI/4, 0, -3*M_PI/4, 0, M_PI/2, M_PI/2}};
    
    // Define target deltas for each joint
    std::array<double, 7> delta_q{{
        0.2,    // Joint 1: +0.2 rad
        -0.15,  // Joint 2: -0.15 rad  
        0.25,   // Joint 3: +0.25 rad
        -0.2,   // Joint 4: -0.2 rad
        0.3,    // Joint 5: +0.3 rad
        -0.25,  // Joint 6: -0.25 rad
        -0.2    // Joint 7: -0.2 rad (same as original example)
    }};
    
    double time = 0.0;
    double duration = 4.0;  // 4 seconds for the motion
    bool motion_started = false;
    
    std::cout << "Moving all 7 joints simultaneously..." << std::endl;
    std::cout << "Target changes (rad): [";
    for (size_t i = 0; i < 7; ++i) {
        std::cout << delta_q[i] << (i < 6 ? ", " : "");
    }
    std::cout << "]" << std::endl;
    
    // Maximum velocities for each joint
    constexpr std::array<double, 7> kMaxVel{{2.0, 1.0, 1.5, 1.25, 3.0, 1.5, 3.0}};
    std::array<double, 7> q_cmd_last{};
    
    robot.control([&](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointPositions {
        time += period.toSec();
        
        if (!motion_started) {
            q_start = robot_state.q_d;
            q_goal = q_start;
            
            // Apply deltas to all joints
            for (size_t i = 0; i < 7; ++i) {
                q_goal[i] += delta_q[i];
            }
            
            q_cmd_last = q_start;
            motion_started = true;
        }
        
        // Smooth cosine trajectory
        double s = 0.5 * (1.0 - std::cos(M_PI * std::min(1.0, time / duration)));
        
        // Interpolate all joints
        std::array<double, 7> q_cmd{};
        for (size_t i = 0; i < 7; ++i) {
            q_cmd[i] = q_start[i] + (q_goal[i] - q_start[i]) * s;
        }
        
        // Apply velocity limits
        q_cmd = franka::limitRate(kMaxVel, q_cmd, q_cmd_last);
        q_cmd_last = q_cmd;
        
        // Check if motion is complete
        if (time >= duration) {
            std::cout << "\nMotion complete! Final errors (rad):" << std::endl;
            for (size_t i = 0; i < 7; ++i) {
                double error = robot_state.q[i] - q_goal[i];
                std::cout << "  Joint " << (i + 1) << ": " << error << std::endl;
            }
            
            // Calculate total error
            double total_error = 0.0;
            for (size_t i = 0; i < 7; ++i) {
                double error = robot_state.q[i] - q_goal[i];
                total_error += error * error;
            }
            total_error = std::sqrt(total_error);
            std::cout << "Total RMS error: " << total_error << " rad" << std::endl;
            
            return franka::MotionFinished(franka::JointPositions(q_cmd));
        }
        
        return franka::JointPositions(q_cmd);
    });
    
    std::cout << "Motion control finished successfully!" << std::endl;
    
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  
  return 0;
}