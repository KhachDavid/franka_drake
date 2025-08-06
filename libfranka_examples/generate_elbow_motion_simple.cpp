#include <cmath>
#include <iostream>
#include <array>
#include <franka/exception.h>
#include <franka/robot.h>

/**
 * @example generate_elbow_motion_simple.cpp
 * A simplified version of elbow motion example without external dependencies.
 * This moves the robot's elbow while keeping the end-effector pose constant.
 */

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  
  try {
    franka::Robot robot(argv[1]);
    
    // Set collision behavior
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, 
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, 
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, 
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, 
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    std::array<double, 16> initial_pose;
    std::array<double, 2> initial_elbow;
    double time = 0.0;
    bool started = false;
    
    std::cout << "Starting elbow motion for 10 seconds..." << std::endl;
    std::cout << "The robot will move its elbow while keeping the end-effector stationary." << std::endl;
    
    robot.control(
        [&time, &initial_pose, &initial_elbow, &started](const franka::RobotState& robot_state,
                                               franka::Duration period) -> franka::CartesianPose {
          time += period.toSec();

          if (!started) {
            initial_pose = robot_state.O_T_EE_c;
            initial_elbow = robot_state.elbow_c;
            started = true;
            std::cout << "Initial elbow configuration: [" << initial_elbow[0] << ", " << initial_elbow[1] << "]" << std::endl;
          }

          // Generate sinusoidal elbow motion
          double angle = M_PI / 10.0 * (1.0 - std::cos(M_PI / 5.0 * time));

          auto elbow = initial_elbow;
          elbow[0] += angle;
          
          // Print progress every second
          static double last_print = 0.0;
          if (time - last_print >= 1.0) {
            std::cout << "Time: " << time << "s, Elbow angle offset: " << angle << " rad" << std::endl;
            std::cout << "Current joint positions: [";
            for (size_t i = 0; i < 7; ++i) {
              std::cout << robot_state.q[i];
              if (i < 6) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
            last_print = time;
          }

          if (time >= 10.0) {
            std::cout << "\nMotion complete!" << std::endl;
            std::cout << "Final elbow configuration: [" << elbow[0] << ", " << elbow[1] << "]" << std::endl;
            std::cout << "Total elbow rotation: " << angle << " rad (" << angle * 180.0 / M_PI << " degrees)" << std::endl;
            return franka::MotionFinished({initial_pose, elbow});
          }

          // Return Cartesian pose with elbow configuration
          return {initial_pose, elbow};
        });
        
    std::cout << "Elbow motion example finished successfully!" << std::endl;
    
  } catch (const franka::Exception& e) {
    std::cout << "Franka exception: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}