// Simple joint motion test - moves only joint 1 for debugging
#include <iostream>
#include <cmath>
#include <array>
#include <franka/robot.h>
#include <franka/exception.h>
#include "examples_common.h"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);
    std::cout << "Connected to robot" << std::endl;
    
    // Set default collision behavior
    setDefaultBehavior(robot);
    std::cout << "Set default behavior" << std::endl;
    
    // Get initial state
    auto state = robot.readOnce();
    std::cout << "Initial joint positions: [";
    for (int i = 0; i < 7; ++i) {
      std::cout << state.q[i] << (i < 6 ? ", " : "");
    }
    std::cout << "]" << std::endl;
    
    // Simple motion: Move joint 1 by 0.5 radians
    double time = 0;
    std::cout << "Starting motion..." << std::endl;
    
    robot.control([&time, &state](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointPositions {
      time += period.toSec();
      
      // Create target positions (copy current positions)
      franka::JointPositions output(robot_state.q);
      
      // Only modify joint 1 with a simple sine motion
      output.q[0] = state.q[0] + 0.5 * std::sin(time);
      
      // Print every 0.5 seconds
      static double last_print = 0;
      if (time - last_print > 0.5) {
        std::cout << "Time: " << time << "s, Joint 1: " << output.q[0] << " rad" << std::endl;
        last_print = time;
      }
      
      // Run for 5 seconds
      if (time >= 5.0) {
        std::cout << "Motion finished!" << std::endl;
        return franka::MotionFinished(output);
      }
      
      return output;
    });
    
    std::cout << "Control loop completed successfully" << std::endl;
    
  } catch (const franka::Exception& e) {
    std::cerr << "Franka exception: " << e.what() << std::endl;
    return -1;
  }
  
  return 0;
}