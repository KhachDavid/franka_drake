// Simple force control example - applies constant downward force without model
#include <array>
#include <cmath>
#include <iostream>
#include <franka/exception.h>
#include <franka/robot.h>
#include "examples_common.h"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    
    std::cout << "WARNING: This example will apply a constant downward force!" << std::endl;
    std::cout << "Make sure the robot has space below and won't hit anything." << std::endl;
    std::cout << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // Set additional parameters for force control
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    double time = 0.0;
    std::cout << "Starting force control..." << std::endl;
    
    robot.control([&time](const franka::RobotState& robot_state,
                          franka::Duration period) -> franka::Torques {
      time += period.toSec();

      // Simple force control without model:
      // Apply a small constant torque to create downward force
      // This is a simplified approach that doesn't use the Jacobian
      
      franka::Torques torques{};
      
      // Apply small torques to create a gentle downward motion
      // These values are tuned for safety - adjust carefully!
      torques.tau_J[0] = 0.0;   // Base joint - no torque
      torques.tau_J[1] = -2.0;  // Shoulder - slight downward
      torques.tau_J[2] = 0.0;   // No torque
      torques.tau_J[3] = -1.0;  // Elbow - slight downward
      torques.tau_J[4] = 0.0;   // No torque
      torques.tau_J[5] = 0.0;   // No torque
      torques.tau_J[6] = 0.0;   // No torque
      
      // Smooth ramp up/down
      double ramp = 1.0;
      if (time < 1.0) {
        // Ramp up
        ramp = time;
      } else if (time > 4.0) {
        // Ramp down
        ramp = std::max(0.0, 5.0 - time);
      }
      
      // Apply ramp to all torques
      for (size_t i = 0; i < 7; ++i) {
        torques.tau_J[i] *= ramp;
      }
      
      // Print status every 0.5 seconds
      static double last_print = 0;
      if (time - last_print > 0.5) {
        std::cout << "Time: " << time << "s, Ramp: " << ramp 
                  << ", Torques: [";
        for (size_t i = 0; i < 7; ++i) {
          std::cout << torques.tau_J[i] << (i < 6 ? ", " : "");
        }
        std::cout << "]" << std::endl;
        last_print = time;
      }
      
      // Stop after 5 seconds
      if (time >= 5.0) {
        std::cout << "\nFinished force control demo." << std::endl;
        return franka::MotionFinished(torques);
      }
      
      return torques;
    });

  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
    return -1;
  }

  return 0;
}