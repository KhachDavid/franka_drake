// Copyright (c) 2024 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
#include <iomanip>
#include <vector>

#include <franka/active_motion_generator.h>
#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

/**
 * @example sine_wave_velocity_control.cpp
 * An example showing smooth joint velocity control using sine waves.
 *
 * This demonstrates velocity control mode by moving multiple joints
 * with synchronized sine wave patterns.
 */

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    // Parameters for sine wave motion
    const double duration = 10.0;  // 10 seconds
    std::array<double, 7> amplitudes = {{0.2, 0.15, 0.2, 0.15, 0.1, 0.1, 0.1}};  // rad/s
    std::array<double, 7> frequencies = {{0.5, 0.6, 0.4, 0.7, 0.8, 0.9, 1.0}};  // Hz
    std::array<double, 7> phases = {{0, M_PI/4, M_PI/2, 3*M_PI/4, M_PI, 5*M_PI/4, 3*M_PI/2}};  // rad

    double time = 0.0;

    // Velocity control callback
    auto sine_wave_velocity = [&time, &amplitudes, &frequencies, &phases, duration](
                                 const franka::RobotState& robot_state,
                                 franka::Duration period) -> franka::JointVelocities {
      time += period.toSec();

      if (time > duration) {
        std::cout << std::endl << "Finished sine wave motion." << std::endl;
        // Return zero velocities to stop
        return franka::MotionFinished(franka::JointVelocities{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
      }

      franka::JointVelocities velocities{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
      
      // Generate sine wave velocities for each joint
      for (size_t i = 0; i < 7; i++) {
        velocities.dq[i] = amplitudes[i] * std::sin(2 * M_PI * frequencies[i] * time + phases[i]);
      }

      // Print status every second
      static double last_print_time = 0.0;
      if (time - last_print_time > 1.0) {
        std::cout << "Time: " << time << "s, Joint velocities: [";
        for (size_t i = 0; i < 7; i++) {
          std::cout << std::fixed << std::setprecision(3) << velocities.dq[i];
          if (i < 6) std::cout << ", ";
        }
        std::cout << "] rad/s" << std::endl;
        last_print_time = time;
      }

      return velocities;
    };

    std::cout << "WARNING: This example will move the robot with velocity control!" << std::endl
              << "Joint velocities will follow sine wave patterns." << std::endl
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // Start velocity control
    std::cout << "Starting sine wave velocity control..." << std::endl;
    robot.control(sine_wave_velocity);

  } catch (const franka::Exception& ex) {
    std::cerr << ex.what() << std::endl;
    return -1;
  }

  return 0;
}