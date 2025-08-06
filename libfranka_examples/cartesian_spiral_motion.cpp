// Copyright (c) 2024 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/active_motion_generator.h>
#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

/**
 * @example cartesian_spiral_motion.cpp
 * An example showing how to generate a Cartesian spiral motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion1(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion1);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    std::array<double, 16> initial_pose;
    double time = 0.0;
    const double radius = 0.05;  // 5cm radius
    const double pitch = 0.02;   // 2cm vertical movement per revolution
    const double angular_velocity = 0.5;  // rad/s
    const double duration = 20.0;  // 20 seconds total

    auto cartesian_spiral_motion = [&time, &initial_pose, radius, pitch, angular_velocity, duration](
                                       const franka::RobotState& robot_state,
                                       franka::Duration period) -> franka::CartesianPose {
      if (time == 0.0) {
        initial_pose = robot_state.O_T_EE_c;
      }

      time += period.toSec();

      if (time > duration) {
        std::cout << std::endl << "Finished spiral motion." << std::endl;
        return franka::MotionFinished(initial_pose);
      }

      // Calculate spiral trajectory
      double angle = angular_velocity * time;
      double x_offset = radius * std::cos(angle);
      double y_offset = radius * std::sin(angle);
      double z_offset = (pitch / (2 * M_PI)) * angle;  // Linear z movement based on angle

      std::array<double, 16> new_pose = initial_pose;
      // Update translation (keeping rotation fixed)
      new_pose[12] = initial_pose[12] + x_offset;  // X
      new_pose[13] = initial_pose[13] + y_offset;  // Y
      new_pose[14] = initial_pose[14] + z_offset;  // Z

      // Optional: print progress every second
      static double last_print_time = 0.0;
      if (time - last_print_time > 1.0) {
        std::cout << "Time: " << time << "s, Z-offset: " << z_offset * 1000 << "mm" << std::endl;
        last_print_time = time;
      }

      return new_pose;
    };

    // Start real-time control loop
    std::cout << "Starting Cartesian spiral motion..." << std::endl;
    robot.control(cartesian_spiral_motion);

  } catch (const franka::Exception& ex) {
    std::cerr << ex.what() << std::endl;
    return -1;
  }

  return 0;
}