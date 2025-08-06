// Copyright (c) 2024 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <thread>

#include <franka/active_torque_control.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

/**
 * @example gravity_compensation_toggle.cpp
 * An example that allows toggling gravity compensation on/off.
 *
 * When gravity compensation is OFF, the robot will feel heavy and drop.
 * When ON, the robot will feel weightless and can be moved by hand.
 *
 * Press Enter to toggle gravity compensation during execution.
 */

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try {
    // Connect to robot
    franka::Robot robot(argv[1]);
    
    // Set collision behavior for safe interaction
    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});

    // Load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    // Atomic flag for gravity compensation state
    std::atomic<bool> gravity_compensation_enabled{true};
    std::atomic<bool> running{true};

    // Thread to handle user input
    std::thread input_thread([&gravity_compensation_enabled, &running]() {
      std::cout << "\nGravity compensation is initially ON." << std::endl;
      std::cout << "Press Enter to toggle gravity compensation, or 'q' + Enter to quit." << std::endl;
      
      std::string line;
      while (running.load() && std::getline(std::cin, line)) {
        if (line == "q") {
          running.store(false);
          break;
        }
        bool new_state = !gravity_compensation_enabled.load();
        gravity_compensation_enabled.store(new_state);
        std::cout << "\nGravity compensation is now " << (new_state ? "ON" : "OFF") << std::endl;
        if (!new_state) {
          std::cout << "WARNING: Robot will drop! Hold it or place it safely." << std::endl;
        }
      }
    });

    // Torque control callback
    auto torque_control_callback = [&model, &gravity_compensation_enabled, &running](
                                       const franka::RobotState& robot_state,
                                       franka::Duration /*period*/) -> franka::Torques {
      if (!running.load()) {
        return franka::MotionFinished(franka::Torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
      }

      // Get gravity vector from model
      std::array<double, 7> gravity_array = model.gravity(robot_state);

      // Apply or remove gravity compensation based on toggle
      franka::Torques torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
      
      if (gravity_compensation_enabled.load()) {
        // Apply gravity compensation
        for (size_t i = 0; i < 7; i++) {
          torques.tau_J[i] = gravity_array[i];
        }
      }
      // else: zero torques (no compensation)

      // Add small damping for stability
      const double damping_factor = 0.1;
      for (size_t i = 0; i < 7; i++) {
        torques.tau_J[i] -= damping_factor * robot_state.dq[i];
      }

      return torques;
    };

    std::cout << "WARNING: This example controls the robot with torque commands!" << std::endl
              << "The robot will be in gravity compensation mode initially." << std::endl
              << "Make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to start..." << std::endl;
    
    std::string dummy;
    std::getline(std::cin, dummy);

    // Start torque control
    std::cout << "Starting torque control with gravity compensation..." << std::endl;
    robot.control(torque_control_callback);

    // Clean up
    running.store(false);
    if (input_thread.joinable()) {
      input_thread.join();
    }

  } catch (const franka::Exception& ex) {
    // running.store(false); // 'running' is out of scope here
    std::cerr << ex.what() << std::endl;
    return -1;
  }

  return 0;
}