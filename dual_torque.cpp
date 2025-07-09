#include <iostream>
#include <array>
#include <string>
#include <chrono>
#include <sstream>
#include <iomanip>

#include <Poco/Net/StreamSocket.h>
#include <Poco/Net/SocketAddress.h>
#include <Poco/Exception.h>

#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/control_types.h>

#include "examples_common.h"
#include "log_error.h"

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <robot-ip> <torque> [sim_host] [sim_port]\n";
    return -1;
  }

  // Parse CLI arguments
  const char* robot_ip = argv[1];
  double max_torque = std::stod(argv[2]);
  const std::string sim_host = (argc >= 4) ? argv[3] : "127.0.0.1";
  uint16_t sim_port = (argc >= 5) ? static_cast<uint16_t>(std::stoi(argv[4])) : 12345;

  if (max_torque < -10.0 || max_torque > 10.0) {
    std::cerr << "Torque must be between -10.0 and 10.0 Nm.\n";
    return -1;
  }

  // ---------------------------------------------------------------------
  // Connect to the Drake simulation (TCP)
  // ---------------------------------------------------------------------
  Poco::Net::StreamSocket sim_socket;
  try {
    Poco::Net::SocketAddress sa(sim_host, sim_port);
    sim_socket.connect(sa);
    sim_socket.setBlocking(false);  // best-effort, don't stall control loop
    std::cout << "Connected to simulation at " << sa.toString() << std::endl;
  } catch (Poco::Exception& e) {
    std::cerr << "WARNING: Could not connect to simulation: " << e.displayText()
              << "\nRobot control will proceed anyway." << std::endl;
  }

  try {
    franka::Robot robot(robot_ip);
    robot.automaticErrorRecovery();
    setDefaultBehavior(robot);  // torque-rate saturation, etc.

    constexpr double ramp_time = 2.0;   // [s]
    constexpr double total_duration = 4.0;

    auto start_time = std::chrono::steady_clock::now();

    robot.control([&](const franka::RobotState& /*state*/, franka::Duration dt) -> franka::Torques {
      // elapsed wall-clock time since start (seconds)
      double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();

      // Linear ramp profile
      double ramped_torque = (t < ramp_time) ? (max_torque * t / ramp_time) : max_torque;

      std::array<double, 7> tau{};  // all zeros by default
      tau[6] = ramped_torque;       // joint-7 only for now

      // -----------------------------------------------------------------
      // Send timestamped packet:  t tau1 … tau7\n
      // -----------------------------------------------------------------
      if (sim_socket.available() >= 0) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6) << t;
        for (double v : tau) oss << ' ' << v;
        oss << '\n';
        std::string msg = oss.str();
        try {
          sim_socket.sendBytes(msg.c_str(), static_cast<int>(msg.size()));
        } catch (Poco::Exception&) {
        }
      }

      // Stop after total_duration
      if (t >= total_duration) {
        std::cout << "Finished torque command." << std::endl;
        return franka::MotionFinished(franka::Torques(tau));
      }

      return franka::Torques(tau);
    });

    // Notify simulation that streaming is done
    if (sim_socket.available() >= 0) {
      const char* stop_msg = "STOP\n";
      try {
        sim_socket.sendBytes(stop_msg, 5);
      } catch (Poco::Exception&) {}
    }

  } catch (const franka::Exception& e) {
    logError(e);
    return -1;
  }

  return 0;
}
