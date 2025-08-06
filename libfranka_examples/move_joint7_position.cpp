#include <cmath>
#include <iostream>
// #include "log_error.h" // Not needed

#include "examples_common.h"
#include <franka/robot.h>
#include <franka/rate_limiting.h>
#include <franka/exception.h>
#include <franka/control_types.h>

int main(int argc, char** argv) {
  if (argc != 2) { std::cerr << "Usage: " << argv[0] << " <robot-ip>\n"; return -1; }

  try {
    franka::Robot robot(argv[1]);
    robot.automaticErrorRecovery();
    setDefaultBehavior(robot);

    std::array<double,7> q_start{}, q_goal{};
    constexpr double delta_q7 = -0.20;   // rad
    constexpr double T        = 2.0;    // s

    double t = 0.0;
    bool first = true;

    std::array<double,7> q_cmd_last{};                // previous command
    constexpr std::array<double,7> kMaxVel{{2.0, 1.0, 1.5, 1.25, 3.0, 1.5, 3.0}};
    robot.control([&](const franka::RobotState& s, franka::Duration dt)
                  -> franka::JointPositions {

      t += dt.toSec();

      if (first) {
        q_start   = s.q_d;
        q_goal    = q_start;
        q_goal[6] += delta_q7;
        q_cmd_last = q_start;
        first = false;
      }

      double a      = std::clamp(t / T, 0.0, 1.0);          // 0‒1
      double s_tra  = 0.5 * (1.0 - std::cos(M_PI * a));     // smooth blend

      std::array<double,7> q_cmd = q_start;
      q_cmd[6] = q_start[6] + delta_q7 * s_tra;

      // Velocity-only rate limit (three-argument overload in 0.13.x)
      q_cmd = franka::limitRate(kMaxVel, q_cmd, q_cmd_last);
      q_cmd_last = q_cmd;

      if (a >= 1.0) {
        std::cout << "Final J7 error = " << (s.q[6] - q_goal[6]) << " rad\n";
        return franka::MotionFinished(franka::JointPositions(q_cmd));
      }
      return franka::JointPositions(q_cmd);
    });

  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
    return -1;
  }
  return 0;
}
