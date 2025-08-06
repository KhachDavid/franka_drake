// Fixed trajectory following example - demonstrates smooth path tracking in joint space
#include <cmath>
#include <iostream>
#include <iomanip>
#include <vector>
#include <array>

#include <franka/exception.h>
#include <franka/robot.h>
#include "examples_common.h"

// Trajectory parameters
constexpr double TRAJECTORY_DURATION = 10.0;  // Total duration in seconds

// Quintic polynomial trajectory generator
class QuinticTrajectory {
 public:
  QuinticTrajectory(double start, double end, double duration)
      : start_(start), end_(end), duration_(duration) {
    // Compute quintic polynomial coefficients for zero velocity/acceleration at endpoints
    a0_ = start;
    a1_ = 0;
    a2_ = 0;
    a3_ = 10 * (end - start) / (duration * duration * duration);
    a4_ = -15 * (end - start) / (duration * duration * duration * duration);
    a5_ = 6 * (end - start) / (duration * duration * duration * duration * duration);
  }

  double position(double t) const {
    if (t <= 0) return start_;
    if (t >= duration_) return end_;
    
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;
    
    return a0_ + a1_ * t + a2_ * t2 + a3_ * t3 + a4_ * t4 + a5_ * t5;
  }

 private:
  double start_, end_, duration_;
  double a0_, a1_, a2_, a3_, a4_, a5_;
};

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    // Connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    std::cout << "Connected to robot and set default behavior" << std::endl;

    // Get current joint positions as starting point
    auto robot_state = robot.readOnce();
    std::array<double, 7> q_start = robot_state.q;
    
    std::cout << "Starting joint positions: [";
    for (int i = 0; i < 7; ++i) {
      std::cout << std::fixed << std::setprecision(3) << q_start[i] << (i < 6 ? ", " : "");
    }
    std::cout << "]" << std::endl;

    // Define trajectory waypoints for each joint
    std::vector<std::array<double, 7>> waypoints = {
      q_start,  // Start from current position
      {{0.2, -0.6, 0.2, -2.0, -0.2, 1.8, 0.9}},
      {{-0.2, -0.9, -0.2, -2.3, 0.2, 2.0, 0.5}},
      {{0.0, -0.785398, 0.0, -2.35619, 0.0, 1.5708, 0.785398}},
      q_start  // Return to start
    };

    std::cout << "\nStarting trajectory following demo..." << std::endl;
    std::cout << "The robot will follow a smooth trajectory through " 
              << waypoints.size() - 1 << " waypoints." << std::endl;

    // Generate trajectories for each segment
    std::vector<std::vector<QuinticTrajectory>> segment_trajectories;
    double segment_duration = TRAJECTORY_DURATION / (waypoints.size() - 1);
    
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
      std::vector<QuinticTrajectory> joint_trajectories;
      for (size_t j = 0; j < 7; ++j) {
        joint_trajectories.emplace_back(
          waypoints[i][j], waypoints[i+1][j], segment_duration
        );
      }
      segment_trajectories.push_back(joint_trajectories);
    }

    // Control callback
    double time = 0;
    size_t current_segment = 0;
    
    robot.control([&](const franka::RobotState& robot_state,
                      franka::Duration period) -> franka::JointPositions {
      time += period.toSec();

      // Determine current segment
      double segment_time = time - current_segment * segment_duration;
      if (segment_time > segment_duration && current_segment < segment_trajectories.size() - 1) {
        current_segment++;
        segment_time = time - current_segment * segment_duration;
      }

      // Compute desired joint positions
      franka::JointPositions output = franka::JointPositions(robot_state.q);
      
      if (current_segment < segment_trajectories.size()) {
        for (size_t i = 0; i < 7; ++i) {
          output.q[i] = segment_trajectories[current_segment][i].position(segment_time);
        }

        // Print progress every 100ms
        static int print_counter = 0;
        if (++print_counter >= 100) {
          print_counter = 0;
          std::cout << "\rProgress: " << std::fixed << std::setprecision(1) 
                    << (time / TRAJECTORY_DURATION * 100) << "% "
                    << "| Segment: " << current_segment + 1 << "/" << segment_trajectories.size()
                    << " | Time: " << std::setprecision(2) << time << "s      " << std::flush;

          // Also print joint positions periodically
          if (static_cast<int>(time * 2) % 4 == 0) {  // Every 2 seconds
            std::cout << "\nJoint positions: [";
            for (size_t i = 0; i < 7; ++i) {
              std::cout << std::fixed << std::setprecision(3) << output.q[i];
              if (i < 6) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
          }
        }
      }

      // Check if trajectory is complete
      if (time >= TRAJECTORY_DURATION) {
        std::cout << "\n\nTrajectory completed!" << std::endl;
        return franka::MotionFinished(output);
      }

      return output;
    });

    // Print final statistics
    std::cout << "\nTrajectory execution finished successfully." << std::endl;
    std::cout << "Total duration: " << TRAJECTORY_DURATION << " seconds" << std::endl;
    std::cout << "Number of waypoints: " << waypoints.size() << std::endl;

  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
    return -1;
  }

  return 0;
}