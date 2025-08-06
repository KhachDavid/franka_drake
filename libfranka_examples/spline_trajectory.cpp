// Copyright (c) 2024 Franka Emika GmbH
// Spline trajectory example - demonstrates smooth cubic spline interpolation
#include <cmath>
#include <iostream>
#include <iomanip>
#include <vector>
#include <array>
#include <algorithm>
#include <Eigen/Dense>

#include <franka/active_motion_generator.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include "examples_common.h"

// Cubic spline interpolation class
class CubicSpline {
 public:
  CubicSpline(const std::vector<double>& x, const std::vector<double>& y) {
    if (x.size() != y.size() || x.size() < 2) {
      throw std::invalid_argument("Invalid input for cubic spline");
    }
    
    x_ = x;
    y_ = y;
    n_ = x.size();
    
    // Calculate spline coefficients
    computeCoefficients();
  }
  
  double evaluate(double x) const {
    // Find the right interval
    size_t i = 0;
    if (x <= x_[0]) return y_[0];
    if (x >= x_[n_-1]) return y_[n_-1];
    
    // Binary search for efficiency
    size_t left = 0, right = n_ - 1;
    while (left < right - 1) {
      size_t mid = (left + right) / 2;
      if (x < x_[mid]) {
        right = mid;
      } else {
        left = mid;
      }
    }
    i = left;
    
    // Evaluate cubic polynomial
    double dx = x - x_[i];
    return a_[i] + b_[i]*dx + c_[i]*dx*dx + d_[i]*dx*dx*dx;
  }
  
  double derivative(double x) const {
    // Find the right interval
    size_t i = 0;
    if (x <= x_[0]) return 0;
    if (x >= x_[n_-1]) return 0;
    
    size_t left = 0, right = n_ - 1;
    while (left < right - 1) {
      size_t mid = (left + right) / 2;
      if (x < x_[mid]) {
        right = mid;
      } else {
        left = mid;
      }
    }
    i = left;
    
    // Evaluate derivative
    double dx = x - x_[i];
    return b_[i] + 2*c_[i]*dx + 3*d_[i]*dx*dx;
  }
  
 private:
  void computeCoefficients() {
    std::vector<double> h(n_-1);
    for (size_t i = 0; i < n_-1; ++i) {
      h[i] = x_[i+1] - x_[i];
    }
    
    // Set up tridiagonal system
    std::vector<double> alpha(n_-1);
    for (size_t i = 1; i < n_-1; ++i) {
      alpha[i] = 3/h[i] * (y_[i+1] - y_[i]) - 3/h[i-1] * (y_[i] - y_[i-1]);
    }
    
    std::vector<double> l(n_), mu(n_), z(n_);
    l[0] = 1;
    mu[0] = 0;
    z[0] = 0;
    
    for (size_t i = 1; i < n_-1; ++i) {
      l[i] = 2 * (x_[i+1] - x_[i-1]) - h[i-1] * mu[i-1];
      mu[i] = h[i] / l[i];
      z[i] = (alpha[i] - h[i-1] * z[i-1]) / l[i];
    }
    
    l[n_-1] = 1;
    z[n_-1] = 0;
    c_.resize(n_);
    c_[n_-1] = 0;
    
    // Back substitution
    for (int j = n_-2; j >= 0; --j) {
      c_[j] = z[j] - mu[j] * c_[j+1];
    }
    
    // Compute remaining coefficients
    a_.resize(n_-1);
    b_.resize(n_-1);
    d_.resize(n_-1);
    
    for (size_t i = 0; i < n_-1; ++i) {
      a_[i] = y_[i];
      b_[i] = (y_[i+1] - y_[i])/h[i] - h[i]*(c_[i+1] + 2*c_[i])/3;
      d_[i] = (c_[i+1] - c_[i])/(3*h[i]);
    }
  }
  
  std::vector<double> x_, y_;
  std::vector<double> a_, b_, c_, d_;
  size_t n_;
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

    // First move to initial position
    std::array<double, 7> q_goal = {{0, -M_PI/4, 0, -3*M_PI/4, 0, M_PI/2, M_PI/4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "Moving to initial position..." << std::endl;
    robot.control(motion_generator);

    // Get current joint positions
    auto robot_state = robot.readOnce();
    std::array<double, 7> q_start = robot_state.q;

    std::cout << "\nStarting cubic spline trajectory demo..." << std::endl;
    std::cout << "The robot will follow smooth spline trajectories through multiple waypoints." << std::endl;

    // Define time points and waypoints for spline
    std::vector<double> time_points = {0.0, 2.0, 4.0, 6.0, 8.0, 10.0};
    
    // Define waypoints for each joint
    std::vector<std::vector<double>> joint_waypoints(7);
    
    // Joint 1 waypoints
    joint_waypoints[0] = {q_start[0], 0.3, -0.3, 0.2, -0.1, q_start[0]};
    // Joint 2 waypoints
    joint_waypoints[1] = {q_start[1], -0.5, -1.0, -0.7, -0.85, q_start[1]};
    // Joint 3 waypoints
    joint_waypoints[2] = {q_start[2], 0.2, -0.2, 0.3, -0.1, q_start[2]};
    // Joint 4 waypoints
    joint_waypoints[3] = {q_start[3], -2.0, -2.5, -2.2, -2.35, q_start[3]};
    // Joint 5 waypoints
    joint_waypoints[4] = {q_start[4], 0.2, -0.2, 0.1, 0.0, q_start[4]};
    // Joint 6 waypoints
    joint_waypoints[5] = {q_start[5], 1.8, 2.0, 1.7, 1.57, q_start[5]};
    // Joint 7 waypoints
    joint_waypoints[6] = {q_start[6], 1.0, 0.5, 0.8, 0.785, q_start[6]};

    // Create splines for each joint
    std::vector<CubicSpline> joint_splines;
    for (size_t i = 0; i < 7; ++i) {
      joint_splines.emplace_back(time_points, joint_waypoints[i]);
    }

    // Control callback
    double time = 0;
    const double total_duration = time_points.back();
    
    auto spline_callback = [&](const franka::RobotState& robot_state,
                              franka::Duration period) -> franka::JointPositions {
      time += period.toSec();

      // Evaluate splines for current time
      franka::JointPositions output{};
      for (size_t i = 0; i < 7; ++i) {
        output.q[i] = joint_splines[i].evaluate(time);
      }

      // Print progress
      static int print_counter = 0;
      if (++print_counter >= 100) {  // Every 100ms
        print_counter = 0;
        
        std::cout << "\rTime: " << std::fixed << std::setprecision(2) << time << "s"
                  << " | Progress: " << std::setprecision(1) << (time / total_duration * 100) << "%";
        
        // Find current waypoint segment
        auto it = std::lower_bound(time_points.begin(), time_points.end(), time);
        if (it != time_points.begin()) {
          size_t segment = std::distance(time_points.begin(), it) - 1;
          std::cout << " | Segment: " << segment + 1 << "/" << (time_points.size() - 1);
        }
        
        // Print velocity magnitude
        double vel_magnitude = 0;
        for (size_t i = 0; i < 7; ++i) {
          double vel = joint_splines[i].derivative(time);
          vel_magnitude += vel * vel;
        }
        vel_magnitude = std::sqrt(vel_magnitude);
        std::cout << " | Velocity: " << std::setprecision(3) << vel_magnitude << " rad/s     ";
        std::cout << std::flush;
        
        // Periodically print joint positions
        if (static_cast<int>(time * 2) % 4 == 0) {
          std::cout << "\nJoint positions: [";
          for (size_t i = 0; i < 7; ++i) {
            std::cout << std::fixed << std::setprecision(3) << output.q[i];
            if (i < 6) std::cout << ", ";
          }
          std::cout << "]" << std::endl;
        }
      }

      // Check if trajectory is complete
      if (time >= total_duration) {
        std::cout << "\n\nSpline trajectory completed!" << std::endl;
        
        // Print smoothness metrics
        std::cout << "\nTrajectory smoothness analysis:" << std::endl;
        for (size_t i = 0; i < 7; ++i) {
          double max_vel = 0;
          for (double t = 0; t <= total_duration; t += 0.01) {
            max_vel = std::max(max_vel, std::abs(joint_splines[i].derivative(t)));
          }
          std::cout << "  Joint " << i+1 << " max velocity: " 
                    << std::fixed << std::setprecision(3) << max_vel << " rad/s" << std::endl;
        }
        
        return franka::MotionFinished(output);
      }

      return output;
    };

    // Execute trajectory
    robot.control(spline_callback);

    std::cout << "\nSpline trajectory execution finished successfully." << std::endl;
    std::cout << "Total duration: " << total_duration << " seconds" << std::endl;
    std::cout << "Number of waypoints: " << time_points.size() << std::endl;
    std::cout << "Interpolation method: Natural cubic splines" << std::endl;

  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
    return -1;
  }

  return 0;
}