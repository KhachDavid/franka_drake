cmake_minimum_required(VERSION 3.10)
project(FrankaExamples)

# Find libfranka
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

# Common source files
set(COMMON_SOURCES
  examples_common.cpp
)

# Function to add an example executable
function(add_franka_example name)
  add_executable(${name} ${name}.cpp ${COMMON_SOURCES})
  target_link_libraries(${name} Franka::Franka Eigen3::Eigen)
  target_compile_features(${name} PRIVATE cxx_std_14)
endfunction()

# Add all examples
add_franka_example(cartesian_spiral_motion)
add_franka_example(sine_wave_velocity_control)
add_franka_example(gravity_compensation_toggle)
add_franka_example(generate_elbow_motion)
add_franka_example(generate_elbow_motion_simple)
add_franka_example(move_all_joints)
add_franka_example(move_joint7_position)
add_franka_example(move_joint7_position_wait)

# Install targets
install(TARGETS 
  cartesian_spiral_motion
  sine_wave_velocity_control
  gravity_compensation_toggle
  generate_elbow_motion
  generate_elbow_motion_simple
  move_all_joints
  move_joint7_position
  move_joint7_position_wait
  DESTINATION bin
)