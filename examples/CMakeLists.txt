# Create the main executable
add_executable(franka_fci_sim_server_main franka_fci_sim_server_main.cpp)
target_link_libraries(franka_fci_sim_server_main franka_drake_module)

# Create the robot movement test executable
add_executable(test_robot_movement test_robot_movement.cpp)
target_link_libraries(test_robot_movement franka_drake_module)

# Example executable for basic functionality
add_executable(drake_simple_example drake_simple_example.cpp)
target_link_libraries(drake_simple_example franka_drake_module)

# Example executable for simulator server
add_executable(run_simulator_server run_simulator_server.cpp)
target_link_libraries(run_simulator_server franka_drake_module)

# Example executable for sandbox testing
add_executable(sim_sandbox sim_sandbox.cpp)
target_link_libraries(sim_sandbox franka_drake_module)