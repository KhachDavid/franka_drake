add_executable(drake_simple_example drake_simple_example.cpp)
target_link_libraries(drake_simple_example franka_drake_module drake::drake)

add_executable(run_simulator_server run_simulator_server.cpp)
target_link_libraries(run_simulator_server franka_drake_module drake::drake Poco::Foundation Poco::Net)

add_executable(sim_sandbox sim_sandbox.cpp)
target_link_libraries(sim_sandbox drake::drake Poco::Foundation Poco::Net)

add_executable(franka_fci_sim_server_main franka_fci_sim_server_main.cpp)
target_link_libraries(franka_fci_sim_server_main franka_drake_module drake::drake pthread)