add_library(franka_drake_module STATIC franka_drake_module.cpp simulator_server.cpp fci_sim_server.cpp)
target_link_libraries(franka_drake_module drake::drake Poco::Foundation Poco::Net)
target_include_directories(franka_drake_module PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include)
