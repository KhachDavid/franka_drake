cmake_minimum_required(VERSION 3.22)
project(sim_franka)

set(CMAKE_CXX_STANDARD 17)
set(DRAKE_DIR "$ENV{DRAKE_INSTALL_DIR}")

# Tell CMake to use Drake's installed CMake modules
list(APPEND CMAKE_PREFIX_PATH "${DRAKE_DIR}")

# Now this works
find_package(drake REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation Net)
find_package(Eigen3 REQUIRED)

add_executable(sim main.cpp)
add_executable(sim_network main_network.cpp)

target_include_directories(sim PRIVATE "${PROJECT_SOURCE_DIR}/include")
target_include_directories(sim_network PRIVATE "${PROJECT_SOURCE_DIR}/include")


target_link_libraries(sim drake::drake)
target_link_libraries(sim_network drake::drake Poco::Foundation Poco::Net)

add_executable(client client.cpp)
target_link_libraries(client Poco::Foundation Poco::Net)

add_executable(dual_torque dual_torque.cpp examples_common.cpp)
target_include_directories(dual_torque PRIVATE "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(dual_torque Poco::Foundation Poco::Net franka Eigen3::Eigen)  # assumes franka library is available

add_subdirectory(examples)
