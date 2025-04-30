cmake_minimum_required(VERSION 3.22)
project(sim_franka)

set(CMAKE_CXX_STANDARD 17)
set(DRAKE_DIR "$ENV{DRAKE_INSTALL_DIR}")

# Tell CMake to use Drake's installed CMake modules
list(APPEND CMAKE_PREFIX_PATH "${DRAKE_DIR}")

# Now this works
find_package(drake REQUIRED)

add_executable(sim main.cpp)
target_link_libraries(sim drake::drake)

add_subdirectory(examples)
