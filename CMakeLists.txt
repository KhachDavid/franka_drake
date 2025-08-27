cmake_minimum_required(VERSION 3.10)
project(franka_drake VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Provide the source dir to C++ as a compile-time macro so runtime code can
# resolve assets relative to the repository root (e.g., scripts/ for monitoring).
add_compile_definitions(FRANKA_DRAKE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# Find required packages
find_package(drake REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation Net)

# Include directories
include_directories(include)

# Add subdirectories
add_subdirectory(src)
add_subdirectory(examples)

enable_testing()
add_subdirectory(test)

# Install rules
install(DIRECTORY include/ DESTINATION include/franka_drake)
install(DIRECTORY models/ DESTINATION share/franka_drake)
install(DIRECTORY docs/ DESTINATION share/franka_drake/docs)
