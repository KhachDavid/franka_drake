cmake_minimum_required(VERSION 3.10)
project(franka_drake VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Find required packages
find_package(drake REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation Net)

# Include directories
include_directories(include)

# Add subdirectories
add_subdirectory(src)
add_subdirectory(apps)

enable_testing()
add_subdirectory(test)

# Install rules
install(DIRECTORY include/ DESTINATION include/franka_drake)
install(DIRECTORY models/ DESTINATION share/franka_drake)
install(DIRECTORY scripts/ DESTINATION share/franka_drake)
install(DIRECTORY kb/ DESTINATION share/franka_drake/kb)
