cmake_minimum_required(VERSION 3.10)
project(franka_drake)

set(CMAKE_CXX_STANDARD 17)

find_package(drake REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation Net)

include_directories(include)

add_subdirectory(src)
add_subdirectory(examples)

# Optionally add urdf and scripts directories if needed
# add_subdirectory(urdf)
# add_subdirectory(scripts)
