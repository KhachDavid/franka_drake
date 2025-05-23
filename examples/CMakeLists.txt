add_executable(move_cube move_cube.cc)
target_link_libraries(move_cube drake::drake)
add_executable(zero_torque zero_torque.cpp)
target_link_libraries(zero_torque drake::drake)