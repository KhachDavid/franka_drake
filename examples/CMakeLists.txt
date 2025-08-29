# Embedding example app
add_executable(franka_fci_sim_embed_example franka_fci_sim_embed_example.cpp)
target_link_libraries(franka_fci_sim_embed_example franka_drake_core)
set_target_properties(franka_fci_sim_embed_example PROPERTIES
    OUTPUT_NAME "franka-fci-sim-embed-example"
)
install(TARGETS franka_fci_sim_embed_example
    RUNTIME DESTINATION bin
)

# Drake-based pick and place controller (fingerless configuration)
add_executable(franka_fci_sim_embed_example_no_gripper franka_fci_sim_embed_example_no_gripper.cpp)
target_link_libraries(franka_fci_sim_embed_example_no_gripper franka_drake_core)
set_target_properties(franka_fci_sim_embed_example_no_gripper PROPERTIES
    OUTPUT_NAME "franka-fci-sim-embed-example-no-gripper"
)
install(TARGETS franka_fci_sim_embed_example_no_gripper
    RUNTIME DESTINATION bin
)