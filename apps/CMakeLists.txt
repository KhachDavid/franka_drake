# Embedding example app
add_executable(franka_fci_sim_embed_example franka_fci_sim_embed_example.cpp)
target_link_libraries(franka_fci_sim_embed_example franka_drake_core)
set_target_properties(franka_fci_sim_embed_example PROPERTIES
    OUTPUT_NAME "franka-fci-sim-embed-example"
)
install(TARGETS franka_fci_sim_embed_example
    RUNTIME DESTINATION bin
)

# Drake-based warehouse controller
add_executable(warehouse_drake_controller warehouse_drake_controller.cpp)
target_link_libraries(warehouse_drake_controller franka_drake_core)
set_target_properties(warehouse_drake_controller PROPERTIES
    OUTPUT_NAME "warehouse-drake-controller"
)
install(TARGETS warehouse_drake_controller
    RUNTIME DESTINATION bin
)