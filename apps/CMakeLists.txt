# Embedding example app
add_executable(franka_fci_sim_embed_example franka_fci_sim_embed_example.cpp)
target_link_libraries(franka_fci_sim_embed_example franka_drake_core)
set_target_properties(franka_fci_sim_embed_example PROPERTIES
    OUTPUT_NAME "franka-fci-sim-embed-example"
)
install(TARGETS franka_fci_sim_embed_example
    RUNTIME DESTINATION bin
)

# Warehouse example app
add_executable(franka_fci_sim_warehouse_example franka_fci_sim_warehouse_example.cpp)
target_link_libraries(franka_fci_sim_warehouse_example franka_drake_core)
set_target_properties(franka_fci_sim_warehouse_example PROPERTIES
    OUTPUT_NAME "franka-fci-sim-warehouse-example"
)
install(TARGETS franka_fci_sim_warehouse_example
    RUNTIME DESTINATION bin
)