# Main FCI simulation server
add_executable(franka_fci_sim_server franka_fci_sim_server_main.cpp)
target_link_libraries(franka_fci_sim_server franka_drake_core)

# Set executable properties
set_target_properties(franka_fci_sim_server PROPERTIES
    OUTPUT_NAME "franka-fci-sim-server"
)

# Install rules
install(TARGETS franka_fci_sim_server
    RUNTIME DESTINATION bin
) 

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