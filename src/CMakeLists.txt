# Create the main library
add_library(franka_drake_core STATIC 
    franka_drake_module.cpp 
    simulator_server.cpp 
    fci_sim_server.cpp
    gripper_sim_server.cpp
    embed_fci_sim.cpp
    embed_fci_sim_helpers.cpp
)

# Set library properties
set_target_properties(franka_drake_core PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    PUBLIC_HEADER ""
)

# Find libfranka
find_package(Franka QUIET)

# Link libraries
target_link_libraries(franka_drake_core 
    drake::drake 
    Poco::Foundation 
    Poco::Net
)

# Conditionally link libfranka if found
if(Franka_FOUND)
    target_link_libraries(franka_drake_core Franka::Franka)
    target_compile_definitions(franka_drake_core PUBLIC FRANKA_AVAILABLE)
endif()

# Include directories
target_include_directories(franka_drake_core 
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
        $<INSTALL_INTERFACE:include/franka_drake>
)

# Install rules
install(TARGETS franka_drake_core
    EXPORT franka_drake_coreTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
