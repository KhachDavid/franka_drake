#include <chrono>
#include <iostream>
#include <thread>
#include <drake/common/find_resource.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/visualization/visualization_config_functions.h>

using namespace drake;

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <path_to_package.xml> <path_to_urdf>" << std::endl;
        return 1;
    }

    std::string package_xml_path = argv[1];
    std::string urdf_path = argv[2];

    systems::DiagramBuilder<double> builder;

    // Set up plant + scene graph
    auto [plant, scene_graph] = multibody::AddMultibodyPlantSceneGraph(&builder, 0.001);

    // Set up parser and load the Franka model
    multibody::Parser parser(&plant, &scene_graph);
    parser.package_map().AddPackageXml(package_xml_path);

    // Load and get model instance
    multibody::ModelInstanceIndex panda = parser.AddModels(urdf_path)[0];

    // Weld base link to world frame
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("base", panda),
        math::RigidTransformd());

    // Finalize plant
    plant.Finalize();

    // Enable Meshcat + Drake visualizer
    visualization::AddDefaultVisualization(&builder);

    // Build the system diagram
    auto diagram = builder.Build();
    systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(1.0);

    // Initialize simulation
    simulator.Initialize();
    simulator.AdvanceTo(5);

    std::cout << "Press Enter to exit..." << std::endl;
    std::cin.get();

    return 0;
}
