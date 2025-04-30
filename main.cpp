#include <chrono>
#include <thread>
#include <drake/common/find_resource.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/visualization/visualization_config_functions.h>

using namespace drake;

int main() {
    systems::DiagramBuilder<double> builder;

    // Set up plant + scene graph
    auto [plant, scene_graph] = multibody::AddMultibodyPlantSceneGraph(&builder, 0.001);

    // Set up parser and load the Franka model
    multibody::Parser parser(&plant, &scene_graph);
    parser.package_map().AddPackageXml("/home/david/ws/franka/src/franka_description/package.xml");

    parser.AddModels("/home/david/final_project/franka_drake_test/fer_drake.urdf");

    plant.Finalize();

    // Enable Meshcat + Drake visualizer
    visualization::AddDefaultVisualization(&builder);

    // Build the system diagram
    auto diagram = builder.Build();
    systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(1.0);

    // Initialize simulation
    simulator.Initialize();
    simulator.AdvanceTo(0.1);

    // Keep alive so Meshcat has time to show it
    std::this_thread::sleep_for(std::chrono::seconds(5));

    return 0;
}
