#include <iostream>
#include <drake/common/find_resource.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/geometry/scene_graph.h>
#include <drake/geometry/meshcat.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>

#include <Eigen/Dense>  // For Eigen::Vector3d

using namespace drake;
using namespace drake::geometry;
using namespace drake::multibody;
using namespace drake::systems;

int main() {
    DiagramBuilder<double> builder;

    // Meshcat
    auto meshcat = std::make_shared<Meshcat>();
    auto* scene_graph = builder.AddSystem<SceneGraph>();
    MeshcatVisualizer<double>::AddToBuilder(&builder, *scene_graph, meshcat);

    // MultibodyPlant
    auto* plant = builder.AddSystem<MultibodyPlant<double>>(0.0);
    plant->RegisterAsSourceForSceneGraph(scene_graph);

    // Ground
    Box ground_box(1.0, 1.0, 0.01);
    plant->RegisterCollisionGeometry(plant->world_body(), math::RigidTransformd(),
                                     ground_box, "ground_collision",
                                     CoulombFriction<double>(0.9, 0.8));
    plant->RegisterVisualGeometry(plant->world_body(), math::RigidTransformd(),
                                  ground_box, "ground_visual",
                                  Vector4<double>(0.2, 0.2, 0.2, 1.0));

    // Cube (0.1m x 0.1m x 0.1m)
    Box cube_box(0.1, 0.1, 0.1);
    double mass = 1.0;
    auto M_BBo_B = SpatialInertia<double>::MakeFromCentralInertia(
        mass, Eigen::Vector3d::Zero(), UnitInertia<double>::SolidBox(0.1, 0.1, 0.1));
    auto& cube = plant->AddRigidBody("cube", M_BBo_B);

    plant->RegisterCollisionGeometry(cube, math::RigidTransformd(), cube_box,
                                     "cube_collision", CoulombFriction<double>(0.9, 0.8));
    plant->RegisterVisualGeometry(cube, math::RigidTransformd(), cube_box,
                                  "cube_visual", Vector4<double>(0.5, 0.5, 1.0, 1.0));

    plant->Finalize();

    // Poses port tells SceneGraph where the plants geometries are.
    builder.Connect(plant->get_geometry_pose_output_port(),
                scene_graph->get_source_pose_port(plant->get_source_id().value()));

    // Connect SceneGraph to the MultibodyPlant
    builder.Connect(scene_graph->get_query_output_port(),
                plant->get_geometry_query_input_port());

    auto diagram = builder.Build();
    auto context = diagram->CreateDefaultContext();
    auto* plant_context = &plant->GetMyMutableContextFromRoot(context.get());

    // Set initial pose of cube (0.2m above ground)
    plant->SetFreeBodyPose(plant_context, cube, math::RigidTransformd(Eigen::Vector3d(0.0, 0.0, 0.2)));

    // Simulate
    Simulator<double> simulator(*diagram, std::move(context));
    simulator.set_target_realtime_rate(1.0);
    simulator.Initialize();
    simulator.AdvanceTo(5.0);

    // Press enter to exit
    std::cout << "Press Enter to exit..." << std::endl;
    std::cin.get();

    return 0;
}
