#include <gtest/gtest.h>

#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>

#include "franka_drake/fci_sim_embed.h"
#include "franka_drake/manipulation_helpers.h"

using drake::multibody::AddMultibodyPlantSceneGraph;
using franka_fci_sim::manipulation::AttachObject;
using franka_fci_sim::manipulation::DetachObject;
using franka_fci_sim::manipulation::ExecuteJointTrajectory;
using franka_fci_sim::manipulation::FindObjectByName;
using franka_fci_sim::manipulation::SolveCollisionFreeIK;
using franka_fci_sim::manipulation::WaitForGripper;

namespace {

TEST(WarehouseHelpers, IKFindsSolutionForNearbyPose) {
  const double dt = 0.001;
  drake::systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, dt);

  // Load a simple robot from built-in URDFs via AutoAttach
  franka_fci_sim::FciSimEmbedder::AutoAttachOptions auto_opts;
  auto_opts.prefer_gripper = true;
  auto_opts.disable_collisions = true;

  franka_fci_sim::FciSimOptions opts;
  auto embed = franka_fci_sim::FciSimEmbedder::AutoAttach(&plant, &scene_graph, &builder, auto_opts, opts);

  auto diagram = builder.Build();
  drake::systems::Simulator<double> sim(*diagram);
  auto& root = sim.get_mutable_context();
  auto& ctx = plant.GetMyMutableContextFromRoot(&root);
  franka_fci_sim::SetDefaultFrankaInitialState(plant, ctx);
  sim.Initialize();

  const auto robot = plant.GetBodyByName("fer_link1").model_instance();
  const auto& ee = plant.GetFrameByName("fer_hand_tcp", robot);
  const auto& world = plant.world_frame();

  const auto X_W_EE = plant.CalcRelativeTransform(ctx, world, ee);
  auto q_full = plant.GetPositions(ctx);
  auto maybe = SolveCollisionFreeIK(plant, ctx, robot, X_W_EE, q_full, false);
  ASSERT_TRUE(maybe.has_value());
}

TEST(WarehouseHelpers, FindObjectByNameReturnsNulloptWhenMissing) {
  const double dt = 0.001;
  drake::systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, dt);

  franka_fci_sim::FciSimEmbedder::AutoAttachOptions auto_opts;
  franka_fci_sim::FciSimOptions opts;
  auto embed = franka_fci_sim::FciSimEmbedder::AutoAttach(&plant, &scene_graph, &builder, auto_opts, opts);

  auto diagram = builder.Build();
  drake::systems::Simulator<double> sim(*diagram);
  auto& root = sim.get_mutable_context();
  auto& ctx = plant.GetMyMutableContextFromRoot(&root);
  franka_fci_sim::SetDefaultFrankaInitialState(plant, ctx);
  sim.Initialize();

  auto idx = FindObjectByName(plant, "nonexistent_object_name");
  ASSERT_FALSE(idx.has_value());
}

TEST(WarehouseHelpers, AttachAndDetachObjectDoNotThrow) {
  const double dt = 0.001;
  drake::systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, dt);

  // Load warehouse scene to get a free body (red cube)
  {
    drake::multibody::Parser parser(&plant, &scene_graph);
    const std::string sdf_path = franka_fci_sim::ResolveModelPath("models/warehouse_scene.sdf");
    parser.AddModels(sdf_path);
  }

  franka_fci_sim::FciSimEmbedder::AutoAttachOptions auto_opts;
  auto_opts.prefer_gripper = true;
  auto_opts.disable_collisions = true;
  franka_fci_sim::FciSimOptions opts;
  auto embed = franka_fci_sim::FciSimEmbedder::AutoAttach(&plant, &scene_graph, &builder, auto_opts, opts);

  auto diagram = builder.Build();
  drake::systems::Simulator<double> sim(*diagram);
  auto& root = sim.get_mutable_context();
  auto& ctx = plant.GetMyMutableContextFromRoot(&root);
  franka_fci_sim::SetDefaultFrankaInitialState(plant, ctx);
  sim.Initialize();

  const auto robot = plant.GetBodyByName("fer_link1").model_instance();
  auto cube = FindObjectByName(plant, "box_red");
  ASSERT_TRUE(cube.has_value());

  EXPECT_NO_THROW({
    auto X_EE_Obj = AttachObject(plant, ctx, *cube, robot);
    const auto& ee = plant.GetFrameByName("fer_hand_tcp", robot);
    const auto& world = plant.world_frame();
    const auto X_W_EE = plant.CalcRelativeTransform(ctx, world, ee);
    const auto X_W_final = X_W_EE * X_EE_Obj; // place where it was attached
    DetachObject(plant, ctx, *cube, X_W_final);
  });
}

TEST(WarehouseHelpers, WaitAndTrajectoryRun) {
  const double dt = 0.001;
  drake::systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, dt);

  franka_fci_sim::FciSimEmbedder::AutoAttachOptions auto_opts;
  auto_opts.prefer_gripper = true;
  auto_opts.disable_collisions = true;
  franka_fci_sim::FciSimOptions opts;
  auto embed = franka_fci_sim::FciSimEmbedder::AutoAttach(&plant, &scene_graph, &builder, auto_opts, opts);

  auto diagram = builder.Build();
  drake::systems::Simulator<double> sim(*diagram);
  auto& root = sim.get_mutable_context();
  auto& ctx = plant.GetMyMutableContextFromRoot(&root);
  franka_fci_sim::SetDefaultFrankaInitialState(plant, ctx);
  sim.Initialize();

  const auto robot = plant.GetBodyByName("fer_link1").model_instance();
  const auto q_start = plant.GetPositions(ctx, robot);
  Eigen::VectorXd q_end = q_start;
  if (q_end.size() >= 3) {
    q_end[2] += 0.05; // small joint change
  }
  EXPECT_NO_THROW({
    ExecuteJointTrajectory(plant, ctx, sim, robot, q_start, q_end, 0.5, "unit-test move");
    WaitForGripper(sim, 0.1);
  });
}

}  // namespace


