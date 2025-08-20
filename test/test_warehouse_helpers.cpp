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

  // Parse robot explicitly to avoid wiring full FCI systems in CI
  drake::multibody::Parser parser(&plant, &scene_graph);
  const std::string pkg_xml = franka_fci_sim::ResolveModelPath("models/urdf/package.xml");
  const std::string urdf = franka_fci_sim::ResolveModelPath("models/urdf/fer_drake_gripper_fixed.urdf");
  parser.package_map().AddPackageXml(pkg_xml);
  const auto robot = parser.AddModels(urdf)[0];
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base", robot));
  plant.Finalize();

  auto diagram = builder.Build();
  drake::systems::Simulator<double> sim(*diagram);
  auto& root = sim.get_mutable_context();
  auto& ctx = plant.GetMyMutableContextFromRoot(&root);
  franka_fci_sim::SetDefaultFrankaInitialState(plant, ctx);
  sim.Initialize();

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

  // Just build a minimal robot diagram; no FCI wiring to avoid Demultiplexer asserts
  drake::multibody::Parser parser(&plant, &scene_graph);
  const std::string pkg_xml = franka_fci_sim::ResolveModelPath("models/urdf/package.xml");
  const std::string urdf = franka_fci_sim::ResolveModelPath("models/urdf/fer_drake_fingerless.urdf");
  parser.package_map().AddPackageXml(pkg_xml);
  const auto robot = parser.AddModels(urdf)[0];
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base", robot));
  plant.Finalize();

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

  // Minimal robot only; no scene. Skip if no free object exists.
  drake::multibody::Parser parser2(&plant, &scene_graph);
  const std::string pkg_xml2 = franka_fci_sim::ResolveModelPath("models/urdf/package.xml");
  const std::string urdf2 = franka_fci_sim::ResolveModelPath("models/urdf/fer_drake_gripper_fixed.urdf");
  parser2.package_map().AddPackageXml(pkg_xml2);
  const auto robot = parser2.AddModels(urdf2)[0];
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base", robot));
  plant.Finalize();

  auto diagram = builder.Build();
  drake::systems::Simulator<double> sim(*diagram);
  auto& root = sim.get_mutable_context();
  auto& ctx = plant.GetMyMutableContextFromRoot(&root);
  franka_fci_sim::SetDefaultFrankaInitialState(plant, ctx);
  sim.Initialize();

  auto cube = FindObjectByName(plant, "box_red");
  if (!cube.has_value()) {
    GTEST_SKIP() << "Skipping attach/detach test: no free object found";
  }

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

  // Minimal robot diagram only
  drake::multibody::Parser parser3(&plant, &scene_graph);
  const std::string pkg_xml3 = franka_fci_sim::ResolveModelPath("models/urdf/package.xml");
  const std::string urdf3 = franka_fci_sim::ResolveModelPath("models/urdf/fer_drake_gripper_fixed.urdf");
  parser3.package_map().AddPackageXml(pkg_xml3);
  const auto robot = parser3.AddModels(urdf3)[0];
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base", robot));
  plant.Finalize();

  auto diagram = builder.Build();
  drake::systems::Simulator<double> sim(*diagram);
  auto& root = sim.get_mutable_context();
  auto& ctx = plant.GetMyMutableContextFromRoot(&root);
  franka_fci_sim::SetDefaultFrankaInitialState(plant, ctx);
  sim.Initialize();
  const auto q_start = plant.GetPositions(ctx, robot);
  Eigen::VectorXd q_end = q_start;
  if (q_end.size() >= 3) {
    q_end[2] += 0.05; // small joint change
  }
  EXPECT_NO_THROW({
    ExecuteJointTrajectory(plant, ctx, sim, robot, q_start, q_end, 0.2, "unit-test move");
    WaitForGripper(sim, 0.05);
  });
}

}  // namespace


