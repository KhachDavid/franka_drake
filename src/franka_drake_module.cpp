#include "franka_drake/franka_drake_module.h"
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/geometry/meshcat.h>
#include <drake/visualization/visualization_config_functions.h>
#include <drake/math/rigid_transform.h>
#include <memory>
#include <vector>
#include <string>
#include <Eigen/Core>

namespace franka_drake {

FrankaDrakeModule::FrankaDrakeModule(drake::systems::DiagramBuilder<double>* builder,
                                     const std::string& package_xml,
                                     const std::string& urdf_path,
                                     double time_step) {
  // Register plant and scene graph with the builder
  auto [plant, scene_graph] = drake::multibody::AddMultibodyPlantSceneGraph(builder, time_step);
  plant_ = &plant;
  scene_graph_ = &scene_graph;

  // Parse the URDF
  drake::multibody::Parser parser(plant_, scene_graph_);
  parser.package_map().AddPackageXml(package_xml);
  const auto model_ids = parser.AddModels(urdf_path);
  if (model_ids.empty()) {
    throw std::runtime_error("Failed to load URDF: " + urdf_path);
  }
  const auto robot = model_ids[0];

  // Weld base to world
  plant_->WeldFrames(plant_->world_frame(), plant_->GetFrameByName("base", robot));

  // Add actuators
  struct J { std::string n; double eff; };
  const std::vector<J> act = {
      {"fer_joint1", 87}, {"fer_joint2", 87}, {"fer_joint3", 87},
      {"fer_joint4", 87}, {"fer_joint5", 12}, {"fer_joint6", 87}, {"fer_joint7", 87},
  };
  for (const auto& a : act) {
    const auto& joint = plant_->GetJointByName<drake::multibody::RevoluteJoint>(a.n, robot);
    plant_->AddJointActuator(a.n + "_act", joint, a.eff);
  }

  // Add visual triads to each joint
  const double axis_len = 0.15, axis_rad = 0.003;
  const drake::geometry::Cylinder cyl_shape(axis_rad, axis_len);
  const drake::math::RigidTransformd X_FGx(
      drake::math::RotationMatrixd::MakeYRotation(M_PI / 2), Eigen::Vector3d(axis_len / 2, 0, 0));
  const drake::math::RigidTransformd X_FGy(
      drake::math::RotationMatrixd::MakeXRotation(-M_PI / 2), Eigen::Vector3d(0, axis_len / 2, 0));
  const drake::math::RigidTransformd X_FGz(
      drake::math::RotationMatrixd(), Eigen::Vector3d(0, 0, axis_len / 2));
  const Eigen::Vector4d red(1, 0, 0, 0.9), grn(0, 1, 0, 0.9), blu(0, 0, 1, 0.9);
  for (const auto& a : act) {
    const auto& joint = plant_->GetJointByName<drake::multibody::RevoluteJoint>(a.n, robot);
    const auto& body = joint.child_body();
    plant_->RegisterVisualGeometry(body, X_FGx, cyl_shape, a.n + ":x", red);
    plant_->RegisterVisualGeometry(body, X_FGy, cyl_shape, a.n + ":y", grn);
    plant_->RegisterVisualGeometry(body, X_FGz, cyl_shape, a.n + ":z", blu);
  }

  // Set per-joint viscous damping
  const std::vector<std::pair<std::string, double>> damp = {
      {"fer_joint1", 1.0}, {"fer_joint2", 1.0}, {"fer_joint3", 1.0},
      {"fer_joint4", 1.0}, {"fer_joint5", 1.0}, {"fer_joint6", 1.0}, {"fer_joint7", 1.0},
  };
  for (const auto& [name, d] : damp) {
    plant_->GetMutableJointByName<drake::multibody::RevoluteJoint>(name, robot).set_default_damping(d);
  }

  // Disable collisions by stripping proximity roles
  {
    const auto sid = plant_->get_source_id().value();
    for (drake::multibody::BodyIndex b(0); b < plant_->num_bodies(); ++b) {
      const auto& body = plant_->get_body(b);
      for (const auto gid : plant_->GetCollisionGeometriesForBody(body)) {
        scene_graph_->RemoveRole(sid, gid, drake::geometry::Role::kProximity);
      }
    }
  }

  plant_->Finalize();
}

drake::multibody::MultibodyPlant<double>* FrankaDrakeModule::get_plant() {
  return plant_;
}
drake::geometry::SceneGraph<double>* FrankaDrakeModule::get_scene_graph() {
  return scene_graph_;
}

} // namespace franka_drake 