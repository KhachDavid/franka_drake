#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/geometry/scene_graph.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/analysis/simulator.h>

#include "franka_drake/fci_sim_server.h"

namespace franka_fci_sim {

// Lightweight options for embedding the FCI simulation server into an
// existing Drake diagram. The embedding code will add a joint-space impedance
// controller, gravity compensation, and the networking server, and wire them
// to the provided plant within the given builder.
struct FciSimOptions {
  // Enable additional std::cout status messages intended for headless runs.
  bool headless = false;
  // No real-time constraints in your own simulation loop; this only toggles
  // logging behavior inside the mirror system.
  bool turbo = false;
  // Hint for which end-effector frame to prefer if multiple are present.
  // If empty, a sensible default is auto-detected.
  std::string ee_frame_hint;
};

// Handle class that encapsulates all wiring/state required to expose a Drake
// MultibodyPlant as a Franka FCI-compatible simulation over TCP/UDP.
//
// Usage sketch (real-time by default):
//   DiagramBuilder<double> builder;
//   auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.001);
//   Parser parser(&plant, &scene_graph);
//   const auto robot = parser.AddModels(urdf_path)[0];
//   plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base", robot));
//   plant.Finalize();
//   auto embed = franka_fci_sim::FciSimEmbedder::Attach(&plant, robot, &builder, {});
//   auto diagram = builder.Build();
//   drake::systems::Simulator<double> sim(*diagram);
//   sim.set_target_realtime_rate(1.0);  // real-time by default
//   sim.Initialize();
//   embed->StartServer(1337, 1338);
//   sim.AdvanceTo(std::numeric_limits<double>::infinity());  // run until externally stopped
//   embed->StopServer();
class FciSimEmbedder {
 public:
  // Attaches the FCI controller + gravity compensation + state mirror and
  // networking server to the provided builder/plant. Returns a handle that
  // manages lifetime and exposes Start/Stop for the server. This must be
  // called before builder.Build().
  static std::unique_ptr<FciSimEmbedder> Attach(
      const drake::multibody::MultibodyPlant<double>* plant,
      const drake::multibody::ModelInstanceIndex& robot_instance,
      drake::systems::DiagramBuilder<double>* builder,
      const FciSimOptions& options = {});

  // Overload: also pass SceneGraph so this call can optionally disable
  // collisions by removing proximity roles immediately after wiring.
  static std::unique_ptr<FciSimEmbedder> Attach(
      const drake::multibody::MultibodyPlant<double>* plant,
      const drake::multibody::ModelInstanceIndex& robot_instance,
      drake::geometry::SceneGraph<double>* scene_graph,
      drake::systems::DiagramBuilder<double>* builder,
      const FciSimOptions& options = {},
      bool disable_collisions = true);

  // Attach and auto-configure convenience: modifies the provided plant so the
  // caller doesn't need to add actuators/damping/welds manually. If a Franka
  // model instance is not found in the plant, a default model is added from
  // the built-in URDFs (fingerless by default, or gripper if preferred).
  struct AutoAttachOptions {
    // Prefer gripper configuration when adding a default model.
    bool prefer_gripper = false;
    // Optional package.xml path. If empty, uses "models/urdf/package.xml".
    std::string package_xml_path;
    // Optional explicit URDF path override. If empty, uses a built-in default
    // based on prefer_gripper.
    std::string urdf_path_override;
    // When true, remove proximity roles (disable collisions) for all robot
    // bodies during AutoAttach if a SceneGraph pointer is provided (see
    // overload below). Defaults to true to mirror server behavior.
    bool disable_collisions = true;
  };

  // Finds or adds a Franka model instance in the plant, applies default
  // configuration (actuators, damping, base weld), finalizes the plant if
  // needed, and attaches the FCI systems. Returns the embedder handle.
  static std::unique_ptr<FciSimEmbedder> AutoAttach(
      drake::multibody::MultibodyPlant<double>* plant,
      drake::systems::DiagramBuilder<double>* builder,
      AutoAttachOptions auto_options,
      FciSimOptions options);

  // Overload: also pass SceneGraph so AutoAttach can disable collisions.
  // If scene_graph is non-null and auto_options.disable_collisions is true,
  // all proximity roles attached to the plant's bodies will be removed.
  static std::unique_ptr<FciSimEmbedder> AutoAttach(
      drake::multibody::MultibodyPlant<double>* plant,
      drake::geometry::SceneGraph<double>* scene_graph,
      drake::systems::DiagramBuilder<double>* builder,
      AutoAttachOptions auto_options,
      FciSimOptions options);

  // Convenience overload with all defaults.
  static std::unique_ptr<FciSimEmbedder> AutoAttach(
      drake::multibody::MultibodyPlant<double>* plant,
      drake::systems::DiagramBuilder<double>* builder);

  // Non-copyable, non-movable (holds internal system pointers owned by builder)
  FciSimEmbedder(const FciSimEmbedder&) = delete;
  FciSimEmbedder& operator=(const FciSimEmbedder&) = delete;

  // Starts the TCP/UDP FCI server threads. Safe to call after Diagram is
  // built/initialized; idempotent.
  void StartServer(uint16_t tcp_port = 1337, uint16_t udp_port = 1338);

  // Stops the server threads; safe to call during shutdown.
  void StopServer();

  // Optional: override mode change callback (e.g., for app-level telemetry).
  void SetModeChangeHandler(FrankaFciSimServer::ModeChangeHandler handler);

  ~FciSimEmbedder();

 private:
  FciSimEmbedder(const drake::multibody::MultibodyPlant<double>* plant,
                 const drake::multibody::ModelInstanceIndex& robot_instance,
                 drake::systems::DiagramBuilder<double>* builder,
                 const FciSimOptions& options);

  // Internal wiring performed by Attach().
  void ConfigureSystems();

  // PIMPL to keep header small and stable.
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Convenience enums for high-level helpers
enum class RobotConfig { Fingerless, Gripper };
enum class RunMode { Visual, Headless, Turbo };

// High-level “one-call” builder that constructs a plant+diagram, wires the
// FCI embed, configures default actuators/damping/collision, initializes a
// simulator (real-time by default unless Turbo), and starts the server.
struct EmbeddedApp {
  std::unique_ptr<drake::systems::Diagram<double>> diagram;
  std::unique_ptr<drake::systems::Simulator<double>> simulator;
  std::unique_ptr<FciSimEmbedder> embedder;
};

struct BuildArgs {
  std::string package_xml_path;
  RobotConfig config = RobotConfig::Fingerless;
  double time_step = 0.001;
  RunMode mode = RunMode::Visual;  // Visual or Headless; Turbo overrides RT rate
};

// Build a complete embedded app from scratch (URDF parse → plant wiring →
// embed → simulator). Starts the server automatically.
EmbeddedApp BuildEmbeddedApp(const BuildArgs& args);

// Utility: run a real-time loop (or turbo) with a small sleep when not turbo.
void RunRealtimeLoop(drake::systems::Simulator<double>& simulator, bool turbo);

  // Utility: set a sensible default initial joint configuration (q) and
  // zero velocities (v) on the provided plant/context. Matches the server
  // main defaults (ROS 2 franka_description style pose, open gripper if any).
  void SetDefaultFrankaInitialState(drake::multibody::MultibodyPlant<double>& plant,
                                    drake::systems::Context<double>& plant_context);

}  // namespace franka_fci_sim


