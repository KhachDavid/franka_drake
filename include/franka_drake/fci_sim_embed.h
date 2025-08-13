#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <drake/multibody/plant/multibody_plant.h>
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

}  // namespace franka_fci_sim


