# Franka Drake Integration

A Drake-based Franka Control Interface (FCI) simulation server and an embeddable library. It emulates the Franka control box networking protocol so libfranka clients can connect unmodified.

## Project structure

```
franka_drake/
├── apps/
│   ├── franka_fci_sim_server_main.cpp        # Full server (CLI: URDF, dt, headless, turbo)
│   └── franka_fci_sim_embed_example.cpp      # Minimal embedding example (+ Meshcat)
├── include/
│   └── franka_drake/
│       ├── fci_sim_embed.h                   # Embedding API
│       ├── fci_sim_server.h                  # FCI protocol server
│       └── franka_drake_module.h             # Internal module glue
├── models/
│   ├── urdf/
│   │   ├── fer_drake_fingerless.urdf
│   │   └── fer_drake_gripper_fixed.urdf
│   └── meshes/
├── src/
│   ├── embed_fci_sim.cpp                     # Embedding implementation
│   ├── embed_fci_sim_helpers.cpp             # One-shot builder + helpers
│   ├── embed_model_library.cpp               # Model discovery utilities
│   └── CMakeLists.txt
├── scripts/
│   └── (utility scripts)
├── CMakeLists.txt
└── README.md
```

## Dependencies

- Drake (simulation)
- Poco (networking)
- libfranka (FCI data structures and examples compatibility)

Ensure Drake is discoverable (e.g., export `DRAKE_INSTALL_DIR`, update PATH/LD_LIBRARY_PATH per your Drake install).

## Build

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

Artifacts:
- `bin/franka-fci-sim-server`
- `bin/franka-fci-sim-embed-example`
- `lib/libfranka_drake_core.a`

## Knowledge base: Warehouse helpers

Public helper APIs for warehouse-style manipulation demos live in `include/franka_drake/warehouse_helpers.h` and are implemented in `src/warehouse_helpers.cpp`.

Functions:

- SolveCollisionFreeIK: Collision-aware IK to the `fer_hand_tcp` frame.
- FindObjectByName: Simple substring search for a body in the plant.
- WaitForGripper: Advance sim for a fixed time to allow gripper motion.
- ExecuteJointTrajectory: Cosine-blend joint space motion with optional payload handling.
- AttachObject: Rigidly attach an object in front of the TCP for simulated grasps.
- DetachObject: Place object at a final pose and zero velocities.

These are used by `apps/warehouse_drake_controller.cpp` and can be reused by clients embedding `franka_drake_core`.

## Running the full server

You can use the convenience script or the binary directly.

### Script

```bash
# Default (fingerless + visual)
./run_server.sh

# Gripper configuration
./run_server.sh gripper

# Headless (no Meshcat)
./run_server.sh fingerless headless

# Visual + faster-than-real-time (turbo)
./run_server.sh fingerless visual turbo
```

### Direct binary

```
bin/franka-fci-sim-server <path/to/franka_description/package.xml> <urdf> <time_step> [headless] [turbo]
```

Examples:
```bash
# Fingerless, visual, dt=1ms
bin/franka-fci-sim-server models/urdf/package.xml models/urdf/fer_drake_fingerless.urdf 0.001

# Gripper, headless, dt=1ms
bin/franka-fci-sim-server models/urdf/package.xml models/urdf/fer_drake_gripper_fixed.urdf 0.001 true

# Fingerless, visual, turbo (faster-than-real-time)
bin/franka-fci-sim-server models/urdf/package.xml models/urdf/fer_drake_fingerless.urdf 0.001 false turbo
```

### Configuration reference

- Headless: pass `true` (4th arg) to disable Meshcat and enable compact logging.
- Turbo: pass `turbo` (5th arg) to remove the real-time constraint (sim runs as fast as possible).
- Sim time step (`<time_step>`): numeric seconds, e.g. `0.001` for 1 kHz.
- Gripper vs fingerless: choose the URDF:
  - Fingerless: `models/urdf/fer_drake_fingerless.urdf`
  - Gripper: `models/urdf/fer_drake_gripper_fixed.urdf`

Meshcat (visual mode) serves at: http://localhost:7000

## Embedding into your own Drake diagram

Use `FciSimEmbedder` to attach the FCI wiring to your plant and diagram. The embedder can also disable collisions (remove proximity roles) and seeds a sensible default joint configuration via a helper.

```cpp
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/geometry/scene_graph.h>
#include <drake/visualization/visualization_config_functions.h>
#include <drake/geometry/meshcat.h>
#include "franka_drake/fci_sim_embed.h"

using drake::multibody::AddMultibodyPlantSceneGraph;

const double dt = 0.001;
drake::systems::DiagramBuilder<double> builder;
auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, dt);

// Auto-configure the Franka model and attach the FCI systems.
franka_fci_sim::FciSimEmbedder::AutoAttachOptions auto_opts;
auto_opts.prefer_gripper = false;        // true to prefer the gripper URDF
auto_opts.disable_collisions = true;     // remove proximity roles (default true)

franka_fci_sim::FciSimOptions opts;      // runtime options
opts.headless = false;                    // true for headless
opts.turbo = false;                       // true to run faster-than-real-time

auto embed = franka_fci_sim::FciSimEmbedder::AutoAttach(&plant, &scene_graph, &builder, auto_opts, opts);

// Optional Meshcat visualization (omit if headless)
auto meshcat = std::make_shared<drake::geometry::Meshcat>();
drake::visualization::AddDefaultVisualization(&builder, meshcat);

auto diagram = builder.Build();
drake::systems::Simulator<double> simulator(*diagram);
simulator.set_target_realtime_rate(opts.turbo ? 0.0 : 1.0);

// Seed default starting pose and zero velocities
auto& root = simulator.get_mutable_context();
auto& plant_ctx = plant.GetMyMutableContextFromRoot(&root);
franka_fci_sim::SetDefaultFrankaInitialState(plant, plant_ctx);

simulator.Initialize();
embed->StartServer(1337, 1338);
```

### Options (embedding API)

- `AutoAttachOptions::prefer_gripper` (bool): choose gripper variant when adding default model.
- `AutoAttachOptions::disable_collisions` (bool, default true): strip proximity roles to disable collisions.
- `FciSimOptions::headless` (bool): enable compact headless logging (visualization remains under your control).
- `FciSimOptions::turbo` (bool): remove real-time target; sim runs as fast as possible.
- Sim time step: either pass your desired `dt` to `AddMultibodyPlantSceneGraph`, or use `AutoBuildAndAttach(builder, time_step, ...)`.

### Common configurations

```cpp
// Fingerless + visual (default)
auto_opts.prefer_gripper = false; opts.headless = false; opts.turbo = false;

// Gripper + headless + turbo
auto_opts.prefer_gripper = true;  opts.headless = true;  opts.turbo = true;

// Visual + turbo (fast motion testing)
opts.headless = false; opts.turbo = true;

// One-call helper that also sets the time step:
auto build = franka_fci_sim::AutoBuildAndAttach(&builder, /*time_step=*/0.001, auto_opts, opts);
// build.plant, build.scene_graph, build.embedder are ready
```

## Notes

- Collisions are disabled by default when using the AutoAttach overload that accepts a `SceneGraph`.
- The default initial pose matches typical ROS 2 `franka_description` start (elbow-up); gripper starts open if present.
- The server mirrors libfranka behavior sufficiently for running upstream examples against `127.0.0.1`.
