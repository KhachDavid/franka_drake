### franka_drake: Drake-based FCI-compatible simulation server

Libfranka compatible simulation server for the Franka FCI control box. It allows to run libfranka examples against a simulated robot by passing the `127.0.0.1` host.

#### Features
- 1 kHz FCI loop, all control modes (pos/vel/torque, impedance), motion generators
- Headless/visual and turbo modes
- Models under `models/` (URDF + meshes)

#### Build
```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

#### Run
```bash
./build/bin/franka-fci-sim-embed-example
```

#### Run with Moveit

Once the server is running, you can use Moveit to plan and execute motions.

```bash
ros2 launch franka_gripper gripper.launch.py arm_id:=fer robot_ip:=127.0.0.1 use_fake_hardware:=false
```

```bash
ros2 launch franka_fer_moveit_config moveit.launch.py robot_ip:=127.0.0.1 use_fake_hardware:=false hand:=true launch_gripper_node:=false
```

### Known issues

When integrating into custom Drake controllers, you may encounter an issue where the gripper is not working as expected. This is because the Franka has an overly conservative collision filter that prevents the gripper from closing. In pick_and_place_drake_controller.cpp, we apply a collision filter to exclude the gripper from colliding with the boxes. This is a workaround but seems to be the only way to get the gripper to work without completely disabling collision detection.

#### Use with libfranka examples
- Start the server, then run libfranka examples against host `127.0.0.1`.

### Live compare: real vs sim (single binary)

Run the same libfranka example on both the real robot and the Drake FCI sim and plot joint positions in sync.

One-time setup (publisher endpoint):
```bash
mkdir -p ~/.config/franka
echo 'udp://127.0.0.1:5601' > ~/.config/franka/mon_endpoint
```

Start the server:
```bash
./build/bin/franka-fci-sim-embed-example
```

Run the example twice (same binary), labeling each stream:
```bash
# Real FCI
libfranka/build/examples/pick_and_place --src=real panda0.robot

# Sim FCI
libfranka/build/examples/pick_and_place --src=sim 127.0.0.1
```

Viewer (single-port mode; streams are separated by label):
```bash
cd libfranka/build/examples
python3 ../../examples/plot_compare_real_vs_sim_udp.py \
  --real-port 5601 --sim-port 5601 \
  --ms 1.0 --hz 60 \
  --align-auto --align-joint 5
```

Tips and options:
- `--ms`: time grid in milliseconds (1.0 = libfranka step); both streams are interpolated to this grid.
- `--align-auto`: cross-correlate a joint (default `--align-joint 5`) to auto-estimate and remove lag.
- `--offset-ms N`: additionally shift the REAL stream by N ms (positive moves real earlier).
- Robust to staggered starts: the plot updates when any stream begins; missing segments show as gaps.
- The publisher advances a discrete step index only when arm joints change, so gripper-only motion does not advance time. This keeps traces aligned during grasp/release phases.
- Endpoint can also be overridden per-binary by placing a file `franka_mon_endpoint` next to the executable containing `udp://host:port`.

#### Repository structure
- include/franka_drake/ - public headers
- src/ - implementation (private headers under src/internal/)
- examples/ - runnable examples and demos
- models/ - URDFs and meshes
- docs/ - guides, design notes, troubleshooting, devlog
- test/ - unit/integration tests
- .github/workflows/ - CI

#### Development
- Build: `cmake .. && make -j$(nproc)`
- Tests: `ctest --output-on-failure -j$(nproc)`

#### License
- MIT