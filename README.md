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


#### Use with libfranka examples
- Start the server, then run libfranka examples against host `127.0.0.1`.

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