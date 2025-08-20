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