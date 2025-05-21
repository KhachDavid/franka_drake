# Simulating Franka FER3 Control Box in Drake

## Week of May 19

The goal was to implement a 1 kHz controller that ensured the stability of Franka. Initially, I attempted to achieve this with no P and only gravity compensation. The arm stabilized only when using a really small time step. This is not the best idea, since the simulation of 20 seconds took over 5 minutes.

![no_pd_0_00001](https://github.com/user-attachments/assets/0692279a-873e-4fe9-bddb-e92be8c31494)

With no P control, and a smaller timestep of 1kHz, Drake's gravity compensation solver is unable to keep up. This was the case even after providing a hypothetical unlimited torque to each motor. Another approach was switching the default implicit euler integration to Runge Kutta, but that does not make a difference. With all of these methods, the drift was always similar to the graph below:

![joint7_test_08_to_07](https://github.com/user-attachments/assets/43112331-d42c-40fb-907a-bd3e7f5fcd04)

Therefore, P control was introduced to keep the robot static. 

P = 26000
![P26000](https://github.com/user-attachments/assets/84fbeaa0-ab5f-4fc0-b37e-c756c3924f06)

P = 16000
![static_p16000](https://github.com/user-attachments/assets/b02502c3-4a53-459a-a082-9eb27310e175)

P = 16000, zoomed in, joint 7 small error
![static_p16000_joint7](https://github.com/user-attachments/assets/0606a7b0-1889-49ed-b63c-9b6077ec3d43)

P = 5500, zoomed in, joint 7 slightly larger error
![static_p5500](https://github.com/user-attachments/assets/adf36f1c-4707-42ce-bd88-98979a730bfd)



## Week of May 12

This week I implemented PD setpoint control with gravity compensation. For a robot with no friction at the joints, $\theta{e}$ converges to 0 when using this controller.

```cpp
  // ----------------------------------------------------------------------
  //  Sum:   τ = τ_PD  +  g̃(q)
  // ----------------------------------------------------------------------
  auto* sum = builder.AddSystem<systems::Adder<double>>(2, n);

  builder.Connect(pd   ->get_output_port(), sum->get_input_port(0));
  builder.Connect(g_comp->get_output_port(), sum->get_input_port(1));
  builder.Connect(sum->get_output_port(), plant.get_actuation_input_port());

```

In the example above torque is calculate by adding the PD and gravity components. When no desired position is indicated, the robot is perfect still at a given `q0`.

This controller was put into use for a task of moving joint 7 by 0.1 radians, going from 0.8 to 0.7. Below is a graph generated using drake's vector log sink.

![joint7_test_08_to_07](https://github.com/user-attachments/assets/43112331-d42c-40fb-907a-bd3e7f5fcd04)

Blue curve (position) – Joint 7 starts at 0.8 rad, crosses the 0.7 rad set-point around sample = 11 000, then keeps drifting down.

Orange curve (velocity) – Small negative velocity the whole time.

Since there is a steady state error, one way to alleviate this would be adding an integral term.

Another idea is to increase feedback gains


## Week of May 5

This week, the goal is to understand libfranka inputs and experiment with the anti gravity component of franka joints in drake. In addition, last week I created technical debt by uncommenting collision stl files in the URDF. This week I used blender to convert them to `.obj` format, as required by Drake. This went smoothly and the files were pushed as a fork of franka_description and can be found [here](https://github.com/KhachDavid/franka_description).

For the purpose of understanding libfranka inputs, I used the deep research functionality on chat gpt (chat [here](https://chatgpt.com/share/681a20e7-df5c-8001-8a9f-806dfbe8daca)). Below are the input types:

    Joint positions – franka::JointPositions (7 joint angles in radians).

    Joint velocities – franka::JointVelocities (7 joint angular rates in rad/s).

    Cartesian pose – franka::CartesianPose (4 × 4 homogeneous matrix, optional elbow hint).

    Cartesian twist (velocity) – franka::CartesianVelocities (linear + angular tool-frame velocities).

    Joint torques – franka::Torques (7 joint torques in N·m).

Any motion-generator command (1–4) can be returned alone or together with a torque command (5) by calling robot.control(motion_cb, torque_cb). Motion commands are terminated with franka::MotionFinished(...).

Examples were written for each type of input, where joint 7 was moved on the real franka. For position, I measured the difference between joint 7 input and output. 

```
Final J7 error = -0.000316827 rad
```

This is about 1/5th of a degree, which is close enough to consider that the output was exactly matching the input. The repository containing these examples can be found [here](https://github.com/KhachDavid/libfranka-examples).

In the case of Drake gravity compensation, I found that PyBullet has a gravity compensation term in this [repo](https://github.com/PaulPauls/franka_emika_panda_pybullet/tree/master):

![image](https://github.com/user-attachments/assets/4239111f-42d4-477a-aa64-10ad67f6c6d9)

For implementing anti-gravity torque in Drake, we have to phrase this as a statics problem.

$$\tau_{applied}(q) = -\tau_{g}(q)$$

```cpp
auto id = builder.AddSystem<InverseDynamics<double>>(
    &plant,
    InverseDynamics<double>::InverseDynamicsMode::kGravityCompensation);
```

During each simulator tick, this evaluates and finds the torque needed to compensate for gravity. 

![image](https://github.com/user-attachments/assets/bc1c36f9-76ef-4a73-a5a2-b91a8dc51bbd)


## Week of April 28

This week, the goal is to have Franka simulation up and running on Drake. This was achieved by using the franka_description repository found [here](https://github.com/m-elwin/franka_description/tree/panda). Note that the you must be on the panda branch.

Then the Panda FER urdf file was generated using the franka description repository.

```
xacro <PATH_TO_FRANKA_REPO>/franka_description/robots/fer/fer.urdf.xacro hand:=true > <PATH_TO_THIS_REPO>/fer_drake.urdf
```

The resulting URDF file included .stl mesh files, which causes issues in Drake because STL files do not contain scale or unit metadata. This can lead to incorrect rendering or physics behavior. At the moment the collisions were commented out to ensure the compilation of the Drake program in C++. In order to use them, they must be converted to .obj or .dae files. Here is a link to the file showing the commented out collisions. [File](https://github.com/KhachDavid/simTorealTosim/blob/main/fer_drake.urdf#L27)

Once the mesh compatibility was resolved, the robot was successfully loaded into Drake using MultibodyPlant and visualized through SceneGraph. The binary was set up to continue running the WebSocket until the user presses enter. This was created to improve the developer experience.

At this point, Franka was just floating in the scene. This [commit](https://github.com/KhachDavid/simTorealTosim/commit/eff95d17b6ee18eaacfdb587a875bab8475700e9) attached franka base frame to `0,0,0`. The result was this. It can be observed that the base is attached and all the other joints are being pulled down by gravity.

![image](https://github.com/user-attachments/assets/2fa1e2d2-06c1-4894-b45b-8007f5293ae1)


### Setup Instructions

#### 1. Get Franka Description
Clone the following repo into `~/ws/franka/src/franka_description`. Make sure you are on the panda branch. 

```
git clone https://github.com/m-elwin/franka_description
```

#### 2. Install Dranka

```
# Note that this is for Ubuntu 24.04 Noble Numbat
wget https://github.com/RobotLocomotion/drake/releases/download/v1.40.0/drake-1.40.0-noble.tar.gz

mkdir -p ~/drake
tar -xvzf drake-v1.40.0-noble.tar.gz -C drake --strip-components=1
```

#### 3. Add Drake to shell environment
```
echo 'export DRAKE_INSTALL_DIR=$HOME/drake' >> ~/.bashrc
echo 'export PATH=$DRAKE_INSTALL_DIR/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$DRAKE_INSTALL_DIR/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

#### 4. Install libfranka
```
git clone https://github.com/KhachDavid/libfranka.git
cd libfranka
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
```

## Week of April 21

The main goal of this week is to understand Drake’s environment and see an example of Franka running and moving in Drake.

I created this Docker pipeline that successfully runs Drake in an Ubuntu 22.04 and ROS2 Humble container. The original Dockerfile provided by ROS2-Drake had to be changed due to a failing CI/CD pipeline. The fork can be found [here](https://github.com/KhachDavid/drake-ros).

Docker Commands to Run:
```
docker build   --build-arg ARCH=amd64   --build-arg BUILD_DRAKE_FROM_SOURCE=false   -t drake-ros:humble .
docker compose up -d

# allow containers to use your X server
xhost +local:docker

# run, mounting your workspace and enabling GUI
docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix drake-ros:humble
```

Below is the iiwa_manipulator on rviz:

It looks like joint orientations are incorrect. Seems to be an issue reported by many users [here](https://github.com/RobotLocomotion/drake-ros/issues/358).
![image](https://github.com/user-attachments/assets/8e621433-6593-4499-9d7c-48f69fd65db5)
