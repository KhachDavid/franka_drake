# Simulating Franka FER3 Control Box in Drake

## Week of April 28

This week, the goal is to have Franka simulation up and running on Drake. This was achieved by using the franka_description repository found [here](https://github.com/m-elwin/franka_description/tree/panda). Note that the you must be on the panda branch.

Then the Panda FER urdf file was generated using the franka description repository.

```
xacro <PATH_TO_FRANKA_REPO>/franka_description/robots/fer/fer.urdf.xacro hand:=true > <PATH_TO_THIS_REPO>/fer_drake.urdf
```

The resulting URDF file included .stl mesh files, which causes issues in Drake because STL files do not contain scale or unit metadata. This can lead to incorrect rendering or physics behavior. At the moment the collisions were commented out to ensure the compilation of the Drake program in C++. In order to use them, they must be converted to .obj or .dae files. Here is a link to the file showing the commented out collisions. [File](https://github.com/KhachDavid/simTorealTosim/blob/main/fer_drake.urdf#L27)

Once the mesh compatibility was resolved, the robot was successfully loaded into Drake using MultibodyPlant and visualized through SceneGraph. The binary was set up to continue running the WebSocket until the user presses enter. This was created to improve the developer experience.

At this point, Franka was just falling in the scene. With `simulator.advanceTo(1)` franka would be below z=0, indicating that Drake's built-in gravity is pulling the robot down. This barebones initalization of franka can be found in main.cpp of this [commit](https://github.com/KhachDavid/simTorealTosim/blob/1797664abb83c181a32993c778f32256fca9bf8c/main.cpp).

![image](https://github.com/user-attachments/assets/7355436f-3199-46e3-bdce-3f141aa78d23)


This [commit](https://github.com/KhachDavid/simTorealTosim/commit/eff95d17b6ee18eaacfdb587a875bab8475700e9) attached franka base frame to `0,0,0`. The result was this. It can be observed that the base is attached and all the other joints are still being pulled down by gravity.

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
