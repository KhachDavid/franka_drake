# Simulating Franka FER3 Control Box in Drake

## Week of April 28

This week, the goal is to have Franka simulation up and running on Drake

### Setup Instructions

#### 1. Get Franka Description
Clone the following repo into `~/ws/franka/src/franka_description`

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
# simTorealTosim

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
