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
