# Simulating Franka FER3 Control Box in Drake

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

#### 5. Run the example of a moving cube

```
mkdir build && cd build
cmake ..
make -j 32
./example/moving_cube
```

You should observe torque being applied to the center of the cube. Here is a demo:



https://github.com/user-attachments/assets/f8c52540-4a6a-4bb5-b292-1737324baa2e

#### 6. Run the main file and provide CLI arguments
```
./sim ~/ws/franka/src/franka_description/package.xml ~/final_project/franka_drake_test/fer_drake.urdf
```