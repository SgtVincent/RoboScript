# Instruction to Policy Module

This module is responsible for converting the instructions in natural to policies for Franka robotic arm based on Code as Policies. 

## Installation

The whole package runs on ROS 1 Noetic. You can either install ROS directly with apt or inside conda environment with Robostack. It is a known issue that native ROS 1 environment is not compatible with conda environment, so Robostack is recommended to use for ROS setup.

The components needed for this project:
- Franka ROS: Integration of [libfranka](https://frankaemika.github.io/docs/libfranka.html) and [ROS control](https://wiki.ros.org/ros_control), which provides low-level connection/ low-level control to Franka robots.
- [MoveIt](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html): Full-stack library for robot mobile manipulation: planning, manipulation, perception, kinematics, control. 
- Code As Policies dependencies.


### Install ROS environment with Robostack (Recommended)

Recommend to use [Robostack](https://robostack.github.io/GettingStarted.html) to install ros libraries inside a virtual environment.

Assuming you have installed conda and mamba in your computer. Create a new environment with conda-forge and robostack channels:

```bash
mamba create -n ros_env
mamba activate ros_env

# this adds the conda-forge channel to the new created environment configuration 
conda config --env --add channels conda-forge
# and the robostack channel
conda config --env --add channels robostack-staging
# remove the defaults channel just in case, this might return an error if it is not in the list which is ok
conda config --env --remove channels defaults
```

Install ROS1 Noetic:
```bash
# Install ros-noetic into the environment (ROS1)
mamba install ros-noetic-desktop-full
```

Install common tools and moveit dependencies:
```bash
# Reactivate the environment to initialize the ros env
mamba deactivate
mamba activate ros_env

# Install common tools
mamba install compilers cxx-compiler cmake pkg-config make ninja colcon-common-extensions catkin_tools

# Install moveit dependencies
pip install catkin-tools wstool
```

### Install MoveIt ROS Packages

Install MoveIt from Robostack Prebuilt Binaries
```bash
mamba install ros-noetic-moveit
```

To test the installation, follow the official instructions in official tutorials [Build your Catkin Workspace](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html#create-a-catkin-workspace) to run the tutorial packages. 

Note: `moveit_tuorials` depends on QT5, which further depends on opengl libraries.
```bash
sudo apt install mesa-common-dev libglu1-mesa-dev freeglut3 freeglut3-dev 
```

### Install Franka ROS

If you **ONLY** need to run the pipeline on the real panda robot, you can install Franka ROS Packages from Robostack prebuilt binaries:
```bash 
mamba install ros-noetic-franka-ros
```
**However**, if you want to test the pipeline of the franka robot in gazebo simulation, you need to build the franka ros packages from source, following the tutorials on [Gazebo Simulation Integration](https://ros-planning.github.io/moveit_tutorials/doc/gazebo_simulation/gazebo_simulation.html?highlight=gazebo#gazebo-simulation-integration).




### Troubleshooting 

Bug of `ros-noetic-genpy` 0.6.15 with gazebo:

```
[ERROR] [xxxxx]: Unpause physics service call failed: transport error completing service call: receive_once[/gazebo/unpause_physics]: DeserializationError cannot deserialize: unknown error handler name 'rosmsg'
```
Solution: Upgrade `ros-noetic-genpy` to 0.6.16 by building from source:

```bash
cd /path/to/catkin_ws/src
git clone --branch 0.6.16 https://github.com/ros/genpy.git
cd ..
catkin build genpy
```

Python 3.9 no longer supports thread.isAlive() method:

```bash
AttributeError: 'Thread' object has no attribute 'isAlive'
```

Solution: https://transang.me/add-apt-repository-does-not-work-after-updating-to-python-3-9/


If you had some link failures during catkin make, please add the corresponding libraries to `target_link_libraries()` in the cmake file of moveit_tutorials (depending on the OS). 

### Install Code as policies dependencies:
```bash
# code as policies dependencies
pip install numpy scipy shapely astunparse pygments openai imageio==2.4.1 imageio-ffmpeg pybullet moviepy
```

## Run (Under Development)
Currently, this module only runs a demo inside pybullet with Code as Policies prompot.