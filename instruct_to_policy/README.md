# Instruction to Policy Module
This module is responsible for converting the instructions in natural to policies for franka robotic arm based on Code as Policies. 

## Installation

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

Install Franka ROS Packages from Robostack Prebuilt Binaries
```bash 
mamba install ros-noetic-franka-ros
```

To test the installation, follow the official instructions in official tutorials [Build your Catkin Workspace](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html#create-a-catkin-workspace) to run the tutorial packages. 

#### Troubleshooting 
If you had some link failures during catkin make, please add the corresponding libraries to `target_link_libraries()` in the cmake file of moveit_tutorials (depending on the OS). 


moveit_tuorials depend on QT5, which further depends on opengl libraries.
```bash
sudo apt install mesa-common-dev libglu1-mesa-dev freeglut3 freeglut3-dev 
```


### Install python dependencies of using openai API:
```bash
# code as policies dependencies
pip install numpy scipy shapely astunparse pygments openai imageio==2.4.1 imageio-ffmpeg pybullet moviepy
```

## Run (Under Development)
Currently, this module only runs a demo inside pybullet with Code as Policies prompot.