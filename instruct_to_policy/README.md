# LLM-manipulation Benchmark

This benchmark is to evaluate the performance of different multimodal large language models (LLM) on the task of code generation for object manipulation. The task is to generate python script to control a robotic arm to execute tasks specified in natural language instructions.

To minimize the sim-to-real gap in code generation for robotics control, the whole pipeline is built on ROS (Robot Operating System) and Gazebo simulation. Thus the generated code tested in Gazebo can be directly deployed to control real robots. 

**TODO** Restructure documentation in a more readable way. 

## Installation

The whole pipeline runs on ROS 1 Noetic. You can either install ROS directly with apt or inside conda environment with Robostack. It is a known issue that native ROS 1 environment is not compatible with conda environment, so Robostack is recommended to use for ROS setup.

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

# Install ros-noetic common dev tools
mamba install compilers cxx-compiler cmake pkg-config make ninja colcon-common-extensions catkin_tools boost-cpp ros-noetic-ros-numpy

# Install ros gazebo packages
mamba instsall ros-noetic-gazebo-ros

# Install realsense ros packages integrated with gazebo 
mamba install ros-noetic-realsense2-camera ros-noetic-realsense2-description ros-noetic-librealsense2

# (Optional) Install moveit dependencies if missing error occurs 
pip install catkin-tools wstool 
```


### Install MoveIt ROS Packages

Install MoveIt from Robostack Prebuilt Binaries
```bash
mamba install ros-noetic-moveit 

# You might also need other dependecies for moveit_tutorials package
# please install the dependencies with: 
rosdep install -y --from-paths . --ignore-src --rosdistro noetic
```

To test the installation, follow the official instructions in official tutorials [Build your Catkin Workspace](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html#create-a-catkin-workspace) to run the tutorial packages. 

Note: `moveit_tuorials` depends on QT5, which further depends on opengl libraries.
```bash
sudo apt install mesa-common-dev libglu1-mesa-dev freeglut3 freeglut3-dev 
```

### Install Franka ROS

<!-- If you **ONLY** need to run the pipeline on the real panda robot, you can install Franka ROS Packages from Robostack prebuilt binaries: -->
```bash 
mamba install ros-noetic-franka-ros
```
<!-- **However**, if you want to test the pipeline of the franka robot in gazebo simulation, you need to build the franka ros packages from source, following the tutorials on [Gazebo Simulation Integration](https://ros-planning.github.io/moveit_tutorials/doc/gazebo_simulation/gazebo_simulation.html?highlight=gazebo#gazebo-simulation-integration). -->


### Install Realsense Plugni for Gazebo

Please refer to this [repo](https://github.com/SgtVincent/realsense_gazebo_plugin) for installation:
```bash
git clone https://github.com/SgtVincent/realsense_gazebo_plugin

cd /path/to/catkin_ws 
catkin build realsense_gazebo_plugin
```


Then you can run example launch file to test the installation:
```bash
source devvel/setup.bash
roslaunch realsense_gazebo_plugin view_d435_model_rviz_gazebo.launch 
```

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

Numpy >= 1.24 removed `np.float` property, which is used in `ros_numpy`:
```
AttributeError: module 'numpy' has no attribute 'float'.
```
Solution: `numpy` downgrade to `<1.24`


If you had some link failures during catkin make, please add the corresponding libraries to `target_link_libraries()` in the cmake file of moveit_tutorials (depending on the OS). 

### Install Code as policies dependencies:
```bash
# code as policies dependencies
pip install numpy scipy shapely astunparse pygments openai imageio==2.4.1 imageio-ffmpeg pybullet moviepy
```

### (Optional) Install Development Tools

#### Jupyter-ROS
[Jupyter-ROS](https://github.com/RoboStack/jupyter-ros) is a set of ROS tools to run in jupyter notebook
with interactive command & visualization. It is not required for the pipeline, but it is useful for debugging and visualization with jupyter. 

```bash
# install dependencies inside the mamba environment
mamba install jupyter bqplot pyyaml ipywidgets ipycanvas
# install jupyter-ros
mamba install jupyter-ros -c robostack
```
Then you need to create a ros kernel, which has loaded catkin_ws environments, from the official [instructions](https://jupyter-ros.readthedocs.io/en/latest/user_troubleshooting.html).


## Data process

### Convert urdf.xacro to sdf

```bash
# urdf.xacro -> urdf
rosrun xacro xacro <model_name>.urdf.xacro > <model_name>.cabinet_0.urdf
# urdf -> sdf
gz sdf -p <model_name>.urdf > <model_name>.sdf
```




## Run

Make sure you have activated the ROS environment and catkin workspace:
```bash
mamba activate <your_ros_env>
cd /path/to/catkin_ws
source devel/setup.bash
```

### Demo of code generation for Pick and Place with MoveIt

First, you need to launch the gazebo simulation and moveit nodes with the following command:
```bash
roslaunch instruct_to_policy run_panda_moveit_gazebo.launch 
```

Then, you can run the code generation demo script by roslauch:
```bash
roslaunch instruct_to_policy run_cap.launch 
```

### Gazebo plugins (deprecated)

**No longer used.**

Our locally-built plugins are in `/path/to/catkin_ws/devel/lib`. To load it in gazebo, you should specify the LD_LIBRARY_PATH to the directory of the plugin. 

```bash
e.g. export LD_LIBRARY_PATH=/path/to/catkin_ws/devel/lib
```
