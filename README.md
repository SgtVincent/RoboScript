# LLM-manipulation Benchmark

This benchmark is to evaluate the performance of different multimodal large language models (LLM) on the task of code generation for object manipulation. The task is to generate python script to control a robotic arm to execute tasks specified in natural language instructions.

To minimize the sim-to-real gap in code generation for robotics control, the whole pipeline is built on ROS (Robot Operating System) and Gazebo simulation. Thus the generated code tested in Gazebo can be directly deployed to control real robots. 

## Installation

The whole package runs on ROS 1 Noetic. You can either install ROS directly with apt or inside conda environment with Robostack. It is a known issue that native ROS 1 environment is not compatible with conda environment, so Robostack is recommended to use for ROS setup.

The components needed for this project:
- Franka ROS: Integration of [libfranka](https://frankaemika.github.io/docs/libfranka.html) and [ROS control](https://wiki.ros.org/ros_control), which provides low-level connection/ low-level control to Franka robots.
- [MoveIt](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html): Full-stack library for robot mobile manipulation: planning, manipulation, perception, kinematics, control. 
- code generation dependencies.

### Download and Install mamba (Recommended)
You should download the mamba package manager to install the dependencies. Mamba is a faster version of conda, which is used to install the C++/python/ROS dependencies in this project.

Please also disable your previously installed (if any, comment out the conda init scritps in ~/.bashrc or ~/.zshrc) conda environment and initialize the conda environment with `conda init` with mamba installation. See [miniforge](https://github.com/conda-forge/miniforge#mambaforge) for download and initialization.

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

Install common tools and dependency pacakges:
```bash
# Reactivate the environment to initialize the ros env
mamba deactivate
mamba activate ros_env

# Install ros-noetic common dev tools
mamba install compilers cxx-compiler cmake pkg-config make ninja colcon-common-extensions catkin_tools boost-cpp ros-noetic-ros-numpy

# Install ros gazebo packages
mamba install ros-noetic-gazebo-ros

# Install realsense ros packages and image processing packages
mamba install ros-noetic-realsense2-camera ros-noetic-realsense2-description ros-noetic-librealsense2 ros-noetic-image-pipeline
```

### Prepare your catkin workspace 

```bash
# make sure you have activated the ros environment
mamba activate ros_env
# create a catkin workspace
mkdir -p /path/to/catkin_ws/src
cd /path/to/catkin_ws
catkin init
cd /path/to/catkin_ws/src
git clone --recurse-submodules https://github.com/SgtVincent/llm-manipulation-bench.git
```


### Install MoveIt ROS Packages

Install MoveIt from Robostack Prebuilt Binaries

```bash
mamba install ros-noetic-moveit 
```

#### Test with moveit_tutorials (Optional)

**You are recommended to skip this part if you only need to run the pipeline and have no trouble running moveit**

To test the installation, follow the official instructions in official tutorials [Build your Catkin Workspace](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html#create-a-catkin-workspace) to run the tutorial packages. 

```
# You might also need other dependecies for moveit_tutorials package
# please install the dependencies with: 
rosdep install -y --from-paths . --ignore-src --rosdistro noetic
```

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


<!-- ### Install GIGA grasp detection package

Please refer to the [GIGA](./GIGA/) package for installation instructions.  -->


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

### Install code generation dependencies:
```bash
# code generation dependencies
pip install numpy=1.23 scipy shapely astunparse pygments openai open3d imageio==2.4.1 imageio-ffmpeg moviepy
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


### Build the catkin packages

```bash
cd /path/to/catkin_ws
catkin build
```


## Data preparation

Please download the preprocessed data from [ADD_LINK_HERE](#).

Please refer to [data](instruct_to_policy/data.md) for more details.


## Run

Please refer to [instruct_to_policy](instruct_to_policy/README.md) package for more details.
