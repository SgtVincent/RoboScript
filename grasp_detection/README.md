# Grasp Detection Package

This package contains the grasp detection node for the Franka robot with two-finger gripper. The node should runs `DetectGrasps`  service and publishes the detected grasp pose in the world frame.

## Installation 
For a specific/custom grasp model, please create a new ROS conda environment to avoid python dependencies conflicts.
We recommend creating your conda env with:

```bash
mamba create -n grasp_env_<model> python=3.9.16
mamba activate grasp_env_<model>
conda config --env --add channels conda-forge
conda config --env --add channels robostack-staging
conda config --env --remove channels defaults
```

Note that you also need some geometric & perception tools in GIGA repo. If you DO NOT need GIGA, you can run the following command to install the tools ONLY without GIGA network installation:

```bash
pip install git+https://github.com/SgtVincent/GIGA.git
```
### Install grasp detection models 

Before you install full ros dependencies, we recommend you install the grasp detection models first.
Since 1) Full ros environment setup adds isolated C/C++ compilters inside mamba environment and overwrites C/C++ environment variables, 2) and a lot of detection models are built locally with system g++/nvcc executables, installing detection models first can avoid a lot of potential issues. 

Models including GIGA, AnyGrasp have been verified and integrated into the package. You can choose any of them to install or deloy your own model. 

#### Install GIGA (Optional)

Please follow the [installation](src/detectors/GIGA/README.md) to install our fork of the GIGA (https://github.com/SgtVincent/GIGA) with some minor fixes.

#### Install AnyGrasp (Optional)

AnyGrasp is a SOTA grasp detection SDK library developed by [GraspNet](https://graspnet.net/). You can get access to the library by applying to its license [here](https://github.com/graspnet/anygrasp_sdk/#license-registration).

Please follow the [Installation](src/detectors/anygrasp_sdk/README.md) to install AnyGrasp.

### Install full ros dependencies

```bash
# Install ros-noetic into the environment (ROS1)
mamba install ros-noetic-desktop-full compilers cxx-compiler cmake pkg-config make ninja colcon-common-extensions catkin_tools boost-cpp
```

### Create a separate catkin workspace

By default, one catkin workspace uses one mamba ros environment to build the packages.
To avoid conflicts, create a separate catkin workspace for your grasp detection models.

```bash
mamba activate grasp_env_<model>

# Create a catkin workspace 
mkdir -p /path/to/grasp_<model>_ws/src
cd /path/to/grasp_<model>_ws
catkin init

# Link the grasp detection models to the new catkin workspace
ln -s /path/to/catkin_ws/src/grasp_detection /path/to/grasp_<model>_ws/src/grasp_detection

# Build the catkin workspace
catkin build
```

## Run

Now you should be good to run the grasp detection node. 

```bash
# Activate the grasp detection environment
cd /path/to/grasp_<model>_ws
source devel/setup.bash

roslaunch grasp_detection run_node.launch
```
For custom model deployment and parameters setting, please refer to [detectors](src/detectors/)