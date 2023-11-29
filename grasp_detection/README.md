# Grasp Detection Package

This package contains the grasp detection node for the Franka robot with two-finger gripper. The node should runs `DetectGrasps`  service and publishes the detected grasp pose in the world frame.

General notes:
For a specific/custom grasp model, please create a new ROS conda environment to avoid python dependencies conflicts.
We recommend creating your conda env with:

```bash
mamba create -n grasp_env_<model> python=3.9.16
mamba activate grasp_env_<model>
conda config --env --add channels conda-forge
conda config --env --add channels robostack-staging
conda config --env --remove channels defaults
mamba install ros-noetic-desktop

...
# install other dependencies
...
```

## Install GIGA

Please follow the installation instruction in this [fork of the GIGA](https://github.com/SgtVincent/GIGA).

## Install AnyGrasp 

Please follow the [installation instruction](./src/detectors/anygrasp_sdk/README.md) to install AnyGrasp.

Note that you also need 3D fusion tools in GIGA repo. If you DO NOT need GIGA, you can run the following command to install the GIGA tools only without network dependecies:

```bash
pip install git+https://github.com/SgtVincent/GIGA.git
```
