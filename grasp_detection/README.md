# Grasp Detection Package

This package contains the grasp detection node for the Franka robot with two-finger gripper. The node should runs `DetectGrasps`  service and publishes the detected grasp pose in the world frame.

## Install GIGA

Please follow the installation instruction in this [fork of the GIGA](https://github.com/SgtVincent/GIGA).

## Install AnyGrasp 

Please follow the [installation instruction](./src/detectors/anygrasp_sdk/README.md) to install AnyGrasp.

Note that you also need 3D fusion tools in GIGA repo. If you DO NOT need GIGA, you can run the following command to install the GIGA tools only without network dependecies:

```bash
pip install git+https://github.com/SgtVincent/GIGA.git
```