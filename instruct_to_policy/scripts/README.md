# Instruction to Policy Module
This module is responsible for converting the instructions in natural to policies for franka robotic arm based on Code as Policies. 

## Installation
Install ROS environment:
Recommend to use [Robostack](https://robostack.github.io/GettingStarted.html) to install ros libraries inside a virtual environment.

Install python dependencies:
```bash
pip install numpy scipy shapely astunparse pygments openai imageio==2.4.1 imageio-ffmpeg pybullet moviepy
```

## Run (Under Development)
Currently, this module only runs a demo inside pybullet with Code as Policies prompot.