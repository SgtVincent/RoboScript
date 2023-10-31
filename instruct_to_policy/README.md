# Instruct_to_policy Module
 
## Data process

Please refer to the [data document](./data.md)s more details about the data generation process.

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
