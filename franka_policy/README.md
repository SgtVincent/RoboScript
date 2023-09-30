## Connecting to Franka
1. Head to `https://172.31.154.43/`, which is the local server of the franka. Make sure to disable the network proxy as othewise the server is not reachable.
2. Deactive that break
3. Switch the FCI mode by pressing the ip on the top navigation bar.
![image](https://github.com/SysCV/franka-sim2real/assets/11755597/465f69f2-80d9-48d6-90e0-93a98c0eeaba)

## Workflow:
1. Start the franka driver and moveit interface on robostation:
```bash
roslaunch panda_moveit_config franka_control.launch robot_ip:=172.31.154.43 load_gripper:=True
```
2. Run the grasp exectuor node with our pytohn API
```bash
roslaunch franka_policy grasp_commander.launch
```

## ROS Best Practices, Conventions and Tricks

This package is built with ROS 1.
