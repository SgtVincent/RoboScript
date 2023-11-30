
## Jupyter-ROS
[Jupyter-ROS](https://github.com/RoboStack/jupyter-ros) is a set of ROS tools to run in jupyter notebook
with interactive command & visualization. It is not required for the pipeline, but it is useful for debugging and visualization with jupyter. 

```bash
# install dependencies inside the mamba environment
mamba install jupyter bqplot pyyaml ipywidgets ipycanvas
# install jupyter-ros
mamba install jupyter-ros -c robostack
```
Then you need to create a ros kernel, which has loaded catkin_ws environments, from the official [instructions](https://jupyter-ros.readthedocs.io/en/latest/user_troubleshooting.html).


## Test with moveit_tutorials (Optional)

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


