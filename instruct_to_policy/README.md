# Instruct_to_policy Module
 
## Data pre-process

You can download the processed data for evaluation from [ADD_LINK_HERE](#).


You can also refer to the [data document](./data.md)s more details about the data generation process to process your custom data.

## Code Generation and Evaluation

### Code Generation

We provide a set of pre-defined worlds and tasks for benchmarking. You can find them in the `data/benchmark` directory. 

Under [task_queries](./data/benchmark/task_queries), you can find the task queries for each world. Each task query file has the same name as the corresponding world file.

You can generate the code for each task query with the following command:

```bash
roscd instruct_to_policy
python scripts/data/generate_code.py \
        --task-queries ./data/benchmark/task_queries/world_1_table_sort.txt \
        --output-dir ./data/benchmark/generated_code
```
you can refer to script [generate_code.py](./scripts/data/generate_code.py) to customize your own code generation process.

### Evaluation

With the processed data, you should be able to run benchmark worlds with the following command:

```bash
roslaunch instruct_to_policy run_pandan_moveit_gazebo.launch world:=$(rospack find instruct_to_policy)/data/benchmark/worlds/world_1_table_sort.world
# or roslaunch instruct_to_policy run_ur5_moveit_gazebo.launch world:=$(rospack find instruct_to_policy)/data/benchmark/worlds/world_1_table_sort.world
```
This will launch all gazebo and moveit services/plugins you need for the task.


Also, our environment waits until all required services are ready. e.g. The environment by defaults waits for grasp detection service to be ready. You should launch this service in grasp detection environment. Please refer to the [grasp_detection](../grasp_detection/README.md) package for more details. The case is similar for grounding model and other services dependant on your task. 

With gazebo, moveit and other services ready, you can run the evaluation with the following command:

```bash
roscd instruct_to_policy
rosrun scripts/eval_generated_code.py \
    _world_name:=world_1_table_sort \
    _code_to_eval:=generated_code
```

The evaluation results should be in `data/benchmark/eval_results/` by default. 

## Run

Make sure you have activated the ROS environment and catkin workspace:
```bash
mamba activate <your_ros_env>
cd /path/to/catkin_ws
source devel/setup.bash
```

### TODO: Add the real-time run demo here 



## Troubleshooting

### Gazebo plugins

Our locally-built plugins are in `/path/to/catkin_ws/devel/lib`. To load it in gazebo, you should specify the LD_LIBRARY_PATH to the directory of the plugin. 

```bash
e.g. export LD_LIBRARY_PATH=/path/to/catkin_ws/devel/lib
```

## TODO List

- [x] Try to decouple the ros plugins and the robot urdf model.
