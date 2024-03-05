# This bash file is used to evaluate code genearted under the ground truth grounding problem setting.

# ROS Installation workspace path
CATKIN_WS="/home/junting/catkin_ws"
CATKIN_CONDA_ENV="catkin_ws"

# activate catkin workspace
cd $CATKIN_WS
mamba activate $CATKIN_CONDA_ENV
source devel/setup.bash

# Define script arguments
robot_name="panda"
package_root="$(rospack find instruct_to_policy)"
benchmark_data_dir="$(rospack find instruct_to_policy)/data/benchmark"
verbose=false
use_gt_planning_scene=false

models_names=(
    "panda_toy_figure"
    "Nintendo_Mario_toy_figure"
    "wood_block"
    "grey_medication_bottle"
    "white_and_brown_box"
    "brown_medication_bottle"
    "blue_medication_bottle"
    "brown_ceramic_cup"
    "toy_bus"
    "dog_figure"
)

# function to kill all descendant processes of a given pid
kill_descendant_processes() {
    local pid="$1"
    local and_self="${2:-false}"
    if children="$(pgrep -P "$pid")"; then
        for child in $children; do
            kill_descendant_processes "$child" true
        done
    fi
    if [[ "$and_self" == true ]]; then
        kill -9 "$pid"
    fi
}

# function to evaluate code in one benchmark world
function eval_model_in_world() {
    local model_name=$1
    local robot_name=$2
    local verbose=$3

    # launch_grasp_detection_node 
    # the script is in the same directory as this script
    gnome-terminal -- bash -c "source $(rospack find instruct_to_policy)/scripts/bash/run_grasp_detection.sh; exec bash"

    # eval all configs 
    echo "-----------------Evaluating $model_name -------------------------------------"
    rosrun instruct_to_policy eval_model_pick_and_place.py \
        _config_to_eval:=perception_few_shot_gpt_4 \
        _model_name:=$model_name \
        _use_gt_planning_scene:=$use_gt_planning_scene
    echo "-----------------Evaluating $model_name End ....--------------------------------------------------------"

    # kill all process in this function 


}

roscore & 
killall -9 python
killall -9 rviz 
killall -9 gzserver

# roslaunch gazebo and moveit
roslaunch instruct_to_policy run_${robot_name}_moveit_gazebo.launch \
    --wait \
    world:=$package_root/worlds/table_cabinet_plate.world \
    moveit_rviz:=true \
    gazebo_gui:=true \
    use_gt_planning_scene:=$use_gt_planning_scene \
    enable_moveit_sensor_plugins:=$(if $use_gt_planning_scene; then echo "false"; else echo "true"; fi) \
    publish_pointcloud:=$(if $use_gt_planning_scene; then echo "false"; else echo "true"; fi) \
    verbose:=$verbose &

# Wait for the Gazebo world to load
sleep 15

# iterate through all the worlds under data/benchmark/worlds
for model_name in "${models_names[@]}"
do
    
    sleep 5

    # evaluate code of all configs in the world
    # also save all logs to a log file
    eval_model_in_world $model_name $robot_name $verbose 2>&1 | tee -a eval_pick_and_place_perception.log

done

# kill all descendant processes of this script
kill_descendant_processes $$

