# This bash file is provide an example of how to evaluate generated code on RoboScript benchmark worlds.
# PLEASE NOTE that currently the project is not well cleaned up.
# There will be ~30 seconds delay for all differnet packages to be loaded and initialized. Please wait patiently.

################### SET YOUR PARAMETERS HERE according to local ROS config ###################

# ROS Installation workspace path
CATKIN_WS="/home/junting/catkin_ws"
CATKIN_CONDA_ENV="ros_env"

# Grass Detection workspace path
GRASP_DETECTION_WS="/home/junting/anygrasp_ws"
GRASP_DETECTION_CONDA_ENV="anygrasp"

# activate catkin workspace
cd $CATKIN_WS
mamba activate $CATKIN_CONDA_ENV
source devel/setup.bash

# Define script arguments
robot_name="panda"
benchmark_data_dir="$(rospack find instruct_to_policy)/data/benchmark"
verbose=false

# Specify the configs to evaluate
config_name=text_gpt_4
repeat_times=1

# Specify the worlds to evaluate, there are 6 worlds in total
world_name="world_2_pick_and_place"
# (
#     "world_1_table_sort"
#     "world_2_pick_and_place"
#     "world_3_mug_to_empty_plate"
#     "world_4_clear_table_to_basket"
#     "world_5_mug_to_cabinet"
#     "world_6_mug_to_same_color_plate"
# )

##################################################################################################

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
function eval_code_in_world() {
    local world_name=$1
    local robot_name=$2
    local verbose=$3

    # launch_grasp_detection_node $robot_name grasp_detection

    # get world name and full path to world
    world_path=$(realpath ${benchmark_data_dir}/worlds/${world_name}.world)

    # roslaunch gazebo and moveit
    roslaunch instruct_to_policy run_${robot_name}_moveit_gazebo.launch \
        --wait \
        world:=$world_path \
        moveit_rviz:=false \
        gazebo_gui:=true \
        use_gt_planning_scene:=true \
        enable_moveit_sensor_plugins:=false \
        verbose:=$verbose &

    # Wait for the Gazebo world to load
    sleep 10

    # run grasp detection node in another terminal with script run_grasp_detection.sh
    gnome-terminal -- bash -c "source $(rospack find instruct_to_policy)/scripts/bash/run_grasp_detection.sh $GRASP_DETECTION_WS $GRASP_DETECTION_CONDA_ENV; exec bash"

    # Eval the config
    echo "-----------------Evaluating $config_name in world $world_name -------------------------------------"
    rosrun instruct_to_policy eval_generated_code.py \
        _world_name:=$world_name \
        _config_to_eval:=$config_name \
        _repeat_times:=$repeat_times
    echo "---------------------------------------------------------------------------------------------"

}

roscore &

# clean up python processes to free model memory
kikkall -9 gzserver
killall -9 gzclient
killall -9 python

# skip if benchmark world does not have eval_items file
if [ ! -f "$benchmark_data_dir/eval_items/${world_name}_eval_items.json" ]; then
    echo "Skipping world $world_name because it does not have ground truth annotations in eval_items dir."
    continue
fi

# evaluate code of all configs in the world
eval_code_in_world $world_name $robot_name $verbose 2>&1 | tee -a "eval_code_text_${world_name}.log"

# kill all descendant processes of this script
kill_descendant_processes $$
