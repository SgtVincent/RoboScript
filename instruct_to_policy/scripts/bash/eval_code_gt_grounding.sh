# This bash file is used to evaluate code genearted under the ground truth grounding problem setting.

# ROS Installation workspace path
CATKIN_WS="/home/junting/catkin_ws"
CATKIN_CONDA_ENV="catkin_ws"

# activate catkin workspace
cd $CATKIN_WS
mamba activate $CATKIN_CONDA_ENV
source devel/setup.bash

# Receive named arguments from the command line.
robot_name="panda"
benchmark_data_dir="$(rospack find instruct_to_policy)/data/benchmark"
code_to_eval="generated_code"
verbose=false

while [ "$#" -gt 0 ]; do
    case "$1" in
        -n|--robot_name)
            robot_name="$2"
            shift 2
            ;;
        -d|--benchmark_data_dir)
            benchmark_data_dir="$2"
            shift 2
            ;;
        -c|--code_to_eval)
            code_to_eval="$2"
            shift 2
            ;;
        -v|--verbose)
            verbose=true
            shift 1
            ;;
        *)
            echo "Unknown option: $1"
            return 1
            ;;
    esac
done

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
    local code_to_eval=$3
    local verbose=$4

    # launch_grasp_detection_node $robot_name grasp_detection

    # roslaunch gazebo and moveit
    roslaunch instruct_to_policy run_${robot_name}_moveit_gazebo.launch \
        world:=$world_name \
        moveit_rviz:=false \
        use_gt_planning_scene:=true \
        verbose:=$verbose &

    # Wait for the Gazebo world to load
    sleep 10

    echo "Evaluating code in world: $world_name"
    rosrun instruct_to_policy eval_generated_code.py \
        _world_name:=$world_name \
        _code_to_eval:=$code_to_eval
}

# run grasp detection node in another terminal with script run_grasp_detection.sh
# the script is in the same directory as this script
gnome-terminal -- bash -c "source $(rospack find instruct_to_policy)/scripts/bash/run_grasp_detection.sh; exec bash"

# iterate through all the worlds under data/benchmark/worlds
for world_file in $(ls $benchmark_data_dir/worlds); do
    # get world name and full path to world
    world_name="${world_file%.*}"
    world_path=$(realpath $benchmark_data_dir/worlds/$world_file)
    
    # skip if benchmark world does not have eval_items file 
    if [ ! -f "$benchmark_data_dir/eval_items/${world_name}_eval_items.json" ]; then
        echo "Skipping world $world_name because it does not have ground truth annotations in eval_items dir."
        continue
    fi 

    # evaluate code in the world
    eval_code_in_world $world_path $robot_name $code_to_eval $verbose

    # kill all descendant processes of this script
    kill_descendant_processes $$
done


