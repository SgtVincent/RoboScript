GRASP_DETECTION_WS=${1:-"/home/junting/anygrasp_ws"}
GRASP_DETECTION_CONDA_ENV=${2:-"anygrasp"}

model_name=anygrasp
service_name=/detect_grasps

# activate grasp detection environment
cd $GRASP_DETECTION_WS
mamba activate $GRASP_DETECTION_CONDA_ENV
source devel/setup.bash

rosnode kill /grasp_detection_node

sleep 3

# roslaunch grasp detection node in the background 
roslaunch grasp_detection run_node.launch \
    model_name:=$model_name \
    service_name:=$service_name
