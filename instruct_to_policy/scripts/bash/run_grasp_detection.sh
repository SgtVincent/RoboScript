GRASP_DETECTION_WS="/home/junting/anygrasp_ws"
GRASP_DETECTION_CONDA_ENV="anygrasp"

model_name=$1
service_name=$2

# activate grasp detection environment
cd $GRASP_DETECTION_WS
mamba activate $GRASP_DETECTION_CONDA_ENV
source devel/setup.bash

# roslaunch grasp detection node in the background 
roslaunch grasp_detection run_node.launch \
    model_name:=$model_name \
    service_name:=$service_name
