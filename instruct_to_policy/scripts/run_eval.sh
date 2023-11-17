#!/bin/bash

# Define an array of ROS parameters
declare -a params=("generated_code_gpt3_few_shot_grasp_preference" "generated_code_gpt4_few_shot_grasp_preference")
# world_name="world_2_pick_and_place"
world_name="world_3_mug_to_empty_plate"

# Iterate over each parameter
for param in "${params[@]}"
do
   echo "Running with parameter: $param"
   
   # Set the ROS parameter
   rosparam set /eval_code/code_to_eval "$param"
   rosparam set /eval_code/world_name "$world_name"
   
   # Run the command
   rosrun instruct_to_policy eval_generated_code.py
done