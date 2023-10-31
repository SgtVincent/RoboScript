# This message defines the grasp detection results from the models 
# TODO: consider all other possible information that can be useful for manipulation

string object_id 

# The grasp pose 
geometry_msgs/Pose grasp_pose

# The grasp width
float64 grasp_width 

# The grasp depth 
float64 grasp_depth

# The score of the grasp, which is the probability of the grasp being successful
float64 grasp_score
