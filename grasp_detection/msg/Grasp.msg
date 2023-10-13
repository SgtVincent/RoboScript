# This message defines the grasp detection results from the models 
# TODO: consider all other possible information that can be useful for manipulation

geometry_msgs/Pose grasp_pose

string object_id 

# The score of the grasp, which is the probability of the grasp being successful
float64 grasp_score
