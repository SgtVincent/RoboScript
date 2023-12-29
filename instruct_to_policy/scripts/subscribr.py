import rospy

# Assuming you have a joint named 'joint1'
joint_name = 'panda_joint1'

rospy.init_node('joint_limits_node')

# Get joint limits
joint_limits = rospy.get_param("/robot_description_planning/joint_limits/" + joint_name)

# Print joint limits
print("Joint: ", joint_limits)

