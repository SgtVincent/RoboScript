# Python 2/3 compatibility imports
from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import copy
from geometry_msgs.msg import Quaternion, Pose, Point


try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

# planning_frame = move_group.get_planning_frame()
# print("============ Planning frame: %s" % planning_frame)

# # We can also print the name of the end-effector link for this group:
# eef_link = move_group.get_end_effector_link()
# print("============ End effector link: %s" % eef_link)

# # We can get a list of all the groups in the robot:
# group_names = robot.get_group_names()
# print("============ Available Planning Groups:", robot.get_group_names())

# # Sometimes for debugging it is useful to print the entire state of the
# # robot:
# print("============ Printing robot state")
print(robot.get_current_state())
# print("")
# pose_goal = [0.3005312874609964, -0.70001046936977, 0.26720704058596956, -2.534149039720234, 0.7999030763639348, 3.058523871024449, 0.17747668076324996]
print(move_group.get_current_pose().pose)

# pose_goal = move_group.get_current_pose().pose
# print(pose_goal)

# pre_defined_horizontal_orientation = Quaternion(0.6832535941073715,0.3440460910674551,0.610755296083018,0.20438880024871378) # for franka hand 
# pose_goal.orientation = pre_defined_horizontal_orientation 
# pose_goal.position.x = 0.4
# pose_goal.position.y = 0
# pose_goal.position.z = 0.6
# pose_goal.orientation.x= -1
# pose_goal.orientation.y= 0.5
# pose_goal.orientation.z= 0.5
# pose_goal.orientation.w= 0.5
# print(pose_goal.orientation)
# pose_goal = [-0.001209562553452295, -0.7798870970324466, -0.0025529672049247384, -2.3749749452691327, -0.0031800321414839528, 1.5748067867097109, 0.7786260150587473]
# move_group.set_pose_target(pose_goal)
# `go()` returns a boolean indicating whether the planning and execution was successful.
# success = move_group.go(wait=True)
# Calling `stop()` ensures that there is no residual movement
# move_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets().
# move_group.clear_pose_targets()

# box_pose = geometry_msgs.msg.PoseStamped()
# box_pose.header.frame_id = "base_link"
# box_pose.pose.orientation.w = 1.0
# box_pose.pose.position.x = -0.4  # above the panda_hand frame
# box_pose.pose.position.z = 1  # above the panda_hand frame

# box_name = "box1"
# scene.add_box(box_name, box_pose, size=(0.1, 0.5, 2))

# rospy.sleep(2)

# box_pose.pose.orientation.w = 1.0
# box_pose.pose.position.x = -0.35  # above the panda_hand frame
# box_pose.pose.position.y = -0.7  # above the panda_hand frame
# box_pose.pose.position.z = 0.2  # above the panda_hand frame

# box_name = "box2"
# scene.add_box(box_name, box_pose, size=(0.9, 0.4, 0.4))
# rospy.sleep(2)
# print("______________done__________")
