#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
import actionlib
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Workaround to use the gripper action client with unified gripper interface
class GripperCommanderGroup:

    def __init__(self) -> None:    

        self.init_clients()
        print("Gripper action clients ready")


    def init_clients(self):
        self.action_gripper = actionlib.SimpleActionClient(
        '/gripper_controller/follow_joint_trajectory',
        control_msgs.msg.FollowJointTrajectoryAction
        )
        print("Waiting for action of gripper controller")
        _ans = self.action_gripper.wait_for_server(rospy.Duration(5))
        if _ans:
            rospy.loginfo("Action server started")
        elif not _ans:
            rospy.loginfo("Action server not started") 

    def open_gripper(self, value=0.08):
        self.set_gripper(value)

    def close_gripper(self, value=0.0):
        self.set_gripper(value)

    def set_gripper(self, value):
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        # Create a trajectory point
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = [(0.08 - value)*10]  # Set the gripper position
        trajectory_point.time_from_start = rospy.Duration(3)  # Move to the position in 3 seconds

        trajectory = JointTrajectory()
        trajectory.joint_names = ["robotiq_85_left_knuckle_joint"]  # This should be the name of your gripper joint
        trajectory.points.append(trajectory_point)
        goal.trajectory = trajectory

        self.action_gripper.send_goal(goal)
        self.action_gripper.wait_for_result(rospy.Duration(10))
        return self.action_gripper.get_result()

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur5_moveit_script', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name_arm = "ur5_arm"  # 修改为你的机械臂组名
    group_name_gripper = "gripper"  # 修改为你的抓手组名
    group_name_manipulator = "ur5_manipulator"  # 修改为你的tcp组名

    move_group_arm = moveit_commander.MoveGroupCommander(group_name_arm)
    move_group_gripper = moveit_commander.MoveGroupCommander(group_name_gripper)
    move_group_manipulator = moveit_commander.MoveGroupCommander(group_name_manipulator)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)


    rospy.loginfo("Planning frame: %s" % move_group_manipulator.get_planning_frame())
    rospy.loginfo("End effector link: %s" % move_group_manipulator.get_end_effector_link())
    rospy.loginfo("Robot Groups: %s" % robot.get_group_names())
    rospy.loginfo("Robot State:")
    rospy.loginfo(robot.get_current_state())

    GripperCommander = GripperCommanderGroup()
    # Planning to a Pose goal
    pose_goal = Pose()
    pose_goal.position.x = 0.0
    pose_goal.position.y = 0.3
    pose_goal.position.z = 0.6
    pose_goal.orientation.w = 1.0
    pose_goal.orientation.x = 0
    pose_goal.orientation.y = 0
    pose_goal.orientation.z = 0

    move_group_manipulator.set_pose_target(pose_goal)

    rospy.loginfo("Planning to move to pose goal")
    plan = move_group_manipulator.go(wait=True)
    move_group_manipulator.stop()
    move_group_manipulator.clear_pose_targets()

    if plan:
        rospy.loginfo("Motion to pose goal succeeded!")
    else:
        rospy.loginfo("Motion to pose goal failed.")

    # Moving the gripper
    rospy.loginfo("Closing the gripper")
    GripperCommander.close_gripper()
    rospy.loginfo("Opening the gripper")
    GripperCommander.open_gripper()
    # Moving the gripper
    rospy.loginfo("Closing the gripper")
    GripperCommander.close_gripper()
    rospy.loginfo("Opening the gripper")
    GripperCommander.open_gripper()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
