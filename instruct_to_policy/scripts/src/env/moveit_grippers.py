import os 
import numpy as np
from typing import List, NamedTuple
import franka_gripper.msg
from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryActionGoal
import rospy

# Brings in the SimpleActionClient
from actionlib import SimpleActionClient
from control_msgs.msg import (
    FollowJointTrajectoryAction, FollowJointTrajectoryGoal,
    GripperCommandAction, GripperCommandActionGoal
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK


class Grasp(NamedTuple):
    orientation: np.ndarray
    position: np.ndarray
    score: float
    width: float
    instance_id: int

class GripperCommanderGroup:
    """
    Base class for gripper commander groups.
    """
    
    def __init__(self) -> None:
        pass
    
    def open_gripper(self, width: float, speed: float, force: float):
        raise NotImplementedError
    
    def close_gripper(self, width: float, speed: float, force: float):
        raise NotImplementedError
    
    #classmethod to get child class instance
    @classmethod
    def get_instance(cls, gripper_name: str):
        # TODO: rename robotiq gripper move group to more specific name
        if gripper_name == "panda_hand":
            return FrankaGripperCommanderGroup()
        elif gripper_name == "gripper":
            return RobotiqGripperCommanderGroup()
        else:
            raise ValueError(f"Unknown gripper type: {gripper_name}")
    

class FrankaGripperCommanderGroup(GripperCommanderGroup):
    """
    Interface to the Franka gripper action server.
    """

    def __init__(self) -> None:    

        # set ROS parameters related to franka_gripper
        rospy.set_param("/franka_gripper/move/width_tolerance", 0.002)
            
        self.init_clients()
        print("Gripper action clients ready")


    def init_clients(self):
        self.gripper_grasp_client = SimpleActionClient(
            "/franka_gripper/grasp", franka_gripper.msg.GraspAction
        )       

        self.gripper_move_client = SimpleActionClient(
            "/franka_gripper/move", franka_gripper.msg.MoveAction
        )

        self.gripper_action_client = SimpleActionClient(
            "/franka_gripper/gripper_action", GripperCommandAction
        )

        self.gripper_stop_client = SimpleActionClient(
            "/franka_gripper/stop", franka_gripper.msg.StopAction)
        
        self.gripper_grasp_client.wait_for_server()
        self.gripper_move_client.wait_for_server()
        self.gripper_action_client.wait_for_server()
        self.gripper_stop_client.wait_for_server()


    def reset(self):
        self.gripper_stop_client.send_goal_and_wait(franka_gripper.msg.StopGoal())
        # self.gripper_grasp_client.cancel_all_goals()
        self.init_clients()


    def open_gripper(self, width=0.08, **kwargs):
        
        cancel_other_goals = kwargs.get("cancel_other_goals", False)
        if cancel_other_goals:
            self.gripper_stop_client.send_goal_and_wait(franka_gripper.msg.StopGoal())
            
        #FIXME /franka_gripper/move has bugs related to https://github.com/frankaemika/franka_ros/issues/172#issuecomment-1819302834
        # goal = franka_gripper.msg.MoveGoal(width=width, speed=0.01)
        # self.gripper_move_client.send_goal(goal)
        # Use gripper_action instead
        goal = GripperCommandActionGoal()
        goal.goal.command.position = width / 2.0
        goal.goal.command.max_effort = 10
        self.gripper_action_client.send_goal(goal.goal)
        
        
        # Manually set 5 seconds sleep for goal sync 
        rospy.sleep(3.0)
        
        done = self.gripper_action_client.wait_for_result(rospy.Duration(5.0))
        return done

    def close_gripper(self, width=0.01, speed=0.01, force=10, **kwargs):
        """Close the gripper."""
        cancel_other_goals = kwargs.get("cancel_other_goals", False)
        if cancel_other_goals:
            self.gripper_stop_client.send_goal_and_wait(franka_gripper.msg.StopGoal())
        
        goal = franka_gripper.msg.GraspGoal(width=width, speed=speed, force=force)
        goal.epsilon.inner = 0.08
        goal.epsilon.outer = 0.08
        self.gripper_grasp_client.send_goal(goal)
        
        # Manually set 5 seconds sleep for goal sync 
        rospy.sleep(3.0)
        
        done = self.gripper_grasp_client.wait_for_result(rospy.Duration(5.0))
        return done
    
    
class RobotiqGripperCommanderGroup(GripperCommanderGroup):

    def __init__(self) -> None:    

        self.init_clients()
        print("Gripper action clients ready")

    def init_clients(self):
        self.action_gripper = SimpleActionClient(
        '/gripper_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        print("Waiting for action of gripper controller")
        _ans = self.action_gripper.wait_for_server(rospy.Duration(5))
        if _ans:
            rospy.loginfo("Action server started")
        elif not _ans:
            rospy.loginfo("Action server not started") 

    def reset(self):
        self.action_gripper.cancel_all_goals()
        self.init_clients()

    def open_gripper(self, value=0.08, **kwargs):
        self.set_gripper(value)

    def close_gripper(self, value=0.0, **kwargs):
        self.set_gripper(value)

    def set_gripper(self, value):
        goal = FollowJointTrajectoryGoal()
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