import os
import numpy as np
from typing import List, NamedTuple
import franka_gripper.msg
from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryActionGoal
import rospy

# Brings in the SimpleActionClient
from actionlib import SimpleActionClient
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    GripperCommandAction,
    GripperCommandActionGoal,
)
from sensor_msgs.msg import JointState
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

    # classmethod to get child class instance
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

    def __init__(
        self, gripper_joint_names=["panda_finger_joint1", "panda_finger_joint2"]
    ) -> None:
        
        self._joint_positions = {}
        self._joint_names = gripper_joint_names
        self._joint_velocity = {}
        self._joint_effort = {}
        
        # set ROS parameters related to franka_gripper
        rospy.set_param("/franka_gripper/move/width_tolerance", 0.002)
        rospy.set_param("/franka_gripper/gripper_action/width_tolerance", 0.002)
        # rospy.set_param("/franka_gripper/gripper_action/speed", 0.01)

        self.init_clients()
        print("Gripper action clients ready")

    def init_clients(self):
        
        self._joint_states_state_sub = rospy.Subscriber(
            "/franka_gripper/joint_states", JointState, self._joint_states_callback, queue_size = 1, tcp_nodelay = True)

        
        self.gripper_homing_client = SimpleActionClient(
            "/franka_gripper/homing", franka_gripper.msg.HomingAction
        )

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
            "/franka_gripper/stop", franka_gripper.msg.StopAction
        )

        self.gripper_homing_client.wait_for_server()
        self.gripper_grasp_client.wait_for_server()
        self.gripper_move_client.wait_for_server()
        self.gripper_action_client.wait_for_server()
        self.gripper_stop_client.wait_for_server()

    def _joint_states_callback(self, msg):
        for idx, name in enumerate(msg.name):
            if name in self._joint_names:
                self._joint_positions[name] = msg.position[idx]
                self._joint_velocity[name] = msg.velocity[idx]
                self._joint_effort[name] = msg.effort[idx]

    def _active_cb(self):
        rospy.logdebug("GripperInterface: '{}' request active.".format(self._caller))

    def _feedback_cb(self, msg):
        rospy.logdebug(
            "GripperInterface: '{}' request feedback: \n\t{}".format(self._caller, msg)
        )

    def _done_cb(self, status, result):
        rospy.logdebug(
            "GripperInterface: '{}' complete. Result: \n\t{}".format(
                self._caller, result
            )
        )

    def reset(self):
        self.stop_action()
        # self.gripper_grasp_client.cancel_all_goals()
        self.init_clients()

    def stop_action(self):
        self._caller = "stop_action"

        goal = franka_gripper.msg.StopGoal()

        self.gripper_stop_client.send_goal(
            goal,
            done_cb=self._done_cb,
            active_cb=self._active_cb,
            feedback_cb=self._feedback_cb,
        )

        result = self.gripper_stop_client.wait_for_result(rospy.Duration(15.0))
        return result

    def open_gripper(self, width=0.08, **kwargs):
        cancel_other_goals = kwargs.get("cancel_other_goals", False)
        if cancel_other_goals:
            self.stop_action()

        # FIXME /franka_gripper/move has bugs related to https://github.com/frankaemika/franka_ros/issues/172#issuecomment-1819302834
        # self.gripper_homing_client.send_goal(franka_gripper.msg.HomingGoal())
        # done = self.gripper_homing_client.wait_for_result(rospy.Duration(5.0))

        # goal = franka_gripper.msg.MoveGoal(width=width, speed=0.03)
        # self.gripper_move_client.send_goal(goal)
        # rospy.sleep(3.0)
        # done = self.gripper_move_client.wait_for_result(rospy.Duration(5.0))

        # # Use gripper_action instead
        goal = GripperCommandActionGoal()
        goal.goal.command.position = width / 2.0
        goal.goal.command.max_effort = 100
        
        # FIXME: wait_for_result cannot properly switch threads in the background
        for i in range(5):
            rospy.logdebug(f"FrankaGripperCommanderGroup: Trying to open gripper for trial {i+1}...")
            self._caller = "open_gripper"
            self.gripper_action_client.send_goal(goal.goal, done_cb=self._done_cb, active_cb=self._active_cb, feedback_cb=self._feedback_cb)
            
            rospy.sleep(1.0)
            
            # wierd bug for gripper opening when gripper is closed
            done = self.gripper_action_client.wait_for_result(rospy.Duration(5.0))
            rospy.logdebug(f"FrankaGripperCommanderGroup: open_gripper done when joint positions:{self._joint_positions}")
            
            if self._joint_positions[self._joint_names[0]] + self._joint_positions[self._joint_names[1]] >= 0.035:
                # open success
                break
                        
            # self._caller = "stop_action"
            # self.gripper_stop_client.send_goal(goal.goal, done_cb=self._done_cb, active_cb=self._active_cb, feedback_cb=self._feedback_cb)
            
        return done

    def close_gripper(self, width=0.01, speed=0.03, force=10, **kwargs):
        """Close the gripper."""
        cancel_other_goals = kwargs.get("cancel_other_goals", False)
        if cancel_other_goals:
            self.stop_action()

        goal = franka_gripper.msg.GraspGoal(width=width, speed=speed, force=force)
        # the size of the object is unknown in this API, so we set the inner and outer tolerance to 0.04
        goal.epsilon.inner = 0.08
        goal.epsilon.outer = 0.08
        self._caller = "close_gripper"
        self.gripper_grasp_client.send_goal(goal, done_cb=self._done_cb, active_cb=self._active_cb, feedback_cb=self._feedback_cb)

        rospy.sleep(3.0)
        done = self.gripper_grasp_client.wait_for_result(rospy.Duration(5.0))
        rospy.logdebug(f"FrankaGripperCommanderGroup: close_gripper done when joint positions:{self._joint_positions}")
        return done


class RobotiqGripperCommanderGroup(GripperCommanderGroup):
    def __init__(self) -> None:
        self.init_clients()
        print("Gripper action clients ready")

    def init_clients(self):
        self.action_gripper = SimpleActionClient(
            "/gripper_controller/follow_joint_trajectory", FollowJointTrajectoryAction
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

    def open_gripper(self, width=0.08, **kwargs):
        self.set_gripper(width)

    def close_gripper(self, width=0.001, **kwargs):
        self.set_gripper(width)

    def set_gripper(self, value):
        goal = FollowJointTrajectoryGoal()
        # Create a trajectory point
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = [(0.08 - value) * 10]  # Set the gripper position
        trajectory_point.time_from_start = rospy.Duration(
            3
        )  # Move to the position in 3 seconds

        trajectory = JointTrajectory()
        trajectory.joint_names = [
            "robotiq_85_left_knuckle_joint"
        ]  # This should be the name of your gripper joint
        trajectory.points.append(trajectory_point)
        goal.trajectory = trajectory

        self.action_gripper.send_goal(goal)
        self.action_gripper.wait_for_result(rospy.Duration(10))
        return self.action_gripper.get_result()
