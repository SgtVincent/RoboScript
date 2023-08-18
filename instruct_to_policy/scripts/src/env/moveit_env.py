#!/usr/bin/env python3
"""Python interface to execute grasps on the Franka Emika fr3 robot.

This module provides a class to execute grasps on the Franka Emika fr3 robot.
It uses MoveIt! to plan and execute the grasps.
"""
from typing import List, NamedTuple
import numpy as np
import os 
import rospkg

from visualization_msgs.msg import Marker, MarkerArray
from visualization_msgs.msg import Marker


import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker

import trimesh

import actionlib
import numpy as np
from scipy.spatial.transform import Rotation
import franka_gripper.msg
from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryActionGoal
import rospy

# Brings in the SimpleActionClient
import actionlib
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import GetPositionIK


from src.env.utils import get_pose_msg, get_stamped_pose
from src.env.utils import load_model_into_moveit
from src.env.gazebo_env import GazeboEnv


class Grasp(NamedTuple):
    orientation: np.ndarray
    position: np.ndarray
    score: float
    width: float
    instance_id: int


class MoveitGazeboEnv(GazeboEnv):
    """Class to execute grasps on the Franka Emika panda robot."""

    def __init__(self, cfg) -> None:
        """Initialize the GraspExecutor class.

        Args:
            frame: The frame in which the poses are given.
            reset_pose: The pose to which the robot should move to reset the scene.
        """
        super().__init__(cfg)

        rospy.init_node("moveit_gazebo_env")

        # self.frame = frame
        self.moving = False
        self.objects = {}
        self.sim = cfg['env']['sim']
        self.use_sim = len(self.sim) > 0

        self.ignore_coll_check = False
        self.wait_at_grasp_pose = False
        self.wait_at_place_pose = False

        # Service to compute IK
        self.compute_ik = rospy.ServiceProxy("/compute_ik", GetPositionIK)

        # Joint limits for Franka panda Emika. Used to check if IK is at limits.
        self.upper_limit = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
        self.lower_limit = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
        # MoveIt! interface
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("panda_manipulator", wait_for_servers=15)
        self.gripper = moveit_commander.MoveGroupCommander("panda_hand", wait_for_servers=15)
        self.grasp_client = actionlib.SimpleActionClient(
            "/franka_gripper/grasp", franka_gripper.msg.GraspAction
        )
        self.move_client = actionlib.SimpleActionClient(
            "/franka_gripper/move", franka_gripper.msg.MoveAction
        )

        self.error_recovery_client = actionlib.SimpleActionClient(
            "/franka_control/error_recovery", ErrorRecoveryAction
        )

        print("Loading static scene information")
        self.object_names = self.get_obj_names()
        self.load_scene()
        self.group.set_planning_time(15)
        self.group.set_max_velocity_scaling_factor(0.15)
        self.group.set_max_acceleration_scaling_factor(0.15)

        if 'reset_pose' in cfg:
            self.reset_pose = cfg['reset_pose']
        else:
            self.reset_pose = self.group.get_current_pose().pose

        print("Set up Franka API. Ready to go!")

    def _block(fn):
        """Decorator to block the execution of a function if the robot is moving.

        Sets the self.moving variable to True before executing the function and to False after.
        """

        def lock_state(self, *args, **kwargs):
            is_moving = self.moving
            self.moving = True
            ret = fn(self, *args, **kwargs)
            self.moving = False if not is_moving else True
            return ret

        return lock_state

    def _go(self, move_group):
        if not move_group.go(wait=True):
            print("Execution failed! Going to retry with error recovery")
            self.error_recovery_client.send_goal_and_wait(ErrorRecoveryActionGoal())
            return move_group.go(wait=True)
        return True

    def _execute(self, move_group, plan, reset_err=True):
        if not move_group.execute(plan, wait=True):
            if reset_err:
                print("Execution failed!. Going to retry with error recovery")
                self.error_recovery_client.send_goal_and_wait(ErrorRecoveryActionGoal())
                move_group.execute(plan, wait=True)

    def computeIK(
        self,
        orientation,
        position,
        ik_link_name="panda_hand_tcp",
        move_group="panda_manipulator",
    ) -> bool:
        """Check if a given pose is reachable for the robot. Return True if it is, False otherwise."""

        # Create a pose to compute IK for
        pose_stamped = get_stamped_pose(position, orientation, self.frame)

        ik_request = PositionIKRequest()
        ik_request.group_name = move_group
        ik_request.ik_link_name = ik_link_name
        ik_request.pose_stamped = pose_stamped
        ik_request.robot_state = self.robot.get_current_state()
        ik_request.avoid_collisions = True

        request_value = self.compute_ik(ik_request)

        if request_value.error_code.val == -31:
            return False

        if request_value.error_code.val == 1:
            # Check if the IK is at the limits
            joint_positions = np.array(request_value.solution.joint_state.position[:7])
            upper_diff = np.min(np.abs(joint_positions - self.upper_limit))
            lower_diff = np.min(np.abs(joint_positions - self.lower_limit))
            return min(upper_diff, lower_diff) > 0.1
        else:
            return False

    def register_object(
        self, mesh: trimesh.Trimesh, object_id: int, position: List[float] = [0, 0, 0]
    ):
        """Adds a given mesh to the scene and registers it as an object."""

        # File to export the mesh to."wall"
        f = "/tmp/mesh_inst_{}.obj".format(object_id)
        if self.ignore_coll_check:
            position[-1] = 5
        # TODO, Add simplification of mesh here?
        mesh.export(f)

        # Register objects internally.
        self.objects[object_id] = {
            "file": f,
            "active": True,
            "position": position,
        }
        print("Registering mesh for fraem", self.frame)
        self.scene.add_mesh(
            f"inst_{object_id}",
            get_stamped_pose(position, [0, 0, 0, 1], self.frame),
            f,
            size=(1, 1, 1),
        )

    def reset_scene(self):
        """Reset the scene to the initial state."""
        self.scene.clear()
        self.objects = {}
        if self.use_sim:
            self.reset_world()
        self.load_scene()

    def load_gazebo_world_into_moveit(self, 
                                    gazebo_models_dir="",
                                    gazebo_models_filter=["panda", "fr3"],
                                    load_dynamic=False):

        # change the models path accordingly
        if gazebo_models_dir == "":
            gazebo_models_dir = os.path.join(
                rospkg.RosPack().get_path("instruct_to_policy"), "models"
            )

        for model in self.object_names:
            if model not in gazebo_models_filter:
                sdf_path = os.path.join(gazebo_models_dir, model, "model.sdf")
                pose = self.get_model_state(model, "world").pose
                properties = self.get_model_properties(model)
                if properties.is_static or load_dynamic:
                    load_model_into_moveit(sdf_path, pose, self.scene, model, link_name="link")

    def load_scene(self):
        """Load the scene in the MoveIt! planning scene."""
        if self.use_sim:
            self.load_gazebo_world_into_moveit()
            for name in self.object_names:
                self.objects[name] = {}
        else:
            raise NotImplementedError("Only simulation is supported for now.")

        # camera not considered for now
        # cam_pose = get_stamped_pose([0.03, 0, 0.01], [0, 0, 0, 1], "world")
        # self.scene.add_box("cam", cam_pose, size = (0.04, 0.14, 0.02))
        # self.scene.attach_mesh("panda_hand", f"cam", touch_links=[*self.robot.get_link_names(group= "panda_hand"), "panda_joint7"])

    @_block
    def reset(self):
        """Reset the robot to the initial state and opens the gripper."""
        self.open_gripper()
        self.group.set_joint_value_target(self.reset_pose)
        self._go(self.group)
        self.group.stop()
        self.group.clear_pose_targets()
        self.reset_scene()

    def get_ee_pose(self):
        """Get the current pose of the end effector."""
        return self.group.get_current_pose().pose

    @_block
    def open_gripper(self):
        """Open the gripper."""
        goal = franka_gripper.msg.MoveGoal(width=0.039 * 2, speed=1.0)
        self.move_client.send_goal(goal)
        self.move_client.wait_for_result(timeout=rospy.Duration(2.0))

    @_block
    def close_gripper(self, width=0.025, speed=0.25, force=20):
        """Close the gripper."""
        goal = franka_gripper.msg.GraspGoal(width=width, speed=speed, force=force)
        goal.epsilon.inner = 0.03
        goal.epsilon.outer = 0.03
        self.grasp_client.send_goal(goal)
        self.grasp_client.wait_for_result()

    @_block
    def move_to_pose(self, pose: Pose):
        """Move the robot to a given pose with given orientation."""
        self.group.set_pose_target(pose)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        return plan

    @_block
    def move_joints_to(self, joint_values):
        """Move the robot to a given joint configuration."""
        self.group.set_joint_value_target(joint_values)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        return plan

    @_block
    def grasp(
        self,
        pose: Pose,
        width=0.025,
        pre_grasp_approach=0.05,
        dryrun=False,
        object_id=None,
        verbose=True,
    ):
        """Executes a grasp at a given pose with given orientation.

        Args:
            position: The position of the grasp.
            orientation: The orientation of the grasp (scipy format, xyzw).
            width: The width of the gripper.
            pre_grasp_approach: The distance to move towards the object before grasping.
            dryrun: If true, the robot will not call the action to close the gripper (not available in simulation).
            verbose: If true, the robot will print information about the grasp.
        """

        self.group.set_max_velocity_scaling_factor(0.2)
        self.group.set_max_acceleration_scaling_factor(0.2)
        self.group.set_goal_position_tolerance(0.001)
        self.group.set_goal_orientation_tolerance(0.02)
        self.group.set_pose_reference_frame(self.frame)

        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        orientation = np.array(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        )

        pre_grasp_pose = get_pose_msg(
            position
            + Rotation.from_quat(orientation).as_matrix()
            @ (pre_grasp_approach * np.array([0, 0, -1])),
            orientation,
        )
        waypoints = [pre_grasp_pose]

        # make sure gripper is open before moving to pre-grasp pose
        if not dryrun:
            self.open_gripper()

        self.group.set_pose_target(waypoints[0])

        # (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.02, 0.0)
        # if fraction < 1:
        #     print("Could not plan to pre-grasp pose. Plan accuracy", fraction)
        #     return False

        if verbose:
            print("Moving to pre-grasp pose")

        # self._execute(self.group, plan)
        plan = self._go(self.group)
        self.group.stop()
        self.group.clear_pose_targets()
        if not plan:
            print("Failed")
            return False

        if verbose:
            print("Moved to pre grasp. Remmoving object")

        if object_id is not None and object_id in self.objects:
            self.scene.remove_world_object(f"inst_{object_id}")
            self.objects[object_id]["active"] = False

        waypoints = [get_pose_msg(position, orientation)]
        # (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.003, 0.001, True)
        # if fraction < 0.7:
        #     print("Could not plan to pre-grasp pose. Plan Accuracy", fraction)
        #     return False

        if verbose:
            print("Moving to grasp pose")

        self.group.set_pose_target(waypoints[0])
        self.error_recovery_client.send_goal_and_wait(ErrorRecoveryActionGoal())
        self._go(self.group)
        # plan = self.group.g/o(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        if not plan:
            print("Failed!")
            return False

        if self.wait_at_grasp_pose:
            import time

            time.sleep(5)

        if not dryrun:
            if verbose:
                print("Closing gripper")
                self.close_gripper(width=width)

        if object_id is not None and object_id in self.objects:
            touch_links = self.robot.get_link_names(group="panda_hand")
            self.scene.add_mesh(
                f"inst_{object_id}",
                get_stamped_pose(self.objects[object_id]["position"], [0, 0, 0, 1], self.frame),
                self.objects[object_id]["file"],
                size=(1, 1, 1),
            )
            self.scene.attach_mesh("panda_hand", f"inst_{object_id}", touch_links=touch_links)
            if verbose:
                print("attached mesh to ", touch_links)

        return True

    @_block
    def place(self, pose: Pose, width=0.025, lift_height=0.1, dryrun=False, verbose=True):
        """Executes place action at a given pose with given orientation.

        Args:
            position: The position of the place.
            orientation: The orientation of the place (scipy format, xyzw).
            width: The width of the gripper.
            dryrun: If true, the robot will not call the action to open the gripper (not available in simulation).
            verbose: If true, the robot will print information about the place.
        """

        self.group.set_max_velocity_scaling_factor(0.2)
        self.group.set_max_acceleration_scaling_factor(0.2)
        self.group.set_goal_position_tolerance(0.001)
        self.group.set_goal_orientation_tolerance(0.02)
        self.group.set_pose_reference_frame(self.frame)
        self.group.set_pose_target(pose)

        if verbose:
            print("Moving to place pose")

        # self._execute(self.group, plan)
        plan = self._go(self.group)
        self.group.stop()
        self.group.clear_pose_targets()
        if not plan:
            print("Failed")
            return False

        if verbose:
            print("Moved to place. Remmoving object")

        if self.wait_at_place_pose:
            import time

            time.sleep(5)

        if not dryrun:
            if verbose:
                print("Opening gripper")
            self.open_gripper()

        return True
