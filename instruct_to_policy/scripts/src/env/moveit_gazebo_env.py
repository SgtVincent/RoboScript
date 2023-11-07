#!/usr/bin/env python3
"""Python interface to execute grasps on the Franka Emika fr3 robot.

This module provides a class to execute grasps on the Franka Emika fr3 robot.
It uses MoveIt! to plan and execute the grasps.
"""
from typing import List, NamedTuple
import numpy as np
import os 
import rospkg


import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped

import trimesh

import numpy as np
from scipy.spatial.transform import Rotation
from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryActionGoal
import rospy

# Brings in the SimpleActionClient
from actionlib import SimpleActionClient, GoalStatus
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK

from src.env.utils import (
    get_pose_msg, 
    get_stamped_pose,
    load_model_into_moveit
)
from src.utils import has_keywords
from src.env.gazebo_env import GazeboEnv
from src.env.moveit_collision_manager import CollisionManager
from src.env.moveit_grippers import GripperCommanderGroup

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
            cfg (dict): Configuration dictionary.
        """
        super().__init__(cfg)

        # self.frame = frame # already initialized in parent class
        self.moving = False
        self.objects = {}

        # load config
        self.sim = cfg['env'].get('sim', "")
        self.verbose = cfg['env'].get('verbose', False)
        self.use_sim = len(self.sim) > 0
        self.config = cfg['env']['moveit_config']
        self.debug = self.config.get('debug', False)
        self.planning_time = self.config.get('planning_time', 15)
        self.max_velocity = self.config.get('max_velocity', 0.2) 
        self.max_acceleration = self.config.get('max_acceleration', 0.2)
        self.goal_position_tolerance = self.config.get('goal_position_tolerance', 0.001)
        self.goal_orientation_tolerance = self.config.get('goal_orientation_tolerance', 0.02)  
        self.reference_frame = self.config.get('reference_frame', self.frame)
        self.cartesian_path = self.config.get('cartesian_path', False)
        self.tentative_approach = self.config.get('tentative_approach', True)

        # group name 
        # franka default
        # self.arm_group_name = self.config.get('arm_group_name', "panda_arm")
        # self.gripper_group_name = self.config.get('gripper_group_name', "panda_hand")
        # self.manipulator_group_name = self.config.get('manipulator_group_name', "panda_manipulator")
        # ur5 default
        self.arm_group_name = self.config.get('arm_group_name', "ur5_arm")
        self.gripper_group_name = self.config.get('gripper_group_name', "gripper")
        self.manipulator_group_name = self.config.get('manipulator_group_name', "ur5_manipulator")


        # environment prior knowledge
        # TODO: consider to parse this from external perception model
        self.static_objects = ['table', 'cabinet']

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
        
        # NOTE: use xxx_manipulator move_group to make grasp center as end effector link 
        self.move_group = moveit_commander.MoveGroupCommander(self.manipulator_group_name, wait_for_servers=15)
        self.move_group_arm = moveit_commander.MoveGroupCommander(self.arm_group_name, wait_for_servers=15)
        # NOTE: use actionlib to control gripper for more detailed control instead of moveit commander 
        # self.move_group_gripper = moveit_commander.MoveGroupCommander(self.gripper_group_name, wait_for_servers=15)
        self.gripper_group = GripperCommanderGroup.get_instance(self.gripper_group_name)
        
        # collision manager to toggle collision between objects
        # used when contrained objects are attached to the robot
        # moveit planning cannot understand translational and rotational joints 
        self.collision_manager = CollisionManager()
        
        self.end_effctor_link = self.move_group.get_end_effector_link()

        # self.error_recovery_client = SimpleActionClient(
        #     "/franka_control/error_recovery", ErrorRecoveryAction
        # )

        print("Loading static scene information")
        self.object_names = self.get_obj_name_list()
        self.load_scene()
        # TODO: moveit configurations should be set inside the config file 

        # Set parameters in move_group
        self.move_group.set_planning_time(self.planning_time)
        self.move_group.set_max_velocity_scaling_factor(self.max_velocity)
        self.move_group.set_max_acceleration_scaling_factor(self.max_acceleration)
        self.move_group.set_goal_position_tolerance(self.goal_position_tolerance)
        self.move_group.set_goal_orientation_tolerance(self.goal_orientation_tolerance)
        self.move_group.set_pose_reference_frame(self.reference_frame)    

        self.reset_joint_values = cfg['env']['initial_joint_values']
        
        # disable collision between gripper and other objects
        # NOTE: currently disable collision in srdf setting file to accelerate system booting 
        # gripper_collision_frames = ["panda_rightfinger", "panda_leftfinger"]
        # self.collision_manager.disable_collisions(gripper_collision_frames)
        # rospy.loginfo(f"Moveit: frames collision turned off: {gripper_collision_frames}")

        # setup debug visualization
        if self.debug:
            ee_name = self.end_effctor_link.replace("/", "")
            self.marker_pub = rospy.Publisher(f"/rviz/moveit/move_marker/goal_{ee_name}", PoseStamped, queue_size=5)
            self.start_state_pub = rospy.Publisher(f"/rviz/moveit/update_custom_start_state", RobotState, queue_size=1)
            self.goal_state_pub = rospy.Publisher(f"/rviz/moveit/update_custom_goal_state", RobotState, queue_size=1)

            
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
            # rospy.logwarn("Execution failed! Going to retry with error recovery")
            # self.error_recovery_client.send_goal_and_wait(ErrorRecoveryActionGoal())
            # return move_group.go(wait=True)
            return False
        return True

    def _execute(self, move_group, plan, reset_err=True):
        if not move_group.execute(plan, wait=True):
            # if reset_err:
            #     rospy.logwarn("Execution failed!. Going to retry with error recovery")
            #     self.error_recovery_client.send_goal_and_wait(ErrorRecoveryActionGoal())
            #     return move_group.execute(plan, wait=True)
            return False
        return True

    def computeIK(
        self,
        orientation,
        position,
        ik_link_name=None,
        move_group=None,
    ) -> bool:
        """Check if a given pose is reachable for the robot. Return True if it is, False otherwise."""

        # Create a pose to compute IK for
        pose_stamped = get_stamped_pose(position, orientation, self.frame)

        ik_request = PositionIKRequest()
        ik_request.group_name = move_group if move_group is not None else self.manipulator_group_name
        ik_request.ik_link_name = ik_link_name if ik_link_name is not None else self.end_effctor_link
        ik_request.pose_stamped = pose_stamped
        ik_request.robot_state = self.robot.get_current_state()
        ik_request.avoid_collisions = False

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
        self.planning_scene.add_mesh(
            f"inst_{object_id}",
            get_stamped_pose(position, [0, 0, 0, 1], self.frame),
            f,
            size=(1, 1, 1),
        )

    def reset_scene(self):
        """Reset the scene to the initial state."""
        self.objects = {}
        if self.use_sim:
            self.reset_simulation()
            self.reset_world()
        
        # reset planning scene 
        # remove all attached objects
        self.planning_scene.remove_attached_object()
        
        # reset visualization marker if debug is enabledss
        if self.debug:              
            self.start_state_pub.publish(self.robot.get_current_state())  
            self.goal_state_pub.publish(self.robot.get_current_state())
            rospy.sleep(1)
        
        self.load_scene()

    def load_gazebo_world_into_moveit(self, 
                                    gazebo_models_dir="",
                                    gazebo_models_filter=["panda", "fr3"],
                                    load_dynamic=True):
        """
        TODO: Deprecated. A gazebo plugin will publish the models in the world into moveit planning scene
        """
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
                    load_model_into_moveit(sdf_path, pose, self.planning_scene, model, link_name="link")

        
    def load_scene(self):
        """Load the scene in the MoveIt! planning scene."""
        if self.use_sim:

            for name in self.object_names:
                self.objects[name] = {}
                    
        else:
            raise NotImplementedError("Only simulation is supported for now.")

    def publish_goal_to_marker(self, goal_pose: Pose):
        """Publish the current goal to the interactive marker for debug visualization."""
        if not self.debug:
            return
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.reference_frame
        pose_stamped.pose = goal_pose
        self.marker_pub.publish(pose_stamped)
        # wait for rviz to update
        rospy.logdebug("Waiting for rviz to update")
        rospy.sleep(1.0)
        
    @_block
    def reset(self, group=None, gripper_group=None):
        """Reset the robot to the initial state and opens the gripper."""
        if group is None:
            group = self.move_group
        if gripper_group is None:
            gripper_group = self.gripper_group
            
        # stop gripper command to stop ongoing action
        gripper_group.reset()
        self.open_gripper(gripper_group)
        
        # reset robot arm pose 
        group.set_joint_value_target(self.reset_joint_values)
        self._go(group)
        group.stop()
        group.clear_pose_targets()
        
        # reset the world and moveit planning scene 
        self.reset_scene()    
        rospy.loginfo("Environment reset.")

    def get_gripper_pose(self, group=None):
        """Get the current pose of the gripper."""
        if group is None:
            group = self.move_group
        return group.get_current_pose().pose

    @_block
    def open_gripper(self, gripper_group=None, width=0.08):
        """Open the gripper."""
        if gripper_group is None:
            gripper_group = self.gripper_group
        return gripper_group.open_gripper(width=width)


    @_block
    def close_gripper(self, gripper_group=None, width=0.01, force=100):
        """Close the gripper."""
        if gripper_group is None:
            gripper_group = self.gripper_group
        return gripper_group.close_gripper(width=width, force=force)


    @_block
    def move_to_pose(self, pose: Pose, group:moveit_commander.MoveGroupCommander=None):
        """Move the robot to a given pose with given orientation."""
        # TODO: add recovery behavior to decrease gripper width and try plan again
        
        if group is None:
            group = self.move_group
            
        # publish debug goal pose to rviz
        if self.debug:
            self.publish_goal_to_marker(pose)
            
        plan_success = False
        # first try to plan with cartesian path if enabled
        if self.cartesian_path:
            (plan, fraction) = group.compute_cartesian_path([pose], 0.02, 0.0)
            if fraction > 0.9:
                plan_success = True
            else:
                rospy.logwarn(f"MoveitEnv: Could not plan cartesian_path to target pose \n{pose}.\n Plan accuracy: {fraction}")


        # if cartesian path is disabled or failed, try to replan non-cartesian path
        if not self.cartesian_path or not plan_success:
            group.set_pose_target(pose)
            # API change for move_group.plan()
            # return: (success flag : boolean, trajectory message : RobotTrajectory, planning time : float, error code : MoveitErrorCodes)
            plan_success, plan, _, _ = group.plan()
            if not plan_success:
                rospy.logwarn(f"MoveitEnv: Could not plan to target pose \n{pose}")
                 
        if not plan_success:
            return False
        
        success = self._execute(group, plan)
        group.stop()
        group.clear_pose_targets()
        return success

    @_block
    def move_joints_to(self, joint_values, group=None):
        """Move the robot to a given joint configuration."""
        if group is None:
            group = self.move_group
        group.set_joint_value_target(joint_values)
        plan = self._go(group)
        group.stop()
        group.clear_pose_targets()
        return plan

    @_block
    def add_object_to_scene(self, object_id):
        """Add an object to the moveit planning scene."""
        if object_id is not None and object_id in self.objects:
            pass

            # currently do noting 
            # collision_dict = self.get_object_collision(object_id)
            # TODO: finish this part after the collision detection is done

            # touch_links = self.robot.get_link_names(group="panda_hand")
            # self.scene.add_mesh(
            #     object_id,
            #     get_stamped_pose(self.objects[object_id]["position"], [0, 0, 0, 1], self.frame),
            #     self.objects[object_id]["file"],
            #     size=(1, 1, 1),
            # )
        else:
            if self.verbose:
                print(f"Moveit: object {object_id} not found in environment.")

    @_block
    def attach_object(self, object_id, link=None):
        """Attach an object to the robot gripper"""

        if link is None:
            link = self.end_effctor_link

        # DO NOT add furniture into moveit planning scene since they are static
        if object_id in self.objects and not has_keywords(object_id, self.static_objects):
            
            # NOTE: since the object name in moveit planning scene is different from the object name in gazebo with Gazebo plugin
            # we need to convert the object name to the name in moveit planning scene
            # e.g. 'apple' in gazebo -> 'apple.link' in moveit planning scene
            moveit_object_names = self.planning_scene.get_known_object_names()
            
            for moveit_object_name in moveit_object_names:
                if object_id in moveit_object_name:
                    self.objects[object_id]["attach_name"] = moveit_object_name
                    self.move_group.attach_object(moveit_object_name, link) 
                    if self.verbose:
                        rospy.loginfo(f"Moveit: attached object object {object_id} to {link}")
                    return True
            
            rospy.logerr(f"Moveit: object {object_id} not found in moveit planning scene. Planning scene objects: {moveit_object_names}")
            return False
        return False


    @_block
    def detach_object(self, object_id):
        """Detach an object from the robot."""
        try:
            moveit_object_name = self.objects[object_id]["attach_name"]
            self.move_group.detach_object(moveit_object_name)
            
            if self.verbose:
                print(f"Moveit: detached object {object_id}")
        except:
            if self.verbose:
                rospy.logerr(f"Moveit: failed to detach object {object_id}")


    @_block
    def grasp(
        self,
        pose: Pose,
        pre_grasp_approach=0.1,
        depth=0.03,
        tentative_depth=[0.03, 0.01, -0.01],
    ):
        """Executes a grasp at a given pose with given orientation.
        It first moves to a pre-grasp pose, then approaches the grasp pose.

        Args:
            pose: The pose of the grasp.
            pre_grasp_approach: The distance to move towards the object before grasping.
            depth: The distance to move towards the object after grasping.
            
        NOTE: for anygrasp and the offset between gripper tip and grasp center is 0.02, depth = 0.05 - 0.02 = 0.03
        # TODO: should add pre_grasp_approach and depth to the robot-specifig config file
        """

        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        orientation = np.array(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        )

        # calculate pre-grasp pose 
        pre_grasp_pose = get_pose_msg(
            position
            + Rotation.from_quat(orientation).as_matrix() @ (pre_grasp_approach * np.array([0, 0, -1])),
            orientation,
        )

        success = self.move_to_pose(pre_grasp_pose)
        if not success:
            rospy.logwarn("MoveitEnv: Failed to move to pre-grasp pose")
            return False
        
        if self.verbose:
            rospy.loginfo("MoveitEnv: Moved to pre-grasp pose")
        
        if not self.tentative_approach:
            # calculate approach pose
            approach_pose = get_pose_msg(
                position
                + Rotation.from_quat(orientation).as_matrix() @ (depth * np.array([0, 0, 1])),
                orientation,
            )
            success = self.move_to_pose(approach_pose)
        
            if not success:
                rospy.logwarn("MoveitEnv: Failed to approach to grasp pose")
                return False
        else:
            # calculate tentative approach pose
            for tentative_depth in tentative_depth:
                tentative_approach_pose = get_pose_msg(
                    position
                    + Rotation.from_quat(orientation).as_matrix() @ (tentative_depth * np.array([0, 0, 1])),
                    orientation,
                )
                success = self.move_to_pose(tentative_approach_pose)
                if success:
                    break
            if not success:
                rospy.logwarn("MoveitEnv: Failed to approach to grasp pose")
                return False
            
        
        if self.verbose:
            rospy.loginfo("MoveitEnv: Approached to grasp pose")

        if self.wait_at_grasp_pose:
            import time
            time.sleep(5)
        
        return True

    @_block
    def place(self, pose: Pose, width=0.025, lift_height=0.1, dryrun=False):
        """Executes place action at a given pose with given orientation.

        Args:
            position: The position of the place.
            orientation: The orientation of the place (scipy format, xyzw).
            width: The width of the gripper.
            dryrun: If true, the robot will not call the action to open the gripper (not available in simulation).
            verbose: If true, the robot will print information about the place.
        """

        self.move_group.set_pose_target(pose)

        if self.verbose:
            print("Moving to place pose")

        # self._execute(self.move_group, plan)
        plan = self._go(self.move_group)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        if not plan:
            print("Failed")
            return False

        if self.verbose:
            print("Moved to place. Remmoving object")

        if self.wait_at_place_pose:
            import time

            time.sleep(5)

        if not dryrun:
            if self.verbose:
                print("Opening gripper")
            self.open_gripper()

        return True
