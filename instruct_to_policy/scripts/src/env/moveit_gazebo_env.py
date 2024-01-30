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
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
import open3d as o3d 
import numpy as np
from scipy.spatial.transform import Rotation
from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryActionGoal
import rospy
import tf2_ros
# Brings in the SimpleActionClient
from actionlib import SimpleActionClient, GoalStatus
from std_srvs.srv import Empty, EmptyRequest
from moveit_msgs.msg import PositionIKRequest, RobotState, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK, GetPositionFKRequest

from src.env.utils import (
    get_pose_msg, 
    get_stamped_pose,
    create_collision_object_from_open3d_mesh,
    calculate_place_position, 
    is_collision, 
    adjust_z
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
        self.use_gt_perception = cfg['perception']['use_ground_truth']
        self.use_gt_planning_scene = cfg["perception"]["use_gt_planning_scene"]
        
        # moveit config 
        self.debug = self.config.get('debug', False)
        self.planning_time = self.config.get('planning_time', 15)
        self.max_velocity = self.config.get('max_velocity', 0.2) 
        self.max_acceleration = self.config.get('max_acceleration', 0.2)
        self.goal_position_tolerance = self.config.get('goal_position_tolerance', 0.001)
        self.goal_orientation_tolerance = self.config.get('goal_orientation_tolerance', 0.02)  
        self.goal_joint_tolerance = self.config.get('goal_joint_tolerance', 0.001)
        self.reference_frame = self.config.get('reference_frame', self.frame)
        self.cartesian_path = self.config.get('cartesian_path', False)
        self.tentative_approach = self.config.get('tentative_approach', True)

        # group name 
        # franka default
        self.base_link = self.config.get('arm_base_link', 'panda_link0')
        self.arm_group_name = self.config.get('arm_group_name', "panda_arm")
        self.gripper_group_name = self.config.get('gripper_group_name', "panda_hand")
        self.manipulator_group_name = self.config.get('manipulator_group_name', "panda_manipulator")
        self.reset_joint_values = self.config.get('initial_joint_values', [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854])

        # ur5 default
        # self.base_link = self.config.get('arm_base_link', 'ur5_base_link')
        # self.arm_group_name = self.config.get('arm_group_name', "ur5_arm")
        # self.gripper_group_name = self.config.get('gripper_group_name', "gripper")
        # self.manipulator_group_name = self.config.get('manipulator_group_name', "ur5_manipulator")
        # self.reset_joint_values = self.config.get('initial_joint_values', [3.1416, -1.5447, -1.5447, -1.5794, 1.5794, 0])
        
        # environment prior knowledge
        # TODO: consider to parse this from external perception model
        self.static_objects = ['table', 'cabinet']

        self.ignore_coll_check = False
        self.wait_at_grasp_pose = False
        self.wait_at_place_pose = False

        # tf buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialized Moveit services
        self.compute_ik = rospy.ServiceProxy("/compute_ik", GetPositionIK)
        self.compute_fk = rospy.ServiceProxy("/compute_fk", GetPositionFK)
        if not self.use_gt_planning_scene:
            self.clear_octomap = rospy.ServiceProxy("/clear_octomap", Empty)

        # Joint limits for Franka panda Emika. Used to check if IK is at limits.
        self.upper_limit = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
        self.lower_limit = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
        # MoveIt! interface
        self.robot = moveit_commander.RobotCommander()
        self.planning_scene = moveit_commander.PlanningSceneInterface()
        
        # NOTE: use xxx_manipulator move_group to make grasp center as end effector link 
        self.move_group = moveit_commander.MoveGroupCommander(self.manipulator_group_name, wait_for_servers=15)
        # self.move_group_arm = moveit_commander.MoveGroupCommander(self.arm_group_name, wait_for_servers=15)
        # NOTE: use actionlib to control gripper for more detailed control instead of moveit commander 
        self.move_group_gripper = moveit_commander.MoveGroupCommander(self.gripper_group_name, wait_for_servers=15)
        self.gripper_group = GripperCommanderGroup.get_instance(self.gripper_group_name)
        
        # collision manager to toggle collision between objects
        # used when contrained objects are attached to the robot
        # moveit planning cannot understand translational and rotational joints 
        self.collision_manager = CollisionManager()
        
        self.end_effctor_link = self.move_group.get_end_effector_link()

        self.error_recovery_client = SimpleActionClient(
            "/franka_control/error_recovery", ErrorRecoveryAction
        )

        # TODO: moveit configurations should be set inside the config file 
        # Set parameters in move_group
        self.move_group.set_planning_time(self.planning_time)
        self.move_group.set_max_velocity_scaling_factor(self.max_velocity)
        self.move_group.set_max_acceleration_scaling_factor(self.max_acceleration)
        self.move_group.set_goal_position_tolerance(self.goal_position_tolerance)
        self.move_group.set_goal_orientation_tolerance(self.goal_orientation_tolerance)
        self.move_group.set_goal_joint_tolerance(self.goal_joint_tolerance)
        self.move_group.set_pose_reference_frame(self.reference_frame)    
        
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
            rospy.logwarn("Execution failed! Going to retry with error recovery")
            self.error_recovery_client.send_goal_and_wait(ErrorRecoveryActionGoal())
            return move_group.go(wait=True)
            # return False
        return True

    def _execute(self, move_group, plan, reset_err=True):
        if not move_group.execute(plan, wait=True):
            if reset_err:
                rospy.logwarn("Execution failed!. Going to retry with error recovery")
                self.error_recovery_client.send_goal_and_wait(ErrorRecoveryActionGoal())
                return move_group.execute(plan, wait=True)
            # return False
        return True

    def computeFK(
        self,
        joint_names: List[str],
        joint_states: List[float],   
    ) -> Pose:
        """
        Compute forward kinematics for the given joint states.
        """
        fk_request = GetPositionFKRequest()
        fk_request.fk_link_names = [self.end_effctor_link]
        fk_request.robot_state.joint_state.name = joint_names
        fk_request.robot_state.joint_state.position = joint_states

        request_value = self.compute_fk(fk_request)
        return request_value.pose_stamped[0].pose
    
    def computeIK(
        self,
        pose: Pose,
        ik_link_name=None,
        move_group=None,
        avoid_colllision=True,
    ) -> bool:
        """
        Check if a given pose is reachable for the robot. Return True if it is, False otherwise.
        
        FIXME: /compute_ik is buggy for moveit 1 until Jan. 2024, see https://github.com/ros-planning/moveit/issues/1568
        Do not use function, use move_group.plan() instead.
        """

        # Create a pose to compute IK for
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "world"
        pose_stamped.pose = pose
        
        ik_request = PositionIKRequest()
        ik_request.group_name = move_group if move_group is not None else self.manipulator_group_name
        ik_request.ik_link_name = ik_link_name if ik_link_name is not None else self.end_effctor_link
        ik_request.pose_stamped = pose_stamped
        ik_request.robot_state = self.robot.get_current_state()
        ik_request.avoid_collisions = avoid_colllision

        request_value = self.compute_ik(ik_request)

        if request_value.error_code.val == -31:
            rospy.logwarn()
            return False

        if request_value.error_code.val == 1:
            # Check if the IK is at the limits
            joint_positions = np.array(request_value.solution.joint_state.position[:7])
            upper_diff = np.min(np.abs(joint_positions - self.upper_limit))
            lower_diff = np.min(np.abs(joint_positions - self.lower_limit))
            return min(upper_diff, lower_diff) > 0.1
        else:
            return False

    def register_object_mesh(
            self, mesh: o3d.geometry.TriangleMesh, object_id: int, position: List[float] = [0, 0, 0]
        ):
        """Adds a given mesh to the scene and registers it as an object."""
        pose_stamped = get_stamped_pose(
            position=position,
            orientation=[0, 0, 0, 1],
            frame_id=self.reference_frame,
        )  
        co = create_collision_object_from_open3d_mesh(
            name=object_id, 
            pose_stamped=pose_stamped,
            triangle_mesh=mesh
        )
        self.planning_scene.add_object(co)
        print(f"Added object {object_id} to planning scene")

    def reset_scene(self):
        """Reset the scene to the initial state."""
        self.objects = {}
        if self.use_sim:
            self.reset_gazebo()
        
        # reset planning scene 
        # remove all attached objects
        self.planning_scene.remove_attached_object()
        
        if not self.use_gt_planning_scene:
            # remove all objects in planning scene from external perception 
            self.planning_scene.remove_world_object()
            # clear octomap from perception if perception used for planning scene
            self.clear_octomap()
        
        if self.use_gt_perception:
            # load GT object names from gazebo
            for object_name in self.get_gazebo_model_names():
                self.objects[object_name] = {}
        else: 
            self.objects = {}
        
        # reset visualization marker if debug is enabledss
        if self.debug:              
            self.start_state_pub.publish(self.robot.get_current_state())  
            self.goal_state_pub.publish(self.robot.get_current_state())
            rospy.sleep(1)

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
        
    # @_block
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

    def get_gripper_pose(self, group=None)->Pose:
        """Get the current pose of the gripper."""
        if group is None:
            group = self.move_group
        return group.get_current_pose().pose

    def get_robot_base_position(self)->np.ndarray: 
        """ 
        Get the current position of the robot base in the reference frame by querying the tf from reference frame to robot base link
        """

        transform:tf2_ros.TransformStamped = self.tf_buffer.lookup_transform(self.reference_frame, self.base_link, rospy.Time(0), rospy.Duration(1.0))
        position = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z], dtype=float)
        return position

    # @_block
    def open_gripper(self, gripper_group=None, width=0.08):
        """Open the gripper."""
        # if gripper_group is None:
        #     gripper_group = self.gripper_group
        # return gripper_group.open_gripper(width=width)

        # use move_group_gripper to open gripper
        for i in range(5):
            self.move_group_gripper.set_joint_value_target([width/2, width/2])
            rospy.logdebug(f"MoveitEnv: wait for open_gripper {i}-th time.....")
            plan = self._go(self.move_group_gripper)
            joint_states = self.move_group_gripper.get_current_joint_values()
            if joint_states[0] + joint_states[1] > 0.035:
                break
    
        self.move_group_gripper.stop()
        self.move_group_gripper.clear_pose_targets()
        return plan
        
        

    # @_block
    def close_gripper(self, gripper_group=None, width=0.01, force=20):
        """Close the gripper."""
        if gripper_group is None:
            gripper_group = self.gripper_group
        return gripper_group.close_gripper(width=width, force=force)


    # @_block
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
                
        success = self._execute(group, plan)
        group.stop()
        group.clear_pose_targets()
        return success

    # @_block
    def follow_path(self, path, group=None, step_size=0.01, jump_threshold=0.0, avoid_collisions=True):
        """Execute a given plan."""
        if group is None:
            group = self.move_group
        
        (plan, fraction) = group.compute_cartesian_path(path, step_size, jump_threshold, avoid_collisions=avoid_collisions)
        return self._execute(group, plan)
        
    # @_block
    def move_joints_to(self, joint_values, group=None):
        """Move the robot to a given joint configuration."""
        if group is None:
            group = self.move_group
        group.set_joint_value_target(joint_values)
        plan = self._go(group)
        group.stop()
        group.clear_pose_targets()
        return plan

    # @_block
    def attach_object(self, object_id, link=None):
        """Attach an object to the robot gripper"""
        # TODO: need to clean & refactor this function
        if link is None:
            link = self.end_effctor_link

        # DO NOT add furniture into moveit planning scene since they are static
        if has_keywords(object_id, self.static_objects):
            rospy.logdebug(f"Moveit: object {object_id} is static. Skip attaching to robot")
            return False
            
        # NOTE: since the object name in moveit planning scene is different from the object name in gazebo with Gazebo plugin
        # we need to convert the object name to the name in moveit planning scene
        # e.g. 'apple' in gazebo -> 'apple.link' in moveit planning scene
        moveit_object_names = self.planning_scene.get_known_object_names()
        
        for moveit_object_name in moveit_object_names:
            if object_id in moveit_object_name:
                if object_id not in self.objects:
                    self.objects[object_id] = {}
                self.objects[object_id]["attach_name"] = moveit_object_name
                self.move_group.attach_object(moveit_object_name, link) 
                if self.verbose:
                    rospy.loginfo(f"Moveit: attached object object {object_id} to {link}")
                return True
        
        rospy.logerr(f"Moveit: object {object_id} not found in moveit planning scene. Planning scene objects: {moveit_object_names}")
        return False


    # @_block
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


    # @_block
    def grasp(
        self,
        pose: Pose,
        pre_grasp_approach=0.1,
        depth=0.03,
        tentative_depth_list=[0.02, 0.0, -0.02],
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
            # return False
        
        if self.verbose:
            rospy.loginfo("MoveitEnv: Moved to pre-grasp pose")

        # FIXME: collision between gripper and other objects closes gripper, re-open to make sure it is open at pre-grasp pose
        # self.gripper_group.open_gripper(width=0.08)
        self.open_gripper()

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
            plan_success = False
            
            # calculate tentative approach pose
            for tentative_depth in tentative_depth_list:
                tentative_approach_pose = get_pose_msg(
                    position
                    + Rotation.from_quat(orientation).as_matrix() @ (tentative_depth * np.array([0, 0, 1])),
                    orientation,
                )

                # try to plan to tentative approach pose
                success = self.move_group.set_pose_target(tentative_approach_pose)
                # return: (success flag : boolean, trajectory message : RobotTrajectory, planning time : float, error code : MoveitErrorCodes)
                plan_success, plan, _, _ = self.move_group.plan()
                
                if plan_success:
                    self.move_to_pose(tentative_approach_pose)
                    break
            
            if not plan_success:
                rospy.logwarn("MoveitEnv: Failed to approach to grasp pose")
                return False
            
        
        if self.verbose:
            rospy.loginfo("MoveitEnv: Approached to grasp pose")

        if self.wait_at_grasp_pose:
            import time
            time.sleep(5)
        
        return True

    # @_block
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
    
    def parse_place_pose(self, object_name, receptacle_name:str=None, **kwargs)->Pose:
        """
        Parse place pose for the object. Use ground truth grounding and heuristic place position calculation.
        Args:
            object_name: str, name of the object
            receptacle_name: Optional(str), name of the receptacle
            position: Optional(np.array), position of the place pose
            description: Optional(str), description of the pose, "canonical pose" or "current pose"
        """
        # get parameters from kwargs
        position = kwargs.get('position', None)
        if isinstance(position, Point):
            position = np.array([position.x, position.y, position.z])
        if isinstance(position, list):
            position = np.array(position)
        # description: str= kwargs.get('description', "current pose") 
        # assert description in ["canonical pose", "current pose"] # only support canonical pose and current pose for now
        predict_place_orientation = kwargs.get('predict_place_orientation', True)
        
        
        # get the bounding box of the object and all other objectss
        object_bbox = self.get_3d_bbox(object_name)
        object_names = self.get_object_name_list()
        obstacle_bbox_list = [
            self.get_3d_bbox(obstacle_name) for obstacle_name in object_names 
            if obstacle_name not in [object_name]
        ]
        
        pose = Pose()
        #################### Solve valid place position #########################
        # If receptacle_name is given, get the receptacle position and bounding box
        if receptacle_name is not None:
            receptacle_bbox = self.get_3d_bbox(receptacle_name)
            # FIXME: for drawer, just hard code the receptacle position x to [max_x-0.2, max_x]
            if "drawer" in receptacle_name.lower():
                receptacle_bbox[0] = receptacle_bbox[3] - 0.2
        
        # If position is given, use it directly, otherwise use grounding model to get the receptacle position
        if position is None:
            assert receptacle_name is not None, "parse_place_pose: position must be given if receptacle_name is not given"
            position = calculate_place_position(
                object_bbox, receptacle_bbox, obstacle_bbox_list, max_tries=100)
        else:
            # position already given, check if the position is valid, if not, adjust it until no collision found 
            collision_mask = np.array([is_collision(object_bbox, obstacle_bbox) for obstacle_bbox in obstacle_bbox_list])
            # adjust the z position if there is collision
            if np.any(collision_mask):
                collided_bbox_list = np.array(obstacle_bbox_list)[collision_mask]
                position[2] = adjust_z(object_bbox, collided_bbox_list, extra_elevation=0.1)          
        
        ###################### Solve valid place orientation ############################
        # Solve the valid place position by move_group 
        if predict_place_orientation:
            self.move_group.set_position_target(position)
            plan_success, plan, _, _ = self.move_group.plan()
            plan: RobotTrajectory
            if plan_success:
                # set the orientation to last plan 
                joint_names = plan.joint_trajectory.joint_names
                last_state:JointTrajectoryPoint  = plan.joint_trajectory.points[-1]
                # convert joint positions to end effector pose
                try:
                    planned_pose = self.computeFK(joint_names, last_state.positions)
                    return planned_pose
                except: 
                    pass
            
            rospy.logwarn("MoveitEnv.parse_place_pose: Failed to plan to target place position")  
        pose.position = Point(*position)
        pose.orientation = Quaternion(-1.0,0.0,0.0,0.0)
        
        return pose