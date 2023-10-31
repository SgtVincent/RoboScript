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
import franka_gripper.msg
from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryActionGoal
import rospy

# Brings in the SimpleActionClient
from actionlib import SimpleActionClient
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

class Grasp(NamedTuple):
    orientation: np.ndarray
    position: np.ndarray
    score: float
    width: float
    instance_id: int

# Workaround to use the gripper action client with unified gripper interface
class GripperCommanderGroup:

    def __init__(self) -> None:    

        self.init_clients()
        print("Gripper action clients ready")


    def init_clients(self):
        self.gripper_grasp_client = SimpleActionClient(
            "/franka_gripper/grasp", franka_gripper.msg.GraspAction
        )       
        self.gripper_grasp_client.wait_for_server()

        self.gripper_move_client = SimpleActionClient(
            "/franka_gripper/move", franka_gripper.msg.MoveAction
        )
        self.gripper_move_client.wait_for_server()

        self.gripper_stop_client = SimpleActionClient(
            "/franka_gripper/stop", franka_gripper.msg.StopAction)
        self.gripper_stop_client.wait_for_server()


    def reset(self):
        self.gripper_stop_client.send_goal_and_wait(franka_gripper.msg.StopGoal())
        self.init_clients()


    def open_gripper(self, width=0.08):
        goal = franka_gripper.msg.MoveGoal(width=width, speed=0.05)
        # self.gripper_move_client.send_goal_and_wait(goal, rospy.Duration(3.0))
        self.gripper_move_client.send_goal(goal)
        done = self.gripper_move_client.wait_for_result()
        return done

    def close_gripper(self, width=0.01, speed=0.05, force=50):
        """Close the gripper."""
        goal = franka_gripper.msg.GraspGoal(width=width, speed=speed, force=force)
        goal.epsilon.inner = 0.08
        goal.epsilon.outer = 0.08
        # self.gripper_grasp_client.send_goal_and_wait(goal, rospy.Duration(3.0))
        self.gripper_grasp_client.send_goal(goal)
        done = self.gripper_grasp_client.wait_for_result()
        return done


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

        # environment prior knowledge
        # TODO: consider to parse this from external perception model
        self.static_objects = ['table']

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
        # NOTE: use panda_manipulator move_group to make grasp center as end effector link 
        self.move_group = moveit_commander.MoveGroupCommander("panda_manipulator", wait_for_servers=15)
        # self.move_group = moveit_commander.MoveGroupCommander("panda_arm", wait_for_servers=15)
        # self.gripper = moveit_commander.MoveGroupCommander("panda_hand", wait_for_servers=15)
        self.gripper_group = GripperCommanderGroup()
        # collision manager to toggle collision between objects
        # used when contrained objects are attached to the robot
        # moveit planning cannot understand translational and rotational joints 
        self.collision_manager = CollisionManager()
        
        self.end_effctor_link = self.move_group.get_end_effector_link()

        self.error_recovery_client = SimpleActionClient(
            "/franka_control/error_recovery", ErrorRecoveryAction
        )

        print("Loading static scene information")
        self.object_names = self.get_obj_names()
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


        gripper_collision_frames = ["panda_rightfinger", "panda_leftfinger"]

        # disable collision between gripper and other objects
        # NOTE: currently disable collision in srdf setting file to accelerate system booting 
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
        return True

    def _execute(self, move_group, plan, reset_err=True):
        if not move_group.execute(plan, wait=True):
            if reset_err:
                rospy.logwarn("Execution failed!. Going to retry with error recovery")
                self.error_recovery_client.send_goal_and_wait(ErrorRecoveryActionGoal())
                return move_group.execute(plan, wait=True)
            return False
        return True

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
        self.scene.add_mesh(
            f"inst_{object_id}",
            get_stamped_pose(position, [0, 0, 0, 1], self.frame),
            f,
            size=(1, 1, 1),
        )

    def reset_scene(self):
        """Reset the scene to the initial state."""
        self.objects = {}
        if self.use_sim:
            self.reset_world()
            # self.reset_simulation()
        
        # reset planning scene 
        # remove all attached objects
        self.scene.remove_attached_object()
        
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
        # first reset robot pose, otherwise there could be collision 
        group.set_joint_value_target(self.reset_joint_values)
        self._go(group)
        group.stop()
        group.clear_pose_targets()
        gripper_group.reset()
        self.open_gripper(gripper_group)
        
        # then reset the world and moveit planning scene 
        self.reset_scene()    
        rospy.loginfo("Environment reset.")

    def get_ee_pose(self, group=None):
        """Get the current pose of the end effector."""
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
    def close_gripper(self, gripper_group=None, width=0.01, force=50):
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
            
        # first try to plan with cartesian path if enabled
        if self.cartesian_path:
            group.set_pose_target(pose)
            (plan, fraction) = group.compute_cartesian_path([pose], 0.02, 0.0)
            if fraction < 0.9:
                rospy.logwarn(f"Could not plan cartesian_path to target pose \n{pose}.\n Plan accuracy: {fraction}")
                success = False
            else:
                success = self._execute(group, plan)
                
        # if cartesian path is disabled or failed, try to plan with joint values
        if not self.cartesian_path or not success:
            group.set_pose_target(pose)
            plan = self._go(group)
            if not plan:
                rospy.logwarn(f"Could not plan to target pose \n{pose}")
                success = False
        
        group.stop()
        group.clear_pose_targets()
        return plan

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
            
            self.move_group.attach_object(f"{object_id}.link", link) 
            # self.scene.attach_mesh(object_id, object_id, 
            #                        touch_links=[*self.robot.get_link_names(group= "panda_hand"), "panda_joint7"])
            if self.verbose:
                rospy.loginfo(f"Moveit: attached object object {object_id} to gripper link")

    @_block
    def detach_object(self, object_id):
        """Detach an object from the robot."""
        try:
            self.move_group.detach_object(object_id)
            
            if self.verbose:
                print(f"Moveit: detached object {object_id} from gripper link")
        except:
            if self.verbose:
                rospy.logerr(f"Moveit: failed to detach object {object_id} from gripper link")


    @_block
    def grasp(
        self,
        pose: Pose,
        width=0.025,
        pre_grasp_approach=0.05,
        dryrun=False,
        object_id=None,
    ):
        """Executes a grasp at a given pose with given orientation.

        Args:
            position: The position of the grasp.
            orientation: The orientation of the grasp (scipy format, xyzw).
            width: The width of the gripper.
            pre_grasp_approach: The distance to move towards the object before grasping.
            dryrun: If true, the robot will not call the action to close the gripper (not available in simulation).       
        """

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

        self.move_group.set_pose_target(waypoints[0])

        if self.cartesian_path:
            (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.02, 0.0)
            if fraction < 0.9:
                print("Could not plan to pre-grasp pose. Plan accuracy", fraction)
                return False

        if self.verbose:
            print("Moving to pre-grasp pose")

        if self.cartesian_path:
            self._execute(self.move_group, plan)
        else:
            plan = self._go(self.move_group)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        if not plan:
            print("Failed")
            return False

        if self.verbose:
            print("Moved to pre grasp. Remmoving object")

        if object_id is not None and object_id in self.objects:
            self.scene.remove_world_object(f"inst_{object_id}")
            self.objects[object_id]["active"] = False

        waypoints = [get_pose_msg(position, orientation)]
        # (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.003, 0.001, True)
        # if fraction < 0.7:
        #     print("Could not plan to pre-grasp pose. Plan Accuracy", fraction)
        #     return False

        if self.verbose:
            print("Moving to grasp pose")

        self.move_group.set_pose_target(waypoints[0])
        self.error_recovery_client.send_goal_and_wait(ErrorRecoveryActionGoal())
        self._go(self.move_group)
        # plan = self.move_group.g/o(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        if not plan:
            print("Failed!")
            return False

        if self.wait_at_grasp_pose:
            import time

            time.sleep(5)

        if not dryrun:
            if self.verbose:
                print("Closing gripper")
                self.close_gripper(width=width)

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
