#!/usr/bin/env python3
"""Python interface to execute grasps on the Franka Emika fr3 robot.

This module provides a class to execute grasps on the Franka Emika fr3 robot.
It uses MoveIt! to plan and execute the grasps.
"""
from typing import List, NamedTuple
import numpy as np

from visualization_msgs.msg import Marker, MarkerArray
from visualization_msgs.msg import Marker


import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from visualization_msgs.msg import Marker

import trimesh

import actionlib
import numpy as np
from scipy.spatial.transform import Rotation
from ros_utils import get_pose_msg, get_stamped_pose
import franka_gripper.msg
from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryActionGoal
import rospy

# Brings in the SimpleActionClient
import actionlib
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import GetPositionIK

class Grasp(NamedTuple):
    orientation: np.ndarray
    position: np.ndarray
    score: float
    width: float
    instance_id: int




class GraspExecutor:
    """Class to execute grasps on the Franka Emika fr3 robot."""

    
    def __init__(self, frame, reset_pose = None, drop_off_pose = None, load_wall = False) -> None:
        """Initialize the GraspExecutor class.
        
        Args:
            frame: The frame in which the poses are given.
            reset_pose: The pose to which the robot should move to reset the scene.
            drop_off_pose: The pose to which the robot should move to drop off the object.
        """
        
        self.moving = False
        self.objects = {}

        self.frame = frame

        self.ignore_coll_check = False
        self.wait_at_grasp_pose = False
        # Service to compute IK
        self.compute_ik = rospy.ServiceProxy('/compute_ik', GetPositionIK)

        # Joint limits for Franka fr3 Emika. Used to check if IK is at limits.
        self.upper_limit = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])
        self.lower_limit = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])

        self.load_wall = load_wall

        # MoveIt! interface
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("fr3_manipulator", wait_for_servers = 15)
        self.gripper = moveit_commander.MoveGroupCommander("fr3_hand", wait_for_servers = 15)
        self.grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)
        self.move_client = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)

        self.error_recovery_client = actionlib.SimpleActionClient('/franka_control/error_recovery', ErrorRecoveryAction)

        print("Loading static scene information")
        self.load_scene(load_wall=self.load_wall)
        self.group.set_planning_time(15)
        self.group.set_max_velocity_scaling_factor(0.15)
        self.group.set_max_acceleration_scaling_factor(0.15)
        
        self.drop_off_pose = drop_off_pose if drop_off_pose is not None else [-0.6861938888210998, -1.0922074558700297, -0.596633734874051, -2.397880921082069, -0.5792871115412288, 1.3971697680950166, -1.9250296761749517]
        self.reset_pose = reset_pose if reset_pose is not None else [-0.41784432355234147, 0.49401059360515726, -0.6039398761251251, -1.1411382317488874, 0.7870978311647195, 1.1255037510962724, -1.456606710367404]
        # self.drop_off_pose =  [-0.3830724722278898, -0.6373905915255127, -0.461236102236845, -2.7075531786406684, -0.1428619671947347, 2.010842381524334, -0.75863185727017]
        
        print("Set up Franka API. Ready to go!")


    def computeIK(self, orientation, position, ik_link_name = "fr3_hand_tcp", move_group="fr3_manipulator") -> bool:
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
            joint_positions = np.array(request_value.solution.joint_state.position[:7] )
            upper_diff = np.min(np.abs(joint_positions - self.upper_limit))
            lower_diff = np.min(np.abs(joint_positions - self.lower_limit))
            return min(upper_diff, lower_diff) > 0.1
        else:
            return False


    def reset_scene(self):
        """Reset the scene to the initial state."""
        self.scene.clear()
        self.objects = {}
        self.load_scene(load_wall=self.load_wall)

    def load_scene(self, load_wall:bool=True):
        """Load the scene in the MoveIt! planning scene.
        
        Loads a static floor and if needed, a static wall to the MoveIt! planning scene.
        """
        if load_wall:
            wall_pose = get_stamped_pose([0, 0.47, 0.0], [0, 0, 0, 1], "fr3_link0")
            self.scene.add_box("wall", wall_pose, size=(3, 0.02, 3))
        # Misc variables
        wall_pose = get_stamped_pose([1.7, 0.0, -0.025], [0, 0, 0, 1], "fr3_link0")
        self.scene.add_box("floor", wall_pose, size=(3, 3, 0))

        wall_pose = get_stamped_pose([-0.50, 0.0, 0.0], [0, 0, 0, 1], "fr3_link0")
        self.scene.add_box("wall", wall_pose, size=(0.02, 3, 3))

        cam_pose = get_stamped_pose([0.03, 0, 0.01], [0, 0, 0, 1], "fr3_hand")
        self.scene.add_box("cam", cam_pose, size = (0.04, 0.14, 0.02))
        self.scene.attach_mesh("fr3_hand", f"cam", touch_links=[*self.robot.get_link_names(group= "fr3_hand"), "fr3_joint7"])

    def _block(fn):
        """Decorator to block the execution of a function if the robot is moving.
        
        Sets the self.moving variable to True before executing the function and to False after.
        """
        def lock_state( self, *args, **kwargs ):
            is_moving = self.moving
            self.moving = True
            ret = fn(self, *args, **kwargs)
            self.moving = False if not is_moving else True
            return ret
        return lock_state

    @_block
    def open_gripper(self):
        """Open the gripper."""
        goal =  franka_gripper.msg.MoveGoal(width=0.039*2, speed=1.0)
        self.move_client.send_goal(goal)
        self.move_client.wait_for_result(timeout=rospy.Duration(2.0))

    @_block
    def reset(self):
        """Reset the robot to the initial state and opens the gripper."""
        self.open_gripper()
        self.group.set_joint_value_target(self.reset_pose)
        self._go(self.group)
        self.group.stop()
        self.group.clear_pose_targets()
        self.reset_scene()

    @_block
    def move_to_pose(self, position: List[float], orientation: List[float] = None):
        """Move the robot to a given pose with given orientation."""
        orientation = orientation if orientation is not None else [0, 0, 0, 1]
        pose = get_stamped_pose(position, orientation, self.frame)
        self.group.set_pose_target(pose)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        return plan

    def _go(self, move_group):
        if not move_group.go(wait=True):
            print("Execution failed! Going to retry with error recovery")
            self.error_recovery_client.send_goal_and_wait(ErrorRecoveryActionGoal())
            return move_group.go(wait=True)
        return True

    def _execute(self, move_group, plan, reset_err = True):
        if not move_group.execute(plan, wait=True):
            if reset_err:
                print("Execution failed!. Going to retry with error recovery")
                self.error_recovery_client.send_goal_and_wait(ErrorRecoveryActionGoal())
                move_group.execute(plan, wait=True)

    def register_object(self, mesh: trimesh.Trimesh, object_id: int, position: List[float] = [0,0,0]):
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
        self.scene.add_mesh(f"inst_{object_id}", get_stamped_pose(position, [0, 0, 0, 1], self.frame), f, size = (1, 1, 1))


    @_block
    def pick_and_drop(self, grasps: List[Grasp], cb = lambda x: x):
        scores = np.array([g.score for g in grasps])
        score_ids = np.argsort(-scores)
        grasps = [grasps[i] for i in score_ids]
        scores = scores[score_ids]
        # drop invalid
        retry_count = 10

        for grasp in grasps:
            if retry_count < 0:
                return False
            

            orientation = grasp.orientation
            # gripper conventions differ
            position = grasp.position + Rotation.from_quat(orientation).as_matrix() @ np.array([0,0,0.045]) # 
            pre_grasp_position = position + Rotation.from_quat(orientation).as_matrix() @ (0.03*np.array([0,0,-1]))
            if self.computeIK(orientation, pre_grasp_position):
                cb(Grasp(orientation, position - Rotation.from_quat(orientation).as_matrix() @ np.array([0,0,0.045]), grasp.score, max(0.01, grasp.width - 2), grasp.instance_id))
                if self.grasp(position, orientation, width = max(0.01, grasp.width - 2), object_id = grasp.instance_id, verbose = True):
                    return True
                else:
                    retry_count -= 1
                
            # Rotate by 180
            ori = np.array(orientation)
            ori = ori[[1,0,3,2]]
            ori[1] *= -1
            ori[3] *= -1
            if self.computeIK(ori, pre_grasp_position):
                cb(Grasp(ori, position-Rotation.from_quat(orientation).as_matrix() @ np.array([0,0,0.045]), grasp.score,  max(0.01, grasp.width - 2), grasp.instance_id))
                if self.grasp(position, ori, width = max(0.01, grasp.width - 2), object_id = grasp.instance_id, verbose = True):
                    return True
                else:
                    retry_count -= 1

    @_block
    def grasp(self, position, orientation, width = 0.025, pre_grasp_approach = 0.05, dryrun = False, object_id = None, verbose = True):
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

        pre_grasp_pose = get_pose_msg(position + Rotation.from_quat(orientation).as_matrix() @ (pre_grasp_approach*np.array([0,0,-1])), orientation)
        waypoints = [pre_grasp_pose]

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

            goal = franka_gripper.msg.GraspGoal()
            goal.width = width
            goal.epsilon.inner = 0.03
            goal.epsilon.outer = 0.03
            goal.speed = 0.25
            goal.force = 20
            self.grasp_client.send_goal(goal)
            self.grasp_client.wait_for_result()

        if object_id is not None and object_id in self.objects:
            touch_links = self.robot.get_link_names(group= "fr3_hand")
            self.scene.add_mesh(f"inst_{object_id}", get_stamped_pose(self.objects[object_id]["position"], [0, 0, 0, 1], self.frame), self.objects[object_id]["file"], size = (1, 1, 1))
            self.scene.attach_mesh("fr3_hand", f"inst_{object_id}", touch_links=touch_links)
            if verbose:
                print("attached mesh to ", touch_links)
        
        if verbose:
            print("Moving to drop off pose")
        
        self.group.set_joint_value_target(self.drop_off_pose)
        self.error_recovery_client.send_goal_and_wait(ErrorRecoveryActionGoal())
        # self.group.go(wait=True)
        self._go(self.group)
        self.moving = False
        self.group.stop()
        self.group.clear_pose_targets()

        if object_id is not None and object_id in self.objects:
            self.scene.remove_attached_object("fr3_hand", name=f"inst_{object_id}")
            self.scene.remove_world_object(f"inst_{object_id}")
            self.objects[object_id]["active"] = False

        
        if verbose:
            print("dropping object")
        self.open_gripper()
        self.reset()
        
        return True


if __name__ == "__main__":
    rospy.init_node("grasp_executor")
    grasp_controller = GraspExecutor("world", load_wall=False, reset_pose = [-0.39999520574177366, 0.4740489757251173, -0.5709029256494976, -1.1338590764153493, 0.7967397934201639, 1.0726936823311086, -1.427030496828192])
    print("Resetted")
    grasp_controller.reset() 
    pos = [0.15 + 0.3, 0.10, 0.6]
    ori = np.array([0, -0.923, 0.382, 0])
    ori = ori / np.linalg.norm(ori)
    print("Moving to", pos)
    # grasp_controller.move_to_pose(pos, ori)
    # grasp_controller.grasp(rospy.get_param("~grasp_position"), rospy.get_param("~grasp_orientation"), dryrun = rospy.get_param("~sim"), object_id = None)
    #grasp_controller.reset()


    # for _ in range(3):
    #     sphere = trimesh.primitives.Sphere(radius=0.05)
    #     sphere.apply_translation([0.3,0.3,0.2+_*0.05])
    #     grasp_controller.register_object(sphere, _)

    # while not rospy.is_shutdown():
    #     grasp_controller.reset()
    #     grasp_controller.grasp([0.3,0.3,0.4], [1,0,0,0], dryrun = True, object_id = 2)
    #     grasp_controller.reset()

    # print("done")
    rospy.spin()
