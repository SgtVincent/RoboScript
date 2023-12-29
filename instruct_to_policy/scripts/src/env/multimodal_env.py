from typing import List, Tuple, Dict
import rospy 
import json
import numpy as np 
from geometry_msgs.msg import Quaternion, Pose, Point
from scipy.spatial.transform import Rotation
import re
# from grasp_detection.msg import Grasp
from .moveit_gazebo_env import MoveitGazeboEnv
from src.grasp_detection import GraspDetectionBase, GraspDetectionRemote
from src.grounding_model import create_grounding_model, GroundingBase
from src.env.utils import (
    calculate_place_position, 
    is_collision, 
    adjust_z, 
    pose_msg_to_matrix, 
    create_collision_object_from_open3d_mesh,
    get_pose_msg
)

from scipy.spatial.transform import Rotation as R

from src.lmp import *
from data.generate_code import load_queries, prepare_vars_detached, process_raw_output,OPENAI_API_KEY

from src.eval.evaluator import Evaluator
from src.config import cfg_tabletop

import cv2
from .gpt_correct import gpt_check

from datetime import datetime

import moveit_commander

from src.perception.scene_manager import SceneManager

import open3d as o3d
import math


class MultiModalEnv(MoveitGazeboEnv):
    """
    Simple grounding environment to use gazebo GT model state as observation, and GIGA for grasp pose prediction.
    """
    def __init__(self, cfg) -> None:
        super().__init__(cfg)
        self.use_gt_perception = False 
        self.object_metadata_files = cfg["env"]["metadata_files"]
        self.grasp_config = cfg["grasp_detection"]
        self.grasp_method = self.grasp_config["method"] # ["heuristic", "model"]
        
        self.grounding_config = cfg["grounding_model"]
        # use glip as default baseline 
        self.grounding_model_name = self.grounding_config.get("model_name", "glip")
        self.grounding_model_args = self.grounding_config.get("model_args", {})

        group_name = "panda_manipulator"
        group = moveit_commander.MoveGroupCommander(group_name, wait_for_servers = 15)
        self.group = group
        
        self.grasp_model = None
        self.groudning_model = None
        self.obj_name_list = None
        self.object_list = None
        self.detect_first = None
        
        self.pos_dict_for_reuse = {}

        self.drawer_normal = None
        self.is_drawer_handle_vertical = False
        self.drawer_rotate_axis_offset = 0.04
        self.drawer_rotate_axis_xy = None
        self.drawer_rotate_axis_dir = 0 # 转轴方向，1是逆时针，-1是顺时针
        self.drawer_panel_center = None
        
        # scene manager to manage the scene objects, 3D reconstruction, detections, etc.
        self.scene = SceneManager()

        self._init_models()

    
    def _init_models(self):
        """
        Initialze all models needed for the environment: grounding, grasp detection, etc.
        """
        if self.grasp_method in ["model"]:
            grasp_model_config = self.grasp_config["model_params"]
            self.grasp_model = GraspDetectionRemote(grasp_model_config)
            self.grasp_model.load_model()
        
        self.grounding_model = create_grounding_model(self.grounding_model_name, **self.grounding_model_args)

    def detect_objects(self, **kwargs):
        """
        Call the perception pipeline to detect all task-specific objects in the scene.
        """
        # get objects to detect
        object_list = kwargs.get('object_list', [])
        sensor_data = self.get_sensor_data()
        detections_list = self.grounding_model.query_2d_bbox_list(
            sensor_data=sensor_data,
            object_list=object_list
        )
        # update scene 
        data = {
            'detections_list': detections_list
        }
        data.update(sensor_data)
        self.scene.update(data)
        
        # update objects in moveit planning scene 
        self.add_scene_objects_to_moveit()

    def detect_objects_with_moving(self, **kwargs):
        """
        Move to the predefined pos_list and call the perception pipeline to detect all task-specific objects in the scene.
        """
        # get objects to detect
        object_list = kwargs.get('object_list', None)
        planning = kwargs.get('planning', False)
        is_load= kwargs.get('is_load', False)
        print("_----------object_list---------",object_list)
        # self.move_joints_to([-0.001209562553452295, -0.7798870970324466, -0.0025529672049247384, -2.3749749452691327, -0.0031800321414839528, 1.5748067867097109, 0.7786260150587473])
        # object list to detect in the detection functions
        if self.object_list is None:
            self.object_list = object_list
        elif object_list is None:
            object_list = self.object_list
        elif object_list is not None:
            self.object_list = object_list

        for i, object_name in enumerate(object_list):
            if '_' in object_name:
                object_list[i] = object_name.split('_', 1)[0]

        print('object_list', object_list)

        for i, pos in enumerate(self.scene.pos_list):
            self.move_joints_to(pos)
            print('--------------------move_joints_to---------------------------')
            if i == 0:
                sensor_data = self.get_sensor_data()
            else:
                image_set = self.get_sensor_data()
                for key, value in sensor_data.items():
                    value.extend(image_set[key])
                    sensor_data[key] = value
        detections_list = self.grounding_model.query_2d_bbox_list_with_idx(
            sensor_data=sensor_data,
            object_list=object_list
        )

        # update scene 
        data = {
            'detections_list': detections_list
        }

        data.update(sensor_data)
        self.scene.update(data)
        
        # update objects in moveit planning scene 
        self.add_scene_objects_to_moveit()

        self.scene.visualize_3d_bboxes()

        self.move_joints_to([-0.001209562553452295, -0.7798870970324466, -0.0025529672049247384, -2.3749749452691327, -0.0031800321414839528, 1.5748067867097109, 0.7786260150587473])

        


    def get_obj_name_list(self) -> List[str]:
        """
        Get object name list from scene manager
        """
        return self.scene.get_object_names()

    def get_object_center_position(self, obj_name, **kwargs):
        """
        Get the position of the object in the world frame. 
        This function uses ground truth model state from gazebo and ignore all other parameters.
        """
        return self.scene.get_object_center_position(obj_name)
        
    def get_object_pose(self, obj_name, **kwargs):
        # return self.scene.get_object_pose(obj_name)
        raise NotImplementedError
        
    def get_3d_bbox(self, obj_name, **kwargs)->np.array:
        """
        Get the bounding box of the object.
        Args:
            obj_name: name of the object
        Returns:
            bbox: np.ndarray, [x_min, y_min, z_min, x_max, y_max, z_max]
        """
        return self.scene.get_3d_bbox(obj_name)
    
    def parse_adaptive_shape_grasp_pose(self, object_name, **kwargs):
        """
        Parse grasp pose for the object. Use ground truth grounding and grasp detection model.
        Args:   
            object_name: name of the object
            preferred_position: Optional(np.array), prefered position of the gripper
            preferred_direction: Optional(np.array), prefered direction of the gripper
            description: str, description of the pose
        """
        # CGY
        self.grasp_object_name = object_name

        camera_idx = self.scene.get_object_camera_idx(object_name)

        self.move_joints_to(self.scene.pos_list[int(camera_idx)])

        object_bbox = self.get_3d_bbox(object_name)
        preferred_position: np.array = kwargs.get('preferred_position', None)
        preferred_direction:np.array = kwargs.get('preferred_direction', None)
        description:str = kwargs.get('description', None)
        
        # get visual input from perception model
        sensor_data = self.get_sensor_data()
        
        # add 2d/3d bounding boxes to sensor data
        # NOTE: currently only support one object 
        # object_2d_bbox_list = self.scene.get_object_2d_bbox_list(object_name)
        # detections_list = []
        # for bbox in object_2d_bbox_list:
        #     if len(bbox) > 0:
        #         detections_list.append({object: bbox})
                
        bbox_center = (object_bbox[:3] + object_bbox[3:]) / 2

        bbox_size = object_bbox[3:] - object_bbox[:3]
        bbox_center[0] += 0.5*bbox_size[2]
                
        data = {
            # 'detections_list': detections_list,
            'bboxes_3d_dict':{
                object_name:{'center': bbox_center, 'size': bbox_size}
            }
        }
        data.update(sensor_data)
        
        # call grasp detection service
        grasp_candidates = self.grasp_model.predict(data)
        pose_list = [grasp.grasp_pose for grasp in grasp_candidates]
        width_list = [grasp.grasp_width for grasp in grasp_candidates]
        score_list = [grasp.grasp_score for grasp in grasp_candidates]
    
        rank = []
        if preferred_position is not None:
            # if preferred_position is given, choose the grasp pose closest to the prefered position
            position_list = np.array([np.array([p.position.x, p.position.y, p.position.z]) for p in pose_list])
            print('position_list', position_list.shape, preferred_position.shape)
            distance = np.linalg.norm(position_list - preferred_position, axis=1)
            distance_rank_idx = np.argsort(distance)
            distance_rank = np.zeros(len(distance), dtype=np.int)
            distance_rank[distance_rank_idx] = np.arange(len(distance))
            rank.append(distance_rank)
            
        if preferred_direction is not None:
            # if preferred_direction is given, choose the grasp pose with z-axis closest to the prefered direction
            rotation_list = np.array([pose_msg_to_matrix(p)[:3, :3] for p in pose_list])
            z_axis_list = rotation_list[:, :, 2]
            z_axis_list = z_axis_list / np.linalg.norm(z_axis_list, axis=1, keepdims=True)
            preferred_direction = preferred_direction / np.linalg.norm(preferred_direction)
            cos_similarity = np.sum(z_axis_list * preferred_direction, axis=1)
            cos_similarity_rank_idx = np.argsort(cos_similarity)[::-1]
            cos_similarity_rank = np.zeros(len(cos_similarity), dtype=np.int)
            cos_similarity_rank[cos_similarity_rank_idx] = np.arange(len(cos_similarity))
            rank.append(cos_similarity_rank)
                  
        # sort by score 
        score_rank_idx = np.argsort(score_list)[::-1]
        score_rank = np.zeros(len(score_list), dtype=np.int)
        score_rank[score_rank_idx] = np.arange(len(score_list))
        rank.append(score_rank)

        # get the best grasp based on all the ranks, the grasp with the lowest rank sum is the best grasps
        rank = np.array(rank)
        rank_sum = np.sum(rank, axis=0)
        best_grasp_idx = np.argmin(rank_sum)

        pose = pose_list[best_grasp_idx]
        pose.position.z += 0.03
        width = width_list[best_grasp_idx]

        return pose

    def parse_place_pose(self, object_name, receptacle_name:str=None, **kwargs):
        """
        Parse place pose for the object. Use ground truth grounding and heuristic place position calculation.
        Args:
            object_name: str, name of the object
            receptacle_name: Optional(str), name of the receptacle
            position: Optional(np.array), position of the place pose
            description: Optional(str), description of the pose, "canonical pose" or "current pose"

        reuse_dict = {'object_name', 'choice'}
            - object_name: the object being operated
            - choice includes 'save', 'forward', 'reversed', 'averaged',
                - save: save the initial pose and final pose of the {object_name} for reuse, such as save the position when opening the drawer
                - forward: reuse the saved position of {object_name} in forward direction, setting the initial and final poses as the saved initial and final poses, respectively
                - reversed: reuse the saved position of {object_name} in reversed direction, setting the initial and final poses as the saved final and initial poses, respectively
                - averaged: reuse the averaged saved position of {object_name}, setting the final poses as the averaged pose of the saved final and initial poses
        """
        # get parameters from kwargs

        print('--------------parse_place_pose--------------------')
        self.move_joints_to([-0.001209562553452295, -0.7798870970324466, -0.0025529672049247384, -2.3749749452691327, -0.0031800321414839528, 1.5748067867097109, 0.7786260150587473])
        position = kwargs.get('position', None)
        reuse_dict = kwargs.get('reuse_dict', None)
        position = None

        reuse_dict = None

        if reuse_dict == None:
            is_calculate = True
        elif reuse_dict['choice'] == 'save':
            is_calculate = True
        else:
            is_calculate = False
        if is_calculate == True:
            if isinstance(position, Point):
                position = np.array([position.x, position.y, position.z])
            if isinstance(position, list):
                position = np.array(position)
            description: str= kwargs.get('description', "current pose") 
            assert description in ["canonical pose", "current pose"] # only support canonical pose and current pose for now
            
            # get the bounding box of the object and all other objectss
            object_bbox = self.get_3d_bbox(object_name)
            object_names = self.get_obj_name_list()
            obstacle_bbox_list = [
                self.get_3d_bbox(obstacle_name) for obstacle_name in object_names 
                if obstacle_name not in [object_name]
            ]
            
            pose = Pose()
            
            # If receptacle_name is given, get the receptacle position and bounding box
            if receptacle_name is not None:
                receptacle_bbox = self.get_3d_bbox(receptacle_name)
                # FIXME: for drawer, just hard code the receptacle position x to [max_x-0.2, max_x]
                # if "drawer" in receptacle_name.lower():
                #     receptacle_bbox[0] = receptacle_bbox[3] + 0.2
            
            # If position is given, use it directly, otherwise use grounding model to get the receptacle position
            if position is None:
                assert receptacle_name is not None, "parse_place_pose: position must be given if receptacle_name is not given"
                position = calculate_place_position(
                    object_bbox, receptacle_bbox, obstacle_bbox_list, max_tries=100)
                # position[2] += 0.05
                # position[1] += 0.1
                # position[0] += 0.1
            else:
                # position already given, check if the position is valid, if not, adjust it until no collision found 
                collision_mask = np.array([is_collision(object_bbox, obstacle_bbox) for obstacle_bbox in obstacle_bbox_list])
                # adjust the z position if there is collision
                if np.any(collision_mask):
                    collided_bbox_list = np.array(obstacle_bbox_list)[collision_mask]
                    print('----------adjust--------------')
                    position[2] = adjust_z(object_bbox, collided_bbox_list, extra_elevation=0.1)          
                
            pose.position = Point(*position)
            # if description == "canonical pose":
            #     # use canonical orientation
            #     pose.orientation = Quaternion(-1.0,0.0,0.0,0.0)
            # else:
            #     # remain current orientation
            #     pose.orientation = self.get_gripper_pose().orientation
            pose.orientation = self.get_gripper_pose().orientation#Zhou Tianxing

        else:
            object_name = reuse_dict['object_name']
            choice = reuse_dict['choice']
            if choice == 'save':
                object_pos_dict = {'place_pose': pose}
                self.pos_dict_for_reuse[object_name] = object_pos_dict
            elif choice == 'forward':
                object_pos_dict = self.pos_dict_for_reuse[object_name]
                pose = object_pos_dict['final_pose']
            elif choice == 'reversed':
                object_pos_dict = self.pos_dict_for_reuse[object_name]
                pose = object_pos_dict['initial_pose']
            elif choice == 'averaged':
                object_pos_dict = self.pos_dict_for_reuse[object_name]
                pose = Pose()
                pose.position.x = (object_pos_dict['initial_pose'].position.x  + object_pos_dict['final_pose'].position.x) / 2
                pose.position.y = (object_pos_dict['initial_pose'].position.y  + object_pos_dict['final_pose'].position.y) / 2
                pose.position.z = (object_pos_dict['initial_pose'].position.z  + object_pos_dict['final_pose'].position.z) / 2
            else:
                raise NotImplementedError
        # print("--------------move_joint_to---------------------")
        group = self.group
        group.set_goal_tolerance(0.01)
        group.set_planner_id("RRTConnect") 
        group.set_max_velocity_scaling_factor(0.4) # 0.4
        group.set_max_acceleration_scaling_factor(0.2) # 0.4
        # pose_goal.position.x = 0.4
        group.set_joint_value_target([-0.001209562553452295, -0.7798870970324466, -0.0025529672049247384, -2.3749749452691327, -0.0031800321414839528, 1.5748067867097109, 0.7786260150587473])
        #self.go_to_pose_goal([-8.66025404e-01, -2.65143810e-17,  1.53080850e-17, -5.00000000e-01], [0.15 + 35     , -0.16160254,  0.47009619])
        #return
        ## Now, we call the planner to compute the plan and execute it.
        plan = group.go(wait=True)
        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()
        print('-----------------------plan---------------------', plan)
        return pose
    
    def parse_central_lift_grasp_pose(self, object_name, description="top"):
        """
        Parse central lift grasp pose for the object. Use ground truth grounding and heuristic place position calculation.
        """
        # CGY
        self.grasp_object_name = object_name
        self.force_judge = False

        object_bbox = self.get_3d_bbox(object_name)
        object_center = (object_bbox[:3] + object_bbox[3:]) / 2
        
        pose = Pose()   
        pre_defined_depth = 0.03 # for franka hand 0.3
        if description == "top":
            pose.position = Point(object_center[0], object_center[1], object_bbox[5])

        elif description == "center":
            pose.position = Point(object_center[0], object_center[1], object_center[2])
        else:
            # by default grasp top 
            pose.position = Point(object_center[0], object_center[1], object_bbox[5])
        pose.orientation = Quaternion(-1.0,0.0,0.0,0.0)

        return pose

    def parse_horizontal_grasp_pose(self, object_name, reuse_dict=None):
        """ 
        Parse horizontal pose for grasping drawer handle. (master pose for cabinet drawer pulling)
        Currently the position of the handle if the GT bounding box center of handle in gazebo. 
        The gripper is horizontal and perpendicular to the handle.

        reuse_dict = {'object_name', 'choice'}
            - object_name: the object being operated
            - choice includes 'save', 'forward', 'reversed', 'averaged'
                - save: save the initial pose and final pose of the {object_name} for reuse, such as save the position when opening the drawer
                - forward: reuse the saved position of {object_name} in forward direction, setting the initial and final poses as the saved initial and final poses, respectively
                - reversed: reuse the saved position of {object_name} in reversed direction, setting the initial and final poses as the saved final and initial poses, respectively
                - averaged: reuse the averaged saved position of {object_name}, setting the final poses as the averaged pose of the saved final and initial poses
        """
        # print('pre_defined_horizontal_orientation-----', pre_defined_horizontal_orientation)

        # CGY
        self.grasp_object_name = object_name
        self.force_judge = True

        self.move_joints_to([-0.001209562553452295, -0.7798870970324466, -0.0025529672049247384, -2.3749749452691327, -0.0031800321414839528, 1.5748067867097109, 0.7786260150587473])
        

        reuse_dict = None
        if reuse_dict == None:
            is_calculate = True
        elif reuse_dict['choice'] == 'save':
            is_calculate = True
        else:
            is_calculate = False
        if is_calculate == True:
            handle_bbox = self.get_3d_bbox(object_name)
            # reference_bbox = self.get_3d_bbox('apple_0')
            handle_center = (handle_bbox[:3] + handle_bbox[3:]) / 2
 
            # reference_center = (reference_bbox[:3] + reference_bbox[3:]) / 2
            # vec = reference_center - handle_center
            # vec_magnitude = np.linalg.norm(vec)
            # unit_vectors = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        
            # # 计算角度
            # angles = []
            # for unit_vec in unit_vectors:
            #     dot_product = np.dot(vec, unit_vec)
            #     angle = np.arccos(dot_product / vec_magnitude)  # 这里假设单位向量的模为1
            #     angles.append(np.degrees(angle))  # 将弧度转换为度

            # angles[2] = 0

            # quat = Rotation.from_euler('xyz', angles, degrees=True).as_quat()
            # pre_defined_horizontal_orientation = Quaternion(quat[0],quat[1],quat[2],quat[3])
            
            # pre_defined_horizontal_orientation = Quaternion(0.6832535941073715,0.3440460910674551,0.610755296083018,0.20438880024871378)# for franka hand 
            pre_defined_horizontal_orientation = Quaternion(0.5,0.5,0.5,0.5)# for franka hand 
            # Zhou Tianxing 1221
            # handle_center[0]-=0.06  
            pose = Pose()
            pose.position = Point(*handle_center)
            pose.orientation = pre_defined_horizontal_orientation
        else:
            object_name = reuse_dict['object_name']
            choice = reuse_dict['choice']
            if choice == 'save':
                object_pos_dict = {'grasp_pose': pose}
                self.pos_dict_for_reuse[object_name] = object_pos_dict
            elif choice == 'forward':
                object_pos_dict = self.pos_dict_for_reuse[object_name]
                pose = object_pos_dict['final_pose']
            elif choice == 'reversed':
                object_pos_dict = self.pos_dict_for_reuse[object_name]
                pose = object_pos_dict['initial_pose']
            elif choice == 'averaged':
                object_pos_dict = self.pos_dict_for_reuse[object_name]
                pose = Pose()
                pose.position.x = (object_pos_dict['initial_pose'].position.x  + object_pos_dict['final_pose'].position.x) / 2
                pose.position.y = (object_pos_dict['initial_pose'].position.y  + object_pos_dict['final_pose'].position.y) / 2
                pose.position.z = (object_pos_dict['initial_pose'].position.z  + object_pos_dict['final_pose'].position.z) / 2
            else:
                raise NotImplementedError
        return pose
    
    def parse_vertical_grasp_pose(self, object_name, reuse_dict=None):
        """ 
        Parse horizontal pose for grasping drawer handle. (master pose for cabinet drawer pulling)
        Currently the position of the handle if the GT bounding box center of handle in gazebo. 
        The gripper is horizontal and perpendicular to the handle.

        reuse_dict = {'object_name', 'choice'}
            - object_name: the object being operated
            - choice includes 'save', 'forward', 'reversed', 'averaged'
                - save: save the initial pose and final pose of the {object_name} for reuse, such as save the position when opening the drawer
                - forward: reuse the saved position of {object_name} in forward direction, setting the initial and final poses as the saved initial and final poses, respectively
                - reversed: reuse the saved position of {object_name} in reversed direction, setting the initial and final poses as the saved final and initial poses, respectively
                - averaged: reuse the averaged saved position of {object_name}, setting the final poses as the averaged pose of the saved final and initial poses
        """
        # print('pre_defined_horizontal_orientation-----', pre_defined_horizontal_orientation)

        # CGY
        print('-----------------------------parse_vertical_grasp_pose--------------------------------')
        self.grasp_object_name = object_name
        self.force_judge = True

        reuse_dict = None
        if reuse_dict == None:
            is_calculate = True
        elif reuse_dict['choice'] == 'save':
            is_calculate = True
        else:
            is_calculate = False
        if is_calculate == True:
            handle_bbox = self.get_3d_bbox(object_name)
            # reference_bbox = self.get_3d_bbox('apple_0')
            handle_center = (handle_bbox[:3] + handle_bbox[3:]) / 2

            # handle_center[0]-=0.05
 
            # reference_center = (reference_bbox[:3] + reference_bbox[3:]) / 2
            # vec = reference_center - handle_center
            # vec_magnitude = np.linalg.norm(vec)
            # unit_vectors = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        
            # # 计算角度
            # angles = []
            # for unit_vec in unit_vectors:
            #     dot_product = np.dot(vec, unit_vec)
            #     angle = np.arccos(dot_product / vec_magnitude)  # 这里假设单位向量的模为1
            #     angles.append(np.degrees(angle))  # 将弧度转换为度

            # angles[2] = 0

            # quat = Rotation.from_euler('xyz', angles, degrees=True).as_quat()
            # pre_defined_horizontal_orientation = Quaternion(quat[0],quat[1],quat[2],quat[3])
            
            # pre_defined_horizontal_orientation = Quaternion(0.6832535941073715,0.3440460910674551,0.610755296083018,0.20438880024871378)# for franka hand 
            pre_defined_horizontal_orientation = Quaternion(-0.707,0.0,-0.5,0.5)# for franka hand 

            # Zhou Tianxing 1221
            pose = Pose()
            pose.position = Point(*handle_center)
            pose.orientation.x = -0.707
            pose.orientation.y = 0
            pose.orientation.z = -0.707
            pose.orientation.w = 0.2
            print('pose', pose)
        return pose
    

    ####################  Moveit planning related functions ####################
    def add_scene_objects_to_moveit(self, **kwargs):
        """Add all objects in the scene to the moveit planning scene."""
        for object_name in self.get_obj_name_list():
            print('object_name', object_name)
            object_mesh = self.scene.get_object_mesh(object_name)
            self.register_object_mesh(object_mesh, object_name)
            if object_name not in self.objects:
                self.objects[object_name] = {}

    def construct_queries(self, obj_name_list, query):
        context = 'object = ['
        print(enumerate(obj_name_list))
        for i, obj_name in enumerate(obj_name_list):
            context = context + obj_name
            if i != len(obj_name_list) - 1:
                context = context + ', '
            else:
                context = context + ']; # '
        context = context + query
        return context

    def generate_exec_code(self, obj_name_list, query, cfg_tabletop):
        openai.api_key = OPENAI_API_KEY

        if self.obj_name_list is None:
            self.obj_name_list = obj_name_list

        # Initialize LMP instances
        cfg_tabletop = copy.deepcopy(cfg_tabletop)
        cfg_tabletop["env"] = dict()
        # TODO: load table top from simulation 
        cfg_tabletop["env"]["coords"] = lmp_tabletop_coords

        # prepare vars including APIs and constants
        fixed_vars, variable_vars = prepare_vars_detached()

        # load task queries
        task_query = self.construct_queries(obj_name_list, query)

        exception_log = ""
        dump_hist_list = []
        # generate one code snippet for each task query
        # try:

        # remove extra '#' and '\n' in query line
        # task_query = task_query.replace('#', '').replace('\n', '')

        # NOTE: 
        # the benchmark test code generation cabalibity from scratch, so every time should create a new LMP instance
        # creating the function-generating LMP
        lmp_fgen = LMPFGen(cfg_tabletop["lmps"]["fgen"], fixed_vars, variable_vars)
        
        # creating the LMP that deals w/ high-level language commands
        cfg_tabletop["lmps"]["tabletop_ui"]["debug_mode"] = True
        
        lmp_tabletop_ui = LMP(
            "tabletop_ui", cfg_tabletop["lmps"]["tabletop_ui"], lmp_fgen, fixed_vars, variable_vars,
        )

        print(f"Generating code for task query...")
        # generate code snippet
        lmp_tabletop_ui(task_query, "")
        # append dump_hist to list 
        dump_hist_list.append(lmp_tabletop_ui.dump_hist[-1])

        print('dump_hist_list', dump_hist_list)

        # except Exception as e:
        #     exception_log += "----------\n"
        #     exception_log += f"Cannot generate code for task query {i}: {task_query} \n"
        #     exception_log += f"Exception: {e} \n"
        #     exception_log += f"Traceback: {traceback.format_exc()} \n"
        #     exception_log += "----------\n"
        #     # add empty history to dump_hist
        #     dump_hist_list.append({})
                
        # write exception log to file
        exception_log_file = 'exception_log.txt'
        exception_log_path = os.path.join('human_query', exception_log_file)
        with open(exception_log_path, 'w') as f:
            f.write(exception_log)

        # save generated code snippets to json 
        raw_output_file = "raw_human_query.json"
        raw_output_path = os.path.join('human_query', raw_output_file)
        with open(raw_output_path, "w") as f:
            json.dump(dump_hist_list, f, indent=4)


    def execute_code(self, detect_first=False):

        if self.detect_first ==None:
            self.detect_first == detect_first

        raw_file = os.path.join('human_query', 'raw_human_query.json')

        current_root = os.getcwd()
        
        # log file should be appended with the formatted current time 
        time_str = datetime.now().strftime("%Y%m%d-%H:%M")
        log_file = rospy.get_param('~log_file', f'human_query/eval_log_{time_str}.log')
        log_file_path = os.path.join(current_root, log_file)
        # make log directory if not exist
        os.makedirs(os.path.dirname(log_file_path), exist_ok=True)
        
        # results file 
        result_file = rospy.get_param('~result_file', 
                                    os.path.join(current_root, f'human_query/eval_results/result_{time_str}.json'))
        result_file_path = os.path.join(current_root, result_file)
        # make result directory if not exist
        os.makedirs(os.path.dirname(result_file_path), exist_ok=True)
        
        # load generated code and eval items 
        with open(raw_file, 'r') as f:
            raw_data = json.load(f)
        # with open(eval_items_file, 'r') as f:
        #     eval_items_list = json.load(f)

        eval_result_list = []

        for task_idx in range(0, len(raw_data)):
            # if task_idx not in [1]: # debug
            #     continue
            # run code for each task
            data = raw_data[task_idx]
            # if data is empty, append empty results to eval_result_list
            if len(data) == 0:
                eval_result_list.append({})
                continue
            query = data['query']
            code_str = data['code_str']
            object_list=[]

            match = re.search(r"object_list\s\s*=\s*\[([^\]]*)\]", code_str)
            # 检查是否有匹配
            if match:
                # 提取匹配的部分
                list_str = match.group(1)

                # 将提取的字符串转换为列表，这里使用了字符串的split方法，并用strip去除空白和引号
                object_list = [item.strip().strip("'") for item in list_str.split(',')]

                # 打印结果            

            else:
                print("没有找到匹配的列表。")
            # object_list = [item + '_0' for item in object_list]
            if self.detect_first == False:
                # first generate the code
                for object_name in object_list:
                    code_str = code_str.replace(f'\'{object_name}\'', f'\'{object_name}_0\'')
            else:
                # first observe the scene
                index = code_str.find('detect_objects_with_moving')

                # 如果找到了，删除这一行
                if index != -1:
                    # 找到下一个换行符的位置
                    end_index = code_str.find('\n', index)
                    # 删除这一行
                    code_str = code_str[:index] + code_str[end_index+1:]

            print('--------------------------object_list-----------------------------')
            print(object_list)
            # from dict to list 
            defined_functions = [v for k, v in data['src_fs'].items()]
            
            rospy.loginfo("Running code for query: {}".format(query))
            rospy.loginfo("Code: \n'''{}\n'''".format(code_str))

            evaluator = Evaluator(self, log_file=log_file_path ,verbose=True, render=False)

            evaluator.run_eval_robot(code_str, defined_functions, eval_items=None, query=query, repeat_times=1)
    
    def franka_state_callback(self,data,**kwargs):
        x_force = data.wrench.force.x
        y_force = data.wrench.force.y
        z_force = data.wrench.force.z
        # rospy.loginfo("Current axis force,%f,%f,%f", x_force,y_force,z_force)
        self.current_force_x = x_force
        self.current_force_y = y_force
        self.current_force_z = z_force

   
    def move_in_direction(self, axis: np.array, distance: float, reuse_dict: dict = None):
        """
        reuse_dict = {'object_name', 'choice'}
            - object_name: the object being operated
            - choice includes 'save', 'forward', 'reversed', 'averaged'
                - save: save the initial pose and final pose of the {object_name} for reuse, such as save the position when opening the drawer
                - forward: reuse the saved position of {object_name} in forward direction, setting the initial and final poses as the saved initial and final poses, respectively
                - reversed: reuse the saved position of {object_name} in reversed direction, setting the initial and final poses as the saved final and initial poses, respectively
                - averaged: reuse the averaged saved position of {object_name}, setting the final poses as the averaged pose of the saved final and initial poses
        """
        print("___________move_in_direction_____________")

        FORCE_THRESHOLD = 30

        current_pose = self.move_group.get_current_pose().pose
        is_calculate = True
        if reuse_dict == None:
            is_calculate = True
        elif reuse_dict['choice'] == 'save':
            is_calculate = True
        else:
            is_calculate = False
        is_calculate = True

        if is_calculate == True:
            if reuse_dict != None and reuse_dict['choice'] == 'reversed':
                 # 初始化变量
                wpose = copy.deepcopy(current_pose)
                wpose.position.x -= axis[0] * 0.20  # 直线移动 0.25 米
                wpose.position.y -= axis[1] * 0.20
                wpose.position.z -= axis[2] * 0.20

                result = self.cartesian_path_with_force(wpose, force_threshold=FORCE_THRESHOLD, avoid_collision=False, singular_threshold=0.10)
                return result == 1
            else:
                wpose = copy.deepcopy(current_pose)
                wpose.position.x += axis[0] * distance
                wpose.position.y += axis[1] * distance
                wpose.position.z += axis[2] * distance
                result = self.cartesian_path_without_force(wpose, avoid_collision=False)
                return result
        else:
            object_name = reuse_dict['object_name']
            choice = reuse_dict['choice']
            if choice == 'save':
                object_pos_dict = {'initial_pose': current_pose, 'final_pose': target_pose}
                self.pos_dict_for_reuse[object_name] = object_pos_dict
            elif choice == 'forward':
                object_pos_dict = self.pos_dict_for_reuse[object_name]
                target_pose = object_pos_dict['final_pose']
            elif choice == 'reversed':
                print("--------------------------reversed--------------------------")
                return True
          
            elif choice == 'averaged':
                object_pos_dict = self.pos_dict_for_reuse[object_name]
                target_pose.position.x = (object_pos_dict['initial_pose'].position.x  + object_pos_dict['final_pose'].position.x) / 2
                target_pose.position.y = (object_pos_dict['initial_pose'].position.y  + object_pos_dict['final_pose'].position.y) / 2
                target_pose.position.z = (object_pos_dict['initial_pose'].position.z  + object_pos_dict['final_pose'].position.z) / 2
            else:
                raise NotImplementedError

    # def move_in_direction(self, axis: np.array, distance: float, reuse_dict: dict = None):
    #     """
    #     reuse_dict = {'object_name', 'choice'}
    #         - object_name: the object being operated
    #         - choice includes 'save', 'forward', 'reversed', 'averaged'
    #             - save: save the initial pose and final pose of the {object_name} for reuse, such as save the position when opening the drawer
    #             - forward: reuse the saved position of {object_name} in forward direction, setting the initial and final poses as the saved initial and final poses, respectively
    #             - reversed: reuse the saved position of {object_name} in reversed direction, setting the initial and final poses as the saved final and initial poses, respectively
    #             - averaged: reuse the averaged saved position of {object_name}, setting the final poses as the averaged pose of the saved final and initial poses
    #     """
    #     print("___________move_in_direction_____________")

    #     waypoints = []
    #     current_pose = self.move_group.get_current_pose().pose
    #     is_calculate = True
    #     if reuse_dict == None:
    #         is_calculate = True
    #     elif reuse_dict['choice'] == 'save':
    #         is_calculate = True
    #     else:
    #         is_calculate = False
    #     is_calculate = True
    #     if is_calculate == True:
    #         if reuse_dict != None and reuse_dict['choice'] == 'reversed':
    #              # 初始化变量
    #             FORCE_THRESHOLD = 20
    #             from geometry_msgs.msg import WrenchStamped
    #             rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, self.franka_state_callback)
    #             import time
    #             time.sleep(1)

    #             # 获取当前位姿
    #             current_pose = self.move_group.get_current_pose().pose
    #             initial_force_x = self.current_force_x
    #             initial_force_y = self.current_force_y
    #             initial_force_z = self.current_force_z
    #             while initial_force_x == 0:
    #                 initial_force_x = self.current_force_x
    #                 initial_force_y = self.current_force_y
    #                 initial_force_z = self.current_force_z
    #                 # print(self.current_force)
    #                 print ("waiting for initial force")
    #                 import time
    #                 time.sleep(5)
    #             # 设置笛卡尔路径点
    #             waypoints = []
    #             wpose = current_pose
    #             wpose.position.x = current_pose.position.x - 0.25 * axis[0]  # 直线移动 0.3 米
    #             wpose.position.y = current_pose.position.y - 0.25 * axis[1]
    #             wpose.position.z = current_pose.position.z - 0.25 * axis[2]
    #             waypoints.append(copy.deepcopy(wpose))

    #             # 计算笛卡尔路径
    #             (plan, fraction) = self.move_group.compute_cartesian_path(
    #                 waypoints, 0.001, 0.0 ,True  # waypoints to follow  # eef_step
    #             )  # jump_threshold


    #             # 执行计划
    #             if fraction>0.2:
    #                 self.move_group.execute(plan, wait=True)
    #                 print("--------------------------begin--------------------------")
    #             else:
    #                 rospy.logwarn(f"MoveitEnv: Could not plan cartesian_path to target pose \n{wpose}.\n Plan accuracy: {fraction}")

    #             # 循环检查力阈值和位置
    #             repeat = True
    #             while repeat:
    #                 repeat = False
    #                 flag_follow = True
    #                 self.move_group.execute(plan, wait=False)
    #                 start_time = time.time()
    #                 while not rospy.is_shutdown() and (flag_follow):
    #                     current_time = time.time()
    #                     vector = np.array([self.current_force_x, self.current_force_y, self.current_force_z])
    #                     length = np.linalg.norm(vector)
    #                     current_position = self.move_group.get_current_pose().pose.position 
    #                     current_pos_array = np.array([current_position.x, current_position.y, current_position.z])
    #                     waypoint_pos_array = np.array([wpose.position.x, wpose.position.y, wpose.position.z])
    #                     print(length)
    #                     if length > FORCE_THRESHOLD:
    #                         print("grasp检测到力超过阈值")
    #                         flag_follow = False
    #                     # elif self.move_group.get_current_pose().pose.position.x >= approach_pose.position.x:
    #                     elif np.allclose(current_pos_array, waypoint_pos_array, rtol=1e-05, atol=1e-08):
    #                         print("达到目标位置")
    #                         flag_follow = False
    #                     elif self.current_force_x == 0 and self.current_force_y  == 0 and self.current_force_z == 0:
    #                         print("奇异值")
    #                         flag_follow = False    
    #                     elif current_time - start_time > 5:
    #                         print("over time")
    #                         flag_follow = False
    #                         repeat = True
    #             # 清除目标并关闭 MoveIt
    #             self.move_group.stop()
    #             self.move_group.clear_pose_targets()
    #             return True
                      
    #         else:
    #             target_pose = copy.deepcopy(current_pose)
    #             target_pose.position.x += axis[0] * distance
    #             target_pose.position.y += axis[1] * distance
    #             target_pose.position.z += axis[2] * distance
    #             print('self.pos_dict_for_reuse', self.pos_dict_for_reuse)
           
    #     else:
    #         print("else")
    #         object_name = reuse_dict['object_name']
    #         choice = reuse_dict['choice']
    #         if choice == 'save':
    #             object_pos_dict = {'initial_pose': current_pose, 'final_pose': target_pose}
    #             self.pos_dict_for_reuse[object_name] = object_pos_dict
    #         elif choice == 'forward':
    #             object_pos_dict = self.pos_dict_for_reuse[object_name]
    #             target_pose = object_pos_dict['final_pose']
    #         elif choice == 'reversed':
    #             print("--------------------------reversed--------------------------")
    #             return True
          
    #         elif choice == 'averaged':
    #             object_pos_dict = self.pos_dict_for_reuse[object_name]
    #             target_pose.position.x = (object_pos_dict['initial_pose'].position.x  + object_pos_dict['final_pose'].position.x) / 2
    #             target_pose.position.y = (object_pos_dict['initial_pose'].position.y  + object_pos_dict['final_pose'].position.y) / 2
    #             target_pose.position.z = (object_pos_dict['initial_pose'].position.z  + object_pos_dict['final_pose'].position.z) / 2
    #         else:
    #             raise NotImplementedError
    #     waypoints.append(target_pose)
    #     (plan, fraction) = self.move_group.compute_cartesian_path(
    #         waypoints, 0.001, 0.0 ,False  # waypoints to follow  # eef_step
    #     )  # jump_threshold
    #     if fraction > 0.2:
    #         self.move_group.execute(plan, wait=True)
    #     else:
    #         print("unable to plan", waypoints,fraction)
    #     print("done")
    
    def figure_direction(self, beginning: np.array = None, ending: np.array = None):
        if beginning is not None and ending is not None:
            vector = -self.get_object_center_position(beginning) + np.array(ending)
            direction = vector / np.linalg.norm(vector)
        elif 'handle_0' in self.scene.object_names:
            bbox_parall = self.get_handle_xy_parall_box("handle_0")
            direction, drawer_plane_model = self.get_part_neighbor_normal(bbox_parall)
            # direction = -direction

        #### visualize ####
        vec_starts = []
        vecs = []
        vec_starts.append(bbox_parall.get_center())
        vecs.append(direction)
        self.scene.visualize_vec(vec_starts, vecs, True)
        # 显示与xy平面平行的框用于调试
        vis_list = []
        vis_list.append(self.scene.scene_tsdf_full.get_cloud())
        vis_list.append(bbox_parall)
        # o3d.visualization.draw_geometries(vis_list)

        return direction
    
    def follow_path(self, path, times=0, angle=None, group=None, step_size=0.1, jump_threshold=0.0, avoid_collisions=True):
        FORCE_THRESHOLD = 30
        TIME_THRESHOLD = 3
        """Execute a given plan."""
        if group is None:
            group = self.move_group
        if angle is None:
            angle = self.angle
        times = times + 1
        joint_axis = self.joint_axis
        joint_position = self.joint_position
        handle_position = self.handle_position
        success = False
        door_initial_direction = np.linalg.norm(joint_position - handle_position)
        initial_position = self.move_group.get_current_pose().pose.position 
        for i, wpoints in enumerate(path):
            result, force_direction = self.cartesian_path_with_force(wpoints, force_threshold=FORCE_THRESHOLD, time_threshold=TIME_THRESHOLD, return_f_direction=True, avoid_collision=False, rtol=1e-1, atol=1e-2)
            if result == 1:
                success = True
            elif result == -1:
                current_position = self.move_group.get_current_pose().pose.position 
                joint_axis = self.joint_axis
                joint_position = self.joint_position
                handle_position = self.handle_position
                door_direction = joint_position - handle_position
                v1 = np.array([initial_position.x - joint_position[0], initial_position.y - joint_position[1]])
                v2 = np.array([current_position.x - joint_position[0], current_position.y - joint_position[1]])
                # 计算两个向量的点积
                dot_product = np.dot(v1, v2)
                # 计算两个向量的范数
                norm_v1 = np.linalg.norm(v1)
                norm_v2 = np.linalg.norm(v2)
                # 计算夹角的余弦值
                cos_theta = dot_product / (norm_v1 * norm_v2)
                # 使用arccos函数计算夹角（以弧度为单位）
                theta_rad = np.arccos(cos_theta)
                # 将弧度转换为度
                theta_deg = np.degrees(theta_rad)
                angle_target = angle-theta_deg
                if np.dot(force_direction, door_direction) > 0:
                    # 计算当前已经经过的角度
                    joint_position = joint_position + door_direction*door_initial_direction*0.02*(0.9**times)
                    motion_plan = self.generate_arc_path_around_joint(joint_axis=joint_axis, joint_position=joint_position, n=int((angle-theta_deg)//20)+1, angle=angle_target)
                    success = self.follow_path(motion_plan, times=times, angle=angle_target)
                    return success
                if np.dot(force_direction, door_direction) < 0:
                    joint_position = joint_position - door_direction*door_initial_direction*0.02*(0.9**times)
                    motion_plan = self.generate_arc_path_around_joint(joint_axis=joint_axis, joint_position=joint_position, n=int((angle-theta_deg)//20)+1, angle=angle_target)
                    success = self.follow_path(motion_plan, times=times, angle=angle_target)
                    return success
        return success


    # # print("go to",wpoints)
    #         (plan, fraction) = group.compute_cartesian_path([wpoints], step_size, jump_threshold, avoid_collisions=avoid_collisions)
    #         self.move_group.execute(plan, wait=False)
    #         flag_follow = True
    #         start_time = time.time()
            
    #         while (not rospy.is_shutdown()) and (flag_follow):
    #             vector = np.array([self.current_force_x, self.current_force_y, self.current_force_z])
    #             magnitude = np.linalg.norm(vector)
    #             direction = vector / magnitude if magnitude != 0 else np.zeros_like(vector)
    #             current_position = self.move_group.get_current_pose().pose.position 
    #             current_pos_array = np.array([current_position.x, current_position.y, current_position.z])
    #             waypoint_position = wpoints.position
    #             waypoint_pos_array = np.array([waypoint_position.x, waypoint_position.y, waypoint_position.z])
    #             current_time = time.time()
                
    #             if np.allclose(current_pos_array, waypoint_pos_array, rtol=1e-05, atol=1e-08):
    #                 print("达到目标位置!")
    #                 flag_follow = False
    #             elif self.current_force_x == 0 and self.current_force_y  == 0 and self.current_force_z == 0:
    #                 print("机械臂状态奇异")
    #                 flag_follow = False 
    #             elif magnitude > FORCE_THRESHOLD:
    #                 print("检测到力超过阈值")
    #                 # print("magnitude",magnitude)
    #                 self.move_group.stop()
    #                 self.move_group.clear_pose_targets()
    #                 flag_follow = False
    #                 # 计算门当前向量方向
    #                 joint_axis = self.joint_axis
    #                 joint_position = self.joint_position
    #                 handle_position = self.handle_position
    #                 door_direction = joint_position - handle_position
    #                 v1 = np.array([initial_position.x - joint_position[0], initial_position.y - joint_position[1]])
    #                 v2 = np.array([current_position.x - joint_position[0], current_position.y - joint_position[1]])
    #                 # 计算两个向量的点积
    #                 dot_product = np.dot(v1, v2)
    #                 # 计算两个向量的范数
    #                 norm_v1 = np.linalg.norm(v1)
    #                 norm_v2 = np.linalg.norm(v2)
    #                 # 计算夹角的余弦值
    #                 cos_theta = dot_product / (norm_v1 * norm_v2)
    #                 # 使用arccos函数计算夹角（以弧度为单位）
    #                 theta_rad = np.arccos(cos_theta)
    #                 # 将弧度转换为度
    #                 theta_deg = np.degrees(theta_rad)
    #                 angle_target = angle-theta_deg
    #                 print("angle_target", angle_target)
    #                 if np.dot(direction,door_direction) > 0:
    #                     # 计算当前已经经过的角度
    #                     joint_position = joint_position + door_direction*door_initial_direction*0.02*(0.9**times)
    #                     motion_plan = self.generate_arc_path_around_joint(current_pose=self.get_gripper_pose(),joint_axis=joint_axis, joint_position=joint_position, n=int((angle-theta_deg)//30)+1, angle=angle_target)
    #                     self.follow_path(motion_plan, times=times, angle=angle_target)
    #                     return 
    #                 if np.dot(direction,door_direction) < 0:
    #                     joint_position = joint_position - door_direction*door_initial_direction*0.02*(0.9**times)
    #                     motion_plan = self.generate_arc_path_around_joint(current_pose=self.get_gripper_pose(),joint_axis=joint_axis, joint_position=joint_position, n=int((angle-theta_deg)//30)+1, angle=angle_target)
    #                     self.follow_path(motion_plan, times=times, angle=angle_target)
    #                     return 
    #             elif current_time - start_time > 3:
    #                 flag_follow = False
    #                 i = i - 1
    #         i = i + 1

    ### LHY begin 判断把手是否垂直、获取xy平行bbox、提取法向量、提取转轴函数定义
    def is_handle_vertical(self, handle_bbox: o3d.geometry.OrientedBoundingBox) -> bool:
        """根据包围框的长轴方向与z轴的夹角判断是不是竖直的把手

        Args:
            handle_bbox (o3d.geometry.OrientedBoundingBox): 把手的包围框

        Returns:
            bool: 是竖直把手则为true
        """
        rot_mat = handle_bbox.R
        bbox_axis_x = rot_mat[:, 0]
        # 通过与z轴的夹角判断是水平的还是垂直的handle
        world_axis_z = np.array([0, 0, 1]).reshape(3, 1)
        angle = math.degrees(
            math.acos(
                np.dot(bbox_axis_x, world_axis_z)[0]
                / (np.linalg.norm(bbox_axis_x) * np.linalg.norm(world_axis_z))
            )
        )
        is_vert = False
        if angle < 45 or angle > 135:
            is_vert = True
        self.is_drawer_handle_vertical = is_vert
        return is_vert
    
    def get_handle_xy_parall_box(self,
        object_name: str
    ) -> o3d.geometry.OrientedBoundingBox:
        """获取一个原bbox的一个bbox，新的bbox有一面与xy平面平行

        Args:
            object_name (str): 把手的名称，用于查询3d bbox

        Returns:
            o3d.geometry.OrientedBoundingBox: 输出bbox
        """
        handle_bbox = self.scene.bbox_oriented_3d_dict[object_name]
        is_vert = self.is_drawer_handle_vertical
        # 获取在x、y平面与原bbox类似，但是长轴在xy平面，且有一面与xy平面平行的包围盒
        rot_mat = handle_bbox.R
        # bbox的R三个列对应长宽高的三个方向，extent对应长宽高的三个大小，长宽高从大到小
        if not is_vert:
            # 水平把手，原框的x轴近似水平
            # 新框的x轴用原来的x轴在世界xy平面上的投影，新框的xy轴用世界z轴
            new_axis_x = rot_mat[:, 0].copy()
            new_axis_x[2] = 0.0
            new_axis_x /= np.linalg.norm(new_axis_x)
            new_axis_y = np.array([0.0, 0.0, 1.0])
            new_axis_z = np.cross(new_axis_x, new_axis_y)
        else:
            # 竖直把手，原框的x轴近似竖直
            # 新框的x轴用世界z轴，新框的y轴用原来的y轴在世界xy平面上的投影
            new_axis_x = np.array([0.0, 0.0, 1.0])
            new_axis_y = rot_mat[:, 1].copy()
            new_axis_y[2] = 0.0
            new_axis_y /= np.linalg.norm(new_axis_y)
            new_axis_z = np.cross(new_axis_x, new_axis_y)
        new_rot_mat = np.vstack((new_axis_x, new_axis_y, new_axis_z.squeeze())).T
        new_extent = np.array([0.0, 0.0, 0.0])
        # 原各角点在新的轴上的投影最大值，加倍后作为新的长宽高的
        # 原bbox的角点与中心的差
        vertexes = np.asarray(handle_bbox.get_box_points()) - handle_bbox.get_center()
        for vertex in vertexes:
            for i in range(3):
                projection = abs(np.dot(vertex, new_rot_mat[:, i]))
                if projection > new_extent[i]:
                    new_extent[i] = projection
        new_extent *= 2
        # obox默认extent大小顺序为长宽高，如果新的extent的大小顺序不是这样，需要调整
        if new_extent[1] < new_extent[2]:
            new_extent[1], new_extent[2] = new_extent[2], new_extent[1]
            new_rot_mat[:, 1], new_rot_mat[:, 2] = new_rot_mat[:, 2], new_rot_mat[:, 1]
        # 使用新的extent、R重新构造bbox，新的extent的大小顺序不再固定
        bbox_parall = o3d.geometry.OrientedBoundingBox(handle_bbox)
        bbox_parall.extent = new_extent
        bbox_parall.R = new_rot_mat
        return bbox_parall

    def get_part_neighbor_normal(self,
        part_bbox: o3d.geometry.OrientedBoundingBox,
        scale_xy: float = 1.5,
        scale_z: float = 2.5,
    ) -> (np.ndarray, np.ndarray):
        """提取物体附近平面的法向量

        Args:
            part_bbox (o3d.geometry.OrientedBoundingBox): 局部物体的与xy平面平行的包围框
            scale_xy (float, optional): 搜索邻域的包围框沿世界xy方向的扩大倍数. Defaults to [1.3, 1.0, 2.0].
            scale_z (float, optional): 搜索邻域的包围框沿世界z方向的扩大倍数. Defaults to [1.3, 1.0, 2.0].
            is_vert (bool, optional): 是否是竖直把手，如果是竖直的则之扩大z方向. Defaults to False.

        Returns:
            np.ndarray: 法向量
            np.ndarray: 平面模型
        """
        pc_scene = self.scene.scene_tsdf_full.get_cloud()
        # 扩大包围盒，中心不变，长宽高均乘scale
        bbox_scaled = o3d.geometry.OrientedBoundingBox(part_bbox)
        # 找到与世界z轴平行的轴，并把该方向的extent扩大
        z_idx = 0
        max_z_proj = 0
        for idx in range(3):
            proj = np.dot(bbox_scaled.R[:, idx], np.array([0.0, 0.0, 1.0]))
            if proj > max_z_proj:
                z_idx = idx
                max_z_proj = proj
        new_extent = bbox_scaled.extent.copy()
        new_extent[z_idx] *= scale_z
        for idx in range(3):
            if idx != z_idx:
                new_extent[idx] *= scale_xy
        bbox_scaled.extent = new_extent
        # 提取包围盒内的点云
        pc_neighbor = pc_scene.crop(bbox_scaled)
        # 去除把手部分
        outlier_indices = part_bbox.get_point_indices_within_bounding_box(
            pc_neighbor.points
        )
        pc_remain = pc_neighbor.select_by_index(outlier_indices, invert=True)
        # 提取平面
        plane_model, _ = pc_remain.segment_plane(
            distance_threshold=0.01, ransac_n=3, num_iterations=10
        )
        # 如果plane_model不是朝向机械臂（x分量为负），则反个向
        normal = -plane_model[:3] if plane_model[0] > 0 else plane_model[:3]
        #################CGY##############
        normal[2] = 0
        #################CGY##############
        normal = np.asarray(normal)
        normal /= np.linalg.norm(normal)
        # 返回平面的法向量
        self.drawer_normal = normal
        return normal, plane_model

    def get_rotation_axis(self,
        handle_bbox: o3d.geometry.OrientedBoundingBox,
        plane_model: np.ndarray,
        xy_extent: float = 0.7,
    ) -> (np.ndarray, int, list):
        """在竖直包围框的条件下，通过沿世界x、y方向扩展包围框提取平面块，并通过比对平面块与之前的邻域平面以及平面块的面积决定那一块是抽屉面，最后得到转轴的大致x、y坐标

        Args:
            handle_bbox (o3d.geometry.OrientedBoundingBox): 握把的与xy平面平行的包围框
            plane_model (np.ndarray): 先前通过邻域判断的平面方程系数
            xy_extent (float, optional): xy方向包围框扩大后的长度，单位是m. Defaults to 0.7.

        Returns:
            np.ndarray: 转轴在世界xy平面的二维坐标
            int: 开门的旋转方向，-1是逆时针，1是顺时针
            list: 把手附近的平面的bbox
        """
        pc_scene = self.scene.scene_tsdf_full.get_cloud()
        hbbox_tmp = o3d.geometry.OrientedBoundingBox(handle_bbox)
        new_extent = hbbox_tmp.extent.copy()
        # new_extent[1], new_extent[2] = xy_extent, xy_extent
        # 找到z轴
        z_idx = 0
        max_z_proj = 0
        for idx in range(3):
            proj = np.dot(hbbox_tmp.R[:, idx], np.array([0.0, 0.0, 1.0]))
            if proj > max_z_proj:
                z_idx = idx
                max_z_proj = proj
        for idx in range(3):
            if idx != z_idx:
                new_extent[idx]= xy_extent
        hbbox_tmp.extent = new_extent
        pc_neighbor = pc_scene.crop(hbbox_tmp)
        # 去除把手部分
        outlier_indices = handle_bbox.get_point_indices_within_bounding_box(
            pc_neighbor.points
        )
        pc_remain = pc_neighbor.select_by_index(outlier_indices, invert=True)
        pc_remain.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=10))
        plane_oboxes = pc_remain.detect_planar_patches()
        # 两个条件判断是不是原来的平面，obox中心与plane_model小于阈值，法向量与plane_model夹角小于阈值
        drawer_oboxes = []
        plane_norm = plane_model[:3] / np.linalg.norm(plane_model[:3])
        dist_thr = 0.1
        angle_thr = 10
        for obox in plane_oboxes:
            center = obox.get_center()
            distance = abs(
                plane_model[0] * center[0]
                + plane_model[1] * center[1]
                + plane_model[2] * center[2]
                + plane_model[3]
            ) / np.sqrt(plane_model[0] ** 2 + plane_model[1] ** 2 + plane_model[2] ** 2)
            angle = math.degrees(math.acos(np.dot(plane_norm, obox.R[:, 2])))
            if distance < dist_thr and (angle < angle_thr or angle > 180 - angle_thr):
                drawer_oboxes.append(obox)
        if len(drawer_oboxes) == 0:
            return None, None, None
        # 在法向量接近的平面中找出面积最大的一个
        obox_drawer = None
        max_plane_size = 0
        for obox in drawer_oboxes:
            plane_size = obox.extent[0] * obox.extent[1]
            if plane_size > max_plane_size:
                max_plane_size = plane_size
                obox_drawer = obox
        self.drawer_panel_center = obox_drawer.get_center()
        # 获取长宽中比较水平的那一个
        plane_axises = obox_drawer.R[:, :2]  # 长宽方向向量，高是最小的在最后
        z_axises = np.array([[0, 0, 1], [0, 0, 1]])
        parall_axis_idx = np.argmin(abs((z_axises @ plane_axises).squeeze()))
        parall_axis = plane_axises[:, parall_axis_idx].squeeze()
        # 确保水平方向向量方向接近把手中心指向柜门中心
        handle2plane = obox_drawer.get_center() - hbbox_tmp.get_center()
        if np.dot(handle2plane, parall_axis) < 0:
            parall_axis = -parall_axis
        axis_coord = obox_drawer.get_center() + parall_axis * (
            obox_drawer.extent[parall_axis_idx] / 2 - self.drawer_rotate_axis_offset
        )
        self.drawer_rotate_axis_xy = axis_coord[:2]
        axis_dir = 1
        if axis_coord[1] > handle_bbox.get_center()[1]:  # 如果转轴在把手的左侧，则顺时针开门
            axis_dir = -1
        self.drawer_rotate_axis_dir = axis_dir
        return axis_coord[:2], axis_dir, plane_oboxes
    
    def get_object_joint_info(self, object_name: str, type: str='revolute')->Dict:
        """
        Get the joint axis closest to the given axis.
        Args:
            object_name: name of the object
            position: np.ndarray, select the joint closest to this position
            type: str, allowed type of the joint, "any", "revolute", "prismatic"
        Returns:
            closest_axis: Dict, the closest joint axis
                {
                    "joint_position":[
                        0.0,
                        0.0,
                        0.0
                    ],
                    "joint_axis": [
                        -1.0,
                        -8.511809568290118e-08,
                        -1.677630052654422e-07
                    ],
                    "type": "prismatic"
                }
        """
        
        if type == "any":
            joint_types = ["revolute", "prismatic"]
        elif type in ["revolute", "prismatic"]:
            joint_types = [type]
        else:
            raise ValueError("Error in get_object_joint_info: Invalid type: {}".format(type))
        
        # get joints axes from joint prediction model
        handle_position = self.get_object_center_position(object_name)

        bbox_parall = self.get_handle_xy_parall_box(object_name)
        normal, drawer_plane_model = self.get_part_neighbor_normal(bbox_parall)
        axis_coord, axis_dir, host_plane_oboxes = self.get_rotation_axis(bbox_parall, drawer_plane_model)

        joint_position = np.array([axis_coord[0], axis_coord[1], handle_position[2]])

        joint_axis = np.array([0, 0, axis_dir])

        closest_axis = {
            "joint_position": joint_position,
            "joint_axis": joint_axis,
            "type": "revolute"
        }

        self.handle_position=handle_position
        self.joint_position=joint_position
        self.joint_axis=joint_axis

        #### visualize ####
        # # 显示提取抽屉面时扩大包围框内的平面
        vis_list = []
        if host_plane_oboxes is not None:
            for obox in host_plane_oboxes:
                mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(
                    obox, scale=[1, 1, 0.0001]
                )
                mesh.paint_uniform_color(obox.color)
                vis_list.append(mesh)
                vis_list.append(obox)
        o3d.visualization.draw_geometries(vis_list)
        # 显示法向量和轴
        vec_starts = [bbox_parall.get_center(), joint_position]
        vecs = [normal, joint_axis]
        self.scene.visualize_vec(vec_starts, vecs, True)
        # 显示与xy平面平行的框用于调试
        vis_list = []
        vis_list.append(self.scene.scene_tsdf_full.get_cloud())
        vis_list.append(bbox_parall)
        o3d.visualization.draw_geometries(vis_list)

        return closest_axis
    
    def generate_arc_path_around_joint(self, object_name=None, joint_axis=None, joint_position=None, n=3, angle=60):
        """
        Generate an arc path around a joint in space using ROS geometry_msgs.msg.Pose.

        :param current_pose: geometry_msgs.msg.Pose, the current pose of the end effector
        :param joint_axis: np.ndarray, a 3D unit vector representing the joint's axis
        :param joint_position: np.ndarray, the 3D position of the joint in space
        :param n: int, the number of intermediate poses to generate along the arc
        :param angle: float, the total angle of the arc in degrees
        :return: List[geometry_msgs.msg.Pose], a list of Pose messages representing the arc path
        """
        import math

        print("Generating Path...")
        self.derive = n
        self.angle = angle
        if object_name is not None:
            self.arc_path_object_name = object_name
        else:
            object_name = self.arc_path_object_name
        if joint_axis is None or joint_position is None:
            joint_info = self.get_object_joint_info(object_name=object_name)
            joint_position = joint_info["joint_position"]
            joint_axis = joint_info["joint_axis"]
            print("joint_axis",joint_axis)
            print("joint_position",joint_position)

        # Convert angle from degrees to radians
        current_pose = self.get_gripper_pose()
        angle_rad = np.deg2rad(angle)
        
        # Calculate the step angle for each intermediate pose
        step_angle = angle_rad / n

        # Generate the arc path
        arc_path = []
        for i in range(n + 1):
            # Calculate the rotation for the current step
            rotation = R.from_rotvec(joint_axis * step_angle * i)
            
            # Calculate the new position by applying the rotation to the vector from the joint to the current position
            position_vector = np.array([current_pose.position.x, current_pose.position.y, current_pose.position.z]) - joint_position
            new_position_vector = joint_position + rotation.apply(position_vector)
            
            # Calculate the new orientation by applying the rotation to the current orientation
            current_orientation = R.from_quat([current_pose.orientation.x,
                                            current_pose.orientation.y,
                                            current_pose.orientation.z,
                                            current_pose.orientation.w])
            new_orientation = rotation * current_orientation
            # new_orientation = current_orientation
            
            # Create a new Pose message for the current step
            new_pose_msg = geometry_msgs.msg.Pose()
            new_pose_msg.position = geometry_msgs.msg.Point(x=new_position_vector[0], y=new_position_vector[1], z=new_position_vector[2])
            new_orientation_quat = new_orientation.as_quat()
            new_pose_msg.orientation = geometry_msgs.msg.Quaternion(x=new_orientation_quat[0],
                                                                    y=new_orientation_quat[1],
                                                                    z=new_orientation_quat[2],
                                                                    w=new_orientation_quat[3])
            arc_path.append(new_pose_msg)
        return arc_path

    def check(
        self,
        object_name,
        task_query
    ):
        """
        move to the corresponding camera pos of the <object_name>, 
        then verify whether the execution is successful
        TODO: should automatically generate the object_name
        """

        print('checking the state')

        if '_0' in task_query:
            task_query = task_query.replace('_0', '')

        self.detect_objects_with_moving()

        ############################ check ##########################
        # camera_idx = self.scene.get_object_camera_idx(object_name)

        # self.move_joints_to(self.scene.pos_list[int(camera_idx)])

        # sensor_data = self.get_sensor_data()

        # rgb_image = sensor_data['rgb_image_list'][0]
        # rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)

        # judge, explanation = gpt_check(rgb_image,task_query)

        # print(f'judge: {judge}, explanation: {explanation}')

        # if judge != True:
        #     self.generate_exec_code(self.obj_name_list, task_query, cfg_tabletop)
        #     self.execute_code()
        ############################ check ##########################
                

        return True
                
    

    
    