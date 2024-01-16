import dis
from typing import List, Tuple, Dict
import rospy 
import json
import numpy as np 
import open3d as o3d 

from geometry_msgs.msg import Quaternion, Pose, Point
from .moveit_gazebo_env import MoveitGazeboEnv
from src.grasp_detection import create_grasp_model, GraspDetectionBase, GraspDetectionRemote
from src.grounding_model import create_grounding_model, GroundingBase
from src.joint_prediction import JointPredictionBase
from src.plane_detection import PlaneDetectionOpen3D
from src.env.utils import (
    calculate_place_position, 
    is_collision, 
    adjust_z, 
    pose_msg_to_matrix, 
    create_collision_object_from_open3d_mesh
)
from src.perception.scene_manager import SceneManager

class MultiModalEnv(MoveitGazeboEnv):
    """
    Simple grounding environment to use gazebo GT model state as observation, and GIGA for grasp pose prediction.
    """
    def __init__(self, cfg) -> None:
        super().__init__(cfg)
        # self.use_gt_perception = False 
        self.grasp_config = cfg["grasp_detection"]
        self.plane_detection_config = cfg["plane_detection"]
        
        
        self.grounding_config = cfg["grounding_model"]
        # use glip as default baseline 
        self.grounding_model_name = self.grounding_config.get("model_name", "glip")
        self.grounding_model_args = self.grounding_config.get("model_args", {})        
        
        self.grasp_model = None
        self.groudning_model = None 
        self.joint_prediction_model = None
        
        
        # scene manager to manage the scene objects, 3D reconstruction, detections, etc.
        self.scene_manager_args = cfg["perception"]["scene_manager"]
        self.scene = SceneManager(**self.scene_manager_args)

        # information for evaluation and GT ablation 
        self.scene_gt_bboxes_3d = {}
        self.object_name_detect2gazebo = {}
        
        self._init_models()
    
    def _init_models(self):
        """
        Initialze all models needed for the environment: grounding, grasp detection, etc.
        """
    
        self.grounding_model = create_grounding_model(self.grounding_model_name, **self.grounding_model_args)
        self.grasp_model = create_grasp_model(self.grasp_config)
        # TODO: implement joint prediction model
        self.joint_prediction_model = JointPredictionBase()
        self.plane_detection_model = PlaneDetectionOpen3D(model_params=self.plane_detection_config['model_params'])
        

    def detect_objects(self, *nargs, **kwargs):
        """
        Call the perception pipeline to detect all task-specific objects in the scene.
        
        Args: 
            object_list: list of objects to detect
            category_list: list of categories to detect, used as an alias for object_list
        """
        # get objects to detect
        object_list = nargs[0] if len(nargs) > 0 else kwargs.get('object_list', [])
        if len(object_list) == 0:
            object_list = kwargs.get('category_list', [])

        # call detection pipeline
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
        
        # update GT data if using gazebo
        if self.sim == 'gazebo':
            # update the ground truth 3D bounding boxes of all objects in scene manager
            self.update_scene_gt_bboxes_3d()
            # associate the detected objects with gazebo model names if using gazebo 
            self.associate_scene_objects_with_gazebo()
        
        # update objects in moveit planning scene 
        self.add_scene_objects_to_moveit()

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
        
    def get_3d_bbox(self, obj_name, **kwargs)->np.ndarray:
        """
        Get the bounding box of the object.
        Args:
            obj_name: name of the object
        Returns:
            bbox: np.ndarray, [x_min, y_min, z_min, x_max, y_max, z_max]
        """
        return self.scene.get_3d_bbox(obj_name)
    
    def get_plane_normal(self, obj_name: str, position: np.ndarray) -> np.ndarray:
        '''
        Get the plane normal of the object at the given position.
        Args:
            obj_name: name of the object
            position: position of the object
        Returns:
            normal: np.ndarray, [x, y, z]
        '''
        # First get the object point cloud from scene manager
        object_point_cloud = self.scene.get_object_cloud(obj_name)

        # Then calculate all planes with plane detection model
        normal_vectors, oboxes = self.plane_detection_model.detect_planes(object_point_cloud)
        
        # Calculate the distance from position to all planes (point-to-plane-distance)
        distances = []
        for i, obox in enumerate(oboxes):
            plane_center = obox.get_center()    
            normal_vector = normal_vectors[i]
            distance = np.abs(np.dot(normal_vector, plane_center - position))
            distances.append(distance)
        
        # Select the plane that is closest to the given position
        min_idx = np.argmin(distances)
        normal = normal_vectors[min_idx]
        return normal
        
    #################### Gazebo/ Ground truth related functions ############### 
    def update_scene_gt_bboxes_3d(self, min_points=10): 
        """
        Get the ground truth 3D bounding boxes of all objects in scene manager:
        - Get GT model bounding boxes from gazebo 
        - Get point cloud from scene manager 
        - Get all scene objects that has more than min_points points in the bounding box 
        """
        # get the ground truth model bounding boxes from gazebo
        gazebo_gt_bboxes_3d = self.get_gt_bboxes()
        # clear the scene objects gt bboxes
        self.scene_gt_bboxes_3d = {}
        
        # get the full point cloud from scene manager
        point_cloud_full: o3d.geometry.PointCloud = self.scene.scene_tsdf_full.get_cloud()
        
        for obj_name, gazebo_bbox in gazebo_gt_bboxes_3d.items():
            # get all scene objects that has more than min_points points in the bounding box 
            center = np.array(gazebo_bbox[0])
            size = np.array(gazebo_bbox[1])
            aabb = o3d.geometry.AxisAlignedBoundingBox(
                min_bound=center - size / 2,
                max_bound=center + size / 2
            )
            obj_point_cloud = point_cloud_full.crop(aabb)
            if len(obj_point_cloud.points) > min_points:
                # re-compute the bounding box of the object with the point cloud
                obj_bbox = obj_point_cloud.get_axis_aligned_bounding_box()
                self.scene_gt_bboxes_3d[obj_name] = obj_bbox
    
    def associate_scene_objects_with_gazebo(self):
        """
        Associate the detected objects with gazebo model names, used in evaluation and ablation study.
        
        Current method: Associate the detected object with the gazebo model name with closest distance between detected and GT bounding boxes.
        
        """
        # get all objects in the scene
        object_names = self.scene.get_object_names()
        # get the ground truth model bounding boxes from gazebo
        # NOTE: Do not use the full gazebo bboxes since it contains the area not detected by the perception pipeline
        # Use the bboxes bounding the point cloud in scene manager instead.
        # gazebo_gt_bboxes_3d = self.get_gt_bboxes()
        self.update_scene_gt_bboxes_3d()
        
        # associate the detected objects from scene with gazebo model names by getting its closest bounding box 
        # TODO: consider optimize this loop and add max distance threshold
        for obj_name in object_names:
            # get the bounding box of the detected object
            obj_bbox = self.scene.get_3d_bbox(obj_name)
            # find its closest gazebo model name
            min_dist = np.inf
            gazebo_name = None
            for gazebo_obj_name, gazebo_bbox in self.scene_gt_bboxes_3d.items():
                # compute the center distance 
                obj_bbox_center = (obj_bbox[:3] + obj_bbox[3:]) / 2.0
                gazebo_bbox_center = gazebo_bbox.get_center()
                dist = np.linalg.norm(obj_bbox_center - gazebo_bbox_center)
                if dist < min_dist:
                    min_dist = dist
                    gazebo_name = gazebo_obj_name
            # associate the detected object with gazebo model name
            self.object_name_detect2gazebo[obj_name] = gazebo_name
        

    ####################  Moveit planning related functions ####################
    def add_scene_objects_to_moveit(self, **kwargs):
        """Add all objects in the scene to the moveit planning scene."""
        # Add detected objects' meshes into moveit 
        for object_name in self.get_obj_name_list():
            object_mesh = self.scene.get_object_mesh(object_name)
            self.register_object_mesh(object_mesh, object_name)
            if object_name not in self.objects:
                self.objects[object_name] = {}
                
        # clear octomap to resolve octomap update bugs
        self.clear_octomap()

    
    