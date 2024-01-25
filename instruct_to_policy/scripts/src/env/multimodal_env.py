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
from src.joint_prediction import JointPredictionBase, JointPredictionGT
from src.plane_detection import PlaneDetectionOpen3D
from src.utils import has_keywords
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
        self.use_gt_2d_detections = cfg["perception"]["use_gt_2d_detections"]
        self.use_gt_3d_bboxes = cfg["perception"]["use_gt_3d_bboxes"]
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
        # for true grounding env, always use ground truth joint prediction model
        self.joint_prediction_model = JointPredictionGT()
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
        
        # Inject ground truth 3d bboxes if use_gt_2d_detections is set 
        
        if self.use_gt_2d_detections or self.use_gt_3d_bboxes:
            gt_bbox_3d_dict = {}
            gazebo_gt_bboxes_3d = self.get_gt_bboxes()
            for object_name in object_list:
                if object_name in gazebo_gt_bboxes_3d:
                    if "." in object_name: # the object is already a part of a model
                        object_id = object_name
                    else:
                        object_id = f"{object_name}_0"
                    # FIXME: consider the case when there are multiple objects with the same name
                    gt_bbox_3d_dict[object_id] = gazebo_gt_bboxes_3d[object_name]
             
            # add the ground truth 3d bboxes to sensor data
            sensor_data['bbox_3d_dict'] = gt_bbox_3d_dict
        
            # check if gt grounding model is used 
            assert self.grounding_model_name == "ground_truth"
        
        detections_list = self.grounding_model.query_2d_bbox_list(
            sensor_data=sensor_data,
            object_list=object_list
        )
        # update scene 
        data = {
            'detections_list': detections_list
        }
        data.update(sensor_data)
        self.scene.update(data, use_gt_3d_bboxes=self.use_gt_3d_bboxes)
        
        # update GT data if using gazebo
        if self.sim == 'gazebo':
            # update the ground truth 3D bounding boxes of all objects in scene manager
            self.update_scene_gt_bboxes_3d()
            # associate the detected objects with gazebo model names if using gazebo 
            self.associate_scene_objects_with_gazebo()
        
        if not self.use_gt_planning_scene:
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
     
    def get_object_joint_info(self, obj_name: str, position: np.ndarray, type="any")->Dict:
        """
        Get the joint axis closest to the given axis.
        # TODO: replace this GT function with external perception models
        Args:
            obj_name: name of the object
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
        
        # convert the object name to gazebo model name
        gazebo_obj_name = self.object_name_detect2gazebo[obj_name]
        
        # get joints axes from joint prediction model
        # FIXME: if the obj_name is part of the model, should only return the joints connecting to this part
        if '.' in gazebo_obj_name:
            gazebo_obj_name = gazebo_obj_name.split('.')[0]
        data = {
            "obj_name": gazebo_obj_name,
            "joint_types": joint_types,
        }
        joints_axes = self.joint_prediction_model.predict(data)
        
        # find the closest joint axis: the distance is between the position (point) and the line (joint axis)
        closest_axis = None
        closest_axis_dist = float('inf')
        for joint_axis in joints_axes:
            joint_position = np.array(joint_axis["joint_position"])
            joint_axis_vector = np.array(joint_axis["joint_axis"])
            
            # compute the distance between the point and the line
            dist = np.linalg.norm(np.cross(joint_axis_vector, position - joint_position)) / np.linalg.norm(joint_axis_vector)
            if dist < closest_axis_dist:
                closest_axis_dist = dist
                closest_axis = joint_axis
    
        return closest_axis   
        
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

    def attach_object(self, object_id, link=None):
        # if using gt planning scene, first convert the perception id to gazebo model name and then attach the object
        if self.use_gt_planning_scene:
            # get the gazebo model name of the object 
            gazebo_id = self.object_name_detect2gazebo[object_id]
        
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
            if gazebo_id in moveit_object_name:
                if object_id not in self.objects:
                    self.objects[object_id] = {}
                self.objects[object_id]["attach_name"] = moveit_object_name
                self.move_group.attach_object(moveit_object_name, link) 
                if self.verbose:
                    rospy.loginfo(f"Moveit: attached object object {object_id} to {link}")
                return True
        
        rospy.logerr(f"Moveit: object {object_id} not found in moveit planning scene. Planning scene objects: {moveit_object_names}")

    def detach_object(self, object_id):
        # if using gt planning scene, first convert the perception id to gazebo model name and then detach the object
        # if self.use_gt_planning_scene:
            # get the gazebo model name of the object 
            # gazebo_id = self.object_name_detect2gazebo[object_id]
        
        try:
            moveit_object_name = self.objects[object_id]["attach_name"]
            self.move_group.detach_object(moveit_object_name)
            
            if self.verbose:
                print(f"Moveit: detached object {object_id}")
        except:
            if self.verbose:
                rospy.logerr(f"Moveit: failed to detach object {object_id}")        

    ####################  Moveit planning related functions ####################
    def add_scene_objects_to_moveit(self, **kwargs):
        """Add all objects in the scene to the moveit planning scene."""
        # Add detected objects' meshes into moveit 
        for object_name in self.get_obj_name_list():
            object_mesh = self.scene.get_object_mesh(object_name)
            
            if object_name not in self.objects:
                self.objects[object_name] = {}
                
            # filter out full cabinet and table since their reconstructed mesh will lead to undesired collision in moveit 
            # e.g. "cabinet_0", "table_0"
            if object_name in ["cabinet_0", "table_0"]:
                continue
            self.register_object_mesh(object_mesh, object_name)

        # clear octomap to resolve octomap update bugs
        self.clear_octomap()

    
    