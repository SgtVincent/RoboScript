import os 
from typing import List, Tuple, Dict
import numpy as np 
import open3d as o3d

from .utils import CameraIntrinsic, Transform, open3d_frustum_filter
from .scalable_tsdf import ScalableTSDFVolume

class SceneManager:
    """
    Manage the scene object names, 3D representations, detections result, etc. 
    """
    def __init__(self, **kwargs) -> None:
        
        self.tsdf_voxel_size = kwargs.get('tsdf_voxel_size', 0.01)
        self.tsdf_color_type = kwargs.get('tsdf_color_type', "rgb")
        self.tsdf_trunc_ratio = kwargs.get('tsdf_trunc_ratio', 4.0)
        self.tsdf_max_depth = kwargs.get('tsdf_max_depth', 3.0)
        
        self.camera_names = []
        self.camera_intrinsics = []
        self.camera_extrinsics = []
        self.object_names = []
        self.detections_dict = {}
        self.bbox_3d_dict = {}
        
        self.scene_tsdf = ScalableTSDFVolume( 
            voxel_size=self.tsdf_voxel_size, 
            color_type=self.tsdf_color_type,
            trunc_ratio=self.tsdf_trunc_ratio
        )
        
    def _update_3d_bounding_boxes(self):
        """
        With tsdf and multi-view 2D detections, calculate object 3D bounding boxes.
        """
                # assume only one object detection in perception message
        bbox_list = []

        full_cloud: o3d.geometry.PointCloud = self.scene_tsdf.get_cloud()
        
        for object_name, bbox_list in self.detections_dict.items():
        
            # use 2D bboxes to filter the cloud and get the 3D bbox
            tight_cloud, mask = open3d_frustum_filter(full_cloud, 
                                                bbox_list,
                                                self.camera_intrinsics,
                                                self.camera_extrinsics,
                                                margin=0)
            
            min_bound = tight_cloud.get_min_bound()
            max_bound = tight_cloud.get_max_bound()
            # TODO: check 3D bounding box format
            self.bbox_3d_dict[object_name] = (min_bound, max_bound)
        
    def update_fusion(self, data: Dict)->None:
        """
        Update the multi-view fusion result.
        Args:
            data: Dictionary of sensor data.
        """
        # TODO: update the multi-view fusion result
        self.camera_names = data['camera_names']
        self.camera_intrinsics = data['depth_camera_intrinsic_list']
        self.camera_extrinsics = data['depth_camera_extrinsic_list']
        
        # reset the tsdf volume and integrate the depth images
        self.scene_tsdf.reset()
        
        for i, name in enumerate(data['camera_names']):
            
            # convert sensor_msgs/Image
            rgb = data['rgb_image_list'][i]
            depth = data['depth_image_list'][i]
            
            # NOTE: assume the depth has been aligned with rgb
            assert rgb.shape[0] == depth.shape[0] and rgb.shape[1] == depth.shape[1]
            intrinsic = data['depth_camera_intrinsic_list'][i]
            extrinsic = data['depth_camera_extrinsic_list'][i]
                          
            self.scene_tsdf.integrate(depth, intrinsic, extrinsic, rgb_img=rgb)

        
    def update_detection(self, detections_dict: Dict[str, List[List[float]]]):
        """
        Update the detection result.
        
        Args: 
            detections_dict: a dictionary of detection results.
                key: object name
                value: a list of 2D bounding boxes for each camera view. Each bbox is a List [x_min, y_min, x_max, y_max]
        """
        self.object_names = detections_dict.keys()
        self.detections_dict = detections_dict
        self._update_3d_bounding_boxes()
    
    def get_object_names(self):
        """
        Get the list of object names in the scene.
        """
        return self.object_names
        
    def get_object_center_position(self, obj_name):
        """
        Get the position of the object in the world frame. 
        This function uses ground truth model state from gazebo and ignore all other parameters.
        """
        # TODO: finish this function
        pass
            
    def get_3d_bbox(self, object_name):
        """
        Get the 3D bounding box of the object in the world frame.
        """
        # TODO: finish this function 
        pass
        