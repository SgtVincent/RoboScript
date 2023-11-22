import os 
from typing import List, Tuple, Dict
from numpy.typing import ArrayLike
import numpy as np 
import open3d as o3d

from .utils import CameraIntrinsic, Transform, open3d_frustum_filter
from .scalable_tsdf import ScalableTSDFVolume

class SceneManager:
    '''
    Manage the scene object names, 3D representations, detections result, etc. 
    '''
    def __init__(self, **kwargs) -> None:
        
        self.tsdf_voxel_size = kwargs.get('tsdf_voxel_size', 0.005)
        self.tsdf_color_type = kwargs.get('tsdf_color_type', "rgb")
        self.tsdf_trunc_ratio = kwargs.get('tsdf_trunc_ratio', 4.0)
        self.tsdf_max_depth = kwargs.get('tsdf_max_depth', 3.0)
        self.hdbscan_min_cluster_size = kwargs.get('hdbscan_min_cluster_size', 100)
        self.hdbscan_cluster_selection_epsilon = kwargs.get('hdbscan_cluster_selection_epsilon', 0.05) # 0.05m minimum distance between objects
        
        self.camera_names = []
        self.camera_intrinsics = []
        self.camera_extrinsics = []
        self.object_names = []
        self.detections_dict = {}
        self.bbox_3d_dict = {}
        
        # full tsdf volume
        self.scene_tsdf_full= ScalableTSDFVolume( 
            voxel_size=self.tsdf_voxel_size, 
            color_type=self.tsdf_color_type,
            trunc_ratio=self.tsdf_trunc_ratio
        )
        
        # tsdf volume masked by bounding boxes of objects
        self.scene_tsdf_masked = ScalableTSDFVolume(
            voxel_size=self.tsdf_voxel_size,
            color_type=self.tsdf_color_type,
            trunc_ratio=self.tsdf_trunc_ratio
        )
        
        
    def update(self, data: Dict, detections_list: List[Dict[ArrayLike]])->None:
        '''
        Update the scene manager with new sensor data and new 2D detections. 
        Args:
            data: Dictionary of sensor data.
            detections_list: a list of detection ditcionaries. 
                Each dictionary contains detection results from a single camera:
                    key: object name
                    value: a list of 2D bounding boxes for each camera view. Each bbox is a List [x_min, y_min, x_max, y_max]
        '''
        
        # TODO: add detections preprocessing here if any 
        data.update({'detections_list': detections_list})
        
        self.update_fusion(data)
        
        self.update_object_instances(data)
        
    def update_fusion(self, data: Dict)->None:
        '''
        Update the multi-view fusion result.
        Args:
            data: Dictionary of sensor data.
        '''
        # TODO: update the multi-view fusion result
        self.camera_names = data['camera_names']
        self.camera_intrinsics = data['depth_camera_intrinsic_list']
        self.camera_extrinsics = data['depth_camera_extrinsic_list']
        
        # reset the tsdf volume and integrate the depth images
        self.scene_tsdf_full.reset()
        self.scene_tsdf_masked.reset()
        
        for i, name in enumerate(data['camera_names']):
            
            # convert sensor_msgs/Image
            rgb = data['rgb_image_list'][i]
            depth = data['depth_image_list'][i]
            detections = data['detections_list'][i]
            bbox_2d_list = detections['bbox_2d_list']
            
            # NOTE: assume the depth has been aligned with rgb
            assert rgb.shape[0] == depth.shape[0] and rgb.shape[1] == depth.shape[1]
            intrinsic = data['depth_camera_intrinsic_list'][i]
            extrinsic = data['depth_camera_extrinsic_list'][i]
                          
            # calculate the mask image with all bounding boxes inner regions set to 1
            mask_img = np.zeros_like(depth)
            for bbox in bbox_2d_list:
                mask_img[bbox[1]:bbox[3], bbox[0]:bbox[2]] = 1
                          
                          
            self.scene_tsdf_full.integrate(depth, intrinsic, extrinsic, rgb_img=rgb)
            self.scene_tsdf_masked.integrate(depth, intrinsic, extrinsic, rgb_img=rgb, mask_img=mask_img)

    def update_object_instances(self, data: Dict)->None:
        '''
        Update the object instances in the scene:
            - object_names: list of object names
            - detections_dict: dictionary of 2D/3D detection results
        This function first clusters the instances on masked tsdf volume and then match 2D detections to 3D instances.
        '''
        detections_list = data['detections_list']
        
        # get point clouds from tsdf volume 
        cloud = self.scene_tsdf_masked.get_cloud()
        
        
    
    def get_object_names(self):
        '''
        Get the list of object names in the scene.
        '''
        return self.object_names
        
    def get_object_center_position(self, obj_name):
        '''
        Get the position of the object in the world frame. 
        This function uses ground truth model state from gazebo and ignore all other parameters.
        '''
        # TODO: finish this function
        pass
            
    def get_3d_bbox(self, object_name):
        '''
        Get the 3D bounding box of the object in the world frame.
        '''
        # TODO: finish this function 
        pass
        