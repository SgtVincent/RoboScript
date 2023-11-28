import os 
from typing import List, Tuple, Dict
from numpy.typing import ArrayLike
import numpy as np 
import open3d as o3d

from .utils import (
    CameraIntrinsic, 
    Transform, 
    open3d_frustum_filter,
    match_bboxes_points_overlapping,
)
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
        
        self.bbox_min_match_th = kwargs.get('bbox_min_match_th', 0.1)
        self.bbox_match_downsample_voxel_size = 4 * self.tsdf_voxel_size
                 
        self.camera_names = []
        self.camera_intrinsics = []
        self.camera_extrinsics = []
        self.object_names = []
        self.object_2d_bbox_dict = {}
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
        
        
    def update(self, data: Dict)->None:
        '''
        Update the scene manager with new sensor data and new 2D detections. 
        Args:
            data: Dictionary of sensor data.
            data = {
                'camera_names': [],
                'rgb_image_list': [],
                'rgb_camera_intrinsic_list': [],
                'rgb_camera_frame_list': [],
                'rgb_camera_extrinsic_list': [],
                'depth_image_list': [],
                'depth_camera_intrinsic_list': [],
                'depth_camera_frame_list': [],
                'depth_camera_extrinsic_list': [],
                'detections_list':[
                    {'<object_id>': <2d bounding box>}, # Dictionary of 2D bounding boxes in the form of [min_x, min_y, max_x, max_y]
                ],
            }  
        '''
        # update 3D fusion by TSDF integration
        self.update_fusion(data)
        # update object instances by matching 2D bounding boxes and unifying matched bboxes names 
        self.update_object_instances(data)
        # update 3D bounding boxes for each object by filtering the points inside the 2D bounding boxes and fitting 3D bounding boxes
        self.update_3d_bboxes(data)
        
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
            bbox_2d_list = list(detections.values())
            
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
            - object_2d_bbox_dict: dictionary of 2D bounding boxes for each object
        This function first match the 2D bounding boxes with the points projection overlapping ratio,
        then unify the matched bounding box names under different camera views.
        '''
        bboxes_2d_list = [
            list(detections.values()) for detections in data['detections_list']
        ]
        bboxes_labels_list = [
            list(detections.keys()) for detections in data['detections_list']
        ]
        # get point clouds from tsdf volume 
        pcl = self.scene_tsdf_masked.get_cloud()
        
        # Match 2D bounding boxes with points projection overlapping ratio
        bboxes_match_tuple_list = match_bboxes_points_overlapping(
            bboxes_2d_list=bboxes_2d_list,
            intrinsics=data['depth_camera_intrinsic_list'],
            extrinsics=data['depth_camera_extrinsic_list'],
            pcl=pcl,
            downsample=True,
            downsample_voxel_size=self.bbox_match_downsample_voxel_size,
            min_match_th=self.bbox_min_match_th,
        )
        
        # unify the matched bounding box names under different camera views
        bboxes_name_list = self.unify_bbox_names(bboxes_match_tuple_list, bboxes_labels_list)
        
        # update object names and 2D bounding boxes
        self.object_names = bboxes_name_list
        self.object_2d_bbox_dict = {}
        for bbox_name, match_idx_tuple in zip(bboxes_name_list, bboxes_match_tuple_list):
            self.object_2d_bbox_dict[bbox_name] = []
            for i, idx in enumerate(match_idx_tuple):
                # idx -1 means no match, append empty bounding box 
                if idx != -1:
                    self.object_2d_bbox_dict[bbox_name].append(bboxes_2d_list[i][idx])
                else:
                    self.object_2d_bbox_dict[bbox_name].append([])
                
    def update_3d_bboxes(self, data: Dict)->None:
        '''
        Update the 3D bounding boxes for each object.
        It filters the points inside the 2D bounding boxes and then fit the 3D bounding boxes.
        TODO: Can we have some clustering or segmentation methods to get tighter 3D bounding boxes?
        '''
        pcl = self.scene_tsdf_masked.get_cloud()
        for object_name, bbox_2d_list in self.object_2d_bbox_dict.items():
            # get point clouds of the object by filtering the points inside the 2D bounding boxes
            obj_pcl, _ = open3d_frustum_filter(
                pcl=pcl,
                bbox_2d_list=bbox_2d_list,
                camera_intrinsic_list=data['depth_camera_intrinsic_list'],
                camera_extrinsic_list=data['depth_camera_extrinsic_list']
            )
            
            # fit 3D bounding box and convert it to [x_min, y_min, z_min, x_max, y_max, z_max] format
            aabb = obj_pcl.get_axis_aligned_bounding_box()
            
            self.bbox_3d_dict[object_name] = [
                aabb.min_bound[0], aabb.min_bound[1], aabb.min_bound[2],
                aabb.max_bound[0], aabb.max_bound[1], aabb.max_bound[2],
            ]

    def unify_bbox_names(self, bboxes_match_tuple_list: List[Tuple[int]], bboxes_labels_list: List[List[str]])-> List[Tuple[str, str]]:
        '''
        Unify the matched bounding box names under different camera views.
        NOTE: the consistency of multi-view detection labels should be garanteed by the grounding model. 
        This function only asserts the consistency of the matched bounding box names.
        '''
        bboxes_name_list = []
        for match_idx_tuple in bboxes_match_tuple_list:
            name_set = set()
            for i, idx in enumerate(match_idx_tuple):
                # idx -1 means no match, so we skip it
                if idx != -1:
                    name_set.add(bboxes_labels_list[i][idx])
            assert len(name_set) == 1, "The matched bounding box names are not consistent!, got names: {}".format(name_set)
            bboxes_name_list.append(name_set.pop())
        
        return bboxes_name_list
        
    
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
        obj_bbox_3d = self.bbox_3d_dict[obj_name]
        return np.array([
            (obj_bbox_3d[0] + obj_bbox_3d[3]) / 2,
            (obj_bbox_3d[1] + obj_bbox_3d[4]) / 2,
            (obj_bbox_3d[2] + obj_bbox_3d[5]) / 2,
        ])
            
    def get_object_2d_bbox_list(self, object_name):
        '''
        Get the list of 2D bounding boxes of the object in different camera views.
        '''
        return self.object_2d_bbox_dict[object_name]
            
    def get_3d_bbox(self, object_name):
        '''
        Get the 3D bounding box of the object in the world frame.
        '''
        return self.bbox_3d_dict[object_name]
        
    def visualize_3d_bboxes(self, show_masked_tsdf=False, show_full_tsdf=True):
        '''
        Visualize the 3D bounding boxes and point cloud in the scene with open3d.
        '''
        vis_list = []
        # add 3D bounding boxes 
        for obj_name in self.object_names:
            bbox_3d = self.bbox_3d_dict[obj_name]
            # create a axis-aligned bounding box 
            bbox = o3d.geometry.AxisAlignedBoundingBox(
                min_bound=np.array(bbox_3d[:3]),
                max_bound=np.array(bbox_3d[3:])
            )
            vis_list.append(bbox)
            
        if show_masked_tsdf:
            vis_list.append(self.scene_tsdf_masked.get_mesh())
            
        if show_full_tsdf:
            vis_list.append(self.scene_tsdf_full.get_mesh())
        
        # visualize the scene
        o3d.visualization.draw_geometries(vis_list)
        
        
        
