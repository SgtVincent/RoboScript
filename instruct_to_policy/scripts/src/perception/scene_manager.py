import os 
from typing import List, Tuple, Dict
from numpy.typing import ArrayLike
import numpy as np 
import open3d as o3d
import cv2
from .utils import (
    CameraIntrinsic, 
    Transform, 
    open3d_frustum_filter,
    match_bboxes_points_overlapping,
    draw_multiview_bbox_matches
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
        self.bbox_drop_base_margin = kwargs.get('bbox_drop_base_margin', True)  
        self.bbox_base_margin = kwargs.get('bbox_base_margin', 0.01)
        
        if not self.bbox_drop_base_margin:
            self.bbox_base_margin = 0.0
        
        self.camera_names = []
        self.camera_intrinsics = []
        self.camera_extrinsics = []
        self.object_names = []
        self.category_to_object_name_dict = {}
        self.object_2d_bbox_dict: Dict[str, List] = {}
        self.bbox_3d_dict: Dict[str, List] = {}
        
        # data for 2d bbox matching debug visualization
        self._last_rgb_image_list = []
        self._last_bbox_2d_list = []
        self._last_bboxes_match_tuple_list = []
        
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
        
        
    def update(self, data: Dict, **kwargs)->None:
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
        use_gt_3d_bboxes = kwargs.get('use_gt_3d_bboxes', False)
        
        # update 3D fusion by TSDF integration
        self.update_fusion(data)
        if use_gt_3d_bboxes:
            # use ground truth 3D bounding boxes
            self.update_3d_bboxes(data, use_gt_3d_bboxes=True)
        else:
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
                if len(bbox) > 0:
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
        NOTE: 
        The raw detections will have label format '<object_category>_<instance_id>', e.g. mug_0, apple_1, etc.
        When matching the bounding boxes, we only consider the object category, e.g. mug, apple, etc.
        Then we assign new instance id to each matched bounding box tuple.
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
        # record debug data
        self._last_rgb_image_list = data['rgb_image_list']
        self._last_bbox_2d_list = bboxes_2d_list
        self._last_bboxes_match_tuple_list = bboxes_match_tuple_list
        
        # unify the matched bounding box names of a same object instance under different camera views,
        # and update self.object_names, self.category_to_object_name_dict
        self.unify_bbox_names(bboxes_match_tuple_list, bboxes_labels_list)
        
        # update object names and 2D bounding boxes
        self.object_2d_bbox_dict = {}
        for bbox_name, match_idx_tuple in zip(self.object_names, bboxes_match_tuple_list):
            self.object_2d_bbox_dict[bbox_name] = []
            for i, idx in enumerate(match_idx_tuple):
                # idx -1 means no match, append empty bounding box 
                if idx != -1:
                    self.object_2d_bbox_dict[bbox_name].append(bboxes_2d_list[i][idx])
                else:
                    self.object_2d_bbox_dict[bbox_name].append([])
                
    def update_3d_bboxes(self, data: Dict, use_gt_3d_bboxes=False)->None:
        '''
        Update the 3D bounding boxes for each object.
        It filters the points inside the 2D bounding boxes and then fit the 3D bounding boxes.
        TODO: Can we have some clustering or segmentation methods to get tighter 3D bounding boxes?
        '''
        pcl = self.scene_tsdf_masked.get_cloud()
        
        if use_gt_3d_bboxes:
            # update self.object_names, self.category_to_object_name_dict
            self.object_names = data['bbox_3d_dict'].keys()
            # FIXME: consider the case there are multiple objects with the same category
            self.category_to_object_name_dict =  {
                object_name.split('_')[0]: [object_name] for object_name in self.object_names
            }
            
            # use ground truth 3D bounding boxes
            for object_name in self.object_names:
                bbox_center, bbox_size = data['bbox_3d_dict'][object_name]
                self.bbox_3d_dict[object_name] = [
                    bbox_center[0] - bbox_size[0] / 2, bbox_center[1] - bbox_size[1] / 2, bbox_center[2] - bbox_size[2] / 2,
                    bbox_center[0] + bbox_size[0] / 2, bbox_center[1] + bbox_size[1] / 2, bbox_center[2] + bbox_size[2] / 2,
                ]
            # FIXME: Should we also modify the bbox by filtering the points inside the 2D bounding boxes?
        
        else:
            # get 3D bounding boxes for each object by filtering the points inside the 2D bounding boxes and get its aabb
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
                    aabb.min_bound[0], aabb.min_bound[1], aabb.min_bound[2] + self.bbox_base_margin,
                    aabb.max_bound[0], aabb.max_bound[1], aabb.max_bound[2],
                ]

    def unify_bbox_names(self, bboxes_match_tuple_list: List[Tuple[int]], bboxes_labels_list: List[List[str]])-> List[Tuple[str, str]]:
        '''
        Unify the matched bounding box names of a same object instance under different camera views and update self.object_names, self.category_to_object_name_dict
        Rename the object instance with max-voted category and instance id. 
        Each bbox_label is in the format of '<object_category>_<instance_id>', e.g. mug_0, apple_1, etc.
        '''
        object_name_list = []
        categories_to_name = {} 
        # Each touple of bboxes belongs to the same object instance
        for match_idx_tuple in bboxes_match_tuple_list:
            match_categories = []
    
            for i, idx in enumerate(match_idx_tuple):
                # idx -1 means no match, so we skip it
                if idx != -1:
                    bbox_label = bboxes_labels_list[i][idx]
                    # split the category and instance id by last '_'
                    category, instance_id = bbox_label.rsplit('_', 1)
                    match_categories.append(category)
            # get max voted category
            max_voted_category = max(match_categories, key=match_categories.count)
            
            # create object name and add it to the category_to_name dict
            if max_voted_category not in categories_to_name:
                categories_to_name[max_voted_category] = []
            object_name = max_voted_category + '_' + str(len(categories_to_name[max_voted_category]))
            categories_to_name[max_voted_category].append(object_name)
            object_name_list.append(object_name)
            
        # update self.object_names, self.category_to_object_name_dict
        self.object_names = object_name_list
        self.category_to_object_name_dict = categories_to_name
        
    def get_object_names(self)->str:
        '''
        Get the list of object names in the scene.
        '''
        return self.object_names
        
    def get_object_center_position(self, obj_name)->ArrayLike:
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
            
    def get_object_mesh(self, object_name):
        '''
        Get object mesh by cropping mesh with the 3D bounding box of the object.
        '''
        object_bbox = self.bbox_3d_dict[object_name]
        bbox_center = np.array([
            (object_bbox[0] + object_bbox[3]) / 2,
            (object_bbox[1] + object_bbox[4]) / 2,
            (object_bbox[2] + object_bbox[5]) / 2,
        ])
        bbox_size = np.array([
            object_bbox[3] - object_bbox[0],
            object_bbox[4] - object_bbox[1],
            object_bbox[5] - object_bbox[2],
        ])
        object_mesh = self.scene_tsdf_masked.crop_mesh(
            crop_center=bbox_center,
            crop_size=bbox_size,
        )             
            
        return object_mesh
    
    def get_object_cloud(self, object_name):
        '''
        Get object point cloud by cropping point cloud with the 3D bounding box of the object.
        '''
        object_bbox = self.bbox_3d_dict[object_name]
        bbox_center = np.array([
            (object_bbox[0] + object_bbox[3]) / 2,
            (object_bbox[1] + object_bbox[4]) / 2,
            (object_bbox[2] + object_bbox[5]) / 2,
        ])
        bbox_size = np.array([
            object_bbox[3] - object_bbox[0],
            object_bbox[4] - object_bbox[1],
            object_bbox[5] - object_bbox[2],
        ])
        object_cloud = self.scene_tsdf_masked.crop_cloud(
            crop_center=bbox_center,
            crop_size=bbox_size,
        )             
            
        return object_cloud
    
            
    def get_object_2d_bbox_list(self, object_name)->List[List]:
        '''
        Get the list of 2D bounding boxes of the object in different camera views.
        '''
        return self.object_2d_bbox_dict[object_name]
            
    def get_3d_bbox(self, object_name)->np.ndarray:
        '''
        Get the 3D bounding box of the object in the world frame.
        '''
        return np.array(self.bbox_3d_dict[object_name])
        
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
        
        
        
    def visualize_2d_bboxes_matching(self, save_path=""):
        '''
        Visualize the 2D bounding boxes matching results in rgb images
        '''
        # use draw_multiview_bbox_matches
        vis_canvas = draw_multiview_bbox_matches(
            rgb_image_list=self._last_rgb_image_list,
            bboxes_2d_list=self._last_bbox_2d_list,
            matched_bboxes_idx_tuple_list=self._last_bboxes_match_tuple_list,
        )
        
        # convert to BGR for cv2
        vis_canvas = cv2.cvtColor(vis_canvas, cv2.COLOR_BGR2RGB)
        
        if len(save_path) > 0:
            cv2.imwrite(save_path, vis_canvas)
        else:
            # visualize the result
            cv2.imshow("vis_canvas", vis_canvas)
            cv2.waitKey(0)
