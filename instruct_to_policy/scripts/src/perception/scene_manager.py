import os 
from typing import List, Tuple, Dict
from numpy.typing import ArrayLike
import numpy as np 
import open3d as o3d
import torch
import time
import matplotlib.pyplot as plt
from collections import defaultdict
import copy 

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
        self.bbox_drop_base_margin = kwargs.get('bbox_drop_base_margin', True)  
        self.bbox_base_margin = kwargs.get('bbox_base_margin', 0.01)

        self.detect_approach = kwargs.get('detect_approach', 'mask') # bbox
        
        if not self.bbox_drop_base_margin:
            self.bbox_base_margin = 0.0
        
        self.camera_names = []
        self.camera_intrinsics = []
        self.camera_extrinsics = []
        self.object_names = []
        self.category_to_object_name_dict = {}
        self.object_detect_dict: Dict[str, List] = {}
        self.bbox_3d_dict: Dict[str, List] = {}

        self.bbox_oriented_3d_dict: Dict[str, o3d.geometry.OrientedBoundingBox] = {}
        
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

        o3d.io.write_point_cloud("/home/cuite/storage/scene.pcd", self.scene_tsdf_full.get_cloud())
        
    def update_fusion(self, data: Dict)->None:
        '''
        Update the multi-view fusion result.
        Args:
            data: Dictionary of sensor data.
        '''
        # TODO: update the multi-view fusion result
        now = time.time()
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
            # detections = data['detections_list'][i]
            # bbox_2d_list = list(detections.values())
            # bbox_2d_list = data['detections_list'][i].values()
            detection_list = data['detections_list'][i].values()
            
            # NOTE: assume the depth has been aligned with rgb
            assert rgb.shape[0] == depth.shape[0] and rgb.shape[1] == depth.shape[1]
            intrinsic = data['depth_camera_intrinsic_list'][i]
            extrinsic = data['depth_camera_extrinsic_list'][i]
                          
            # calculate the mask image with all bounding boxes inner regions set to 1
            mask_img = np.zeros_like(depth)
            for detect_result in detection_list:
                if self.detect_approach == 'mask':
                    mask = detect_result['mask']
                    mask_img[mask.cpu().numpy()] = 1
                else:
                    bbox = detect_result['bbox']
                    mask_img[bbox[1]:bbox[3], bbox[0]:bbox[2]] = 1
                          
                          
            self.scene_tsdf_full.integrate(depth, intrinsic, extrinsic, rgb_img=rgb)
            self.scene_tsdf_masked.integrate(depth, intrinsic, extrinsic, rgb_img=rgb, mask_img=mask_img)
        
        # pcl = self.scene_tsdf_full.get_cloud()
        # print('------------begin--------------')
        # o3d.visualization.draw_geometries([pcl])

        print(f'-------------------------------------------cost_time:   {time.time()-now}----------------------------------------')

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
        detection_list = [
            list(detections.values()) for detections in data['detections_list']
        ]
        bboxes_labels_list = [
            list(detections.keys()) for detections in data['detections_list']
        ]
        # get point clouds from tsdf volume 
        pcl = self.scene_tsdf_masked.get_cloud()
        
        # Match 2D bounding boxes with points projection overlapping ratio
        bboxes_match_tuple_list = match_bboxes_points_overlapping(
            detection_list=detection_list,
            intrinsics=data['depth_camera_intrinsic_list'],
            extrinsics=data['depth_camera_extrinsic_list'],
            pcl=pcl,
            downsample=True,
            downsample_voxel_size=self.bbox_match_downsample_voxel_size,
            min_match_th=self.bbox_min_match_th,
            detect_approach=self.detect_approach
        )
        
        # unify the matched bounding box names of a same object instance under different camera views,
        # and update self.object_names, self.category_to_object_name_dict
        self.unify_bbox_names(bboxes_match_tuple_list, bboxes_labels_list)
        
        # update object names and 2D bounding boxes
        self.object_detect_dict = {}
        for bbox_name, match_idx_tuple in zip(self.object_names, bboxes_match_tuple_list):
            self.object_detect_dict[bbox_name] = []
            for i, idx in enumerate(match_idx_tuple):
                # idx -1 means no match, append empty bounding box 
                if idx != -1:
                    self.object_detect_dict[bbox_name].append(detection_list[i][idx])
                else:
                    self.object_detect_dict[bbox_name].append([])

        self.update_object_name_with_confidence()

    def update_object_name_with_confidence(self)->None:
        CONF_CAL = 'mean' # mean, max
        object_detect_dict = copy.deepcopy(self.object_detect_dict)
        # object_name_list_with_idx = [key for key in object_detect_dict.keys()]
        object_name_dict = defaultdict(list)
        for object_name_with_idx in object_detect_dict.keys():
            object_name = object_name_with_idx.split('_')[0]
            object_name_dict[object_name].append(object_name_with_idx)
        for object_name in object_name_dict.keys():
            object_name_list_with_idx = object_name_dict[object_name]
            confidence_list = []
            for object_name_with_idx in object_name_list_with_idx:
                detect_list = self.object_detect_dict[object_name_with_idx]
                object_conf = []
                for detect_result in detect_list:
                    if len(detect_result) != 0:
                        object_conf.append(detect_result['score'])
                if CONF_CAL == 'mean':
                    confidence = sum(object_conf) / len(object_conf)
                elif CONF_CAL == 'max':
                    confidence = max(object_conf)
                confidence_list.append(confidence)
            confidence_array = np.array(confidence_list)
            sorted_index = np.argsort(confidence_array)
            sorted_index = sorted_index[::-1]
            for i, index in enumerate(sorted_index):
                object_detect_dict[f'{object_name}_{i}'] = self.object_detect_dict[object_name_list_with_idx[index]]
        self.object_detect_dict = object_detect_dict

                
    def update_3d_bboxes(self, data: Dict)->None:
        '''
        Update the 3D bounding boxes for each object.
        It filters the points inside the 2D bounding boxes and then fit the 3D bounding boxes.
        TODO: Can we have some clustering or segmentation methods to get tighter 3D bounding boxes?
        '''
        #################### AxisAlignedBoundingBox either has zeros size, or has wrong bounds #############33
        pcl = self.scene_tsdf_masked.get_cloud()
        # print('------------begin--------------')
        # o3d.visualization.draw_geometries([pcl])
        ### lhy debug
        object_cnt = 0
        for object_name, detection_list in self.object_detect_dict.items():
            # get point clouds of the object by filtering the points inside the 2D bounding boxes
            obj_pcl, _ = open3d_frustum_filter(
                pcl=pcl,
                detection_list=detection_list,
                camera_intrinsic_list=data['depth_camera_intrinsic_list'],
                camera_extrinsic_list=data['depth_camera_extrinsic_list'],
                detect_approach = self.detect_approach
            )
            # print('---------before-----------------')
            # o3d.visualization.draw_geometries([obj_pcl])
            object_cnt += 1
            o3d.io.write_point_cloud("/home/cuite/storage/"+object_name+str(object_cnt)+".pcd", obj_pcl)
            fig_id_list = []
            for detect_result in detection_list:
                if len(detect_result) != 0:
                    fig_id_list.append(detect_result['fig_id'])
            print('object_name', object_name, fig_id_list)
            MinPts = 5
            R = 0.05
            obj_pcl, idx = obj_pcl.remove_radius_outlier(MinPts,R)
            # print('---------after-----------------')
            # o3d.visualization.draw_geometries([obj_pcl])
            # fit 3D bounding box and convert it to [x_min, y_min, z_min, x_max, y_max, z_max] format
            aabb = obj_pcl.get_axis_aligned_bounding_box()
            
            self.bbox_3d_dict[object_name] = [
                aabb.min_bound[0], aabb.min_bound[1], aabb.min_bound[2] + self.bbox_base_margin,
                aabb.max_bound[0], aabb.max_bound[1], aabb.max_bound[2],
            ]

            self.bbox_oriented_3d_dict[object_name] = obj_pcl.get_oriented_bounding_box()

    def update_pos_list(self, data: Dict)->None:
        self.pos_list = data['pos_list']

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
        print("____________________obj_bbox_3d_____________________")
        print(obj_bbox_3d)
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
        # print('self.bbox_3d_dict', self.bbox_3d_dict)
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
            
    def get_object_2d_bbox_list(self, object_name)->List[List]:
        '''
        Get the list of 2D bounding boxes of the object in different camera views.
        '''
        return self.object_detect_dict[object_name]
            
    def get_3d_bbox(self, object_name)->ArrayLike:
        '''
        Get the 3D bounding box of the object in the world frame.
        '''
        return np.array(self.bbox_3d_dict[object_name])
    
    def get_object_camera_idx(self, object_name)->int:
        detect_result_list = self.object_detect_dict[object_name]
        for detect_result in detect_result_list:
            if len(detect_result) != 0:
                print('camera_id', detect_result['camera_id'])
                return detect_result['camera_id']
        
    def visualize_3d_bboxes(self, show_masked_tsdf=False, show_full_tsdf=True, show_oriented_box=False):
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

            if show_oriented_box:
                print("~~~show oriented box~~~")
                obbox = self.bbox_oriented_3d_dict[obj_name]
                obbox.color = [1., 0, 0]
                vis_list.append(obbox)
            
        if show_masked_tsdf:
            vis_list.append(self.scene_tsdf_masked.get_mesh())
            
        if show_full_tsdf:
            vis_list.append(self.scene_tsdf_full.get_mesh())
        
        # visualize the scene
        o3d.visualization.draw_geometries(vis_list)
        

    def visualize_vec(self, start_points: list, vecs: list, draw_axis: bool=False):
        vis_list = []
        vis_list.append(self.scene_tsdf_full.get_mesh())
        for start, vec in zip(start_points, vecs):
            end_point = start + vec
            line = o3d.geometry.LineSet()
            line.points = o3d.utility.Vector3dVector(np.vstack((start, end_point)))
            line.lines = o3d.utility.Vector2iVector([[0, 1]])
            line.colors = o3d.utility.Vector3dVector([[1, 0, 0]])  # 设置线段颜色为红色
            vis_list.append(line)
        if draw_axis:
            axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
            vis_list.append(axis)

        o3d.visualization.draw_geometries(vis_list)
        
        
