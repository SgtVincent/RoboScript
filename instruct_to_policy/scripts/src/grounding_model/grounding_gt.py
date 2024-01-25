import os 
import numpy as np
from typing import List, Dict, Tuple

from .grounding_base import GroundingBase
from src.perception.utils import (
    CameraIntrinsic, 
    Transform,
    project_2d_to_3d,
    get_instance_bbox_2d
)


class GroundingGT(GroundingBase):
    """
    Class to provide GT 2D detection result from GT 3D bounding box and inverse-projection onto 2D.
    """
    def __init__(self, **kwargs):
        self.model = kwargs.get('model', None)
        self.model_type = kwargs.get('model_type', None)

    def load_model(self):
        """
        This function should initialize/load model with member parameters.
        
        Example: 
        model = GroundingModelXXX()
        model.load_model()
        """
        raise NotImplementedError
    
    def query_text(self, **kwargs)-> str:
        """
        This function should take in input and return query answer as text.
        """
        raise NotImplementedError
    
    def query_2d_bbox_list(self, sensor_data, object_list, **kwargs)-> List[Dict[str, List[float]]]:

        min_bbox_size = kwargs.get('min_bbox_size', 5)
        bbox_3d_dict = sensor_data['bbox_3d_dict']
        bboxes_list=[]
        # collect detection annotations for each camera
        for i, camera in enumerate(sensor_data['camera_names']):
            
            depth = sensor_data['depth_image_list'][i]
            intrinsics: CameraIntrinsic = sensor_data['depth_camera_intrinsic_list'][i]
            extrinsics: Transform = sensor_data['depth_camera_extrinsic_list'][i]
        
            camera_info_dict = intrinsics.to_dict() 
            camera_info_dict.update(extrinsics.to_dict())
                        
            # back-project depth image to 3D point cloud
            points, pixel_coords = project_2d_to_3d(depth, intrinsics, extrinsics)
            bbox_dict = {}
            # generate 2d bbox for each object 
            for j, category_name in enumerate(object_list):
                for object_id in bbox_3d_dict.keys():
                    if category_name in object_id:
                        bbox_3d_center, bbox_3d_size = bbox_3d_dict[object_id]
                        
                        # get the instance 2d bbox in the depth image 
                        bbox_2d, instance_pixels = get_instance_bbox_2d(
                            np.array(bbox_3d_center), np.array(bbox_3d_size), points, pixel_coords)            # filter empty bbox 
                        
                        # filter out empty bbox
                        if bbox_2d is None or instance_pixels is None:
                            bbox_dict[object_id] = [] 
                            continue
                        
                        # filter out too small bbox 
                        if bbox_2d[2] - bbox_2d[0] < min_bbox_size or bbox_2d[3] - bbox_2d[1] < min_bbox_size:
                            bbox_dict[object_id] = [] 
                            continue
                        
                        bbox_dict[object_id] = bbox_2d.tolist()
                        
            bboxes_list.append(bbox_dict)
             
        return bboxes_list