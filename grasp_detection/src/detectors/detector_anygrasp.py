import os 
import sys
import numpy as np 
from typing import List, Tuple, Dict
import open3d as o3d 
from scipy.spatial.transform import Rotation as R
import graspnetAPI 

# ROS
import rospy 
from grasp_detection.msg import Grasp, Perception, PerceptionSingleCamera, BoundingBox2D
from grasp_detection.srv import DetectGrasps, DetectGraspsRequest, DetectGraspsResponse
from geometry_msgs.msg import Pose, Point, Quaternion

# vgn utils 
from vgn.perception import TSDFVolume, ScalableTSDFVolume

# anygrasp models
current_dir = os.path.dirname(os.path.realpath(__file__))
anygrasp_detection_dir = os.path.join(current_dir, "anygrasp_sdk", "grasp_detection")
sys.path.append(anygrasp_detection_dir)
from gsnet import AnyGrasp
from .detector_base import DetectorBase
from .config import ConfigAnygrasp
from .utils import CameraIntrinsic, Transform, Rotation, get_mask_from_2D_bbox, open3d_frustum_filter


class DetectorAnygrasp(DetectorBase):
    def __init__(self, service_name, config):
        super().__init__(service_name, config)
        self.config: ConfigAnygrasp
        self.model = None
        self.max_grasp_num = self.config.max_grasp_num
        self.volume_type = self.config.volume_type
        
    def load_model(self):
        self.model = AnyGrasp(self.config)
        self.model.load_net()

    def detect_callback(self, req: DetectGraspsRequest)->DetectGraspsResponse:
        """
        Callback function for the ROS service. 
        """
        # preprocess perception data: integrate depth images into TSDF volume and point cloud 
        cloud, min_bound, max_bound = self._preprocess(req)
        points = np.asarray(cloud.points)
        colors = np.asarray(cloud.colors)
        # lims: [xmin, xmax, ymin, ymax, zmin, zmax]
        lims = [min_bound[0], max_bound[0], min_bound[1], max_bound[1], min_bound[2], max_bound[2]]
        
        # get prediction
        gg, cloud = self.model.get_grasp(points, colors, lims)
        
        if len(gg) == 0:
            print('No Grasp detected by Anygrasp model!')

        gg:graspnetAPI.GraspGroup = gg.nms().sort_by_score()
        gg = gg[0:self.max_grasp_num]
        
        # compose response
        grasps_msg = []
        for i, grasp in enumerate(gg):
            grasp: graspnetAPI.Grasp
            grasp_msg = Grasp()
            grasp_msg.object_id = "" # not used yet 
            
            grasp_msg.grasp_pose = Pose()
            grasp_msg.grasp_pose.position = Point(*grasp.translation)
            grasp_msg.grasp_pose.orientation = Quaternion(*R.from_matrix(grasp.rotation).as_quat())
            
            grasp_msg.grasp_score = grasp.score
            grasp_msg.grasp_width = grasp.width

        response = DetectGraspsResponse(grasps=grasps_msg)
        return response

    def _preprocess(self, data: Perception)->Tuple[TSDFVolume, np.ndarray]:
        """
        Preprocess data for grasp detection:
        - integrate depth images into TSDF volume
        - return TSDF volume and point cloud
        """
        if self.volume_type == "uniform":
            tsdf = TSDFVolume(self.voxel_grid_size, self.resolution, color_type=self.color_type)
        elif self.volume_type == "scalable":
            tsdf = ScalableTSDFVolume(self.voxel_grid_size, self.resolution, color_type=self.color_type, voxel_size=self.voxel_size)

        for i, camera_data in enumerate(data.cameras_data):
            camera_data: PerceptionSingleCamera
            rgb = camera_data.rgb_image
            depth = camera_data.depth_image 
            
            camera_info_msg = camera_data.depth_camera_info
            intrinsics = CameraIntrinsic(
                width=camera_info_msg.width,
                height=camera_info_msg.height,
                fx=camera_info_msg.K[0],
                fy=camera_info_msg.K[4],
                cx=camera_info_msg.K[2],
                cy=camera_info_msg.K[5],   
            )
            
            camera_pose_msg = camera_data.depth_camera_pose
            extrinsics = Transform.from_dict(
                {
                    "translation": [
                        camera_pose_msg.position.x,
                        camera_pose_msg.position.y,
                        camera_pose_msg.position.z,
                    ],
                    "rotation": [
                        camera_pose_msg.orientation.x,
                        camera_pose_msg.orientation.y,
                        camera_pose_msg.orientation.z,
                        camera_pose_msg.orientation.w,
                    ]
                }
            )
            
            # NOTE: Assume only one detection in single request
            detection = camera_data.detections[0]
            mask=None
            if "depth_bboxes" in data:
                bbox = np.array([
                    detection.x_min,
                    detection.y_min,
                    detection.x_max,
                    detection.y_max
                ])
                mask = get_mask_from_2D_bbox(bbox, depth) 

            tsdf.integrate(depth, intrinsics, extrinsics, rgb_img=rgb, mask_img=mask)


        full_cloud: o3d.geometry.PointCloud = tsdf.get_cloud()
        
        # TODO: check this filtering part is really needed
        if 'bbox_3d' in data:
            # filter the cloud by the 3D bounding box
            bbox_3d_center = data['bbox_3d']['center']
            bbox_3d_size = data['bbox_3d']['size']
            
            min_bound = np.array(bbox_3d_center) - np.array(bbox_3d_size) / 2
            max_bound=np.array(bbox_3d_center) + np.array(bbox_3d_size) / 2
            filtered_cloud = full_cloud.crop(min_bound=min_bound, max_bound=max_bound)
                
        else:
            # filter the cloud with frustum filters and get the bounding box center 
            assert 'bbox_2d_list' in data, 'bbox_2d_list must be provided if bbox_3d is not provided'
            
            filtered_cloud, mask = open3d_frustum_filter(full_cloud, data['bbox_2d_list'],
                                                        data['depth_camera_intrinsic_list'],
                                                        data['depth_camera_extrinsic_list'])
            min_bound = filtered_cloud.get_min_bound()
            min_bound = filtered_cloud.get_max_bound()
            
        return filtered_cloud, min_bound, max_bound
        
    





