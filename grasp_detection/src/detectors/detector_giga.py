import os 
import numpy as np 
from typing import List, Tuple, Dict

# ROS
import rospy 
from grasp_detection.msg import Grasp, Perception, PerceptionSingleCamera, BoundingBox2D
from grasp_detection.srv import DetectGrasps, DetectGraspsRequest, DetectGraspsResponse

# GIGA models 
from vgn.detection_implicit import VGNImplicit
from vgn.experiments.clutter_removal import State
from vgn.perception import TSDFVolume, ScalableTSDFVolume
from vgn.utils.visual import grasp2mesh, plot_voxel_as_cloud, plot_tsdf_with_grasps
from vgn.utils.implicit import get_mesh_pose_list_from_world, get_scene_from_mesh_pose_list

from detectors.detector_base import DetectorBase
from detectors.utils import CameraIntrinsic, Transform, Rotation, get_mask_from_2D_bbox, open3d_frustum_filter


class DetectorGIGA(DetectorBase):
    def __init__(self, service_name, config):
        super().__init__(service_name, config)
        self.model = None
        
    def load_model(self):
        self.model = VGNImplicit(self.model_path,
            self.model_type,
            best=True, # rank grasps by score
            qual_th=self.qual_th,
            visualize=False, # DO NOT use vgn visualization since gt mesh not available in the pipeline 
            force_detection=True,
            out_th=self.out_th,
            resolution=self.resolution)

    def detect_callback(self, req: DetectGraspsRequest)->DetectGraspsResponse:
        """
        Callback function for the ROS service. 
        """
        
        # preprocess raw data to get model input
        tsdf, pc = self._preprocess(req)
        state = State(tsdf, pc)
        grasps, scores, _ = self.model(state)
        
        print(f"{len(grasps)} Grasps generated.")

        # shift grasp translation by tsdf origin
        tsdf_origin = tsdf.cropped_grid.origin
        for i, grasp in enumerate(grasps):
            grasps[i].pose.translation = grasp.pose.translation + tsdf_origin

        if len(grasps) == 0:
            # return [], [], [tsdf.get_mesh()]
            return [], [], [tsdf.get_cloud()]
        pos = grasps[0].pose.translation
        # pos[2] += 0.05
        angle = grasps[0].pose.rotation.as_euler('xyz')
        print(pos, angle)
        if angle[2] > np.pi / 2 or angle[2] < - np.pi / 2:
            reflect = Transform(Rotation.from_euler('xyz', (0, 0, np.pi)), np.zeros((3)))
            grasps[0].pose = grasps[0].pose * reflect

        # compose response
        grasps_msg = []
        for grasp, score in zip(grasps, scores):
            grasp_msg = Grasp()
            grasp_msg.grasp_pose.position.x, grasp_msg.grasp_pose.position.y, grasp_msg.grasp_pose.position.z = grasp.pose.translation
            grasp_msg.grasp_pose.orientation.x, grasp_msg.grasp_pose.orientation.y, grasp_msg.grasp_pose.orientation.z, grasp_msg.grasp_pose.orientation.w = grasp.pose.rotation.as_quat()
            grasp_msg.grasp_score = score
            grasps_msg.append(grasp_msg)
        response = DetectGraspsResponse(grasps=grasps_msg)
        return response

    def _preprocess(self, data: Perception)->Tuple[TSDFVolume, np.ndarray]:
        """
        Preprocess data for grasp detection:
        - integrate depth images into TSDF volume
        - return TSDF volume and point cloud
        """
        if self.volume_type == "uniform":
            tsdf = TSDFVolume(self.voxel_grid_size, self.resolution)
        elif self.volume_type == "scalable":
            tsdf = ScalableTSDFVolume(self.voxel_grid_size, self.resolution)

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

        if self.volume_type == "scalable":
            
            if 'bbox_3d' in data:
                bbox_3d_center = data['bbox_3d']['center']
                bbox_3d_size = data['bbox_3d']['size']
            else:
                # filter the cloud with frustum filters and get the bounding box center 
                assert 'bbox_2d_list' in data, 'bbox_2d_list must be provided if bbox_3d is not provided'
                full_cloud = tsdf._volume.extract_point_cloud()
                filtered_pc, mask = open3d_frustum_filter(full_cloud, data['bbox_2d_list'],
                                                            data['depth_camera_intrinsic_list'],
                                                            data['depth_camera_extrinsic_list'])
                bbox_3d_center = filtered_pc.get_center()
                bbox_3d_size = self.voxel_grid_size * np.ones((3))
            
            tsdf.set_region_of_interest(bbox_3d_center, bbox_3d_size)
            
        pc = tsdf.get_cloud()
        
        return tsdf, pc 
    





